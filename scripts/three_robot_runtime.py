"""Three independent RGB clients with saved requests and bounded inference."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import io
import json
import time
from urllib.request import urlopen

from harness.gemini_proxy import GeminiProxyCompleter
from harness.research_execution_recovery import request_with_recovery
from harness.three_robot_plan import (ROBOTS, TeamAgreement, build_plan_request,
    build_inspection_request, validate_plan_reply, validate_inspection_reply, fixture_plan)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def refs(frames):
    return {r: {'own': f['own_rgb'], 'top': f['shared_top_rgb'], 'frame_id': f['frame_id']}
            for r, f in frames.items()}


class ThreeRobotRuntime:
    def __init__(self, output, *, run_id, mode='llm', fixture_timing='during_approach',
                 agreement=None, request_builder=build_plan_request,
                 reply_validator=validate_plan_reply, plan_fixture=None):
        self.output, self.mode, self.fixture_timing = output, mode, fixture_timing
        self.agreement = agreement or TeamAgreement(run_id)
        self.request_builder, self.reply_validator = request_builder, reply_validator
        self.plan_fixture = plan_fixture
        self.inbox = {r: [] for r in ROBOTS}
        self.calls, self.rounds, self.inspections = [], [], []
        self.execution_events = []
        self.started = time.monotonic()
        self.wires = {r: 0 for r in ROBOTS}
        self.clients = {}
        self.pool = ThreadPoolExecutor(max_workers=3, thread_name_prefix='three-rgb')
        self.inspection_future = None
        self.inspection_result = None
        self.closed = False
        for rid in ROBOTS:
            (output/rid).mkdir(parents=True, exist_ok=True)
            def audited_open(req, *, timeout, _rid=rid):
                self.wires[_rid] += 1
                path = output/_rid/f'wire-{self.wires[_rid]:03d}.json'
                path.write_bytes(req.data)
                with urlopen(req, timeout=timeout) as response:
                    data = response.read()
                path.with_name(path.stem+'-response.json').write_bytes(data)
                return io.BytesIO(data)
            self.clients[rid] = GeminiProxyCompleter(model='gemini-3.8-flash', max_tokens=950,
                timeout=30., reasoning_effort='none', http_open=audited_open)

    def _invoke(self, rid, request, validate, *, fixture_reply=None):
        path = f'{rid}/{request["request_id"]}-request.json'
        write(self.output/path, request)
        if self.mode == 'fixture':
            raw = json.dumps(fixture_reply)
            reply = validate(raw, request['request_id'])
            record = {'attempt': 0, 'request_id': request['request_id'], 'reply': reply,
                      'raw_response': raw, 'model': 'scripted-fixture-not-llm', 'usage': None,
                      'latency_ms': 0., 'request': path, 'robot_id': rid, 'wire_index': None,
                      'returned_wall_s': time.monotonic()-self.started}
            return reply, None, [record]
        records = []
        def record(row):
            row.update(request=path, robot_id=rid, wire_index=self.wires[rid],
                       returned_wall_s=time.monotonic()-self.started)
            records.append(row)
            write(self.output/rid/f'{request["request_id"]}-decision.json', row)
        reply, stop = request_with_recovery(self.clients[rid], lambda attempt, retry: request,
            validate, record, max_attempts=1,
            can_request=lambda: time.monotonic() - self.started < 600.)
        return reply, stop, records

    def negotiate(self, frames, own_history, turn, sim_time):
        context = self.agreement.context()
        requests, futures = {}, {}
        for rid in ROBOTS:
            request_id = f'{self.agreement.run_id}-{rid}-plan-{turn}'
            frame = frames[rid]
            requests[rid] = self.request_builder(rid, request_id=request_id,
                own_rgb=frame['own_bytes'], top_rgb=frame['top_bytes'], agreement=context,
                inbox=self.inbox[rid], own_history=own_history[rid])
            p = context['proposal']
            fixture = {'request_id': request_id, 'proposal_id': p['proposal_id'] if p else None,
                       'plan_hash': p['plan_hash'] if p else None, 'accept': True,
                       'plan': p['plan'] if p else (self.plan_fixture or fixture_plan(self.fixture_timing)),
                       'reason': 'scripted protocol fixture, not visual reasoning', 'message': ''}
            futures[rid] = self.pool.submit(self._invoke, rid, requests[rid],
                lambda raw, req, c=context: self.reply_validator(raw, req, c), fixture_reply=fixture)
        batch = {r: f.result() for r, f in futures.items()}
        replies = {r: v[0] for r, v in batch.items()}
        for reply, stop, records in batch.values():
            self.calls.extend(records)
        committed = self.agreement.receive(replies, turn)
        self.rounds.append({'turn': turn, 'sim_time_s': sim_time, 'agreement': context,
            'images': refs(frames), 'own_history': own_history,
            'replies': replies, 'stops': {r: v[1] for r, v in batch.items()}, 'committed': committed})
        for sender, reply in replies.items():
            if reply and reply['message']:
                for receiver in ROBOTS:
                    if sender != receiver:
                        self.inbox[receiver].append({'from_robot': sender, 'turn': turn, 'message': reply['message']})
        self.save()
        print(json.dumps({'team_turn': turn, 'committed': committed, 'stops': self.rounds[-1]['stops']}), flush=True)
        return committed

    def start_inspection(self, frame, sim_time):
        if self.inspection_future is not None or self.agreement.committed is None:
            raise ValueError('inspection already active or plan not committed')
        index = len(self.inspections)
        request_id = f'{self.agreement.run_id}-r2-inspect-{index}'
        request = build_inspection_request(request_id=request_id, own_rgb=frame['own_bytes'],
            top_rgb=frame['top_bytes'], committed_plan=self.agreement.committed)
        self.inspections.append({'index': index, 'observed_at_sim_s': sim_time,
            'started_wall_s': time.monotonic()-self.started, 'images': refs({'r2': frame})['r2'],
            'request_id': request_id, 'reply': None, 'stop': None})
        fixture = {'request_id': request_id, 'status': 'UNCERTAIN', 'confidence': 0.,
                   'reason': 'scripted fixture; no visual inspection claim', 'message': ''}
        self.inspection_result = None
        self.inspection_future = self.pool.submit(self._invoke, 'r2', request,
            validate_inspection_reply, fixture_reply=fixture)
        self.save()

    def collect_inspection(self, sim_time, *, wait=False):
        future = self.inspection_future
        if future is None or not (wait or future.done()):
            return self.inspection_result
        reply, stop, records = future.result()
        self.calls.extend(records)
        row = self.inspections[-1]
        row.update(reply=reply, stop=stop, received_at_sim_s=sim_time,
                   received_wall_s=time.monotonic()-self.started,
                   completed_wall_s=records[-1]['returned_wall_s'] if records else None)
        self.inspection_result = row
        self.inspection_future = None
        self.save()
        return row

    def save(self):
        write(self.output/'team.json', self.snapshot())

    def event(self, name, sim_time, **details):
        self.execution_events.append({'event': name, 'sim_time_s': sim_time,
                                      'wall_s': time.monotonic()-self.started, **details})
        self.save()

    def snapshot(self):
        return {'schema': 'ugrp.three_robot_runtime.v1', 'mode': self.mode,
                'run_id': self.agreement.run_id, 'rounds': self.rounds,
                'agreement_events': self.agreement.events, 'committed': self.agreement.committed,
                'inspections': self.inspections, 'calls': self.calls,
                'execution_events': self.execution_events,
                'transport_roles_fixed_by_skill': True,
                'inspection_has_no_motor_permission': self.plan_fixture is None,
                'physical_task_plan': self.plan_fixture is not None,
                'cost_usd': None, 'cost_note': 'provider billing unavailable'}

    def close(self, sim_time):
        if self.closed:
            return
        try:
            self.collect_inspection(sim_time, wait=True)
        finally:
            self.pool.shutdown(wait=True, cancel_futures=True)
            self.closed = True
            self.save()
