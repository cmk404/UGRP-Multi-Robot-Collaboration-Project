"""Ordered, CPU-only RGB analysis for the open pair carry.

The child receives immutable JPEGs and detached visual/map state. It never
owns a simulation, actor permission, controller, output file, or GL context.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import copy
import math
import multiprocessing
import time

from harness.camera_goal_transport import own_payload
from harness.dispatch_beam_tracker import CarriedBeamTracker
from harness.dispatch_skill_binding import BeamContinuity, ImageRoute, canonical_pair_top
from harness.dispatch_translation_skew import translation_skew


def detached_beam_route(route):
    """Copy only authored route and RGB-derived state, never live bindings."""
    if route.obj != 'beam':
        raise ValueError('process perception supports the open beam route only')
    return {'map': copy.deepcopy(route.map), 'task': copy.deepcopy(route.task),
            'dock': route.dock, 'route_overlap': route.route_overlap,
            'points': copy.deepcopy(route.points), 'index': route.index,
            'confirmations': route.confirmations,
            'beam_previous': copy.deepcopy(route.beam_tracker.previous)}


class PairPerceptionState:
    """The same sequential transformations as the original carry analyzer."""
    def __init__(self, reference, anchor_own, carried_previous,
                 continuity_previous, route_state):
        self.reference = bytes(reference)
        self.anchor_own = {slot: bytes(value) for slot, value in anchor_own.items()}
        self.carried_beam = CarriedBeamTracker()
        self.carried_beam.previous = copy.deepcopy(carried_previous)
        self.continuity = BeamContinuity()
        self.continuity.previous = copy.deepcopy(continuity_previous)
        route = ImageRoute.__new__(ImageRoute)
        route.obj = 'beam'
        route.map = copy.deepcopy(route_state['map'])
        route.task = copy.deepcopy(route_state['task'])
        route.dock = route_state['dock']
        route.route_overlap = bool(route_state['route_overlap'])
        route._permission = None
        route.points = copy.deepcopy(route_state['points'])
        route.index = int(route_state['index'])
        route.confirmations = int(route_state['confirmations'])
        route.beam_tracker = CarriedBeamTracker()
        route.beam_tracker.previous = copy.deepcopy(route_state['beam_previous'])
        self.route = route
        self.last_frame_id = None
        self.last_observed_at_s = None

    def analyze(self, frame_id, observed_at_s, raw_top, own_by_slot):
        if (self.last_frame_id is not None and frame_id <= self.last_frame_id) or (
                self.last_observed_at_s is not None and observed_at_s < self.last_observed_at_s):
            raise ValueError('pair perception RGB frames must be accepted in order')
        if set(own_by_slot) != {'r1', 'r3'}:
            raise ValueError('both own RGB images are required')
        started = time.monotonic()
        observed_beam = self.carried_beam.observe(raw_top)
        canonical, transform = canonical_pair_top(
            raw_top, self.reference, hue_upper=35, observed_beam=observed_beam)
        self.continuity.observe(transform['observed_beam'])
        bind_completed = time.monotonic()
        perception_started = time.monotonic()
        motion, evidence = self.route.observe(raw_top)
        decisions = {}
        for slot in ('r1', 'r3'):
            current = own_payload(own_by_slot[slot], hue_upper=35)
            initial = own_payload(self.anchor_own[slot], hue_upper=35)
            held = bool(current and initial and .25 <= current[0] / initial[0] <= 4
                        and math.dist(current[1:], initial[1:]) <= .15)
            decisions[slot] = {'ok': held, 'held_estimate': held,
                'ready': evidence['done'], 'forward': abs(motion['forward']),
                'current_own_rgb_features': current,
                'anchor_own_rgb_features': initial,
                'appearance': 'orange-to-yellow beam hue 3..35; same shape/consistency gates'}
        try:
            skew, skew_evidence = translation_skew(raw_top, self.carried_beam.previous)
        except ValueError as error:
            skew, skew_evidence = None, {'unresolved': str(error)}
        perception_wall_s = time.monotonic() - perception_started
        self.last_frame_id = frame_id
        self.last_observed_at_s = observed_at_s
        return {'frame_id': frame_id, 'observed_at_s': observed_at_s,
                'canonical_top': canonical, 'transform': transform,
                'carried_previous': copy.deepcopy(self.carried_beam.previous),
                'continuity_previous': copy.deepcopy(self.continuity.previous),
                'motion': motion, 'route': evidence, 'decisions': decisions,
                'skew': skew, 'skew_evidence': skew_evidence,
                'bind_started_wall_s': started,
                'bind_completed_wall_s': bind_completed,
                'perception_wall_s': perception_wall_s}


def _worker(connection, initialization):
    try:
        state = PairPerceptionState(**initialization)
        connection.send(('ready', None))
        while True:
            message = connection.recv()
            if message is None:
                return
            try:
                result = state.analyze(**message)
                connection.send(('ok', result))
            except BaseException as error:
                connection.send(('error', (type(error).__name__, str(error))))
                return
    except (EOFError, BrokenPipeError):
        return
    finally:
        connection.close()


class _PendingResult:
    def __init__(self, future, process, timeout_s):
        self.future, self.process = future, process
        self.deadline = time.monotonic() + timeout_s
        self.consumed = False

    def done(self):
        if self.future.done():
            return True
        if not self.process.is_alive():
            raise RuntimeError('pair perception process exited before RGB result')
        if time.monotonic() >= self.deadline:
            raise TimeoutError('pair perception RGB result timed out')
        return False

    def result(self):
        result = self.future.result(timeout=0)
        self.consumed = True
        return result


class PairPerceptionProcess:
    """One persistent spawned process and no more than one pending analysis."""
    def __init__(self, initialization, *, timeout_s=10.):
        context = multiprocessing.get_context('spawn')
        parent, child = context.Pipe()
        self.connection = parent
        self.process = context.Process(target=_worker, args=(child, initialization),
                                       name='dispatch-pair-perception')
        try:
            self.process.start()
        except BaseException:
            parent.close()
            child.close()
            raise
        child.close()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='dispatch-pair-ipc')
        self.timeout_s = timeout_s
        self.pending = None
        self.closed = False

    def wait_ready(self, pump):
        deadline = time.monotonic() + self.timeout_s
        while not self.connection.poll():
            if not self.process.is_alive():
                raise RuntimeError('pair perception process failed during startup')
            if time.monotonic() >= deadline:
                raise TimeoutError('pair perception process startup timed out')
            pump()
        status, detail = self.connection.recv()
        if status != 'ready':
            raise RuntimeError(f'pair perception process startup: {detail}')

    def submit(self, frame_id, observed_at_s, raw_top, own_by_slot):
        if self.closed or (self.pending is not None and not self.pending.consumed):
            raise RuntimeError('pair perception accepts one ordered RGB batch at a time')
        message = {'frame_id': frame_id, 'observed_at_s': observed_at_s,
                   'raw_top': bytes(raw_top),
                   'own_by_slot': {slot: bytes(value) for slot, value in own_by_slot.items()}}
        future = self.executor.submit(self._roundtrip, message)
        self.pending = _PendingResult(future, self.process, self.timeout_s)
        return self.pending

    def _roundtrip(self, message):
        self.connection.send(message)
        status, payload = self.connection.recv()
        if status != 'ok':
            name, detail = payload
            raise RuntimeError(f'pair perception {name}: {detail}')
        return payload

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.pending is None or self.pending.future.done():
            try:
                self.connection.send(None)
            except (BrokenPipeError, OSError):
                pass
        self.process.join(timeout=.5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1.)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=1.)
        self.connection.close()
        self.executor.shutdown(wait=False, cancel_futures=True)

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
