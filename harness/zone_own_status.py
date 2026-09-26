"""Package F executor: own-camera judgments and the own status (split from ``zone_own_executor``, issue #221).

``OwnStatusMixin`` is mixed into ``ZoneOwnExecutor``. It reads only the executor's own state: own
frames in the agreed postures (PR #193 ``judge_route_blockage`` / ``judge_holding_item``), own issued
servo PWM, the own PoseReport and uncertainty gate, and the wrist skill's own-RGB checks.
"""
from __future__ import annotations

import math

from harness import zone_own_guards as guards
from harness.m1_owncam_delivery import LIMITS as M1_LIMITS
from harness.owncam_drive import CARRY_POSTURE, LOOK_P20
from harness.owncam_pose_source import PoseReport
from harness.zone_own_contract import finite_number

STATUS_SCHEMA = 'ugrp.zone_own_executor_status.v2'
UNCERTAINTY_LEVELS = ('low', 'medium', 'high', 'unknown')
LOW_STD = (M1_LIMITS['look_back'].max_std_xy_m, M1_LIMITS['look_back'].max_std_yaw_rad)
MEDIUM_STD = (M1_LIMITS['nav_unloaded'].max_std_xy_m, M1_LIMITS['nav_unloaded'].max_std_yaw_rad)
POSTURE_TOLERANCE_PWM = 90
PAN_CENTRE_PWM, PAN_TOLERANCE_PWM = 1500, 40
JUDGE_PERIOD_S = 1.
COMMIT_CONFIDENCE = .65
BLOCKAGE_CONSECUTIVE = 2
HOLDING_CHECK_MAX_AGE_S = 3.
GRIPPER_OPEN_MIN_PWM = 1900
DOOR_LANE_RANGE_M = 1.2
BLOCKED_AHEAD_MAX_AGE_S = 5.
CARRY_PHASES = ('to_carry_posture', 'nav_preplace', 'grip_check', 'pre_release')
STATUS_JOB_KEYS = ('order_id', 'target_ref', 'target_zone', 'passage', 'waypoints', 'duration_s', 'observe',
                   'slot_id', 'pickup_slot')


def uncertainty_level(report: PoseReport) -> str:
    if not report.initialized or not finite_number(report.std_xy_m) or not finite_number(report.std_yaw_rad):
        return 'unknown'
    if report.std_xy_m <= LOW_STD[0] and report.std_yaw_rad <= LOW_STD[1]:
        return 'low'
    if report.std_xy_m <= MEDIUM_STD[0] and report.std_yaw_rad <= MEDIUM_STD[1]:
        return 'medium'
    return 'high'


class OwnStatusMixin:
    """Own-camera judgments, holding answer, region, ``status()`` and ``belief_projection()``."""

    def _posture(self, obs):
        pose = {int(k): int(v) for k, v in obs['actuator_state']['servo_pulses'].items()}
        if abs(pose.get(6, 0) - PAN_CENTRE_PWM) > PAN_TOLERANCE_PWM:
            return None, pose
        for name, ref in (('look_p20', LOOK_P20), ('carry', CARRY_POSTURE)):
            if all(abs(pose.get(s, -9999) - v) <= POSTURE_TOLERANCE_PWM for s, v in ref.items() if s in (3, 4, 5)):
                return name, pose
        return None, pose

    def _judge(self, now, obs, rgb, report) -> bool:
        name, pose = self._posture(obs)
        moving = any(abs(float(v)) > 1e-9 for v in obs['actuator_state'].get('motor_commands', ()))
        if name is None or moving or not report.initialized:
            return False
        from harness import zone_own_perception as perception
        level = uncertainty_level(report)
        belief = {'x_m': report.x_m, 'y_m': report.y_m, 'yaw_rad': report.yaw_rad,
                  'confidence': {'low': 'high', 'medium': 'medium'}.get(level, 'low')}
        near_door = (math.hypot(self.door_xy[0] - report.x_m, self.door_xy[1] - report.y_m) < DOOR_LANE_RANGE_M
                     and abs(math.cos(report.yaw_rad)) > .8)
        passage = self.door_id if near_door else None
        block = perception.judge_route_blockage(rgb, pose, static_map=self.map, pose_belief=belief, passage_id=passage)
        committed = block['answer'] == 'yes' and block['confidence'] >= COMMIT_CONFIDENCE
        key = guards.BlockageStreak.key_of(passage, guards.OwnPose.from_report(report))
        row = {'t': round(now, 3), 'judgment': 'route_blockage', 'posture': name, 'frame_id': int(obs['frame_id']),
               'answer': block['answer'], 'confidence': block['confidence'], 'reason': block['reason'],
               'passage_id': passage, 'belief_confidence': belief['confidence'], 'location_key': key}
        self.judgment_log.append(row)
        self._last_blockage = row
        if self.streak.observe(now, block['answer'], committed, key):
            self._emit(now, 'blockage_seen', passage_id=passage, confidence=block['confidence'], reason=block['reason'],
                       frame_id=int(obs['frame_id']), region=self._region(report), source='own_rgb_route_blockage')
        if name == 'carry':
            hold = perception.judge_holding_item(rgb, pose, expected_kind=self._held_kind())
            hrow = {'t': round(now, 3), 'judgment': 'holding_item', 'frame_id': int(obs['frame_id']),
                    'answer': hold['answer'], 'confidence': hold['confidence'], 'reason': hold['reason']}
            self.judgment_log.append(hrow)
            self._last_holding_check = hrow
        return True

    def _held_kind(self):
        job = self.job
        if job is not None and job.kind == 'deliver':
            return self.orders[job.args['order_id']]['kind']
        return 'cyan'

    def holding(self) -> dict:
        job = self.job
        sk = job.ctl.skill if job is not None and job.ctl is not None else None
        if sk is not None:
            phase = sk.phase
            box = getattr(sk, 'box', None)
            if phase in CARRY_PHASES and box is not None and box.held:
                base = {'answer': 'yes', 'source': 'own_rgb_attachment_check (wrist skill)', 'skill_phase': phase}
            elif phase in ('look_back', 'finished') and any(e.get('event') == 'release_confirmed'
                                                              for e in getattr(sk, 'events', ())):
                base = {'answer': 'no', 'source': 'own_rgb_release_confirmed (wrist skill)', 'skill_phase': phase}
            elif phase in ('nav_pregrasp',) and self.servo.get(1, 0) >= GRIPPER_OPEN_MIN_PWM:
                base = {'answer': 'no', 'source': 'gripper_open_issued_since_last_release', 'skill_phase': phase}
            else:
                base = {'answer': 'unknown', 'source': 'wrist skill mid-manipulation', 'skill_phase': phase}
        else:
            base = dict(self._holding_after)
        check = self._last_holding_check
        if check is not None and self.now - check['t'] <= HOLDING_CHECK_MAX_AGE_S:
            base['camera_check'] = {k: check[k] for k in ('answer', 'confidence', 'reason', 't')}
            if (check['answer'] in ('yes', 'no') and base['answer'] in ('yes', 'no') and check['answer'] != base['answer']
                    and check['confidence'] >= COMMIT_CONFIDENCE):
                base['answer'], base['conflict'] = 'unknown', True
        return base

    def _region(self, report):
        if report is None or not report.initialized:
            return 'unknown'
        x, y = report.x_m, report.y_m
        if math.hypot(x - self.door_xy[0], y - self.door_xy[1]) < .35:
            return self.door_id
        for name, r in self.map['regions'].items():
            (cx, cy), (hx, hy) = r['center_m'], r['half_extents_m']
            if abs(x - cx) <= hx and abs(y - cy) <= hy:
                return name
        return 'west_floor' if x < self.door_xy[0] else 'east_floor'

    def status(self) -> dict:
        """Own executor state and own-camera judgments only. No peer, no simulator field."""
        rep = self.last_report
        job = self.job
        blocked = self._last_blockage
        if blocked is None or self.now - blocked['t'] > BLOCKED_AHEAD_MAX_AGE_S:
            blocked_ahead = {'answer': 'unknown', 'reason': 'no recent own look in an agreed posture'}
        else:
            blocked_ahead = {k: blocked[k] for k in ('answer', 'confidence', 'reason', 'passage_id', 't')}
        loc = {'level': self._level, 'gate': self.gate.state, 'initialized': bool(rep is not None and rep.initialized)}
        if rep is not None and rep.initialized:
            loc.update(std_xy_m=round(rep.std_xy_m, 4), std_yaw_rad=round(rep.std_yaw_rad, 4),
                       since_tag_s=None if rep.since_tag_s is None else round(rep.since_tag_s, 2),
                       own_estimate_xy_yaw=[round(rep.x_m, 3), round(rep.y_m, 3), round(rep.yaw_rad, 4)])
        return {'schema': STATUS_SCHEMA, 'robot_id': self.robot_id, 'mode': self.mode, 'sim_s': round(self.now, 3),
                'local_state': self._local_state, 'stopped': self.stopped is not None,
                'job': None if job is None else {'job_id': job.job_id, 'kind': job.kind, 'phase': self._job_phase(job),
                                                 'started_at_sim_s': round(job.started_at, 3),
                                                 'arguments': {k: v for k, v in job.args.items() if k in STATUS_JOB_KEYS}},
                'holding': self.holding(), 'blocked_ahead': blocked_ahead, 'localization': loc,
                'region': self._region(rep), 'jobs_finished': len(self.jobs_done)}

    def belief_projection(self) -> dict:
        """Package A ``BELIEF_KEYS`` projection of the status (for the robot's own prompt)."""
        st = self.status()
        last_done = next((j for j in reversed(self.jobs_done) if j['confirmation'] == 'own_camera_confirmed'), None)
        conf = 'low' if st['localization']['gate'] != 'ok' else \
            {'low': 'high', 'medium': 'medium'}.get(st['localization']['level'], 'low')
        return {'region': st['region'], 'last_visual_anchor': None if self.last_frame_id is None else
                f'own-{self.robot_id}-{self.last_frame_id:05d}',
                'last_requested_destination': None if self.job is None else self.job.args.get('slot_id') or
                self.job.args.get('target_ref') or self.job.args.get('target_zone'),
                'last_visually_confirmed_region': None if last_done is None else last_done.get('slot_id'),
                'confidence': conf, 'sources': ['own_rgb', 'own_commands', 'static_map'],
                'held_item_guess': st['holding']['answer'],
                'blocked_passages': [st['blocked_ahead']['passage_id']] if st['blocked_ahead']['answer'] == 'yes'
                and st['blocked_ahead'].get('passage_id') else [],
                'notes_ko': ''}

    def _job_phase(self, job):
        if job.ctl is not None:
            return f"{job.ctl.phase}:{getattr(job.ctl.skill, 'phase', '')}"
        if job.driver is not None:
            return f'drive:{job.driver.state}'
        return job.phase
