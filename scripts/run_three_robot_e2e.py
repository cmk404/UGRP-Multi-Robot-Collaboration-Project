#!/usr/bin/env python3
"""Three-peer plan -> r1/r3 RGB transport + independent r2 visual inspection."""
from __future__ import annotations

from pathlib import Path
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.run_camera_goal_transport import GoalScene, build_parser, run, setup_poses
from scripts.camera_approach_scene import image_record
from scripts.camera_skill_gate import CameraSkillGate
from scripts.three_robot_runtime import ThreeRobotRuntime
from harness.three_robot_plan import ROBOTS, stage_deliveries, carry_deliveries


class ThreeRobotScene(GoalScene):
    team = None

    def team_capture(self, tag):
        frames = self.capture(tag)
        own = self.world.render_jpeg(robot_id='r2', camera='robot_cam', quality=95)
        own_ref = image_record(self.out/'rgb'/f'{tag}-r2-own.jpg', self.out, own)
        self.observer_frame_id += 1
        frames['r2'] = {'own_bytes': own, 'top_bytes': frames['r1']['top_bytes'],
            'own_rgb': own_ref, 'shared_top_rgb': frames['r1']['shared_top_rgb'],
            'frame_id': self.observer_frame_id}
        return frames

    def invariant_record(self):
        import mujoco
        row = super().invariant_record()
        m = self.world.model
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, 'r2__robot_cam')
        row['policy_cameras']['r2__robot_cam'] = {'position': m.cam_pos[cid].tolist(),
            'quaternion': m.cam_quat[cid].tolist(), 'fov_y_deg': float(m.cam_fovy[cid])}
        return row

    def configure_run(self, args, report):
        self.fault, self.observer_frame_id = args.fault, 0
        self.team = ThreeRobotRuntime(self.out/'team', run_id=uuid.uuid4().hex[:12],
            mode=args.team_planner, fixture_timing=args.fixture_timing)
        report['config'].update(team_planner=args.team_planner, team_fault=args.fault)
        report['scope'] = ('three independent RGB plan proposals/ACKs, capability-constrained pair; '
            'r2 stationary destination inspection; r1/r3 visual skill permissions and existing RGB transport; '
            'not free role allocation, three-way grasp, or distributed real-time control')
        history = {r: [] for r in ROBOTS}
        for row in self.trace:
            for rid, targets in row.get('command', {}).get('targets', {}).items():
                history[rid].append({'stage': row['stage'], 'targets': targets,
                                    'duration_s': row['command']['duration_s']})
        for turn in range(6):
            frames = self.team_capture(f'team-plan-{turn}')
            if self.team.negotiate(frames, history, turn, self.time()):
                break
            self.tick(.2)
        else:
            raise RuntimeError('three-peer plan not committed; no task actuation allowed')
        if args.planner == 'llm':
            self.gate = CameraSkillGate(self.out/'llm', team_plan=self.team.agreement.committed,
                delivery_filter=lambda skill, events: stage_deliveries(self.fault, skill, events))
        elif self.fault == 'ready_delay':
            raise ValueError('ready_delay requires real pair visual replies (--planner llm)')

    def drive_mecanum(self, commands, duration_s=.2):
        if self.team and not any(e['event'] == 'PAIR_MOTION_START' for e in self.team.execution_events):
            if any(any(c.values()) for c in commands.values()):
                self.team.event('PAIR_MOTION_START', self.time())
        return super().drive_mecanum(commands, duration_s)

    def checkpoint(self, skill):
        if self.team is not None:
            self.team.event('PAIR_CHECKPOINT', self.time(), skill=skill)
            committed = self.team.agreement.committed
            if not committed or not self.team.agreement.authorize(committed['proposal_id'], committed['plan_hash']):
                raise RuntimeError('plan authorization revoked; replan while stopped')
            self.team.collect_inspection(self.time())
            if skill == 'CARRY':
                if not self.team.inspections:
                    self.team.start_inspection(self.team_capture('team-inspection-0')['r2'], self.time())
                inspection = self.team.collect_inspection(self.time(), wait=True)
                # Reports are advisory, never substitute for fresh carrier RGB.
                # A reported obstruction or missing report prevents starting carry.
                if inspection is None or inspection['reply'] is None:
                    raise RuntimeError('inspection unavailable; stopped for replanning')
                if inspection['reply']['status'] == 'BLOCKED':
                    self.team.agreement.invalidate('r2 reported destination obstruction')
                    raise RuntimeError('destination obstruction reported; stopped for replanning')
            if skill == 'FINISH' and not self.team.inspection_result:
                raise RuntimeError('inspection task has no report')
        result = super().checkpoint(skill)
        if self.team is not None:
            self.team.event('PAIR_AUTHORIZED', self.time(), skill=skill)
            # Launch after the pair's initial LLM barrier, so the third model
            # can run while physics advances, rather than only during a pause.
            if (skill == 'APPROACH' and not self.team.inspections
                    and committed['plan']['inspection']['timing'] == 'during_approach'):
                self.team.start_inspection(self.team_capture('team-inspection-0')['r2'], self.time())
        return result

    def delivered_reports(self, index):
        return carry_deliveries(self.fault, index)

    def extra_report(self):
        return {'team': self.team.snapshot() if self.team else None}

    def close(self):
        # Stop motors before waiting for the bounded outstanding model request.
        for port in self.ports.values():
            port.stop()
        try:
            if self.team is not None:
                self.team.close(self.time() if self.world else 0.)
        finally:
            super().close()


def main():
    parser = build_parser()
    parser.description = __doc__
    parser.set_defaults(planner='llm')
    parser.add_argument('--team-planner', choices=('llm', 'fixture'), default='llm')
    parser.add_argument('--fixture-timing', choices=('during_approach', 'before_carry'), default='during_approach')
    parser.add_argument('--fault', choices=('none', 'ready_delay', 'carry_report_loss'), default='none')
    args = parser.parse_args()
    try:
        setup_poses(args.distance, args.lateral, args.yaw_deg)
    except ValueError as exc:
        parser.error(str(exc))
    if args.fault == 'ready_delay' and args.planner != 'llm':
        parser.error('ready_delay requires --planner llm')
    return run(args, scene_factory=ThreeRobotScene)


if __name__ == '__main__':
    raise SystemExit(main())
