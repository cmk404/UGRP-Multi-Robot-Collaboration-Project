#!/usr/bin/env python3
"""60 cm image approach -> demonstrated/RGB grasp -> visible goal -> placement.

Local execution curriculum, optionally gated by two visual LLM actors. No
teacher feedback at runtime. Arm segments are explicitly demonstrated
playback with RGB recovery, not learned autonomy.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))

from scripts.camera_short_transport_scene import ShortTransportScene
from scripts.camera_approach_scene import ROBOTS, validate_start_poses
from scripts.run_camera_approach_student import models, sha, write
from scripts.run_camera_varied_start_student import load_stage_models, run_approach
from scripts.evaluate_camera_short_transport import evaluate_transport_samples
from harness.camera_goal_transport import coarse_approach, goal_carry, dock_command, preclose_supported
from harness.camera_varied_start_student import predict_stage
from harness.pair_carry_policy import PairCarryPolicy, payload_skew


class GoalScene(ShortTransportScene):
    gate=None
    grasp_check=None

    def configure_run(self, args, report):
        if args.planner == 'llm':
            from scripts.camera_skill_gate import CameraSkillGate
            self.gate = CameraSkillGate(self.out/'llm')
            report['scope'] = 'two independent LLM visual skill permissions; RGB wheels; demonstrated arm sequence plus RGB correction; fixed workflow, not raw-action LLM control'

    def delivered_reports(self, index):
        return ROBOTS

    def extra_report(self):
        return {}

    def checkpoint(self,skill):
        if self.gate is None: return
        for attempt in range(3):
            frames=self.capture(f'skill-{skill}-{attempt}')
            own_commands={r:[] for r in ROBOTS}
            for row in self.trace:
                for r in ROBOTS:
                    if 'actions' in row:
                        own_commands[r].append(dict(stage=row['stage'],action=row['actions'][r]))
                    elif r in row['command']['targets']:
                        own_commands[r].append(dict(stage=row['stage'],targets=row['command']['targets'][r],
                            duration_s=row['command']['duration_s']))
            own_commands={r:h[-16:] for r,h in own_commands.items()}
            if self.gate.decide(skill,frames,own_commands): return
            self.tick(.2)
        raise RuntimeError(f'two independent LLMs did not authorize {skill}')

    def replay(self,commands,stage):
        if stage=='grasp_close':
            from harness.grasp_student_inference import predict_student
            frames=self.capture('preclose-support')
            decisions={r:predict_student(self.grasp_models[r],frames[r]['own_bytes'],
                frames[r]['top_bytes'],max_step=25) for r in ROBOTS}
            self.grasp_check=dict(decisions=decisions,ok=preclose_supported(decisions),
                images={r:dict(own=frames[r]['own_rgb'],top=frames[r]['shared_top_rgb']) for r in ROBOTS})
            if not self.grasp_check['ok']:
                raise RuntimeError('preclose RGB outside learned grasp support')
        skill={'grasp_initialization':'PREPARE_GRASP','grasp_close':'CLOSE',
               'grasp_lift':'LIFT','place_lower':'LOWER','place_open':'RELEASE',
               'place_retract':'RETRACT'}.get(stage)
        if skill: self.checkpoint(skill)
        return super().replay(commands,stage)

    def evaluation_snapshot(self, *, full_state=True):
        from sim.cooperative_payload import beam_pose, evaluate_beam_mission
        row=super().evaluation_snapshot(full_state=full_state)
        row['goal']=evaluate_beam_mission(beam_pose(self.world.data,self.world.model))
        return row


def setup_poses(distance, lateral, yaw_deg):
    """Experiment setup only; never pass these values into either controller."""
    if any(len(values) != 2 for values in (distance, lateral, yaw_deg)):
        raise ValueError('exactly two values required for each start axis')
    starts = validate_start_poses({r: dict(distance_m=distance[i], lateral_m=lateral[i],
        yaw_deg=yaw_deg[i]) for i, r in enumerate(ROBOTS)}, max_distance_m=.7)
    if any(s['distance_m'] < .15 for s in starts.values()):
        raise ValueError('distance must be .15...7 m')
    return starts


def run(args, *, scene_factory=GoalScene):
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        raise RuntimeError('commit and freeze source before physical experiments')
    starts=setup_poses(args.distance,args.lateral,args.yaw_deg)
    out=args.out_dir.resolve()
    if out.exists(): raise FileExistsError(out)
    skill, grasp=models(args.grasp_model_dir.resolve(),'student-skill.json')
    stage_skill, stages=load_stage_models(args.stage_model_dir.resolve())
    reference=args.reference_top.read_bytes()
    scene=scene_factory(out,args.grasp_model_dir.resolve(),fps=10)
    report=dict(schema='ugrp.camera_goal_transport.v1',
        source_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        scope='local RGB wheel feedback; demonstrated arm sequence plus learned RGB recovery; no LLM',
        config=dict(coarse_control='rgb-wheel-heading-v1',reacquire_on_settle=True,final_refinement_steps=40,
                    start_distance_m=dict(zip(ROBOTS,args.distance)),
                    start_poses_setup_only=starts,weld=False,planner=args.planner),
        assets=dict(grasp_skill=dict(path=str(args.grasp_model_dir.resolve()),sha256=sha(args.grasp_model_dir/'student-skill.json')),
                    stage_skill=dict(path=str(args.stage_model_dir.resolve()),sha256=sha(args.stage_model_dir/'varied-start-skill.json')),
                    reference_top=dict(path=str(args.reference_top.resolve()),sha256=sha(args.reference_top))),
        coarse_calls=[],dock_calls=[],carry_calls=[],error=None,success=False,carry_ready=False)
    scene.grasp_models=grasp
    started=time.monotonic()
    try:
        import mujoco
        from sim.camera_robot_port import CameraRobotPort
        from harness.grasp_student_inference import predict_student
        report['environment']=dict(python=sys.version,platform=platform.platform(),mujoco=mujoco.__version__)
        scene.open(start_poses=starts,max_start_distance_m=.7)
        scene.ports={r:CameraRobotPort(scene.world,r,allow_reverse=True,allow_mecanum=True) for r in ROBOTS}
        report['invariants_initial']=scene.invariant_record()
        scene.configure_run(args, report)
        # Reference pixels were produced offline, before this run. No teacher
        # state or live evaluation result can enter either controller.
        (out/'reference-top.jpg').write_bytes(reference)
        scene.checkpoint('APPROACH')
        for index in range(120):
            frames=scene.capture(f'coarse-{index:03d}')
            decisions={r:coarse_approach(frames[r]['top_bytes'],reference,r) for r in ROBOTS}
            report['coarse_calls'].append(dict(index=index,decisions=decisions,
                images={r:dict(own=frames[r]['own_rgb'],top=frames[r]['shared_top_rgb']) for r in ROBOTS}))
            if not all(d['ok'] for d in decisions.values()): raise RuntimeError('coarse RGB unresolved')
            scene.drive_mecanum({r:dict(forward=d['forward'],left=0.,turn=d['turn'])
                                 for r,d in decisions.items()})
            if all(d['ready'] for d in decisions.values()):
                scene.stop_dwell()
                break
        else: raise RuntimeError('coarse approach budget')
        report.update(run_approach(scene,stages,reacquire_on_settle=True,final_refinement_steps=40))
        print(json.dumps(dict(stage='approach',ok=report['approach_ok'],results=report['stage_results'])),flush=True)
        if not report['approach_ok']: raise RuntimeError('fine RGB alignment failed')
        confirmations=0
        for index in range(30):
            frames=scene.capture(f'dock-{index:03d}')
            predictions={r:predict_stage(stages[r]['forward'],frames[r]['own_bytes'],
                frames[r]['top_bytes']) for r in ROBOTS}
            decisions={r:dock_command(p) for r,p in predictions.items()}
            report['dock_calls'].append(dict(index=index,predictions=predictions,decisions=decisions,
                images={r:dict(own=frames[r]['own_rgb'],top=frames[r]['shared_top_rgb']) for r in ROBOTS}))
            if not all(d['ok'] for d in decisions.values()): raise RuntimeError('dock RGB unresolved')
            scene.drive({r:d['forward'] for r,d in decisions.items()},.2)
            confirmations=confirmations+1 if all(d['ready'] for d in decisions.values()) else 0
            if confirmations>=2: break
        else: raise RuntimeError('precise RGB docking budget')
        report['grasp_calls']=scene.finish_grasp(predict_student,grasp)
        write(out/'grasp-result.json',scene.grasp_report)
        scene.checkpoint('CARRY')
        anchor=scene.capture('carry-anchor')
        report['carry_anchor']={r:dict(own=anchor[r]['own_rgb'],top=anchor[r]['shared_top_rgb']) for r in ROBOTS}
        policy=PairCarryPolicy('visible-goal-carry')
        for index in range(160):
            frames=anchor if index==0 else scene.capture(f'carry-{index:03d}')
            decisions={r:goal_carry(frames[r]['own_bytes'],frames[r]['top_bytes'],anchor[r]['own_bytes'],anchor[r]['top_bytes']) for r in ROBOTS}
            skew=payload_skew(frames['r1']['top_bytes'])
            delivered=scene.delivered_reports(index)
            control=policy.step(decisions,skew,{r:frames[r]['frame_id'] for r in ROBOTS},scene.time(),delivered=delivered)
            report['carry_calls'].append(dict(index=index,decisions=decisions,skew=skew,control=control,
                frame_ids={r:frames[r]['frame_id'] for r in ROBOTS},sim_time_s=scene.time(),delivered=list(delivered),
                images={r:dict(own=frames[r]['own_rgb'],top=frames[r]['shared_top_rgb']) for r in ROBOTS}))
            scene.carry_drive(control['forwards'],control['duration_s'],stage='carry' if any(control['forwards'].values()) else 'carry_stop')
            if index%10==0 or control['abort'] or control['done']:
                print(json.dumps(dict(stage='carry',index=index,control=control,decisions=decisions)),flush=True)
            if control['abort']: raise RuntimeError('visual carry aborted')
            if control['done']:
                report['carry_ready']=True
                break
        if not report['carry_ready']: raise RuntimeError('carry budget')
        scene.place()
        scene.checkpoint('FINISH')
    except Exception:
        report['error']=traceback.format_exc()
    finally:
        # Evaluation is exclusively after action generation has ended.
        report['evaluation']=evaluate_transport_samples(scene.evaluation_samples,target_distance_m=.50)
        tail=scene.evaluation_samples[-11:]
        report['goal_stable_at_end']=len(tail)==11 and all(s['goal']['success'] for s in tail)
        report['approach_payload_contact_steps']=scene.approach_payload_contact_steps
        report['weld_active_ticks']=scene.weld_active_ticks
        report['preclose_check']=scene.grasp_check
        report['wall_s']=time.monotonic()-started
        report['llm']=None if scene.gate is None else dict(
            call_count=len(scene.gate.calls),events=scene.gate.events,
            usage={k:sum((c.get('usage') or {}).get(k,0) for c in scene.gate.calls)
                   for k in ('prompt_tokens','completion_tokens','total_tokens')},
            usage_unknown_calls=sum(c.get('usage') is None for c in scene.gate.calls),
            cost_usd=None,cost_note='proxy does not report billing; SIM pauses during inference')
        if scene.world:
            report['invariants_final']=scene.invariant_record()
            report['sim_s']=scene.time()
        report['success']=bool(report['error'] is None and report['evaluation']['success']
            and report['goal_stable_at_end'] and scene.approach_payload_contact_steps==0
            and scene.weld_active_ticks==0 and report['invariants_initial']==report['invariants_final'])
        scene.close()
        report.update(scene.extra_report())
        if scene.grasp_report is not None:
            write(out/'grasp-result.json',scene.grasp_report)
        if out.exists(): write(out/'result.json',report)
    print(json.dumps({k:report[k] for k in ('success','error','evaluation','goal_stable_at_end','wall_s')}),flush=True)
    return 0 if report['success'] else 1


def build_parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--grasp-model-dir',type=Path,required=True)
    p.add_argument('--stage-model-dir',type=Path,required=True)
    p.add_argument('--reference-top',type=Path,required=True)
    p.add_argument('--out-dir',type=Path,required=True)
    p.add_argument('--distance',type=float,nargs=2,default=(.6,.6))
    p.add_argument('--lateral',type=float,nargs=2,default=(0.,0.),help='setup-only world y offsets in m, r1/r3')
    p.add_argument('--yaw-deg',type=float,nargs=2,default=(0.,0.),help='setup-only headings in degrees, r1/r3')
    p.add_argument('--planner',choices=('local','llm'),default='local')
    return p


def main():
    p=build_parser()
    a=p.parse_args()
    try: setup_poses(a.distance,a.lateral,a.yaw_deg)
    except ValueError as exc: p.error(str(exc))
    return run(a)


if __name__=='__main__': raise SystemExit(main())
