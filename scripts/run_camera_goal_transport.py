#!/usr/bin/env python3
"""60 cm image approach -> demonstrated/RGB grasp -> visible goal -> placement.

Local execution curriculum. No LLM or teacher feedback at runtime. Arm segments
are explicitly demonstrated playback with RGB recovery, not learned autonomy.
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
from scripts.camera_approach_scene import ROBOTS
from scripts.run_camera_approach_student import models, sha, write
from scripts.run_camera_varied_start_student import load_stage_models, run_approach
from scripts.evaluate_camera_short_transport import evaluate_transport_samples
from harness.camera_goal_transport import coarse_approach, goal_carry
from harness.pair_carry_policy import PairCarryPolicy, payload_skew


class GoalScene(ShortTransportScene):
    def evaluation_snapshot(self, *, full_state=True):
        from sim.cooperative_payload import beam_pose, evaluate_beam_mission
        row=super().evaluation_snapshot(full_state=full_state)
        row['goal']=evaluate_beam_mission(beam_pose(self.world.data,self.world.model))
        return row


def run(args):
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        raise RuntimeError('commit and freeze source before physical experiments')
    out=args.out_dir.resolve()
    if out.exists(): raise FileExistsError(out)
    skill, grasp=models(args.grasp_model_dir.resolve(),'student-skill.json')
    stage_skill, stages=load_stage_models(args.stage_model_dir.resolve())
    reference=args.reference_top.read_bytes()
    scene=GoalScene(out,args.grasp_model_dir.resolve(),fps=10)
    report=dict(schema='ugrp.camera_goal_transport.v1',
        source_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        scope='local RGB wheel feedback; demonstrated arm sequence plus learned RGB recovery; no LLM',
        config=dict(start_distance_m=dict(zip(ROBOTS,args.distance)),weld=False),
        assets=dict(grasp_skill=dict(path=str(args.grasp_model_dir.resolve()),sha256=sha(args.grasp_model_dir/'student-skill.json')),
                    stage_skill=dict(path=str(args.stage_model_dir.resolve()),sha256=sha(args.stage_model_dir/'varied-start-skill.json')),
                    reference_top=dict(path=str(args.reference_top.resolve()),sha256=sha(args.reference_top))),
        coarse_calls=[],carry_calls=[],error=None,success=False,carry_ready=False)
    started=time.monotonic()
    try:
        import mujoco
        from sim.camera_robot_port import CameraRobotPort
        from harness.grasp_student_inference import predict_student
        report['environment']=dict(python=sys.version,platform=platform.platform(),mujoco=mujoco.__version__)
        scene.open(dict(zip(ROBOTS,args.distance)))
        scene.ports={r:CameraRobotPort(scene.world,r,allow_reverse=True,allow_mecanum=True) for r in ROBOTS}
        report['invariants_initial']=scene.invariant_record()
        # Reference pixels were produced offline, before this run. No teacher
        # state or live evaluation result can enter either controller.
        (out/'reference-top.jpg').write_bytes(reference)
        for index in range(120):
            frames=scene.capture(f'coarse-{index:03d}')
            decisions={r:coarse_approach(frames[r]['top_bytes'],reference,r) for r in ROBOTS}
            report['coarse_calls'].append(dict(index=index,decisions=decisions,
                images={r:dict(own=frames[r]['own_rgb'],top=frames[r]['shared_top_rgb']) for r in ROBOTS}))
            if not all(d['ok'] for d in decisions.values()): raise RuntimeError('coarse RGB unresolved')
            scene.drive({r:d['forward'] for r,d in decisions.items()})
            if all(d['ready'] for d in decisions.values()):
                scene.stop_dwell()
                break
        else: raise RuntimeError('coarse approach budget')
        report.update(run_approach(scene,stages))
        print(json.dumps(dict(stage='approach',ok=report['approach_ok'],results=report['stage_results'])),flush=True)
        if not report['approach_ok']: raise RuntimeError('fine RGB alignment failed')
        report['grasp_calls']=scene.finish_grasp(predict_student,grasp)
        write(out/'grasp-result.json',scene.grasp_report)
        anchor=scene.capture('carry-anchor')
        report['carry_anchor']={r:dict(own=anchor[r]['own_rgb'],top=anchor[r]['shared_top_rgb']) for r in ROBOTS}
        policy=PairCarryPolicy('visible-goal-carry')
        for index in range(160):
            frames=anchor if index==0 else scene.capture(f'carry-{index:03d}')
            decisions={r:goal_carry(frames[r]['own_bytes'],frames[r]['top_bytes'],anchor[r]['own_bytes'],anchor[r]['top_bytes']) for r in ROBOTS}
            skew=payload_skew(frames['r1']['top_bytes'])
            control=policy.step(decisions,skew,{r:frames[r]['frame_id'] for r in ROBOTS},scene.time())
            report['carry_calls'].append(dict(index=index,decisions=decisions,skew=skew,control=control,
                frame_ids={r:frames[r]['frame_id'] for r in ROBOTS},sim_time_s=scene.time(),
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
    except Exception:
        report['error']=traceback.format_exc()
    finally:
        # Evaluation is exclusively after action generation has ended.
        report['evaluation']=evaluate_transport_samples(scene.evaluation_samples,target_distance_m=.50)
        tail=scene.evaluation_samples[-11:]
        report['goal_stable_at_end']=len(tail)==11 and all(s['goal']['success'] for s in tail)
        report['approach_payload_contact_steps']=scene.approach_payload_contact_steps
        report['weld_active_ticks']=scene.weld_active_ticks
        report['wall_s']=time.monotonic()-started
        if scene.world:
            report['invariants_final']=scene.invariant_record()
            report['sim_s']=scene.time()
        report['success']=bool(report['error'] is None and report['evaluation']['success']
            and report['goal_stable_at_end'] and scene.approach_payload_contact_steps==0
            and scene.weld_active_ticks==0 and report['invariants_initial']==report['invariants_final'])
        scene.close()
        if out.exists(): write(out/'result.json',report)
    print(json.dumps({k:report[k] for k in ('success','error','evaluation','goal_stable_at_end','wall_s')}),flush=True)
    return 0 if report['success'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--grasp-model-dir',type=Path,required=True)
    p.add_argument('--stage-model-dir',type=Path,required=True)
    p.add_argument('--reference-top',type=Path,required=True)
    p.add_argument('--out-dir',type=Path,required=True)
    p.add_argument('--distance',type=float,nargs=2,default=(.6,.6))
    a=p.parse_args()
    if any(not .15<=d<=.7 for d in a.distance): p.error('distance must be .15...7 m')
    return run(a)


if __name__=='__main__': raise SystemExit(main())
