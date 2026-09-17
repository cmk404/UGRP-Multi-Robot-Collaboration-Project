#!/usr/bin/env python3
"""Build, inspect and negotiate the dispatch arena. No autonomous transport claim."""
from __future__ import annotations

import argparse
from functools import partial
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))

from sim.research_dispatch_arena import VARIANTS,ROBOTS,actor_task,episode,digest
from harness.dispatch_plan import (validate_dispatch_plan,validate_dispatch_reply,
                                  build_dispatch_request,compile_programs,fixture_plan)
from harness.three_robot_plan import TeamAgreement
from scripts.three_robot_runtime import ThreeRobotRuntime,write
from scripts.research_dispatch_scene import DispatchScene


def opaque_run_id():
    """An actor-visible identifier must not encode condition, seed or answer."""
    return 'dispatch-'+uuid.uuid4().hex[:12]


def run(args):
    config=episode(args.variant,args.seed)
    scene=DispatchScene(config,args.output)
    team=None
    started=time.monotonic()
    result={'scope':'physical environment + RGB common-plan negotiation; not autonomous cargo delivery',
        'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'variant':args.variant,'seed':args.seed,'mode':args.planner,
        'scene_ok':False,'plan_committed':False,'transport_success':None,
        'error':None,'cost_usd':None}
    try:
        scene.open()
        write(args.output/'episode-setup-only.json',config)
        write(args.output/'actor-mission.json',actor_task(config['static_map']))
        write(args.output/'scene-manifest.json',scene.manifest)
        initial=scene.evaluate_positions()
        frames=scene.capture('initial')
        if args.planner!='none':
            run_id=opaque_run_id()
            team=ThreeRobotRuntime(args.output/'team',run_id=run_id,mode=args.planner,
                agreement=TeamAgreement(run_id,plan_validator=validate_dispatch_plan),
                request_builder=partial(build_dispatch_request,task=actor_task(config['static_map'])),
                reply_validator=validate_dispatch_reply,plan_fixture=fixture_plan(),
                request_timeout=60.,max_tokens=1400,roles_fixed_by_skill=False,planning_only=True)
            for turn in range(8):
                if team.negotiate(frames,scene.command_history,turn,float(scene.world.data.time)):
                    break
                frames=scene.capture(f'plan-{turn+1}')
            if not team.agreement.committed:
                raise RuntimeError('no unanimous valid dispatch plan within 8 rounds')
            plan=team.agreement.committed
            programs=compile_programs(plan['plan'],config['static_map'])
            write(args.output/'committed-plan.json',plan)
            write(args.output/'robot-programs.json',programs)
            result['plan_committed']=True
            result['plan']=plan
            result['model_attempts']=len(team.calls)
            result['execution_status']='awaiting_new_arena_local_executor; no cargo actions issued'
        # Explicit diagnostic, never driven by evaluator or represented as a
        # planned transport. No motion if --motion-smoke is absent.
        if args.motion_smoke:
            command={'kind':'drive','forward':.10,'turn':0.,'duration_s':.7}
            for r,p in scene.ports.items():
                p.apply(command,float(scene.world.data.time))
                scene.command_history[r].append(dict(command))
            scene.step(.9)
            scene.capture('motion-smoke')
        else:
            scene.step(.5)
        final=scene.evaluate_positions()
        final_inv=scene.invariants()
        result.update(scene_ok=True,physics_steps=scene.physics_steps,weld_steps=scene.weld_steps,
            obstacle_contact_steps=scene.obstacle_contact_steps,
            invariants_unchanged=scene.initial_invariants==final_inv,
            motion_smoke=args.motion_smoke,
            robot_displacement_m={r:math.dist(initial[r][:2],final[r][:2]) for r in ROBOTS},
            initial_invariants=scene.initial_invariants,final_invariants=final_inv)
        write(args.output/'evaluation-only.json',{'initial':initial,'final':final,
            'scope':'environment settling and optional diagnostic drive only; transport untested'})
        write(args.output/'issued-commands.json',scene.command_history)
        if scene.weld_steps or scene.obstacle_contact_steps or scene.initial_invariants!=final_inv:
            raise RuntimeError('environment smoke invariant/contact failure')
        if args.motion_smoke and any(x<.01 for x in result['robot_displacement_m'].values()):
            raise RuntimeError('not all robots moved in diagnostic')
    except Exception as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
    finally:
        try:
            if team:team.close(float(scene.world.data.time) if scene.world else 0.)
        finally:
            scene.close()
        result['wall_seconds']=time.monotonic()-started
        if args.output.exists():write(args.output/'result.json',result)
    print(json.dumps(result,ensure_ascii=False,sort_keys=True))
    return 1 if result['error'] else 0


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--variant',choices=VARIANTS,default='shared_crossing')
    p.add_argument('--seed',type=int,default=11)
    p.add_argument('--planner',choices=('none','fixture','llm'),default='none')
    p.add_argument('--motion-smoke',action='store_true',help='explicit fixed-command motor diagnostic; not plan execution')
    args=p.parse_args()
    if args.output.exists():p.error('output already exists; choose a new experiment directory')
    return run(args)


if __name__=='__main__':raise SystemExit(main())
