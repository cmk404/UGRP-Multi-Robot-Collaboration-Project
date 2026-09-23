#!/usr/bin/env python3
"""Finite paired render benchmark; no control or physics parameter changes."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from scripts.run_dispatch_skills import SkillScene
from sim.research_dispatch_arena import episode


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--pairs',type=int,default=8)
    args=parser.parse_args()
    if args.pairs<2:parser.error('--pairs must be at least two')
    sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    if subprocess.check_output(['git','status','--porcelain','--untracked-files=no'],cwd=ROOT,text=True).strip():
        parser.error('commit execution source before benchmarking')
    config=episode('open',11);config['contact_solver_profile']='local_contact_fine'
    scene=SkillScene(config,args.output)
    rows=[]
    def actor_hashes(frames,carriers):
        return {rid:{key:hashlib.sha256(frames[rid][key]).hexdigest()
                     for key in ('top_bytes','own_bytes')} for rid in carriers}
    def state_hash():
        return hashlib.sha256(scene.world.data.qpos.tobytes()+scene.world.data.qvel.tobytes()).hexdigest()
    try:
        scene.open()
        start_sim=scene.time();before=state_hash()
        # Warm all cameras equally before paired timings. Rendering never steps physics.
        scene.capture('warmup')
        for index in range(args.pairs):
            carriers=(('r1','r3'),('r2','r3'))[index%2]
            order=('baseline','efficient') if index%2==0 else ('efficient','baseline')
            pair={'index':index,'carriers':carriers,'order':order}
            hashes={}
            for mode in order:
                scene.efficient_capture=mode=='efficient'
                started=time.perf_counter()
                frames=scene.capture(f'{index:02}-{mode}',own_robots=carriers,overview=False)
                pair[mode+'_wall_s']=time.perf_counter()-started
                hashes[mode]=actor_hashes(frames,carriers)
            pair['actor_images_identical']=hashes['baseline']==hashes['efficient']
            pair['actor_hashes']=hashes
            rows.append(pair)
            print(json.dumps({key:val for key,val in pair.items() if key!='actor_hashes'}),flush=True)
        baseline=statistics.median(row['baseline_wall_s'] for row in rows)
        efficient=statistics.median(row['efficient_wall_s'] for row in rows)
        result={'source_sha':sha,'platform':platform.platform(),'python':sys.version,
            'scope':'Paired capture only at a frozen physical state; excludes training, physics, inference and continuous video.',
            'config':config,'invariants':scene.invariants(),'render_resolution':[960,720],
            'render_count':{'baseline':5,'efficient':3},'pairs':rows,
            'baseline_median_s':baseline,'efficient_median_s':efficient,
            'capture_speedup':baseline/efficient,'capture_time_reduction_fraction':1-efficient/baseline,
            'state_before_sha256':before,'state_after_sha256':state_hash(),
            'sim_time_unchanged':scene.time()==start_sim,
            'all_actor_images_identical':all(row['actor_images_identical'] for row in rows)}
        result['complete']=True
        result['passed']=result['all_actor_images_identical'] and result['sim_time_unchanged'] and before==state_hash()
        (args.output/'benchmark.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({key:result[key] for key in ('passed','baseline_median_s','efficient_median_s','capture_speedup')}),flush=True)
        return 0 if result['passed'] else 1
    finally:scene.close()


if __name__=='__main__':raise SystemExit(main())
