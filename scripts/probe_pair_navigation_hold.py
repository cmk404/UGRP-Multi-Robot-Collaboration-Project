#!/usr/bin/env python3
"""Output-only diagnostic: does the unchanged grasp survive a stationary hold?"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import traceback
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))
from scripts.run_pair_navigation import PairNavigationScene, ROBOTS
from scripts.run_camera_approach_student import models, write
from harness.grasp_student_inference import predict_student


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out-dir', type=Path, required=True)
    p.add_argument('--grasp-model-dir', type=Path, required=True)
    p.add_argument('--map', type=Path, required=True)
    p.add_argument('--wrist-delta', type=int, default=0)
    p.add_argument('--impratio', type=int, choices=(1, 10, 100), default=1)
    p.add_argument('--seconds', type=int, default=16)
    a = p.parse_args()
    if not -100 <= a.wrist_delta <= 100: p.error('bounded wrist diagnostic only')
    if not 1 <= a.seconds <= 120: p.error('hold must be 1..120 seconds')
    out = a.out_dir.resolve()
    if out.exists(): raise FileExistsError(out)
    _, ms = models(a.grasp_model_dir.resolve(), 'student-skill.json')
    scene = PairNavigationScene(out, a.grasp_model_dir.resolve(), json.loads(a.map.read_text()), impratio=a.impratio)
    rec = {'source_sha': subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
           'diagnostic_only': True, 'wrist_delta': a.wrist_delta, 'impratio': a.impratio,
           'seconds': a.seconds, 'error': None}
    try:
        scene.open(); rec['invariants_initial'] = scene.invariant_record()
        scene.finish_grasp(predict_student, ms)
        write(out/'grasp-result.json', scene.grasp_report)
        if a.wrist_delta:
            scene.replay([{'targets':{r:{3:scene.commands[r][3]+a.wrist_delta} for r in ROBOTS},
                           'duration_s':.5,'settle_s':.2}], 'diagnostic_wrist')
        for index in range(a.seconds*5):
            scene.capture(f'hold-{index:03d}')
            scene.execute({r:{'kind':'mecanum','forward':0.,'left':0.,'turn':0.,'duration_s':.2} for r in ROBOTS})
        rows = [r for r in scene.evaluation_samples if r['phase'] in ('carry','carry_stop')]
        good = lambda r: all(c['bilateral'] for c in r['contacts'].values()) and r['height_above_start_m']>=.03
        bad = [r for r in rows if not good(r)]
        rec.update(success=not bad, first_failure_s=None if not bad else bad[0]['sim_time_s']-rows[0]['sim_time_s'],
                   initial=rows[0], final=rows[-1], invariants_final=scene.invariant_record())
    except Exception:
        rec['error'] = traceback.format_exc()
    finally:
        scene.close(); out.mkdir(parents=True,exist_ok=True); write(out/'result.json',rec)
    print(json.dumps({k:rec.get(k) for k in ('success','first_failure_s','error')}))

if __name__ == '__main__': main()
