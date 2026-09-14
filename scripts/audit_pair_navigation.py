#!/usr/bin/env python3
"""Replay exact serialized RGB inputs, plan agreement, commands and evaluation."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))
from harness.pair_navigation import PairNavigator, ROBOTS, authorize_pair, digest
from harness.pair_carry_sync import PairCarrySync
from harness.pair_grasp_spacing import PairGraspSpacing, HOLD_DT
from scripts.audit_pair_carry_sync import _rgb, _same, _safe_file
from scripts.audit_camera_grasp_student import audit as audit_grasp
from scripts.evaluate_pair_navigation import evaluate_samples, evaluate_grasp_stability


def audit(run_dir, grasp_dir=None):
    root = Path(run_dir).resolve()
    report = json.loads(_safe_file(root, 'result.json').read_text())
    if report['schema'] != 'ugrp.pair_navigation_trial.v1' or digest(report['map']) != report['map_sha256']:
        raise ValueError('map/schema mismatch')
    import hashlib
    if hashlib.sha256((ROOT/'harness/assets/pair_navigation/manifest.json').read_bytes()).hexdigest() != report['appearance_manifest_sha256']:
        raise ValueError('appearance model manifest mismatch')
    actors = {r: PairNavigator(report['map'], r, vision_mode=report.get('vision_mode','legacy')) for r in ROBOTS}
    sync = PairCarrySync(report['map']['map_id'])
    seen = {r:set() for r in ROBOTS}
    spacing = {r:PairGraspSpacing(report['map'],r) for r in ROBOTS}
    spacing_sync = PairCarrySync(report['map']['map_id']+'-grasp-spacing')
    for i,row in enumerate(report.get('spacing_steps',[])):
        if set(row)!={'index','images','frame_ids','decisions','permission','issued_actions','sim_time_s','executed'} or row['index']!=i:
            raise ValueError('spacing step schema mismatch / hidden actor input')
        decisions = {}
        for r in ROBOTS:
            if set(row['images'][r])!={'own','top'} or row['frame_ids'][r] in seen[r]:
                raise ValueError('spacing image schema or freshness mismatch')
            seen[r].add(row['frame_ids'][r])
            decisions[r]=spacing[r].decide(_rgb(root,row['images'][r]['own']),_rgb(root,row['images'][r]['top']))
            if not _same(decisions[r],row['decisions'][r]):raise ValueError(f'spacing RGB decision mismatch at {i}/{r}')
        if row['images']['r1']['top']!=row['images']['r3']['top']:raise ValueError('different common spacing image')
        permission=authorize_pair(spacing_sync,decisions,row['frame_ids'],i,interval_s=HOLD_DT)
        if permission!=row['permission']:raise ValueError('spacing permission mismatch')
        for r in ROBOTS:
            action=dict(decisions[r]['action'])
            if permission['phase']!='GO':action.update(forward=0.,left=0.,turn=0.)
            if not _same(action,row['issued_actions'][r]):raise ValueError('spacing issued action mismatch')
    if not _same(spacing_sync.events,report.get('spacing_sync_events',[])):
        raise ValueError('spacing coordination trace mismatch')
    if report.get('grasp_spacing')=='visual' and not report.get('error'):
        if len(report.get('spacing_steps',[]))<104 or not all(a.stable_frames>=5 for a in spacing.values()):
            raise ValueError('missing full visual pre-drive hold')
    for i, row in enumerate(report['steps']):
        if set(row) != {'index','images','frame_ids','decisions','permission','issued_actions','sim_time_s'} or row['index'] != i:
            raise ValueError('step schema mismatch / hidden actor input')
        if any(set(row[k]) != set(ROBOTS) for k in ('images','frame_ids','decisions','issued_actions')):
            raise ValueError('participant coverage mismatch')
        decisions = {}
        for rid in ROBOTS:
            if set(row['images'][rid]) != {'own','top'}: raise ValueError('unexpected input field')
            frame = row['frame_ids'][rid]
            if frame in seen[rid]: raise ValueError('reused frame')
            seen[rid].add(frame)
            decisions[rid] = actors[rid].decide(_rgb(root,row['images'][rid]['own']), _rgb(root,row['images'][rid]['top']))
            if not _same(decisions[rid], row['decisions'][rid]):
                raise ValueError(f'RGB decision mismatch at {i}/{rid}')
        if row['images']['r1']['top'] != row['images']['r3']['top']:
            raise ValueError('participants received different common camera frame')
        permission = authorize_pair(sync, decisions, row['frame_ids'], i)
        if permission != row['permission']: raise ValueError('permission mismatch')
        for rid in ROBOTS:
            action = dict(decisions[rid]['action'])
            if permission['phase'] != 'GO': action.update(forward=0.,left=0.,turn=0.)
            if not _same(action,row['issued_actions'][rid]): raise ValueError('issued action mismatch')
    if not _same(sync.events, report['sync_events']): raise ValueError('coordination trace mismatch')
    final = report['steps'][-1]['decisions'] if report['steps'] else None
    if report['arrived'] != bool(final and all(final[r]['done'] for r in ROBOTS)): raise ValueError('false arrival')
    rows = [json.loads(x) for x in _safe_file(root, 'evaluation-only.jsonl').read_text().splitlines()]
    collisions = json.loads(_safe_file(root,'contact-events-evaluation-only.json').read_text())
    wall, unexpected = sum(c['wall'] for c in collisions), sum(c['unexpected'] for c in collisions)
    if wall != report['wall_contact_ticks'] or unexpected != report['unexpected_contact_ticks']:
        raise ValueError('contact count mismatch')
    evaluation = evaluate_samples(rows, report['map'], arrived=report['arrived'],
                  invariants_match=report['invariants_initial']==report['invariants_final'],
                  weld_ticks=report['weld_active_ticks'],wall_contact_ticks=wall,unexpected_contact_ticks=unexpected,
                  require_full_grasp=report.get('grasp_spacing')=='visual')
    if not _same(evaluation,report['evaluation']): raise ValueError('referee evaluation mismatch')
    grasp_stability=evaluate_grasp_stability(rows)
    if 'grasp_stability' in report and not _same(grasp_stability,report['grasp_stability']):raise ValueError('grasp stability evaluation mismatch')
    scored=grasp_stability if report.get('grasp_only') else evaluation
    if report['success'] != bool(not report['error'] and scored['success']): raise ValueError('false success')
    grasp = audit_grasp(root, Path(grasp_dir).resolve(), report_name='grasp-result.json') if grasp_dir else None
    if grasp is not None and not grasp.get('ok'): raise ValueError('grasp input audit failed: '+str(grasp))
    if grasp_dir and hashlib.sha256((Path(grasp_dir)/'student-skill.json').read_bytes()).hexdigest() != report['grasp_skill_sha256']:
        raise ValueError('grasp manifest mismatch')
    if hashlib.sha256(_safe_file(root,'scene.xml').read_bytes()).hexdigest() != report['scene_xml_sha256']:
        raise ValueError('compiled scene hash mismatch')
    return {'passed': True, 'steps': len(report['steps']), 'spacing_steps':len(report.get('spacing_steps',[])), 'grasp': grasp,
            'physical_success': evaluation['success'], 'grasp_stability':grasp_stability,
            'refused_no_route':report.get('refused_no_route',False),
            'scope': 'Exact RGB/command replay and output-only scoring; no actual physics rerun'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run_dir',type=Path);p.add_argument('--grasp-model-dir',type=Path)
    a=p.parse_args(); result=audit(a.run_dir,a.grasp_model_dir)
    (a.run_dir/'input-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__ == '__main__': main()
