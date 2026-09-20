#!/usr/bin/env python3
"""Prepare all 12 multi-object scenes and static admission evidence, no transport."""
from __future__ import annotations

import argparse
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from sim.multi_object_suite import load_pilot
from sim.multi_object_scene import configuration, static_task, geometry_admission, SPEC
from scripts.prepare_act_map_suite import write, file_sha


def diagram(config, geometry, path):
    from PIL import Image, ImageDraw
    from scripts.build_pair_terrain_gallery import font
    img = Image.new('RGB',(640,560),'white'); draw = ImageDraw.Draw(img)
    draw.text((16,10),config['case_id'],fill='#18344b',font=font(24))
    draw.text((16,44),'SETUP / EVALUATION ONLY',fill='#617386',font=font(16))
    xmin,xmax,ymin,ymax = config['static_map']['bounds_m']
    scale = min(600/(xmax-xmin),390/(ymax-ymin))
    def xy(p): return (20+(p[0]-xmin)*scale,76+(ymax-p[1])*scale)
    def rectangle(center,half,**kwargs):
        x,y = center; hx,hy = half
        draw.rectangle((*xy((x-hx,y+hy)),*xy((x+hx,y-hy))),**kwargs)
    rectangle([(xmin+xmax)/2,(ymin+ymax)/2],[(xmax-xmin)/2,(ymax-ymin)/2],fill='#f2f5f8',outline='#687d90')
    for did,slot in config['static_map']['destinations'].items():
        rectangle(slot['center_m'],slot['half_extents_m'],outline='#8b598e',width=2)
        x,y = xy(slot['center_m']); draw.text((x-12,y+10),did,font=font(10),fill='#8b598e')
    for rid,pose in config['setup_only']['spawns'].items():
        rectangle(pose[:2],[.13,.12],fill='#4d5965')
        x,y = xy(pose); draw.text((x-8,y-10),rid,font=font(14),fill='white')
    for oid,item in config['setup_only']['objects'].items():
        rectangle(item['position_m'][:2],item['half_extents_m'][:2],fill='#d87528' if item['kind']=='beam' else '#329fa7')
        x,y = xy(item['position_m']); draw.text((x+8,y-10),oid,font=font(12),fill='#18344b')
    draw.text((16,488),geometry['classification'],fill='#18344b',font=font(20))
    draw.text((16,522),'정적 배치·기하 검사 / 실제 파지·운반 미실행',fill='#617386',font=font(17))
    img.save(path)


def render_scene(config, out):
    from scripts.multi_object_scene_preview import MultiObjectScene
    scene = MultiObjectScene(config,out)
    try:
        scene.open()
        audit = scene.compiled_audit()
        frames = scene.capture('preview')
        audit['rgb'] = {r:{k:v for k,v in row.items() if not k.endswith('_bytes')} for r,row in frames.items()}
        visibility = scene.visibility(audit)
        write(out/'visibility-evaluation-only.json',visibility)
        audit['visibility'] = visibility
        audit['top_visibility_gate'] = all(row['top']['visible_pixels'] >= 16 and
            row['top']['approximate_visible_fraction'] >= .85 for row in visibility.values())
        audit['initialization_clear'] = not (audit['initial_contacts'] or audit['settled_contacts'])
        audit['cargo_xy_stable'] = all(row['initial_xy_drift_m'] <= .002 for row in audit['cargo'].values())
        audit['camera_calibration_id'] = scene.world.controllers['r1'].camera_calibration_id
        return audit
    finally:
        scene.close()


def gallery(output, cases, render):
    from PIL import Image,ImageDraw
    from scripts.build_pair_terrain_gallery import font
    for view in ('diagram','top','r1','r2','r3') if render else ('diagram',):
        sheet = Image.new('RGB',(1600,4*345),'white'); draw = ImageDraw.Draw(sheet)
        for i,case in enumerate(cases):
            name = case['id']; x,y = (i%4)*400,(i//4)*345
            draw.text((x+5,y+5),name,font=font(17),fill='#18344b')
            source = output/name/'diagram.png' if view=='diagram' else output/name/'preview/rgb'/f'preview-{view}.jpg'
            if source.exists():
                with Image.open(source) as img:
                    img.thumbnail((390,310)); sheet.paste(img,(x+5,y+30))
        # 12 cases; crop the unused fourth row when the full pilot is selected.
        sheet.crop((0,0,1600,((len(cases)+3)//4)*345)).save(output/f'{view}-overview.jpg',quality=92)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--render',action='store_true')
    p.add_argument('--require-static-candidates',action='store_true',help='CI gate: fail if any rendered case fails static admission')
    p.add_argument('--case',choices=[c['id'] for c in load_pilot()[1]])
    args = p.parse_args()
    if args.require_static_candidates and not args.render: p.error('static admission gate requires --render')
    if args.output.exists(): p.error('output exists; choose a new path')
    if subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        p.error('commit source and configuration before running a cohort')
    source = subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    _,cases = load_pilot()
    # Start with the requested three-object scene; it also exercises staging.
    cases.sort(key=lambda c:(c['id']!='staging_three',len(c['mission']['objects']),c['id']))
    if args.case: cases = [c for c in cases if c['id']==args.case]
    args.output.mkdir(parents=True)
    write(args.output/'layout-spec.json',json.loads(SPEC.read_text()))
    write(args.output/'environment.json',{'python':sys.version,'platform':platform.platform(),
        'source_sha':source,'command':sys.argv,
        'packages':{n:importlib.metadata.version(n) for n in ('numpy','Pillow','mujoco')},
        'scope':'static scene preparation only'})
    started = time.monotonic(); results = []; baseline = None
    for case in cases:
        tick = time.monotonic(); folder = args.output/case['id']; folder.mkdir()
        row = {'id':case['id'],'objects':len(case['mission']['objects']),'tasks':len(case['mission']['tasks']),
               'geometry':None,'render':None,'error':None,'static_candidate':False}
        try:
            config = configuration(case)
            write(folder/'scene-setup-only.json',config)
            write(folder/'static-task.json',static_task(config))
            row['geometry'] = geometry_admission(config)
            write(folder/'geometry-evaluation-only.json',row['geometry'])
            diagram(config,row['geometry'],folder/'diagram.png')
            if args.render:
                row['render'] = render_scene(config,folder/'preview')
                inv = row['render']['invariants']
                fixed = {k:inv[k] for k in ('robot_xml_sha256','prototype_shape_sha256','policy_cameras')}
                if baseline is None: baseline = fixed
                if fixed != baseline: raise ValueError('robot, cargo prototype or camera changed across cases')
                row['static_candidate'] = (row['geometry']['classification']=='geometric_candidate' and
                    all(row['render'][k] for k in ('top_visibility_gate','initialization_clear','cargo_xy_stable')))
        except Exception as exc:
            row['error'] = f'{type(exc).__name__}: {exc}'
        row['wall_seconds'] = time.monotonic()-tick
        write(folder/'result.json',row); results.append(row)
        print(json.dumps({'id':row['id'],'static_candidate':row['static_candidate'],'error':row['error'],
                          'geometry':row['geometry']['classification'] if row['geometry'] else None,
                          'wall_seconds':round(row['wall_seconds'],2)}),flush=True)
    gallery(args.output,cases,args.render)
    manifest = {'source_sha':source,'layout_spec_file_sha256':file_sha(SPEC),'scope':'static admission only',
        'physical_map_instances':1,'split':'train','cases':[]}
    for row in results:
        folder = args.output/row['id']
        cfgpath = folder/'scene-setup-only.json'
        config = json.loads(cfgpath.read_text()) if cfgpath.exists() else {}
        manifest['cases'].append({'id':row['id'], 'objects':row['objects'],'tasks':row['tasks'],
            **{k:config.get(k) for k in ('source_map_id','source_map_sha256','mission_sha256','static_map_sha256')},
            'setup_file_sha256':file_sha(cfgpath) if cfgpath.exists() else None,
            'static_task_file_sha256':file_sha(folder/'static-task.json') if (folder/'static-task.json').exists() else None,
            'scene_xml_sha256':row['render']['invariants']['scene_xml_sha256'] if row['render'] else None})
    summary = {'source_sha':source,'cases':len(cases),'render_requested':args.render,
        'static_candidates':sum(r['static_candidate'] for r in results),
        'geometry_candidates':sum(bool(r['geometry'] and r['geometry']['classification']=='geometric_candidate') for r in results),
        'errors':sum(r['error'] is not None for r in results),'transport_attempts':0,'policy_calls':0,'api_cost_usd':0,
        'wall_seconds':time.monotonic()-started,'input_boundary':'setup, geometry, masks and referee output never enter static-task.json',
        'scope':'static candidates require visual review; actual grasp, joint execution and transport untested'}
    write(args.output/'manifest.json',manifest); write(args.output/'results.json',results); write(args.output/'summary.json',summary)
    write(args.output/'artifact-hashes.json',{str(p.relative_to(args.output)):file_sha(p) for p in sorted(args.output.rglob('*')) if p.is_file()})
    print(json.dumps(summary),flush=True)
    return int(summary['errors']>0 or (args.require_static_candidates and summary['static_candidates']!=len(cases)))


if __name__=='__main__':
    raise SystemExit(main())
