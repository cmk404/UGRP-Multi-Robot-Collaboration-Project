#!/usr/bin/env python3
"""Build map diagrams and bounded static MuJoCo previews, not navigation trials."""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from harness.pair_navigation import digest, footprint_clear, plan_route, swept_clear, validate_map

CATALOG = ROOT / 'experiments/2026-09-14-pair-terrain-examples/catalog.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def geometry(data, nominal):
    validate_map(data)
    goal = [*data['goal']['center_m'], math.radians(data['goal']['relative_yaw_deg'])]
    assert footprint_clear(nominal, data) and footprint_clear(goal, data)
    route = plan_route(nominal, goal, data)
    if route:
        assert all(swept_clear(a, b, data) for a, b in zip(route, route[1:]))
    nearby = []
    for x in (.60, .62, .64):
        for y in (-2.01, -2., -1.99):
            nearby.append({'nominal_pose_m_rad': [x, y, 0.],
                           'route_found': plan_route([x, y, 0.], goal, data) is not None})
    probe_clear = all(swept_clear((.58, -2., angle),
        (.58+.065*math.cos(angle), -2.+.065*math.sin(angle), angle), data)
        for angle in [math.radians(-15+5*i) for i in range(7)])
    return {'scope': 'Static geometry only, never observed robot state or physical success',
            'nominal_start_m_rad': nominal, 'goal_pose_m_rad': goal,
            'direct_swept_clear': swept_clear(nominal, goal, data),
            'route_found': route is not None, 'route_m_rad': route,
            'nearby_planning_starts': nearby, 'fixed_probe_envelope_clear': probe_clear}


def font(size):
    from PIL import ImageFont
    for path in ('/System/Library/Fonts/AppleSDGothicNeo.ttc',
                 '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    raise RuntimeError('Korean preview font missing: install fonts-noto-cjk on Ubuntu')


def diagram(entry, data, check):
    from PIL import Image, ImageDraw
    canvas = Image.new('RGB', (620, 490), '#ffffff')
    draw = ImageDraw.Draw(canvas)
    draw.text((22, 14), f"{entry['number']:02d}  {entry['title_ko']}", font=font(27), fill='#182b42')
    xmin, xmax, ymin, ymax = data['bounds_m']
    left, top, width = 22, 63, 576
    scale = width/(xmax-xmin)
    height = (ymax-ymin)*scale
    def xy(p):
        return (left+(p[0]-xmin)*scale, top+(ymax-p[1])*scale)
    draw.rectangle((left, top, left+width, top+height), fill='#eef3f8', outline='#bac9d8', width=2)
    for x in range(-2, 5):
        a, b = xy((x*.5, ymin)), xy((x*.5, ymax))
        if left < a[0] < left+width: draw.line((a,b), fill='#dce5ed')
    for y in (-2.5, -2., -1.5):
        draw.line((xy((xmin,y)),xy((xmax,y))),fill='#dce5ed')
    for box in data['obstacles']:
        x,y=box['center_m']; hx,hy=box['half_extents_m']
        draw.rectangle((*xy((x-hx,y+hy)),*xy((x+hx,y-hy))), fill='#3c4b61')
    def footprint(pose, color):
        x,y,angle=pose;f=data['footprint']
        hx=f['half_forward_m']+f['margin_m'];hy=f['half_lateral_m']+f['margin_m']
        corners=[xy((x+math.cos(angle)*u-math.sin(angle)*v,
                     y+math.sin(angle)*u+math.cos(angle)*v)) for u,v in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]]
        draw.line(corners+[corners[0]], fill=color, width=2)
    route=check['route_m_rad']
    if route:
        for a,b in zip(route,route[1:]):
            pa,pb=xy(a),xy(b);length=math.dist(pa,pb)
            for t in range(0, int(length), 13):
                lo,hi=t/max(length,1),min(t+7,length)/max(length,1)
                draw.line([(pa[0]+(pb[0]-pa[0])*lo,pa[1]+(pb[1]-pa[1])*lo),
                           (pa[0]+(pb[0]-pa[0])*hi,pa[1]+(pb[1]-pa[1])*hi)], fill='#147dc2', width=4)
    for pose,label,color in [(check['nominal_start_m_rad'],'S','#147dc2'),
                             (check['goal_pose_m_rad'],'G','#127967')]:
        footprint(pose,color);x,y=xy(pose)
        draw.ellipse((x-15,y-15,x+15,y+15),fill=color)
        draw.text((x-7,y-12),label,font=font(23),fill='white')
    status='계산 경로 있음 · 실제 주행 전' if check['route_found'] else '계산 경로 없음 · 완전 차단 대조'
    draw.text((22,424),status,font=font(21),fill='#127967' if check['route_found'] else '#b43e46')
    draw.text((22,457),'영역 2.93 × 1.75 m  |  벽 높이 0.30 m',font=font(17),fill='#576d80')
    return canvas


def metadata_model(out):
    """Only setup metadata is needed; no prediction models are run in previews."""
    source=ROOT/'experiments/2026-09-10-rgb-varied-start'
    manifest=json.loads((source/'models-manifest.json').read_text())
    assert sha(source/'models.zip')==manifest['archive_sha256']
    target=out/'_setup_metadata';target.mkdir()
    with zipfile.ZipFile(source/'models.zip') as archive:
        for name in ('student-skill.json','evaluation-fixture.json'):
            member='models/grasp/'+name
            expected=next(f['sha256'] for f in manifest['files'] if f['archive_path']==member)
            raw=archive.read(member);assert hashlib.sha256(raw).hexdigest()==expected
            (target/name).write_bytes(raw)
    return target,manifest['archive_sha256']


def preview(data, out, model_root):
    import mujoco
    import numpy as np
    from scripts.run_pair_navigation import PairNavigationScene
    scene=PairNavigationScene(out,model_root,data,impratio=1)
    record={}
    try:
        scene.open()
        initial=scene.invariant_record()
        top_camera = initial['policy_cameras']['cctv_top']
        assert top_camera['position'] == data['top_camera']['position_m']
        assert top_camera['quaternion'] == data['top_camera']['quaternion_wxyz']
        assert top_camera['fov_y_deg'] == data['top_camera']['fov_y_deg']
        assert initial['contact_solver']['impratio'] == 1
        assert initial['contact_solver']['noslip_iterations'] == 0
        scene.capture('preview')
        (out/'observer.jpg').write_bytes(scene.world.render_team_jpeg(camera='cctv_warehouse',quality=95))
        model, state=scene.world.model,scene.world.data
        wall_matches=[]
        for wall in data['obstacles']:
            gid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'pair_wall_'+wall['id'])
            assert gid>=0
            expected_position=[*wall['center_m'],wall['height_m']/2]
            expected_size=[*wall['half_extents_m'],wall['height_m']/2]
            assert np.allclose(model.geom_pos[gid],expected_position,atol=1e-10)
            assert np.allclose(model.geom_size[gid],expected_size,atol=1e-10)
            wall_matches.append({'id':wall['id'],'position_m':model.geom_pos[gid].tolist(),
                                 'half_extents_m':model.geom_size[gid].tolist()})
        overlap=[]
        for contact in state.contact[:state.ncon]:
            pair={int(contact.geom1),int(contact.geom2)}
            if pair & scene.wall_ids and pair & (scene.robot_geoms | {scene.beam_geom}):
                overlap.append(sorted(pair))
        assert not overlap
        assert not any(scene.world._beam_constraint_active(r) for r in ('r1','r3'))
        assert initial==scene.invariant_record()
        # Compare the original geoms by name across previews, excluding only the
        # added walls. Never feed these output-only fingerprints to a controller.
        original=[]
        for gid in range(model.ngeom):
            if gid in scene.wall_ids: continue
            original.append({'name':scene.geom_names[gid], 'type':int(model.geom_type[gid]),
                **{k:getattr(model,k)[gid].tolist() for k in ('geom_size','geom_pos','geom_quat','geom_friction','geom_rgba')}})
        record={'compiled_walls':wall_matches,'initial_wall_robot_or_payload_contacts':overlap,
                'invariants':initial,'original_geoms_sha256':digest(original),
                'scene_xml_sha256':sha(out/'scene.xml'),'setup_sim_seconds':scene.time(),
                'weld_off':True,'grasp_or_navigation_executed':False,
                'scope':'Static scene after unchanged folded-arm setup, not physical passage validation'}
    finally:
        scene.close()
    return record


def main():
    started = time.monotonic()
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--catalog',type=Path,default=CATALOG)
    p.add_argument('--out-dir',type=Path,required=True)
    p.add_argument('--geometry-only',action='store_true')
    a=p.parse_args();out=a.out_dir.resolve()
    if out.exists():raise FileExistsError(out)
    if not a.geometry_only and subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).strip():
        raise RuntimeError('commit the source, maps and protocol before static rendering')
    catalog=json.loads(a.catalog.read_text());out.mkdir(parents=True);(out/'plans').mkdir()
    from PIL import Image,ImageDraw
    overview=Image.new('RGB',(1920,1110),'#e6edf4');draw=ImageDraw.Draw(overview)
    draw.text((28,18),'공동 운반을 위한 예시 지형 5종 + 차단 대조',font=font(36),fill='#182b42')
    draw.text((28,69),'S 시작  ·  G 목표  ·  테두리: 두 로봇과 짐의 계획 공간  ·  점선: 계산 경로, 실제 주행 아님',font=font(22),fill='#435e75')
    record={'schema':'ugrp.pair_terrain_gallery.v1','source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
            'catalog_sha256':sha(a.catalog),'environment':{'python':sys.version,'platform':platform.platform()},
            'geometry_only':a.geometry_only,'physical_navigation_tested':False,'external_model_calls':0,'cost_usd':0,'entries':[]}
    model_root=None
    if not a.geometry_only:
        import mujoco
        record['environment']['mujoco']=mujoco.__version__
        model_root,record['model_archive_sha256']=metadata_model(out)
        baseline=json.loads((a.catalog.parent/catalog['entries'][0]['map']).read_text())
        baseline['obstacles']=[]
        record['no_added_walls_reference']=preview(baseline,out/'_baseline',model_root)
    cards=[]
    for i,entry in enumerate(catalog['entries']):
        data=json.loads((a.catalog.parent/entry['map']).read_text())
        check=geometry(data,catalog['nominal_planning_start_m_rad'])
        assert check['route_found']==entry['expected_geometric_route']
        assert all(x['route_found']==entry['expected_geometric_route'] for x in check['nearby_planning_starts'])
        assert check['fixed_probe_envelope_clear']
        pic=diagram(entry,data,check);pic.save(out/'plans'/f"{entry['id']}.png")
        overview.paste(pic,(20+(i%3)*640,115+(i//3)*495))
        item={**entry,'map_sha256':digest(data),'geometry':check}
        if model_root:
            item['scene']=preview(data,out/entry['id'],model_root)
        record['entries'].append(item)
        links=''
        if model_root:
            links=f'<a href="{entry["id"]}/rgb/preview-top.jpg">실제 top 카메라</a> · <a href="{entry["id"]}/observer.jpg">관찰 시점</a>'
        cards.append(f'<article><h2>{entry["number"]:02d} {html.escape(entry["title_ko"])}</h2><a href="plans/{entry["id"]}.png"><img src="plans/{entry["id"]}.png" alt="{html.escape(entry["title_ko"])} 도면"></a><p>{html.escape(entry["description_ko"])}</p><p>{links}</p></article>')
        print(json.dumps({'id':entry['id'],'route_found':check['route_found'],'rendered':model_root is not None}),flush=True)
    if model_root:
        scenes=[e['scene'] for e in record['entries']]
        reference=record['no_added_walls_reference']
        assert all(s['original_geoms_sha256']==reference['original_geoms_sha256'] for s in scenes)
        assert all(s['invariants']['policy_cameras']==reference['invariants']['policy_cameras'] for s in scenes)
        assert all(s['invariants']['contact_solver']==reference['invariants']['contact_solver'] for s in scenes)
        assert all(s['invariants']['geometry_sha256'][field]==reference['invariants']['geometry_sha256'][field]
                   for s in scenes for field in ('body_mass', 'body_inertia'))
        record['original_geometry_and_cameras_equal_across_examples']=True
        record['mass_inertia_and_solver_equal_across_examples']=True
        actual=Image.new('RGB',(1920,1130),'#e6edf4');labels=ImageDraw.Draw(actual)
        labels.text((28,18),'실제 MuJoCo 장면 · 고정 top 카메라',font=font(36),fill='#182b42')
        labels.text((28,69),'같은 지도 배치의 정적 렌더 · 파지와 운반은 실행하지 않음',font=font(22),fill='#435e75')
        for i,entry in enumerate(catalog['entries']):
            x,y=20+(i%3)*640,120+(i//3)*505
            labels.text((x,y),f"{entry['number']:02d} {entry['title_ko']}",font=font(23),fill='#182b42')
            pic=Image.open(out/entry['id']/'rgb/preview-top.jpg').convert('RGB').resize((620,465))
            actual.paste(pic,(x,y+33))
        actual.save(out/'simulator-overview.jpg',quality=95)
    overview.save(out/'overview.png')
    (out/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>공동 운반 예시 지형</title><style>body{margin:0;padding:30px;font:16px system-ui;background:#e6edf4;color:#182b42}header,main{max-width:1500px;margin:auto}h1{font-size:32px}header p{line-height:1.7}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(350px,1fr));gap:20px}article{background:white;border-radius:15px;padding:20px}h2{font-size:23px;margin:0}img{width:100%;height:auto}p{line-height:1.6}a{color:#096caa}</style><header><h1>공동 운반 예시 지형</h1><p>평지 대표 배치 5종과 완전 차단 대조 1개입니다. 도면의 점선은 계산 경로이며, 파지·주행·통과 시험은 아직 수행하지 않았습니다. 실제 카메라 영상은 지도와 같은 MuJoCo 장면을 렌더한 것입니다.</p></header><main>'+''.join(cards)+'</main></html>')
    record['elapsed_wall_seconds'] = time.monotonic() - started
    record['navigation_actions'] = 0
    assert record['source_sha'] == subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    record['files']=[{'path':str(f.relative_to(out)),'bytes':f.stat().st_size,'sha256':sha(f)} for f in sorted(out.rglob('*')) if f.is_file()]
    write(out/'gallery-record.json',record)


if __name__=='__main__':main()
