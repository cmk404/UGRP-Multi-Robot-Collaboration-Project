import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from sim.multi_object_suite import load_pilot
from sim.multi_object_scene import (configuration, static_task, shape_signature, build_multi_object_xml,
                                    geometry_admission, translation_path)
from sim.research_dispatch_arena import digest, FIXED_TOP
from harness.pair_navigation import swept_clear


def config(name='staging_three'):
    return configuration(next(c for c in load_pilot()[1] if c['id']==name))


def source():
    robots = ''.join(f'<body name="{r}__robot"><geom name="{r}__body" size=".1 .1 .1"/><camera name="{r}__robot_cam" pos=".1 0 .2" fovy="61"/></body>' for r in ('r1','r2','r3'))
    return f'''<mujoco><worldbody><geom name="floor" type="plane" size="5 5 .1"/>{robots}
      <body name="team_beam" pos=".5 -2 .02"><freejoint name="team_beam_free"/>
      <geom name="team_beam_geom" type="box" size=".025 .225 .02" mass=".196" rgba="1 .4 0 1"/>
      <body name="team_beam_r1_endpoint" pos="0 -.2 0"/></body>
      <camera name="cctv_top" pos=".55 -2 2.5" fovy="55"/></worldbody>
      <equality><weld name="old" body1="r1__robot" body2="team_beam" active="true"/></equality></mujoco>'''


def test_twelve_scenes_keep_exact_counts_on_one_train_map():
    configs = [configuration(c) for c in load_pilot()[1]]
    assert len(configs)==12 and len({c['source_map_sha256'] for c in configs})==1
    assert all(c['split']=='train' for c in configs)
    for c in configs:
        assert len(c['setup_only']['objects']) == len(c['mission']['objects'])
        assert c['static_map']['top_camera']==FIXED_TOP
        assert c['static_map_sha256']==digest(c['static_map'])
        assert len(c['static_map']['destinations'])==len(c['mission']['tasks'])
    assert {len(c['setup_only']['objects']) for c in configs}=={1,2,3,4,5,6,8}


@pytest.mark.parametrize('name',[c['id'] for c in load_pilot()[1]])
def test_xml_clones_preserve_geometry_and_cameras_without_extra_active_cargo(name):
    c = config(name); raw=source(); xml,record=build_multi_object_xml(raw,c)
    before,after=ET.fromstring(raw),ET.fromstring(xml)
    assert len(record['cargo_replicas'])==len(c['mission']['objects'])
    for rid in ('r1','r2','r3'):
        assert ET.tostring(before.find(f"worldbody/body[@name='{rid}__robot']"))==ET.tostring(after.find(f"worldbody/body[@name='{rid}__robot']"))
    joints=[]
    for oid,item in c['setup_only']['objects'].items():
        body=after.find(f"worldbody/body[@name='{item['body_name']}']")
        assert shape_signature(body)==record['prototype_shape_sha256'][item['kind']]
        joint=body.find('freejoint')
        if joint is None: joint=body.find('joint')
        joints.append(joint.get('name'))
    assert len(set(joints))==len(joints)
    for name in ('team_beam','dispatch_box'):
        for geom in after.find(f"worldbody/body[@name='{name}']").iter('geom'):
            assert geom.get('rgba')=='0 0 0 0' and geom.get('contype')=='0' and geom.get('conaffinity')=='0'
    assert all(eq.get('active')=='false' for eq in after.findall('equality/weld'))


def test_setup_and_evaluation_cannot_enter_static_task():
    c=config(); expected=static_task(c)
    c['setup_only']['objects']['object_01']['position_m']=[99,99,99]
    c['evaluation_only']={'success':True,'segmentation_ids':[1,2]}
    assert static_task(c)==expected
    for word in ('setup_only','position_m','body_name','segmentation','geometry_parameters','spawns'):
        # Camera position_m is an explicitly permitted static calibration.
        text=json.dumps({k:v for k,v in expected.items() if k!='static_map'})
        assert word not in text
    for place in ('map','obstacle','slot'):
        polluted=copy.deepcopy(c)
        target=(polluted['static_map'] if place=='map' else polluted['static_map']['obstacles'][0]
                if place=='obstacle' else next(iter(polluted['static_map']['destinations'].values())))
        target['live_truth']=[1,2]
        polluted['static_map_sha256']=digest(polluted['static_map'])
        with pytest.raises(ValueError,match='static map field'): static_task(polluted)


def test_source_map_split_and_output_identifiers_fail_closed():
    case=copy.deepcopy(load_pilot()[1][0]); case['id']='../../elsewhere'
    with pytest.raises(ValueError,match='unsafe'): configuration(case)
    from sim.multi_object_scene import SPEC
    layout=json.loads(SPEC.read_text()); layout['source_case']['split']='test_a'
    with pytest.raises(ValueError,match='train-only'): configuration(load_pilot()[1][0],layout)


def test_translation_search_checks_whole_footprint_and_continuous_sweeps():
    data={'bounds_m':[0,3,0,3],'obstacles':[{'center_m':[1.5,1.5],'half_extents_m':[.08,.6]}],
          'footprint':{'half_forward_m':.2,'half_lateral_m':.2,'margin_m':.025}}
    path=translation_path([.5,1.5],[2.5,1.5],data)
    assert path and len(path)>2 and all(swept_clear(a,b,data) for a,b in zip(path,path[1:]))
    data['obstacles'][0]['half_extents_m'][1]=1.5
    assert translation_path([.5,1.5],[2.5,1.5],data) is None


def test_geometry_reports_overlapping_slots_without_changing_counts():
    c=config('mixed_pair')
    slots=list(c['static_map']['destinations'].values()); slots[1]['center_m']=list(slots[0]['center_m'])
    result=geometry_admission(c)
    assert result['slot_overlaps'] and result['classification']=='unresolved_layout'
    assert len(c['setup_only']['objects'])==2


def test_geometry_keeps_delivered_and_remaining_objects_in_placement_ledger():
    c=config('three_boxes'); result=geometry_admission(c)
    assert result['complete_placement_order_found']
    for row in result['trace']:
        assert set(row['obstacle_object_ids'])==set(c['setup_only']['objects'])-{row['object_id']}
    assert result['grasp_and_transport']=='not_run' and result['joint_robot_trajectory']=='not_planned'


def test_staging_layout_can_unload_boxes_after_beam_with_full_footprint():
    c=config('staging_three'); result=geometry_admission(c)
    assert result['complete_placement_order_found'] and len(result['trace'])==5
    order=[r['task_id'] for r in result['trace']]
    assert max(order.index('stage_02'),order.index('stage_03')) < order.index('deliver_01')
    assert order.index('deliver_01') < min(order.index('deliver_02'),order.index('deliver_03'))
    # Previous goal centers blocked the box chassis after the beam was placed.
    old=copy.deepcopy(c)
    for slot in old['static_map']['destinations'].values():
        if slot['center_m'][0]==1.68: slot['center_m'][0]=1.55
    assert not geometry_admission(old)['complete_placement_order_found']


@pytest.mark.parametrize('name',['single_box','mixed_eight'])
def test_production_scene_compiles_with_exact_cargo_and_inactive_welds(name,tmp_path):
    pytest.importorskip('mujoco')
    from scripts.multi_object_scene_preview import MultiObjectScene
    c=config(name); scene=MultiObjectScene(c,tmp_path/'scene',render=False)
    try:
        scene.open(); audit=scene.compiled_audit()
        assert len(audit['cargo'])==len(c['mission']['objects'])
        assert not audit['invariants']['weld_active']
        assert not audit['initial_contacts'] and not audit['settled_contacts']
    finally:
        scene.close()
