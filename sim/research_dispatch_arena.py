"""Authored dispatch laboratory. Scene setup/referee data never enter actors.

The arena fits the approved fixed TOP calibration; robot geometry and own
cameras are copied unchanged. No simulator state is needed to build its map.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import random
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MAP_PATH = ROOT / 'maps/research/dispatch_v1.json'
ROBOTS = ('r1', 'r2', 'r3')
VARIANTS = ('open', 'shared_crossing', 'north_blocked', 'narrow_south', 'rough_south')
FIXED_TOP = {'name': 'cctv_top', 'position_m': [.55, -2., 2.5],
             'quaternion_wxyz': [1., 0., 0., 0.], 'fov_y_deg': 55.}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def authored_map(variant='shared_crossing'):
    if variant not in VARIANTS:
        raise ValueError('unknown arena variant')
    value = json.loads(MAP_PATH.read_text())
    # These are authored course changes, not sensed or measured obstacles.
    if variant == 'open':
        value['obstacles'] = [o for o in value['obstacles'] if o['id'] != 'service_island']
    if variant == 'narrow_south':
        value['obstacles'].append({'id':'south_chicane', 'center_m':[.58,-2.99],
            'half_extents_m':[.20,.13], 'height_m':.14, 'kind':'barrier'})
        value['routes']['south']['declared_min_width_m'] = .46
        value['routes']['south']['loaded_passage'] = 'unvalidated_narrow'
    if variant == 'rough_south':
        value['terrain'].append({'id':'south_rough', 'center_m':[.58,-2.66],
            'half_extents_m':[.25,.19], 'height_m':.008, 'kind':'low_ridge',
            'loaded_passage':'unvalidated'})
        value['routes']['south']['loaded_passage'] = 'unvalidated_terrain'
    # north_blocked deliberately has the same PRIOR map as shared_crossing.
    # The unexpected obstruction is physical evidence, never a planner label.
    value['map_id'] = 'dispatch_' + ('shared_crossing' if variant == 'north_blocked' else variant)
    return value


def episode(variant='shared_crossing', seed=11):
    static = authored_map(variant)
    slots = [(-.77,-1.25),(-.77,-2.),(-.77,-2.75)]
    order = list(ROBOTS)
    random.Random(seed).shuffle(order)
    return {'schema':'ugrp.dispatch_episode.v1', 'seed':seed, 'variant':variant,
        'static_map':static, 'static_map_sha256':digest(static),
        'setup_only':{'spawns':{rid:[*slot,.032355118817659255,0.]
                              for rid,slot in zip(order,slots)},
            'cargo':{'beam':[-.18,-1.65,.020], 'box':[-.18,-2.65,.016]},
            'unexpected_obstacles':([{'id':'unannounced_north_barrier',
                'center_m':[.60,-1.18], 'half_extents_m':[.14,.33],
                'height_m':.16, 'kind':'barrier'}] if variant=='north_blocked' else [])}}


def actor_task(static):
    """Detached allowlist; no seed, spawns, event schedule, truth or goal label."""
    return {'mission_id':'dispatch_one_kit',
        'instruction':('Deliver the orange beam AND the cyan box to their marked slots '
          'in ONE common dispatch dock, A or B. Choose that dock, the pair, the solo '
          'carrier, routes and dependencies together. Independent preparation may '
          'overlap. Shared passages and the unloading apron require coordination.'),
        'objects':{'beam':{'appearance':'orange plain beam', 'carriers':2},
                   'box':{'appearance':'small cyan box', 'carriers':1}},
        'robots':list(ROBOTS), 'static_map':copy.deepcopy(static),
        'static_map_sha256':digest(static),
        'input_boundary':'own RGB + shared fixed TOP RGB + authored map + own issued commands + peer claims',
        'capability_scope':'Planning contract for interchangeable robots; loaded execution in this arena is not yet validated.'}


def _geom(world, name, center, half, height, rgba, *, collision=True, z=None):
    return ET.SubElement(world, 'geom', name='dispatch_'+name, type='box',
        pos=f'{center[0]} {center[1]} {height/2 if z is None else z}',
        size=f'{half[0]} {half[1]} {height/2}', rgba=rgba,
        contype='1' if collision else '0', conaffinity='3' if collision else '0',
        group='0', mass='0')


def build_scene_xml(source, config):
    """Replace legacy room props, keep inert API placeholders and exact robots."""
    from sim.warehouse_mission import CargoSpec, _cargo_body
    root = ET.fromstring(source)
    world = root.find('worldbody')
    robots = {n.get('name'): ET.tostring(n) for n in world.findall('body')
              if n.get('name') in {r+'__robot' for r in ROBOTS}}
    if len(robots) != 3:
        raise ValueError('expected three production robots')
    beam = world.find("body[@name='team_beam']")
    if beam is None:
        raise ValueError('missing plain beam')
    beam_bytes = ET.tostring(beam)
    for node in list(world):
        if node.tag == 'geom' and node.get('name') != 'floor':
            world.remove(node)
        elif node.tag == 'body' and node.get('name') not in {*robots, 'team_beam'}:
            for b in node.iter('body'):
                b.set('gravcomp','1')
            for g in node.iter('geom'):
                g.attrib.update(contype='0',conaffinity='0',rgba='0 0 0 0',group='5')
    static = config['static_map']
    for item in static['obstacles'] + config['setup_only']['unexpected_obstacles']:
        _geom(world,item['id'],item['center_m'],item['half_extents_m'],item['height_m'],
              '.90 .40 .10 1' if item['kind']=='barrier' else '.23 .28 .33 1')
    for item in static['terrain']:
        _geom(world,item['id'],item['center_m'],item['half_extents_m'],item['height_m'],'.55 .43 .25 1')
    # Floor paint is part of the authored map, not a fake obstacle or target.
    for rid, region in static['regions'].items():
        _geom(world,rid,region['center_m'],region['half_extents_m'],.001,
              region['rgba'],collision=False,z=.0006)
    for dock, value in static['docks'].items():
        for obj, slot in value['slots'].items():
            _geom(world,dock+'_'+obj,slot['center_m'],slot['half_extents_m'],.001,
                  '.12 .78 .36 .55' if obj=='beam' else '.8 .12 .7 .55',
                  collision=False,z=.0013)
    # Exact existing production cyan box appearance, dimensions and mass.
    spec = CargoSpec('small_box_01','small_box','small_box','dispatch_box','dispatch_box_free',
                     (.034,.040,.032),.03,tuple(config['setup_only']['cargo']['box']),
                     (0,0,.016),.020,required_carriers=1)
    world.append(_cargo_body(spec))
    top = world.find("camera[@name='cctv_top']")
    top.attrib.pop('xyaxes',None)
    top.set('pos',' '.join(map(str,FIXED_TOP['position_m'])))
    top.set('quat','1 0 0 0'); top.set('fovy','55')
    for eq in root.findall('equality/weld'):
        eq.set('active','false')
    after = {n.get('name'): ET.tostring(n) for n in world.findall('body') if n.get('name') in robots}
    if after != robots or ET.tostring(beam) != beam_bytes:
        raise ValueError('arena changed robot or plain beam geometry')
    xml = ET.tostring(root,encoding='unicode')
    return xml, {'scene_xml_sha256':hashlib.sha256(xml.encode()).hexdigest(),
                 'robot_xml_sha256':{k:hashlib.sha256(v).hexdigest() for k,v in robots.items()},
                 'beam_xml_sha256':hashlib.sha256(beam_bytes).hexdigest(),
                 'static_map_sha256':digest(static)}
