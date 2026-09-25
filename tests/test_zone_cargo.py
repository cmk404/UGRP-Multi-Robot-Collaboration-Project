"""Zone cargo catalogue: static parameters, scene integration and byte identity."""
import hashlib
import math

import pytest

from sim import zone_cargo as zc

# Unmodified ZoneScene XML (seed 11, DEFAULT_GOAL), computed from main 3cbf4e4
# before the cargo profile existed. zone_open v2 / zone_wide v1 must not change.
PINNED_XML_SHA256 = {
    ('zone_open', None): 'cd7065ab6a0144248fb6ed47834a93ffa1730c043af52320dc158512befcbc52',
    ('zone_open', 'local_contact_fine'): '58bc404e8f39985e53101745d8ecbf8b5a2651dd394d8be426f29e3ad271bb10',
    ('zone_wide', None): 'ba0e7c8dd8dc06e0fdf0318b53ebad95ec4053d1898e84401f063af44de4c3a6',
    ('zone_wide', 'local_contact_fine'): 'b7dc69cac432193aaeaec8151b5051080654af4d1413c2c8aea78a76bd02da47',
}
# Gripper jaws (sim/masterpi_dynamics_v2.py): pads at +-30.5 mm when open,
# each jaw closes 16 mm -> graspable widths between 29 mm and 61 mm.
JAW_MIN_M, JAW_MAX_M = .029, .061


def _scene_xml(scene):
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    world = MultiMasterPiProductionV2(seed=11, width=64, height=48, render=False,
                                      warehouse_layout=scene.engine_layout, warehouse_cargo_ids=None,
                                      xml_transform=scene.transform)
    return world


def test_catalogue_tiers_carriers_and_formations_are_consistent():
    tiers = {'solo': 1, 'pair': 2, 'trio': 3}
    assert {k: v.tier for k, v in zc.CATALOGUE.items()} == {
        'can': 'solo', 'tile': 'solo', 'long_beam': 'pair', 'heavy_crate': 'pair', 'tri_frame': 'trio'}
    for spec in zc.CATALOGUE.values():
        assert spec.required_carriers == tiers[spec.tier]
        roles = {g.role for g in spec.grasps}
        assert all(len(f) == spec.required_carriers and set(f) <= roles for f in spec.formations)
        assert {p.name for p in spec.parts} >= {g.geom for g in spec.grasps}
    record = zc.catalogue_record()
    assert record['schema'] == zc.SCHEMA and len(record['sha256']) == 64
    assert 'cal_block' not in record['kinds'] and zc.kind('cal_block').tier == 'calibration'


def test_masses_follow_the_measured_single_robot_capacity():
    lo, hi = zc.MEASURED_SINGLE_ROBOT_CAPACITY_KG
    assert lo < hi
    for spec in zc.CATALOGUE.values():
        share = spec.mass_kg / spec.required_carriers
        assert share < lo, spec.kind                      # each carrier stays inside capacity
        if spec.required_carriers >= 2 and spec.kind != 'long_beam':
            # one robot fewer would need more than it can lift (tipping, measured)
            assert spec.mass_kg / (spec.required_carriers - 1) > hi, spec.kind
    assert zc.CATALOGUE['heavy_crate'].mass_kg > hi


def test_grasps_fit_the_jaws_and_the_calibrated_ik_reaches_them():
    from harness.visual_arm import solve_grip_ik
    for spec in list(zc.CATALOGUE.values()) + [zc.kind('cal_block')]:
        parts = {p.name: p for p in spec.parts}
        for g in spec.grasps:
            part = parts[g.geom]
            width = 2*part.size[0] if part.shape == 'cylinder' else 2*part.size[1]
            assert JAW_MIN_M < width < JAW_MAX_M, (spec.kind, g.role)
            solve_grip_ik(zc.GRASP_RADIUS_M, 0., g.grip_xyz[2], -90)   # raises when unreachable
            solve_grip_ik(zc.GRASP_RADIUS_M, 0., .095, -90)
            bx, by, byaw = g.approach_base()
            assert math.isclose(math.hypot(g.grip_xyz[0]-bx, g.grip_xyz[1]-by), zc.GRASP_RADIUS_M, abs_tol=1e-5)
    crate = zc.CATALOGUE['heavy_crate']
    body = next(p for p in crate.parts if p.name == 'box')
    assert 2*body.size[1] > JAW_MAX_M   # only the lugs can be gripped


def test_beam_is_only_grippable_near_its_ends():
    beam = zc.CATALOGUE['long_beam']
    half = next(p for p in beam.parts if p.name == 'bar').size[0]
    assert 2*half >= .5
    for g in beam.grasps:
        # jaws close across the reach and the arm has no wrist roll: the robot
        # stands on the long axis, its front (6 cm) beyond the end
        assert half - abs(g.grip_xyz[0]) <= zc.GRASP_RADIUS_M - .06
        assert abs(g.grip_xyz[1]) < 1e-9 and abs(math.sin(g.approach_yaw)) < 1e-9


def test_approach_poses_keep_robots_apart_and_outside_the_cargo():
    for spec in zc.CATALOGUE.values():
        for formation in spec.formations:
            bases = [g.approach_base() for g in spec.grasps if g.role in formation]
            for a, b in zip(bases, bases[1:]):
                assert math.hypot(a[0]-b[0], a[1]-b[1]) > .30
            for g in spec.grasps:
                bx, by, _ = g.approach_base()
                reach = math.hypot(bx, by)
                assert reach > math.hypot(g.grip_xyz[0], g.grip_xyz[1])   # the base is outside the grip point


def test_mass_properties_match_mujoco_compilation():
    mujoco = pytest.importorskip('mujoco')
    import numpy as np
    import xml.etree.ElementTree as ET
    for name, spec in zc.CATALOGUE.items():
        inst = zc.CargoInstance('t', name, (0., 0., 0.))
        root = ET.fromstring('<mujoco><worldbody/></mujoco>')
        root.find('worldbody').append(zc.body_element(inst))
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        props = zc.mass_properties(spec)
        assert math.isclose(model.body('cargo_t').mass[0], spec.mass_kg, rel_tol=1e-6)
        want = np.sort(np.linalg.eigvalsh(np.array(props['inertia_kgm2'])))
        assert np.allclose(np.sort(model.body('cargo_t').inertia), want, rtol=1e-3, atol=1e-8), name


def test_existing_zone_variants_stay_byte_identical():
    pytest.importorskip('mujoco')
    from sim.zone_arena import DEFAULT_GOAL, episode
    from sim.zone_cargo_scene import CargoZoneScene
    from sim.zone_scene import ZoneScene
    for (variant, profile), sha in PINNED_XML_SHA256.items():
        config = episode(variant, 11, goal=DEFAULT_GOAL)
        config['contact_solver_profile'] = profile
        plain = _scene_xml(ZoneScene.from_zone_config(config)).scene_xml
        empty = _scene_xml(CargoZoneScene.from_cargo_config(variant, 11, cargo=[], contact_profile=profile)).scene_xml
        assert hashlib.sha256(plain.encode()).hexdigest() == sha, (variant, profile)
        assert empty == plain


def test_cargo_profile_adds_bodies_with_mirrored_finger_pairs_and_no_weld():
    pytest.importorskip('mujoco')
    from sim.zone_cargo_scene import CargoZoneScene
    items = [{'item_id': k, 'kind': k, 'pose': [2.5 + i, .5, 0.]} for i, k in enumerate(zc.CATALOGUE)]
    scene = CargoZoneScene.from_cargo_config('zone_wide', 11, cargo=items)
    world = _scene_xml(scene)
    scene.setup(world)
    m, d = world.model, world.data
    geoms = [f'cargo_{i["item_id"]}__{p.name}' for i in items for p in zc.kind(i['kind']).parts if p.collision]
    assert scene.manifest['cargo']['finger_contact_pairs'] == 6*len(geoms)
    pair_geoms = {m.geom(int(m.pair_geom2[k])).name for k in range(m.npair)}
    assert set(geoms) <= pair_geoms
    assert not d.eq_active.any() and scene.manifest['weld'] == 'off'
    for item in items:
        assert math.isclose(m.body('cargo_' + item['item_id']).mass[0], zc.kind(item['kind']).mass_kg, rel_tol=1e-6)
        assert abs(d.body('cargo_' + item['item_id']).xpos[0] - item['pose'][0]) < 1e-6
    assert scene.config['cargo_set']['items'][0]['kind'] == 'can'
    with pytest.raises(ValueError):
        zc.instances([{'item_id': 'x', 'kind': 'piano', 'pose': [0, 0, 0]}])


def test_formation_reference_and_world_grasps():
    from scripts.cargo_formation_teacher import Reference, compose
    ref = Reference((0., 0., 0.), [('move', .8), ('turn', math.pi/2), ('move', .5)])
    assert ref.poses[-1] == pytest.approx((.8, .5, math.pi/2))
    ref.advance(1e6)
    assert ref.finished and ref.pose == pytest.approx((.8, .5, math.pi/2))
    inst = zc.instances([{'item_id': 'c', 'kind': 'heavy_crate', 'pose': [1., 2., math.pi/2]}])[0]
    grasps = zc.world_grasps(inst)
    assert grasps['west']['grip_xyz'][:2] == pytest.approx((1., 1.9))
    assert grasps['west']['base_xyyaw'] == pytest.approx(compose((1., 2., math.pi/2), inst.spec().grasps[0].approach_base()))


def test_cargo_noslip_profile_is_opt_in_versioned_and_changes_only_the_solver_option():
    pytest.importorskip('mujoco')
    import xml.etree.ElementTree as ET
    from sim.zone_cargo_contact import CARGO_PROFILES, base_profile, profile_record
    from sim.zone_cargo_scene import CargoZoneScene
    record = profile_record('cargo_noslip_v1')
    assert record == profile_record('cargo_noslip_v1') and len(record['sha256']) == 64
    assert base_profile('cargo_noslip_v1') == 'local_contact_fine' and base_profile('local_contact_fine') == 'local_contact_fine'
    items = [{'item_id': 'c', 'kind': 'heavy_crate', 'pose': [2.5, .2, 0.]}]
    plain = CargoZoneScene.from_cargo_config('zone_wide', 11, cargo=items)
    noslip = CargoZoneScene.from_cargo_config('zone_wide', 11, cargo=items, contact_profile='cargo_noslip_v1')
    a, b = _scene_xml(plain), _scene_xml(noslip)
    assert a.model.opt.noslip_iterations == 0 and b.model.opt.noslip_iterations == 10
    assert noslip.manifest['cargo_contact_profile']['sha256'] == record['sha256']
    ra, rb = ET.fromstring(a.scene_xml), ET.fromstring(b.scene_xml)
    rb.find('option').attrib.pop('noslip_iterations')
    assert ET.tostring(ra) == ET.tostring(rb)          # nothing else differs: pairs, bodies, weld OFF
    assert not b.data.eq_active.any()
    with pytest.raises(ValueError):
        CargoZoneScene.from_cargo_config('zone_wide', 11, cargo=items, contact_profile='glue')
    assert set(CARGO_PROFILES) == {'cargo_noslip_v1'}


def test_dispatch_contact_profiles_are_unchanged():
    from sim.dispatch_contact_profile import PROFILES
    assert PROFILES == ('legacy', 'global_noslip', 'local_contact', 'local_contact_fine')
