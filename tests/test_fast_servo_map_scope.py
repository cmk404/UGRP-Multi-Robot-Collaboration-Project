"""The authored open map includes boundary walls and must enable the option."""
import copy
from types import SimpleNamespace

from scripts.run_dispatch_skills import fast_servo_map_supported
from sim.research_dispatch_arena import authored_map


def test_scene_wires_both_opt_ins_from_the_actual_map(monkeypatch):
    from scripts import run_dispatch_skills as module
    monkeypatch.setattr(module,'ImageRoute',lambda *args,**kwargs: kwargs)
    monkeypatch.setattr(module,'SoloBoxTransport',lambda **kwargs: SimpleNamespace(**kwargs,phase='approach'))
    monkeypatch.setattr(module,'VisualMacroExecutor',lambda *args,**kwargs: None)
    scene=SimpleNamespace(bindings=SimpleNamespace(static_map=authored_map('open'),solo='r2'),
                          ports={'r2':object()},solo_raw=[],realtime_control=True)
    module.SkillScene.start_solo(scene)
    assert scene.solo.fast_near_field_servo is True
    assert scene.solo.navigator['time_aware_box_reacquisition'] is True
    scene.realtime_control=False
    module.SkillScene.start_solo(scene)
    assert scene.solo.fast_near_field_servo is False
    assert scene.solo.navigator['time_aware_box_reacquisition'] is False


def test_actual_open_map_enables_fast_servo_with_boundary_walls():
    static = authored_map('open')
    assert static['obstacles']
    assert fast_servo_map_supported(static, realtime_control=True)
    assert not fast_servo_map_supported(static, realtime_control=False)


def test_unsupported_map_obstacle_or_terrain_keeps_original_servo():
    assert not fast_servo_map_supported(authored_map('shared_crossing'), realtime_control=True)
    static = copy.deepcopy(authored_map('open'))
    static['obstacles'].append({'id': 'new_inside_obstacle'})
    assert not fast_servo_map_supported(static, realtime_control=True)
    static = copy.deepcopy(authored_map('open'))
    static['terrain'] = [{'id': 'new_slope'}]
    assert not fast_servo_map_supported(static, realtime_control=True)
