"""Behavior boundaries for the markerless skill, independently of detector fitting."""
import base64
import hashlib
from pathlib import Path
import numpy as np
import cv2
import pytest
from harness import visual_box_skill as mod


def obs(n=1, pan=1500):
    frame = np.zeros((480,640,3),np.uint8)
    ok, data = cv2.imencode('.jpg',frame)
    assert ok
    payload = data.tobytes()
    return {'robot_id':'r1','frame_id':n,'sim_time':float(n),'camera':'robot_cam',
            'image':base64.b64encode(payload).decode(),'sha256':hashlib.sha256(payload).hexdigest(),
            'actuator_state':{'servo_pulses':{'1':2000,'3':740,'4':2320,'5':1320,'6':pan}}}


def test_default_never_constructs_fiducial_tracker(monkeypatch):
    def forbidden(*a,**kw):raise AssertionError('fiducial path forbidden')
    monkeypatch.setattr(mod,'CameraBoxTracker',forbidden)
    monkeypatch.setattr(mod,'observe_ground_box',lambda *a,**kw:{'visible':False,'reason':'AMBIGUOUS'})
    skill=mod.VisualBoxSkill()
    assert skill.perception_mode=='markerless'
    assert skill.decide(obs())['kind']=='drive'
    assert skill.last_target is None


def test_ground_hypothesis_not_queried_while_carrying(monkeypatch):
    def forbidden(*a,**kw):raise AssertionError('held object cannot be projected to ground')
    monkeypatch.setattr(mod,'observe_ground_box',forbidden)
    skill=mod.VisualBoxSkill();skill.phase='carry'
    monkeypatch.setattr(skill,'_carry',lambda *a:{'kind':'wait','duration':.05})
    skill.decide(obs())
    assert skill.last_target is None


def test_one_ground_fit_cannot_finish_release_without_camera_sweep(monkeypatch):
    monkeypatch.setattr(mod,'observe_ground_box',lambda *a,**kw:{'visible':True,
        'estimated_box_center_base_m':[.16,0,.016],'pixel_centroid':[320,219],'provenance':'test'})
    monkeypatch.setattr(mod,'forward_grip',lambda *a:(.16,0,.024))
    skill=mod.VisualBoxSkill();skill.phase='verify_release';skill._grasp={1:2000};skill.held=True
    first=skill.decide(obs())
    assert first=={'kind':'pose','pulses':{6:1560}}
    assert skill.phase=='release_ground_left'
    for n, pan in ((2,1560),(3,1440)):assert skill.decide(obs(n,pan))['kind']=='pose'
    assert skill.decide(obs(4))['reason']=='VISUAL_RELEASE_CONFIRMED'
    assert skill.held is False


def test_apparent_ground_target_moving_with_camera_fails_release(monkeypatch):
    points=iter(([.16,0,.016],[.16,.02,.016]))
    monkeypatch.setattr(mod,'observe_ground_box',lambda *a,**kw:{'visible':True,
        'estimated_box_center_base_m':next(points),'pixel_centroid':[320,219],'provenance':'test'})
    monkeypatch.setattr(mod,'forward_grip',lambda *a:(.16,0,.024))
    skill=mod.VisualBoxSkill();skill.phase='verify_release';skill._grasp={1:2000}
    skill.decide(obs())
    assert skill.decide(obs(2,1560))['reason']=='RELEASE_OBJECT_NOT_GROUND_STATIONARY'


def test_release_requires_observed_commanded_sweep_pose(monkeypatch):
    monkeypatch.setattr(mod,'observe_ground_box',lambda *a,**kw:{'visible':True,
        'estimated_box_center_base_m':[.16,0,.016],'pixel_centroid':[320,219],'provenance':'test'})
    monkeypatch.setattr(mod,'forward_grip',lambda *a:(.16,0,.024))
    skill=mod.VisualBoxSkill();skill.phase='verify_release';skill._grasp={1:2000}
    skill.decide(obs())
    assert skill.decide(obs(2))['reason']=='RELEASE_PROBE_POSE_UNCONFIRMED'


def test_release_opposite_endpoints_must_agree_even_within_origin_tolerance(monkeypatch):
    points=iter(([.16,0,.016],[.16,.008,.016],[.16,-.008,.016]))
    monkeypatch.setattr(mod,'observe_ground_box',lambda *a,**kw:{'visible':True,
        'estimated_box_center_base_m':next(points),'pixel_centroid':[320,219],'provenance':'test'})
    monkeypatch.setattr(mod,'forward_grip',lambda *a:(.16,0,.024))
    skill=mod.VisualBoxSkill();skill.phase='verify_release';skill._grasp={1:2000}
    skill.decide(obs())
    assert skill.decide(obs(2,1560))['kind']=='pose'
    assert skill.decide(obs(3,1440))['reason']=='RELEASE_OBJECT_NOT_GROUND_STATIONARY'


def test_markerless_approach_uses_stable_visual_face_before_center_drive(monkeypatch):
    import math
    monkeypatch.setattr(mod,'observe_ground_box',lambda *a,**kw:{'visible':True,
        'estimated_box_center_base_m':[.4,0,.016],'pixel_centroid':[320,219],
        'estimated_yaw_mod_pi_rad':math.radians(60),
        'floor_hypothesis_projection_iou':.82,'provenance':'test'})
    skill=mod.VisualBoxSkill()
    assert skill.decide(obs())['kind']=='wait'
    assert skill.decide(obs(2))['kind']=='wait'
    action=skill.decide(obs(3))
    assert action['kind']=='drive' and action['fwd']==0 and action['turn']>0
    assert skill._face_approach is False
    assert skill.last_face_alignment['ready'] is True


def test_recorded_reflection_top_supplies_controller_pixel_centroid():
    payload = (Path(__file__).parent/"fixtures/markerless_box/reflection_step7.jpg").read_bytes()
    observation = {
        'robot_id': 'r1', 'frame_id': 7, 'sim_time': 8.154, 'camera': 'robot_cam',
        'image': base64.b64encode(payload).decode(),
        'sha256': hashlib.sha256(payload).hexdigest(),
        'actuator_state': {'servo_pulses': {
            '1': 2000, '3': 650, '4': 2320, '5': 1320, '6': 1500}},
    }
    skill = mod.VisualBoxSkill()

    action = skill.decide(observation)

    assert action['kind'] in {'wait', 'drive', 'pose'}
    assert skill.last_target is not None
    assert np.allclose(skill.last_target, [.2873, .0002, .0206], atol=.004)


def test_far_box_requires_closer_inspection_before_binding_face(monkeypatch):
    import math
    monkeypatch.setattr(mod,'observe_ground_box',lambda *a,**kw:{'visible':True,
        'estimated_box_center_base_m':[.5,0,.016],'pixel_centroid':[320,219],
        'estimated_yaw_mod_pi_rad':math.radians(45),
        'floor_hypothesis_projection_iou':.9,'provenance':'test'})
    skill=mod.VisualBoxSkill()
    action=skill.decide(obs())
    assert action=={'kind':'drive','fwd':.1,'turn':0.0,'duration':.6}
    assert skill.last_face_alignment['reason']=='CLOSER_FACE_INSPECTION_REQUIRED'
    assert skill._face_aligner._previous_normal is None
