"""Own-wrist-camera pair beam perception (feasibility study helpers)."""
import base64
import importlib.util
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness import owncam_pair_beam as ob
from harness.owncam_view import project_base_points

ROOT = Path(__file__).resolve().parents[1]
LIME_BGR = (40, 230, 190)          # hue ~45 (OpenCV), inside the lime band


def _beam_image(pose, end_xy, heading, length=.30, width=.03, z=ob.BEAM_TOP_Z_M):
    """Synthetic own view: the beam top (lime) from its near end along ``heading``."""
    u = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-u[1], u[0]])
    e = np.asarray(end_xy, float)
    corners = [e - n * width / 2, e + n * width / 2, e + u * length + n * width / 2, e + u * length - n * width / 2]
    pts = np.array([[c[0], c[1], z] for c in corners])
    # densify edges so the fisheye curvature is followed
    dense = []
    for a, b in zip(pts, np.roll(pts, -1, axis=0)):
        for t in np.linspace(0, 1, 30, endpoint=False):
            dense.append(a + t * (b - a))
    px = project_base_points(pose, np.array(dense))
    img = np.full((480, 640, 3), 60, np.uint8)
    ok = np.all(np.isfinite(px), axis=1)
    cv2.fillPoly(img, [px[ok].astype(np.int32)], LIME_BGR)
    return base64.b64encode(cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()).decode()


@pytest.mark.parametrize('dist, lateral, heading', [(.40, .00, 0.), (.30, .02, .05), (.20, -.01, -.03)])
def test_observe_beam_recovers_grip_point_and_axis(dist, lateral, heading):
    name, pose = ob.look_posture(dist)
    end = (dist - ob.GRIP_INSET_M, lateral)
    obs = ob.observe_beam(_beam_image(pose, end, heading), pose)
    assert obs['visible'] and obs['end_visible'], (name, obs)
    gx, gy = obs['grip_base_m']
    assert abs(gx - dist) < .015 and abs(gy - lateral) < .006
    assert abs(obs['axis_heading_rad'] - heading) < .03


def test_look_posture_switches_nearer_with_distance():
    assert ob.look_posture(None)[0] == 'search'
    assert ob.look_posture(.40)[0] == 'search'
    assert ob.look_posture(.30)[0] == 'p45'
    assert ob.look_posture(.17)[0] == 'inspect'


def test_align_command_signs_and_stop():
    at = {'grip_base_m': [ob.GRASP_RADIUS_M, 0.], 'axis_heading_rad': 0.}
    assert ob.align_command(at) is None
    far = ob.align_command({'grip_base_m': [ob.GRASP_RADIUS_M + .10, .03], 'axis_heading_rad': .10})
    assert far['forward'] > 0 and far['left'] > 0 and far['turn'] > 0
    near = ob.align_command({'grip_base_m': [ob.GRASP_RADIUS_M - .03, -.02], 'axis_heading_rad': -.08})
    assert near['forward'] < 0 and near['left'] < 0 and near['turn'] < 0


def _strip(present=True):
    img = np.full((480, 640, 3), 60, np.uint8)
    if present:
        cv2.rectangle(img, (40, 420), (600, 450), LIME_BGR, -1)
    return base64.b64encode(cv2.imencode('.jpg', img)[1].tobytes()).decode()


def test_hold_ratio_drops_when_the_strip_leaves_the_frame():
    anchor = ob.held_signature(_strip())
    kept = ob.signature_fraction(ob.held_signature(_strip())) / ob.signature_fraction(anchor)
    gone = ob.signature_fraction(ob.held_signature(_strip(False))) / ob.signature_fraction(anchor)
    assert kept > .95 and gone < .05


def test_carry_schedule_uses_the_recorded_odometry_calibration():
    spec = importlib.util.spec_from_file_location('study', ROOT / 'scripts/study_owncam_pair_beam.py')
    study = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(study)
    r1, r2 = study.build_schedule('r1', 0.), study.build_schedule('r2', 0.)
    for (s1, e1, c1), (s2, e2, c2), (kind, dist) in zip(r1, r2, study.LEGS):
        assert (s1, e1) == (s2, e2)                             # same timing: formation by issued-command timing
        achieved = study.SPEED_M_S * study.CARRY_ODOM_SCALE[kind] * (e1 - s1)
        assert abs(achieved - dist) < 1e-9
        assert c1 == {k: -v if v else v for k, v in c2.items()}   # mirror image for the facing partner
