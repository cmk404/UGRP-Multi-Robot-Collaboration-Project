"""Saved camera replay for a cargo silhouette hidden by its carrier."""
import base64
from pathlib import Path
from unittest import mock

import cv2
import numpy as np
import pytest

from harness.dispatch_skill_binding import ImageRoute, SkillBindings
from harness.visual_attachment import compare_box_comotion
from harness.solo_box_transport import SoloBoxTransport
from test_dispatch_skill_binding import authored_map, committed


FRAMES = Path(__file__).parent / 'fixtures' / 'dispatch_carrier_occlusion'
REGRESSION = Path(__file__).parent / 'fixtures' / 'dispatch_carrier_regression'


def _bytes(name):
    return (FRAMES / name).read_bytes()


def _frame(name):
    return cv2.imdecode(np.frombuffer(_bytes(name), np.uint8), cv2.IMREAD_COLOR)


def _route_through_284():
    route = ImageRoute(SkillBindings(committed(), authored_map('open')), 'box')
    # State recorded in the v26 repeat's decision 283, with all image memory
    # restored from the archived TOP RGB inputs rather than simulator state.
    route.box_center = np.array([815.7171436656605, 344.70885675603694])
    route.box_delta = np.array([13.069488525390625, -0.081939697265625])
    route.box_origin = np.array([274.59183673469386, 544.6734693877551])
    route.box_background = _frame('solo-142-top.jpg')
    route.box_background_sha = '82908ce8590cb69ef5f65761f52823cfba6f14a8fc1a4f7f181af0891b8f39a7'
    route.box_previous = _frame('solo-283-top.jpg')
    route.points = [np.array([880.6010681115795, 359.5]), np.array([880.6010681115795, 176.93])]
    _, accepted = route.observe(_bytes('solo-284-top.jpg'))
    assert np.allclose(accepted['cargo_center_px'], [828.7489429820668, 344.6551153009588], atol=.01)
    assert accepted['tracking']['method'].startswith('bidirectional RGB feature motion')
    return route


def _current_own_attachment():
    before, after = [base64.b64encode(_bytes(f'solo-{i}-own.jpg')).decode() for i in (284, 285)]
    evidence = compare_box_comotion(before, after, min_saturation=150)
    assert evidence['attached'] and evidence['mask_iou'] > .99
    return evidence, after, after


def test_saved_occluded_cargo_uses_prelinked_visual_carrier():
    route = _route_through_284()
    assert route.box_carrier_points is not None and len(route.box_carrier_points) >= 8
    _, result = route.observe(_bytes('solo-285-top.jpg'), own_attachment=_current_own_attachment())
    tracking = result['tracking']
    assert tracking['method'] == 'linked TOP RGB carrier motion during cargo occlusion'
    assert tracking['consistent_features'] >= 8
    assert tracking['consistent_features'] >= .6 * tracking['linked_feature_count']
    assert np.allclose(result['cargo_center_px'], [841.78, 344.60], atol=1.)
    assert not result['done']


@pytest.mark.parametrize('attachment', ['missing', 'detached', 'mismatched', 'stale'])
def test_occluded_cargo_requires_fresh_current_own_attachment(attachment):
    route = _route_through_284()
    valid = _current_own_attachment()
    if attachment == 'missing':
        check = None
    elif attachment == 'detached':
        check = ({**valid[0], 'attached': False}, valid[1], valid[2])
    elif attachment == 'mismatched':
        check = (valid[0], valid[1], base64.b64encode(_bytes('solo-284-own.jpg')).decode())
    else:
        check = ({**valid[0], 'camera_pan_delta_pwm': 60.}, valid[1], valid[2])
    with pytest.raises(RuntimeError, match='own RGB attachment'):
        route.observe(_bytes('solo-285-top.jpg'), own_attachment=check)


def test_occluded_cargo_rejects_mixed_robot_and_stationary_background_motion():
    route = _route_through_284()
    mixed = _frame('solo-285-top.jpg')
    previous = _frame('solo-284-top.jpg')
    # Keep one side of the carrier at its old position while the other side
    # moves. A patch-selection shortcut could wrongly pick either cluster.
    mixed[320:395, 710:775] = previous[320:395, 710:775]
    with pytest.raises(RuntimeError, match='linked carrier motion ambiguous'):
        route.observe(cv2.imencode('.png', mixed)[1].tobytes(),
                      own_attachment=_current_own_attachment())


def test_occluded_cargo_rejects_missing_carrier_appearance():
    route = _route_through_284()
    no_carrier = _frame('solo-285-top.jpg')
    background = _frame('solo-142-top.jpg')
    no_carrier[320:395, 710:840] = background[320:395, 710:840]
    with pytest.raises(RuntimeError, match='linked carrier'):
        route.observe(cv2.imencode('.png', no_carrier)[1].tobytes(),
                      own_attachment=_current_own_attachment())


def test_occluded_cargo_does_not_reaccept_stale_top_as_direct_box_tracking():
    route = _route_through_284()
    route.observe(_bytes('solo-285-top.jpg'), own_attachment=_current_own_attachment())
    with pytest.raises(RuntimeError, match='stale TOP RGB'):
        route.observe(_bytes('solo-285-top.jpg'), own_attachment=_current_own_attachment())


def test_occluded_cargo_never_confirms_delivery_from_proxy():
    route = _route_through_284()
    route.points = [np.array([842., 345.])]
    with pytest.raises(RuntimeError, match='fresh cargo silhouette'):
        route.observe(_bytes('solo-285-top.jpg'), own_attachment=_current_own_attachment())


def test_solo_adapter_passes_same_observation_attachment_to_image_route():
    route = _route_through_284()
    transport = SoloBoxTransport(navigator=route)
    transport.initialized = True
    transport.box.phase = 'carry'
    attachment = _current_own_attachment()
    transport.box.last_attachment = attachment[0]
    transport.box._carry_previous_image = attachment[1]
    transport.box.decide = lambda own: {'kind': 'wait', 'duration': .05}
    with mock.patch.object(route, 'observe', return_value=(
            {'kind': 'mecanum', 'forward': .02, 'left': 0., 'turn': 0., 'duration_s': .2},
            {'done': False})) as observed:
        transport.decide({'image': attachment[1]}, _bytes('solo-285-top.jpg'))
    assert observed.call_args.kwargs['own_attachment'] == attachment


@pytest.mark.parametrize('run,previous,motion,origin,expected', [
    ('v19', [630.589111328125, 454.1775207519531],
     [-.78021240234375, -4.6805267333984375], [274.26666666666665, 545.32],
     [629.7890625, 449.44606018066406]),
    ('v25', [669.4752197265625, 353.3267517089844],
     [7.93206787109375, -1.339996337890625], [274.1159420289855, 545.2608695652174],
     [677.8642578125, 351.8575134277344]),
    ('v26', [835.6815795898438, 344.4997100830078],
     [10.2974853515625, .06036376953125], [274.45454545454544, 545.0181818181818],
     [845.8092651367188, 344.66412353515625]),
])
def test_saved_successful_tracking_transitions_keep_direct_cargo_identity(
        run, previous, motion, origin, expected):
    def rgb(kind):
        return (REGRESSION / f'{run}-{kind}.jpg').read_bytes()

    route = ImageRoute(SkillBindings(committed(), authored_map('open')), 'box')
    route.box_center = np.asarray(previous)
    route.box_delta = np.asarray(motion)
    route.box_origin = np.asarray(origin)
    route.box_background = cv2.imdecode(np.frombuffer(rgb('background'), np.uint8), cv2.IMREAD_COLOR)
    route.box_previous = cv2.imdecode(np.frombuffer(rgb('previous'), np.uint8), cv2.IMREAD_COLOR)
    route.points = [np.array([880., 359.]), np.array([850., 177.])]
    _, result = route.observe(rgb('current'))
    assert result['tracking']['method'] == 'bidirectional RGB feature motion; own attachment independently required'
    assert np.allclose(result['cargo_center_px'], expected, atol=.01)
