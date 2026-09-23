"""Final-slot RGB control regression from the 300 SIM s solo replay.

Only the saved TOP frames, the authored map, and recorded actor RGB evidence
are supplied to the route. Referee position/contact/success are not inputs.
"""

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from harness.dispatch_skill_binding import ImageRoute, pixel_from_map
from sim.research_dispatch_arena import authored_map


FIXTURE = Path(__file__).parent / 'fixtures' / 'solo_route_goal_margin'


def _actor_route(*, dock='dock_a', center_shift_x=0.):
    provenance = json.loads((FIXTURE / 'provenance.json').read_text())
    final_jpeg = (FIXTURE / 'r2-001630-top_rgb.jpg').read_bytes()
    background_jpeg = (FIXTURE / 'r2-000267-top_rgb.jpg').read_bytes()
    assert hashlib.sha256(final_jpeg).hexdigest() == provenance['image_sha256']
    assert hashlib.sha256(background_jpeg).hexdigest() == provenance['background_sha256']
    final_frame = cv2.imdecode(np.frombuffer(final_jpeg, np.uint8), cv2.IMREAD_COLOR)
    background_frame = cv2.imdecode(np.frombuffer(background_jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert final_frame.shape == background_frame.shape == (720, 960, 3)

    evidence = provenance['recorded_actor_evidence']
    static_map = authored_map('open')
    route = ImageRoute(SimpleNamespace(static_map=static_map,
        tasks={'box': {'route': 'south'}}, plan={'dock': dock}), 'box')
    route.points = [np.array(point) for point in evidence['waypoints_px']]
    if dock != 'dock_a':
        destination = static_map['docks'][dock]['slots']['box']['center_m']
        route.points[-1] = pixel_from_map([destination[0] + .04, destination[1]],
                                           static_map, final_frame.shape)
    route.index = len(route.points) - 1
    route.box_center = np.array(evidence['cargo_center_px']) + [center_shift_x, 0.]
    route.box_delta = np.zeros(2)
    route.box_previous = final_frame.copy()
    route.box_background = background_frame
    route.box_background_sha = provenance['background_sha256']
    route.box_origin = np.array(evidence['initial_cargo_center_px'])
    return route, final_jpeg, provenance


def test_saved_rgb_final_slot_miss_commands_inward_correction():
    route, final_jpeg, provenance = _actor_route()
    action, evidence = route.observe(final_jpeg)
    destination = evidence['destination_region']
    recorded = provenance['recorded_actor_evidence']

    assert evidence['tracking']['method'].startswith('bidirectional RGB feature motion')
    assert evidence['waypoint_index'] == 5
    assert evidence['cargo_center_px'] == pytest.approx(recorded['cargo_center_px'], abs=.001)
    assert destination['padded_cargo_bounds_px'][1][0] == pytest.approx(
        recorded['padded_cargo_upper_x_px'], abs=.001)
    assert destination['slot_interior_px'][1][0] == pytest.approx(
        recorded['slot_interior_upper_x_px'], abs=.001)
    assert destination['padded_cargo_bounds_px'][1][0] - destination['slot_interior_px'][1][0] == pytest.approx(.071218, abs=.001)
    assert not evidence['ready'] and not evidence['done']
    # The guided centre must leave room inside the slot instead of targeting
    # its exact edge, where a 0.5px wheel deadband otherwise commands a stop.
    padded_half_width = (destination['padded_cargo_bounds_px'][1][0]
                         - destination['padded_cargo_bounds_px'][0][0]) / 2
    assert destination['guided_center_px'][0] <= (
        destination['slot_interior_px'][1][0] - padded_half_width - .5)
    assert action['kind'] == 'mecanum' and action['forward'] < 0
    assert action['turn'] == 0


def test_final_slot_already_inside_requires_two_rgb_confirmations():
    route, final_jpeg, _ = _actor_route(center_shift_x=-1.)
    first, first_evidence = route.observe(final_jpeg)
    second, second_evidence = route.observe(final_jpeg)
    assert first_evidence['ready'] and not first_evidence['done']
    assert second_evidence['ready'] and second_evidence['done']
    assert first['forward'] == first['left'] == first['turn'] == 0
    assert second['forward'] == second['left'] == second['turn'] == 0


def test_same_rgb_other_dock_preserves_southward_direction():
    route, final_jpeg, _ = _actor_route(dock='dock_b')
    action, evidence = route.observe(final_jpeg)
    assert not evidence['ready']
    assert evidence['error_px'][1] > 300
    assert action['kind'] == 'mecanum' and action['left'] < 0
    assert action['turn'] == 0
