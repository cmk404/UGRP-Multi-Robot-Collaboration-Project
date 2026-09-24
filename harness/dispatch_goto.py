"""Dispatch glue for model-chosen destinations reached by map A*.

Only the solo box robot is bound in this version: its loaded path to the
agreed dock and its unloaded move to the model-chosen park place. The beam
pair keeps its agreed corridor. Inputs are the authored map, the committed
plan, protocol state (finished jobs) and current/past TOP RGB estimates.
"""
from __future__ import annotations

import copy
import hashlib
import math

import numpy as np

from harness.camera_goal_transport import decode
from harness import map_goto
from harness.map_goto import (CARRY_PLANE_M, SOLO_CARRY_ENVELOPE, UNLOADED_ENVELOPE,
                              beam_delivered_keepout, beam_team_keepout, box_slot_goal,
                              direction_preserving, map_to_pixel, pixel_to_map, plan_path)

PREDOCK_OFFSET_M = .14   # the fixed-route skill's east staging line (x=2.00)
REPLAN_CROSS_TRACK_M = .08
MAX_REPLANS = 4


def beam_center_from_top(jpeg, static_map):
    from harness.dispatch_skill_binding import beam_feature
    feature = beam_feature(jpeg, hue_upper=35)
    frame = decode(jpeg)
    center = np.array(feature['center']) * feature['image_size']
    return pixel_to_map(center, static_map, frame.shape), hashlib.sha256(jpeg).hexdigest()


def beam_keepouts(static_map, dock, *, beam_finished, top_jpeg=None, fallback_center=None):
    """Where the beam team is or will be while the box robot moves.

    Serial planned dispatch never lets the box move while the pair carries, so
    the beam is either delivered (authored dock slot) or still at pickup (RGB).
    """
    if beam_finished:
        return [beam_delivered_keepout(static_map, dock)]
    try:
        center, sha = beam_center_from_top(top_jpeg, static_map)
        source = 'current TOP RGB beam at pickup with its carriers; image ' + sha[:12]
    except Exception as error:  # an occluded shaft may fall back to the planning view
        if fallback_center is None:
            raise RuntimeError('BEAM_KEEPOUT_UNRESOLVED: ' + str(error))
        center = fallback_center
        source = 'planning-time TOP RGB beam at pickup (beam has not moved: it holds no transit lock)'
    return [beam_team_keepout(center, source)]


def rgb_obstacles(static_map, jpeg):
    from harness.dispatch_navigation_map import visual_barriers
    return [map_goto.rect(o['id'], o['center_m'], o['half_extents_m'], o['source'])
            for o in visual_barriers(jpeg, static_map)]


def predock_goal(static_map, dock):
    """A* goal: the staging point on the slot's far side from the beam slot.

    The last straight move from here into the slot is the existing authored
    docking maneuver; RGB slot containment still decides the release.
    """
    slots = static_map['docks'][dock]['slots']
    direction = math.copysign(1., slots['box']['center_m'][0] - slots['beam']['center_m'][0])
    goal = box_slot_goal(static_map, dock)
    return [goal[0] + direction * PREDOCK_OFFSET_M, goal[1]]


def box_delivery_route(static_map, dock, start_xy, *, beam_finished, top_jpeg=None,
                       fallback_beam_center=None, reserved_resources=None):
    keepouts = beam_keepouts(static_map, dock, beam_finished=beam_finished, top_jpeg=top_jpeg,
                             fallback_center=fallback_beam_center)
    obstacles = keepouts + (rgb_obstacles(static_map, top_jpeg) if top_jpeg is not None else [])
    blocked = []
    if reserved_resources is not None:
        blocked = [r for r in map_goto.resource_regions(static_map)
                   if r not in reserved_resources and r != 'dispatch_apron']
    # A delivered beam team leaves only the far-side entry: stage there and use
    # the authored straight docking move. Otherwise A* plans to the slot itself.
    goal = predock_goal(static_map, dock) if beam_finished else box_slot_goal(static_map, dock)
    route = plan_path(static_map, start_xy, goal, SOLO_CARRY_ENVELOPE,
                      obstacles=obstacles, blocked_regions=blocked, escape_start_m=.05)
    if route is None:
        return None
    route['final_docking_m'] = box_slot_goal(static_map, dock)
    route['predock_used'] = bool(beam_finished)
    route['final_docking_scope'] = ('existing authored straight docking move + RGB slot containment'
                                    if beam_finished else 'A* segment to the slot + RGB slot containment')
    route['cargo_feature_plane_m'] = CARRY_PLANE_M
    return route


def planning_resources(static_map, plan, top_jpeg):
    """Reserve what a box A* path from the current RGB cargo position needs.

    Called before motors move. Beam order follows the agreed dependencies.
    """
    from harness.dispatch_navigation_map import visual_boxes
    boxes = visual_boxes(top_jpeg, static_map)
    if len(boxes) != 1:
        raise RuntimeError('BOX_START_UNRESOLVED: expected one cyan cargo in TOP RGB, saw ' + str(len(boxes)))
    start = boxes[0]['center_m']
    tasks = {t['object']: t for t in plan['tasks']}
    beam_first = tasks['beam']['id'] in tasks['box']['after']
    route = box_delivery_route(static_map, plan['dock'], start, beam_finished=beam_first,
                               top_jpeg=top_jpeg)
    return start, route


def park_keepouts(static_map, plan, *, beam_center, beam_route=None):
    """The beam team's future swept space when the box robot waits for it."""
    tasks = {t['object']: t for t in plan['tasks']}
    rows = [beam_delivered_keepout(static_map, plan['dock'])]
    if beam_route:
        rows += map_goto.pose_route_corridor(beam_route)
    else:
        rows += map_goto.open_beam_corridor(static_map, plan['dock'], tasks['beam']['route'], beam_center)
    return rows


def check_plan_park(static_map, plan, *, beam_center, beam_route=None):
    tasks = {t['object']: t for t in plan['tasks']}
    if tasks['beam']['id'] in tasks['box']['after']:
        # The beam is delivered before the box, so the box job finishes on
        # release and never parks. Only the destination syntax is checked.
        report = map_goto.resolve_destination(static_map, tasks['box']['park'])
        return {**report, 'feasible': True, 'needed': False,
                'reason': 'box follows beam; box_job finishes at release without parking'}
    report = map_goto.check_park(static_map, tasks['box']['park'],
                                 keepouts=park_keepouts(static_map, plan, beam_center=beam_center,
                                                        beam_route=beam_route))
    report['needed'] = True
    return report


class MapGoToYield:
    """Folded-arm move of the solo robot to its agreed park place by A*.

    Same RGB wheel observer and command envelope as the authored yield; only
    the waypoints come from map A* toward the model-chosen destination.
    """
    def __init__(self, static_map, plan, cargo_center_px, *, top_jpeg, beam_center, destination=None):
        from harness.dispatch_yield import WheelObserver
        self.map = static_map
        self.plan = copy.deepcopy(plan)
        self.vision = WheelObserver(static_map, np.array(cargo_center_px) - [50, 0])
        self.cargo_center_px = list(map(float, cargo_center_px))
        self.beam_center = beam_center
        self.top_for_obstacles = top_jpeg
        self.route = None
        self.points = None
        self.index = 0
        self.confirmations = 0
        self.done = False
        self.steps = 0
        self.replans = []
        tasks = {t['object']: t for t in plan['tasks']}
        # Reuse the pre-motion feasibility verdict (it knows the SE2 beam route
        # in clutter); otherwise check against the open-map beam corridor.
        self.destination = copy.deepcopy(destination) if destination is not None else map_goto.check_park(
            static_map, tasks['box']['park'], keepouts=park_keepouts(static_map, plan, beam_center=beam_center))
        if map_goto.parse_destination(tasks['box']['park']) != self.destination['destination']:
            raise RuntimeError('park verdict does not match the committed plan')
        if not self.destination['feasible']:
            raise RuntimeError('PARK_REJECTED: ' + '; '.join(self.destination['problems']))

    def _obstacles(self, jpeg, shape):
        # The released cargo stays where it was put; the beam has not moved.
        released = pixel_to_map(self.cargo_center_px, self.map, shape)
        rows = [map_goto.rect('released_box', released, [.04, .04], 'last TOP RGB cargo silhouette at release')]
        rows += beam_keepouts(self.map, self.plan['dock'], beam_finished=False, top_jpeg=jpeg,
                              fallback_center=self.beam_center)
        return rows + rgb_obstacles(self.map, jpeg)

    def _plan(self, jpeg, xy, reason):
        route = plan_path(self.map, xy, self.destination['xy_m'], UNLOADED_ENVELOPE,
                          obstacles=self._obstacles(jpeg, decode(jpeg).shape), escape_start_m=.06)
        if route is None:
            raise RuntimeError('PARK_PATH_UNAVAILABLE: no A* path from the RGB robot position to '
                               + self.destination['source'])
        self.replans.append({'reason': reason, 'from_m': list(map(float, xy)), 'plan_sha256': route['plan_sha256']})
        self.route = route
        self.points = route['waypoints_m'][1:]
        self.index = 0
        self.confirmations = 0

    def decide(self, jpeg):
        obs = self.vision.observe(jpeg)
        xy = np.array(obs['xy_m'])
        self.steps += 1
        if self.steps > 400:
            raise RuntimeError('RGB park decision budget exhausted')
        if self.points is None:
            self._plan(jpeg, xy, 'initial')
        previous = np.array(self.route['waypoints_m'][self.index])
        target = np.array(self.points[self.index])
        segment = target - previous
        along = float(np.clip(np.dot(xy - previous, segment) / max(1e-9, float(np.dot(segment, segment))), 0., 1.))
        cross_track = float(np.linalg.norm(xy - (previous + along * segment)))
        if cross_track > REPLAN_CROSS_TRACK_M:
            if len(self.replans) > MAX_REPLANS:
                raise RuntimeError('PARK_PATH_DEVIATION: replan budget exhausted')
            self._plan(jpeg, xy, 'cross_track_%.3f_m' % cross_track)
            target = np.array(self.points[0])
            cross_track = 0.
        final = self.index == len(self.points) - 1
        error = target - xy
        tolerance = .012 if final else .025
        ready = float(np.linalg.norm(error)) < tolerance
        self.confirmations = self.confirmations + 1 if ready else 0
        if self.confirmations >= (3 if final else 1):
            if final:
                self.done = True
            else:
                self.index += 1
                self.confirmations = 0
        velocity = np.zeros(2) if ready else error / max(np.linalg.norm(error) / .06, .2)
        from harness.dispatch_pair_navigation import rotate
        local = rotate(velocity, -obs['heading_rad'])
        command = direction_preserving([local[0] / 1.57, local[1] / 1.18], [(-.05, .06), (-.06, .06)])
        action = {'kind': 'mecanum', 'forward': float(command[0]), 'left': float(command[1]),
                  'turn': 0., 'duration_s': .2}
        return action, {'observation': obs, 'destination': self.destination['destination'],
                        'destination_source': self.destination['source'],
                        'destination_xy_m': self.destination['xy_m'],
                        'waypoints_m': self.route['waypoints_m'], 'index': self.index,
                        'plan_sha256': self.route['plan_sha256'], 'replans': len(self.replans) - 1,
                        'error_m': error.tolist(), 'cross_track_m': cross_track, 'done': self.done,
                        'phase': 'released cargo; A* move to the agreed park place'}


def box_waypoints_px(route, static_map, shape):
    """A* waypoints on the lifted-cargo plane, then the authored floor docking target."""
    carried = route['waypoints_m'][1:] if route['predock_used'] else route['waypoints_m'][1:-1]
    points = [map_to_pixel(p, static_map, shape, height=CARRY_PLANE_M) for p in carried]
    # The final target keeps the validated floor-plane projection; the RGB
    # slot-containment check then decides the release as before.
    points.append(map_to_pixel(route['final_docking_m'], static_map, shape))
    return points
