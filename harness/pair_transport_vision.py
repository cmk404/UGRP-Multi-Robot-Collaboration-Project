"""Wheel landmarks, visible shaft geometry, and own-camera carry monitoring.

All measurements come from the two RGB inputs. The authored camera calibration
and the existing appearance model are the only fixed geometric information.
"""
from __future__ import annotations

import copy
import math

import cv2
import numpy as np

from harness.known_map_navigation import _decode_jpeg, pixel_to_world
from harness.pair_navigation import PairVision, ROBOTS, VisionUncertain


class GripUncertain(ValueError):
    """The own-camera image no longer supports continuing the carry."""


class OwnCarryMonitor:
    """Detect loss of the close, wide beam silhouette in the fixed own camera.

    This is a visual continuation guard, not a contact sensor. It deliberately
    does not certify a physical grasp. Physical scoring stays output-only.
    """
    def __init__(self):
        self.initial_width = None
        self.bad_frames = 0
        self.last = None

    def observe(self, frame):
        h, w = frame.shape[:2]
        image = frame[round(h*.07):round(h*.93), round(w/12):round(w*11/12)]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (5, 90, 45), (22, 255, 255)) > 0
        widths = mask.mean(axis=1)
        visible = widths[widths > .08]
        width = float(np.quantile(visible, .9)) if len(visible) else 0.
        if self.initial_width is None:
            if width < .85 or mask.mean() < .15:
                raise GripUncertain('initial own-camera beam silhouette unsupported')
            self.initial_width = width
        uncertain = width < .75*self.initial_width or mask.mean() < .12
        self.bad_frames = self.bad_frames+1 if uncertain else 0
        self.last = {'wide_beam_fraction': width, 'orange_fraction': float(mask.mean()),
                     'uncertain_frames': self.bad_frames,
                     'continuation_supported': not uncertain}
        if uncertain:
            raise GripUncertain('own-camera beam silhouette changed or disappeared')
        return dict(self.last)


class GeometryPairVision(PairVision):
    """Track four wheel clusters instead of rotating a whole color template."""
    def __init__(self, data):
        super().__init__(data)
        self.wheel_origins = {}
        self.geometry_angles = {}
        self.carry_monitor = OwnCarryMonitor()

    @staticmethod
    def wheel_landmarks(mask, center, angle):
        x, y = np.round(center).astype(int)
        radius = 40
        h, w = mask.shape
        if min(x-radius, y-radius) < 0 or x+radius >= w or y+radius >= h:
            raise VisionUncertain('wheel landmark search outside calibrated image')
        yy, xx = np.where(mask[y-radius:y+radius+1, x-radius:x+radius+1] > 0)
        points = np.column_stack((xx+x-radius, yy+y-radius)).astype(float)
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s], [s, c]])
        # Refine the center twice so translation does not truncate one wheel.
        estimate = np.array(center, dtype=float)
        for _ in range(2):
            local = (points-estimate) @ rotation.T
            corners = []
            counts = []
            for sx, sy in ((-1,-1), (1,-1), (1,1), (-1,1)):
                selected = ((local[:,0]*sx > 10) & (local[:,1]*sy > 10)
                            & (local[:,0]*sx < 29) & (local[:,1]*sy < 29))
                if selected.sum() < 3:
                    raise VisionUncertain('four separate wheel landmarks not visible')
                corners.append(points[selected].mean(axis=0))
                counts.append(int(selected.sum()))
            corners = np.array(corners)
            estimate = corners.mean(axis=0)
        horizontal = (corners[1]-corners[0]+corners[2]-corners[3])/2
        vertical = (corners[3]-corners[0]+corners[2]-corners[1])/2
        raw = math.atan2(-(horizontal[1]-vertical[0]), horizontal[0]+vertical[1])
        raw = angle+(raw-angle+math.pi)%(2*math.pi)-math.pi
        lengths = np.linalg.norm(np.roll(corners,-1,axis=0)-corners,axis=1)
        diagonal_midpoint_error = np.linalg.norm((corners[0]+corners[2]-corners[1]-corners[3])/2)
        if (lengths.min() < 28 or lengths.max() > 46 or diagonal_midpoint_error > 7
                or abs(raw-angle) > math.radians(9) or np.linalg.norm(estimate-center) > 17):
            raise VisionUncertain('wheel landmark geometry or continuity inconsistent')
        return estimate, raw, corners, counts

    def _visible_payload(self, frame, observations):
        # The independently observed robot centers locate a search corridor;
        # orange pixels still have to provide the shaft's position and angle.
        a, b = [np.array(observations[r]['center_uv']) for r in ROBOTS]
        center = (a+b)/2
        direction = b-a
        spacing = np.linalg.norm(direction)
        if not 120 < spacing < 190:
            raise VisionUncertain('robot image spacing outside carry envelope')
        direction /= spacing
        normal = np.array([-direction[1], direction[0]])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (8,100,70), (19,255,255))
        yy, xx = np.where(mask > 0)
        pixels = np.column_stack((xx,yy)).astype(float)
        local = pixels-center
        selected = (np.abs(local@direction) < spacing*.24) & (np.abs(local@normal) < 14)
        pixels = pixels[selected]
        if len(pixels) < 100:
            raise VisionUncertain('visible central shaft pixels insufficient')
        box = cv2.boxPoints(cv2.minAreaRect(pixels.astype(np.float32)))
        edge = max((box[(j+1)%4]-box[j] for j in range(4)),key=np.linalg.norm)
        axis_image = edge/np.linalg.norm(edge)
        along = pixels@axis_image
        across = pixels@np.array([-axis_image[1],axis_image[0]])
        span, width = float(np.ptp(along)), float(np.ptp(across))
        if span < 18 or width > 22 or span/max(width,1) < 2.5:
            raise VisionUncertain('central shaft is not an identifiable straight strip')
        axis = np.array([axis_image[0],-axis_image[1]])
        previous = np.array(self.payload['axis_xy'])
        if axis@previous < 0: axis = -axis
        raw = math.atan2(axis[1],axis[0])
        delta = (raw-self.payload_angle+math.pi/2)%math.pi-math.pi/2
        if abs(delta) > .12:
            raise VisionUncertain('shaft direction changed too quickly')
        angle = self.payload_angle+delta
        points = np.array([pixel_to_world(p,frame.shape,self.map['top_camera']) for p in pixels])
        projections = points@axis
        return {'xy_m':points.mean(axis=0).tolist(), 'relative_yaw_rad':angle-self.payload_origin_angle,
                'feature_area_px':len(pixels), 'feature_plane_height_m':.09,
                'xy_is_visible_fragment_centroid':True,'axis_xy':axis.tolist(),
                'center_projection_interval_m':[float(projections.max()-self.payload_length_m/2),
                                                float(projections.min()+self.payload_length_m/2)]}, angle

    def observe(self, own_rgb, top_rgb):
        own = _decode_jpeg(own_rgb,'own_rgb')
        frame = _decode_jpeg(top_rgb,'shared_top_rgb')
        if frame.shape != (720,960,3):
            raise ValueError('wheel geometry requires the calibrated 960x720 top camera')
        self.carry_monitor.observe(own)
        before = copy.deepcopy(self.__dict__)
        try:
            initializing = not self.templates
            if initializing:
                super().observe(own_rgb,top_rgb)
            hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,(18,80,55),(38,255,255))
            observations = {}
            for rid in ROBOTS:
                previous = self.geometry_angles.get(rid,0.)
                center, raw, corners, counts = self.wheel_landmarks(mask,np.array(self.centers[rid]),previous)
                if initializing:
                    self.wheel_origins[rid] = raw
                # Half-frame smoothing reduces stripe pixel jitter without
                # treating a command or a previous angle as a fresh observation.
                angle = raw if initializing else previous+.65*(raw-previous)
                self.geometry_angles[rid] = angle
                self.centers[rid] = tuple(center)
                self.angles[rid] = math.degrees(angle-self.wheel_origins[rid])
                observations[rid] = {'xy_m':list(pixel_to_world(center,frame.shape,self.map['top_camera'])),
                                     'relative_yaw_rad':angle-self.wheel_origins[rid],
                                     'confidence':1., 'center_uv':center.tolist(),
                                     'feature_plane_height_m':.09,
                                     'wheel_landmarks_uv':corners.tolist(),'wheel_pixel_counts':counts,
                                     'raw_geometric_yaw_rad':raw-self.wheel_origins[rid]}
            self.payload,self.payload_angle = self._visible_payload(frame,observations)
            if initializing:
                self.payload_origin_angle = self.payload_angle
                self.payload['relative_yaw_rad'] = 0.
            return observations
        except ValueError as error:
            self.__dict__.clear();self.__dict__.update(before)
            raise VisionUncertain(str(error)) from error
