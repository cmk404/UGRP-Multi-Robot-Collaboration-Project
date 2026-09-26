"""Grasp -> lift co-motion check v3: beam colour with a low-light brightness floor (own robot_cam only).

M2 stage 2b (frozen ed15489), seed 824, ON and OFF: r2's THIRD grasp (checkpoint 2, base x 2.84, east of
the door on the darker floor) raised ``LOAD_NOT_HELD_AFTER_LIFT`` while the evaluation-only log had the
beam lifted to 0.061 m, both r2 fingers at 5.48 N and the beam fixed relative to r2's base (0.437 m ahead,
-0.011 m aside, before and after). The v2 signature (``owncam_pair_beam_v2.co_motion_signature``) uses
only the lower 60 % of the view, where at grasp range the camera sees the beam's lower face. Its colour
mask needs V >= 120; that face had hue 41, S 166, V ~103 (5th-95th pct 32-116) at x 2.84 versus V ~221 on
the west floor, so the grasp signature had 166 px and the lift signature 0 px -> IoU 0.0. The same
lighting gave the marginal v2 IoU of the other third lifts (822 r2 0.479, dev11-811 r2 0.528; limit 0.45).

v3 changes ONE thing: the beam colour mask in the co-motion signature accepts V >= 60 (same hue 25-54,
S >= 100). The grip band stays excluded (band V <= 60 AND S < 90; beam needs S >= 100). Same region, same
dilation, same IoU and the same limit 0.45 as v2. The carry hold check (``owncam_pair_hold_v3``) and the
grip view are unchanged.

Specificity (a lift with the beam NOT in the jaws) had no recorded frames; the M2 runner's experimenter
intervention ``--inject-open-at-lift`` (jaw forced open during the lift, evaluation-only record) provides
them in development runs before the threshold is frozen.
"""
from __future__ import annotations

import cv2
import numpy as np

from harness import owncam_pair_beam as v1
from harness import owncam_pair_beam_v2 as ob2

PROFILE = 'owncam_pair_lift_v3'
BEAM_V_MIN_LOW = 60
HOLD_MIN_IOU = .45                # unchanged from the v2 lift co-motion limit


def beam_colour_mask_low(frame: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return ((hsv[..., 0] >= ob2.BEAM_HUE[0]) & (hsv[..., 0] <= ob2.BEAM_HUE[1]) & (hsv[..., 1] >= ob2.BEAM_S_MIN)
            & (hsv[..., 2] >= BEAM_V_MIN_LOW))


def co_motion_signature(image) -> np.ndarray:
    """``ob2.co_motion_signature`` with the low-light beam mask (lower 60 % of the view)."""
    frame = v1.decode(image)
    beam = beam_colour_mask_low(frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    band = (hsv[..., 2] <= ob2.BAND_V_MAX) & (hsv[..., 1] < ob2.BAND_S_MAX)
    near = cv2.dilate(beam.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    h = frame.shape[0]
    sig = np.zeros(frame.shape[:2], bool)
    sig[int(.4 * h):] = ((beam | band) & (beam | near))[int(.4 * h):]
    return sig


def lift_iou(grasp_image, lift_image) -> float:
    return v1.signature_iou(co_motion_signature(grasp_image), co_motion_signature(lift_image))
