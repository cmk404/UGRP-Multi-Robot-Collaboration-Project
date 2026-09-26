"""Pair-carry hold check v3: whole-view beam mask anchored on the lift view (own robot_cam only).

v2 kept v1's lime strip signature for the carry hold. At grasp range the wrist
camera sees almost only the yellow beam and its black grip band; the lime strip
is ~0.8 % of the frame, so a few pixels of settling halve its ratio. Pair v2
seed 626: r1 raised LOAD_CHANGED_IN_CARRY 1.1 s into the carry (ratio 0.57 ->
0.42) while GT had both grips holding (0.81 deg tilt) -- a false drop alarm.

v3 compares the WHOLE valid view: pixels that are beam colour (hue 25-54,
S >= 100, V >= 120) or grip band (dark, low saturation), as an IoU against the
same mask of the lift view. The camera rides on the gripper, so a held beam keeps
this mask; a beam that leaves the jaws changes it. Recorded v2 carries (38
robot-runs, 1720 frames, all GT holds): IoU min 0.887, median 0.981; 626 r1
>= 0.984. The threshold is set from deliberate-drop dev runs (study script
``--inject-drop``) and frozen before the cohort.
"""
from __future__ import annotations

import cv2
import numpy as np

from harness import owncam_pair_beam as v1
from harness import owncam_pair_beam_v2 as ob2

PROFILE = 'owncam_pair_hold_v3'
HOLD_MIN_IOU = .70
LOST_FRAMES = 2


def hold_view_mask(image) -> np.ndarray:
    frame = v1.decode(image)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    band = (hsv[..., 2] <= ob2.BAND_V_MAX) & (hsv[..., 1] < ob2.BAND_S_MAX)
    return (ob2.beam_colour_mask(frame) | band) & ob2._valid()


def hold_iou(anchor: np.ndarray, image) -> float:
    mask = hold_view_mask(image)
    union = np.logical_or(anchor, mask).sum()
    return float(np.logical_and(anchor, mask).sum() / union) if union else 0.
