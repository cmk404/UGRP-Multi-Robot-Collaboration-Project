"""Bind TOP cargo to the solo arm's observed, reversible attachment probe.

Colour supplies candidates, never identity. The caller must independently pass
the own-camera attachment checks before using this binding for navigation.
"""
import hashlib
import cv2
import numpy as np
from harness.camera_goal_transport import decode

PAN_PHASES = ('verify_lift', 'attachment_left', 'attachment_right', 'attachment_home')


def candidates(jpeg):
    hsv = cv2.cvtColor(decode(jpeg), cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array((80, 125, 35), np.uint8), np.array((102, 255, 255), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, stats, centers = cv2.connectedComponentsWithStats(mask)
    return [centers[i] for i in range(1, n) if 25 <= stats[i, 4] <= 600 and max(stats[i, 2:4]) < 40]


def bind_attachment_pan(frames):
    if set(frames) != set(PAN_PHASES):
        raise RuntimeError('TOP cargo identity needs the complete attachment pan sequence')
    views = [candidates(frames[phase]) for phase in PAN_PHASES]
    tracks = []
    for home in views[-1]:
        track = []
        for view in views[:-1]:
            nearby = [p for p in view if np.linalg.norm(p-home) <= 20]
            if len(nearby) != 1:
                break
            track.append(nearby[0])
        if len(track) != 3:
            continue
        track.append(home)
        first, sweep, back = np.diff(track, axis=0)
        lengths = np.linalg.norm([first, sweep, back], axis=1)
        # A stationary painted patch fails minimum motion. A monotonically
        # moving object fails reversal. Return near the initial home image.
        # The initial grasp need not sit at the midpoint of the observed arm
        # sweep. Require a resolved full sweep and reversals, not two symmetric
        # half-sweeps. A one-pixel minimum on each return leg still rejects
        # stationary patches; sub-pixel jitter cannot provide the 4px sweep.
        if (np.all(lengths >= 1.) and lengths[1] >= 4. and np.all(lengths <= 20.)
                and first@sweep/(lengths[0]*lengths[1]) < -.8
                and sweep@back/(lengths[1]*lengths[2]) < -.8
                and .25 <= lengths[0]/lengths[1] <= .8
                and .25 <= lengths[2]/lengths[1] <= .8
                and np.linalg.norm(track[0]-home) <= 2.):
            tracks.append(np.asarray(track))
    if len(tracks) != 1:
        raise RuntimeError('TOP cargo identity unresolved or ambiguous across attachment pan')
    return tracks[0][-1], {
        'method': 'TOP RGB reversible motion during own arm attachment pan',
        'own_attachment_independently_required': True,
        'centers_px': tracks[0].tolist(),
        'frames_sha256': {phase: hashlib.sha256(frames[phase]).hexdigest() for phase in PAN_PHASES},
        'minimum_motion_px': 1., 'minimum_full_sweep_px': 4., 'maximum_home_error_px': 2.,
    }
