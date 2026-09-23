"""Independent RGB arrival admission; no measured state and no action fallback."""
import math
import numpy as np
from harness.dispatch_skill_binding import pixel_from_map


def arrival_evidence(feature, static_map, dock):
    """Use the same fixed camera/nominal held plane as the RGB baseline.

    This is a visual admission estimate, never proof of contact or delivery.
    Final released-outline verification and the independent referee still run.
    """
    slots = static_map['docks'][dock]['slots']
    goal = list(slots['beam']['center_m'])
    goal[0] -= math.copysign(.06, slots['box']['center_m'][0]-goal[0])
    w, h = feature['image_size']
    target = pixel_from_map(goal, static_map, (h, w), height=.09)
    center = np.asarray(feature['center'])*[w, h]
    error = target-center
    return {'source': 'current TOP RGB shaft + authored map and fixed nominal 0.09m plane',
            'target_px': target.tolist(), 'center_px': center.tolist(),
            'error_px': error.tolist(), 'tolerance_px': 4.,
            'arrived': bool(np.max(np.abs(error)) <= 4.),
            'physical_success': 'unavailable to controller'}


class PrematureCarryStop(RuntimeError):
    pass
