"""Privileged demonstration/evaluation teacher; never a student observation."""
import math


def teacher_command(remaining_m, speed_multiplier=1.0):
    if not math.isfinite(remaining_m) or speed_multiplier not in (1., 2.):
        raise ValueError('finite remaining distance and 1x/2x teacher required')
    ready = abs(remaining_m) <= .004
    original = 0. if ready or remaining_m <= 0 else min(.15, max(.01, .2 * remaining_m))
    return {'ok': True, 'ready': ready,
            'forward': min(.15, speed_multiplier * original),
            'reason': 'privileged_x_teacher', 'stop_score': float(ready)}
