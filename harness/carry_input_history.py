"""Causal, robot-local windows shared by training, runtime, and input audits."""
import base64
import math


def window_indices(index, length):
    if type(index) is not int or index < 0 or length not in (1, 4):
        raise ValueError('nonnegative index and history 1 or 4 required')
    return [max(0, index - lag) for lag in reversed(range(length))]


def validate_context(value):
    if (not isinstance(value, list) or len(value) != 8 or
            not all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 2 for v in value)):
        raise ValueError('invalid authored/issued context')


def validate_frames(frames, length):
    if not isinstance(frames, list) or len(frames) != length:
        raise ValueError('wrong history length')
    for frame in frames:
        if not isinstance(frame, dict) or set(frame) != {'own_rgb', 'top_rgb', 'context'}:
            raise ValueError('unapproved history field')
        validate_context(frame['context'])
        if frame['context'][:5] != frames[-1]['context'][:5]:
            raise ValueError('history crossed task or robot boundary')
        for key in ('own_rgb', 'top_rgb'):
            if not isinstance(frame[key], bytes) or not frame[key]:
                raise ValueError('nonempty original JPEG required')
    return frames


def wire_request(frames, length):
    validate_frames(frames, length)
    return {'frames': [{**frame, **{k: base64.b64encode(frame[k]).decode('ascii')
                                  for k in ('own_rgb', 'top_rgb')}} for frame in frames]}


def decode_request(value, length):
    if not isinstance(value, dict) or set(value) != {'frames'}:
        raise ValueError('unapproved worker field')
    frames = value['frames']
    if not isinstance(frames, list) or len(frames) != length:
        raise ValueError('wrong history length')
    decoded = []
    for f in frames:
        if not isinstance(f, dict) or set(f) != {'own_rgb', 'top_rgb', 'context'}:
            raise ValueError('unapproved history field')
        if any(not isinstance(f[k], str) or len(f[k]) > 8000000 for k in ('own_rgb', 'top_rgb')):
            raise ValueError('invalid image encoding')
        decoded.append({**f, **{k: base64.b64decode(f[k], validate=True)
                               for k in ('own_rgb', 'top_rgb')}})
    return validate_frames(decoded, length)
