#!/usr/bin/env python3
"""Apply two device-only fixes to the exact pinned optional LeRobot install.

Upstream declares observation.state optional but uses its .device in both VAE
training and inference. Use an existing parameter device instead; no fake state.
Reject unknown source. Reinstalling the pinned package reverses this patch.
"""
import hashlib
import importlib.util
from pathlib import Path

ORIGINAL_SHA256 = '8d622935319af7d03f1efd95d51e726af2921010f7e4f5ec3742f62280493111'
BEFORE = 'batch[OBS_STATE].device'
AFTER = 'self.encoder_latent_input_proj.weight.device'


def patch():
    spec = importlib.util.find_spec('lerobot.policies.act.modeling_act')
    path = Path(spec.origin)
    source = path.read_text()
    original = source.replace(AFTER, BEFORE)
    if hashlib.sha256(original.encode()).hexdigest() != ORIGINAL_SHA256:
        raise ValueError('unexpected upstream ACT source; refusing to patch')
    if original.count(BEFORE) != 2:
        raise ValueError('expected exactly two device references')
    patched = original.replace(BEFORE, AFTER)
    if source != patched:
        path.write_text(patched)
    return {'path': str(path), 'original_sha256': ORIGINAL_SHA256,
            'patched_sha256': hashlib.sha256(patched.encode()).hexdigest(),
            'replacements': 2}


if __name__ == '__main__':
    import json
    print(json.dumps(patch(), indent=2))
