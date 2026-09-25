"""Opt-in, versioned contact profiles for zone cargo scenes.

``local_contact_fine`` (sim/dispatch_contact_profile.py) stays unchanged: pinned
RGB bundles and the ZC1/ZC2/v61 records depend on it. A cargo profile is that
base profile plus named solver changes, selected explicitly in
``CargoZoneScene`` and recorded with its own hash.

``cargo_noslip_v1`` (2026-09-25 user request "remove the slip"):
under ``local_contact_fine`` a held load creeps down through the jaws. The
finger/cargo friction rows are soft constraints with a damping-only reference
(``solreffriction 0 -6000``), so a steady tangential force needs a steady slip
velocity (about R*f/b with R the soft-constraint regularizer). Measured with a
60 s static hold of 0.5 kg: 70.6 mm (slid out). Changing mu (x2), condim (6) or
the normal solref did nothing; ``noslip_iterations`` removes the residual slip
of the friction rows after the main solve (MuJoCo's documented remedy) and
leaves normal-contact softness, friction coefficients, joint limits and
actuators untouched: 0.14 mm in 60 s. It is a global solver option, so the
profile applies it to every contact of the scene; the robot drive (a body
wrench over mu=0.001 wheels) and resting boxes were measured unchanged.
No weld or equality constraint is involved.
"""
from __future__ import annotations

import copy
import hashlib
import json
import xml.etree.ElementTree as ET

CARGO_PROFILES = {
    'cargo_noslip_v1': {
        'version': 1,
        'base': 'local_contact_fine',
        'option': {'noslip_iterations': '10'},
        'scope': 'global solver option (all contacts)',
        'evidence': 'experiments/2026-09-25-zone-cargo-catalogue (section: slip removal)',
    },
}


def profile_record(name):
    if name not in CARGO_PROFILES:
        raise ValueError(f'unknown cargo contact profile: {name}')
    value = {'name': name, **copy.deepcopy(CARGO_PROFILES[name])}
    value['sha256'] = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
    return value


def base_profile(name):
    """The dispatch contact profile a cargo profile builds on (or the name itself)."""
    return CARGO_PROFILES[name]['base'] if name in CARGO_PROFILES else name


def apply(xml, name):
    """Apply a cargo profile's changes to XML already built with its base profile."""
    spec = CARGO_PROFILES[name]
    root = ET.fromstring(xml)
    option = root.find('option')
    for key, value in spec['option'].items():
        option.set(key, value)
    if any(eq.get('active') != 'false' for eq in root.findall('equality/weld')):
        raise ValueError('weld assistance must be OFF')
    return ET.tostring(root, encoding='unicode')
