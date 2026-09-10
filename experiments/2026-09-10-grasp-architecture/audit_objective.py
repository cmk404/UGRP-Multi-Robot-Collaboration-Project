"""Read-only diagnostic of the existing objective, not a grasp benchmark.

Only stored observations/commands are selected from result.json. The evaluator
is not consulted. Counterfactual feature perturbations prove code invariance,
not physical equivalence or observability of the original full RGB streams.
"""
import copy
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from harness.camera_pixel_grasp import alignment_features
from harness.camera_pixel_jacobian import _error

raw = Path(sys.argv[1])
result = json.loads(raw.read_text())
rows = []
for n in (140, 638, 743, 764):
    c = result['calls'][n]
    o = c['observation']
    a = o['alignment']
    own = o['own_beam']
    rows.append({'round': n, 'own_center': own['center'],
                 'own_width_px': own['width_px'], 'own_area_px': own['area_px'],
                 'own_endpoint_available': a['own_endpoint'] is not None,
                 'jacobian_error': _error(a).tolist(),
                 'trial_action': c['action']})
# Mutate only the own-view endpoint's vertical coordinate and segmentation
# scale; preserve x, aspect ratio and valid in-image endpoints. Top is fixed.
o = result['calls'][140]['observation']
base = copy.deepcopy(o['own_beam'])
alt = copy.deepcopy(base)
for p in alt['endpoints']:
    p[1] = .5 + (p[1] - .5) * .5
alt['center'][1] = .5 + (alt['center'][1] - .5) * .5
alt['area_px'] *= .25
alt['width_px'] *= .5
alt['length_px'] *= .5
args = (o['gripper'], o['beam'], o['alignment']['endpoint'])
a = alignment_features(*args, base, o['alignment']['own_endpoint'])
b = alignment_features(*args, alt, o['alignment']['own_endpoint'])
assert a['cost'] == b['cost']
assert (_error(a) == _error(b)).all()
report = {'source_sha': subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
          'input_file': str(raw), 'input_sha256': hashlib.sha256(raw.read_bytes()).hexdigest(),
          'scope': 'code objective invariance and saved RGB-derived features only; no new robot actions',
          'actual_trial_rows': rows,
          'counterfactual_invariance': {'own_endpoint_before': a['own_endpoint'],
                                      'own_endpoint_after': b['own_endpoint'],
                                      'cost_before': a['cost'], 'cost_after': b['cost'],
                                      'jacobian_error_before': _error(a).tolist(),
                                      'jacobian_error_after': _error(b).tolist()},
          'limitation': 'Does not prove two real physical states have identical full RGB; does not demonstrate a new successful controller.'}
print(json.dumps(report,indent=2))
