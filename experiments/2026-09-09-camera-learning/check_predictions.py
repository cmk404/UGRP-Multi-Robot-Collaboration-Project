"""Causal-time offline check of learned pixel deltas against next recorded RGB."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from harness.camera_action_learning import CameraActionLearner, visual_features

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('run', type=Path)
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
root = args.run
report = json.loads((root / 'result.json').read_text())
result = {}
for rid in ('r1', 'r3'):
    learner = CameraActionLearner()
    errors, zero_errors, rows = [], [], []
    calls = [c for c in report['calls'] if c['robot_id'] == rid]
    for i, call in enumerate(calls[:-1]):
        before = visual_features((root/rid/f'{i:03d}-own.jpg').read_bytes(),
                                 (root/rid/f'{i:03d}-overhead.jpg').read_bytes())
        after = visual_features((root/rid/f'{i+1:03d}-own.jpg').read_bytes(),
                                (root/rid/f'{i+1:03d}-overhead.jpg').read_bytes())
        # Forecast using only earlier transitions, before observing this outcome.
        forecast = learner.summarize([call['action']])['predictions'][0]
        prediction = forecast['predicted_pixel_feature_delta']
        row = {'round': i, 'action': call['action'], 'supported': prediction is not None,
               'sample_count': forecast['sample_count']}
        if prediction is not None:
            delta = after - before
            error = float(np.mean((np.asarray(prediction) - delta)**2))
            zero_error = float(np.mean(delta**2))
            errors.append(error)
            zero_errors.append(zero_error)
            row.update(prediction_rmse=error**0.5, zero_change_rmse=zero_error**0.5)
        else:
            row['unsupported_reason'] = forecast['unsupported_reason']
        rows.append(row)
        learner.observe(call['action'], before, after)
    result[rid] = {
        'evaluated_transitions': len(rows), 'supported_predictions': len(errors),
        'held_out_prediction_rmse': float(np.sqrt(np.mean(errors))) if errors else None,
        'zero_change_rmse_same_transitions': float(np.sqrt(np.mean(zero_errors))) if errors else None,
        'rows': rows,
    }
args.out.write_text(json.dumps(result, indent=2) + '\n')
print({rid: {k:v for k,v in data.items() if k != 'rows'} for rid,data in result.items()})
