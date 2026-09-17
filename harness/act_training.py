"""Training-only image feature cache and balanced sampling for frozen ACT vision."""
from contextlib import contextmanager
from unittest.mock import patch

import numpy as np
import torch


def sampling_weights(rows, balanced):
    if not balanced:
        return torch.ones(len(rows), dtype=torch.double)
    # Equal mass for stop, slow approach, and remaining approach. Only training labels.
    groups = [2 if row['stop'] else 1 if row['forward'] <= .015 else 0 for row in rows]
    counts = {g: groups.count(g) for g in set(groups)}
    return torch.tensor([1. / counts[g] for g in groups], dtype=torch.double)


@contextmanager
def frozen_features(policy):
    """Bypass only frozen CNN computation with cached, RGB-derived feature maps.

    No state_dict/architecture mutation: restore the normal backbone even on error.
    The cache is training-only; deployed workers always consume original RGB.
    """
    if any(p.requires_grad for p in policy.model.backbone.parameters()):
        raise ValueError('feature cache requires every backbone parameter frozen')
    with patch.object(policy.model.backbone, 'forward', side_effect=lambda value: {'feature_map': value}):
        yield


def prediction_metrics(values, rows):
    from harness.reference_act import decode_prediction
    decisions = [decode_prediction(v) for v in values]
    stop = np.array([bool(r['stop']) for r in rows])
    ready = np.array([r['ready'] for r in decisions])
    forward = np.array([r['forward'] for r in decisions])
    missed = int((stop & ~ready).sum())
    false = int((~stop & ready).sum())
    mae = float(np.mean(np.abs(forward - np.array([r['forward'] for r in rows]))))
    return {'samples': len(rows), 'stop_samples': int(stop.sum()),
            'missed_ready': missed, 'false_ready': false, 'forward_mae': mae,
            'stop_recall': float((stop & ready).sum() / max(1, stop.sum())),
            'selection_score': missed / max(1, stop.sum()) + 10 * false / max(1, (~stop).sum()) + mae / .15}
