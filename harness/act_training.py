"""Training-only image feature cache and balanced sampling for frozen ACT vision."""
from contextlib import contextmanager
from unittest.mock import patch
from contextlib import ExitStack

import numpy as np
import torch
from torch.utils.checkpoint import checkpoint


@contextmanager
def checkpoint_encoder(policy, enabled=False):
    """Recompute encoder activations in backward; preserve batch, weights and RNG.

    This only changes training memory use. It does not wrap modules or alter
    state_dict names, inference, attention semantics, precision or the optimizer.
    """
    with ExitStack() as stack:
        if enabled:
            for layer in policy.model.encoder.layers:
                native = layer.forward

                def forward(*args, _native=native, **kwargs):
                    return checkpoint(_native, *args, use_reentrant=False, preserve_rng_state=True, **kwargs)

                stack.enter_context(patch.object(layer, 'forward', side_effect=forward))
        yield


def sampling_weights(rows, balanced, slow_threshold=.015):
    if not balanced:
        return torch.ones(len(rows), dtype=torch.double)
    # Equal mass for stop, slow approach, and remaining approach. Only training labels.
    if not 0 < slow_threshold <= .15:
        raise ValueError('slow threshold must be in (0, .15]')
    groups = [2 if row['stop'] else 1 if row['forward'] <= slow_threshold else 0 for row in rows]
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
