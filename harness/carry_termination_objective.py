"""Training-only supervision of the ACT output used for deployed carry stops.

The deployed adapter consumes the raw done coordinate of action chunk position
zero and compares it with its existing fixed threshold. This auxiliary loss
regresses that same raw coordinate toward the recorded 0/1 decision. It adds
no policy parameters and does not alter the ACT checkpoint format.
"""

from contextlib import contextmanager

import torch
from lerobot.utils.constants import OBS_IMAGES

from harness.pair_carry_act import CONTEXT_KEY
from harness.reference_act import IMAGE_KEYS


@contextmanager
def deployment_mode(policy):
    """Use the zero-latent inference branch while retaining autograd.

    ACT conditions its VAE latent on target actions only in training mode.
    Calling predict_action_chunk would disable gradients, so run the same
    model input in eval mode directly and restore every module's prior mode.
    Cached frozen image features are already supplied by the caller.
    """
    modes = [(module, module.training) for module in policy.modules()]
    policy.eval()
    try:
        yield
    finally:
        for module, mode in modes:
            module.training = mode


def balanced_raw_done_mse(raw_scores, targets):
    """Class-balance regression to raw 0/1 scores, with no sigmoid transform."""
    if raw_scores.ndim != 1 or targets.shape != raw_scores.shape:
        raise ValueError('first-action stop scores and labels must be one vector')
    if not torch.isfinite(raw_scores).all() or not torch.isfinite(targets).all():
        raise ValueError('nonfinite first-action stop score or label')
    if not torch.logical_or(targets == 0, targets == 1).all():
        raise ValueError('stop labels must be exactly zero or one')
    positive = targets == 1
    negative = ~positive
    squared = (raw_scores - targets).square()
    class_losses = []
    if positive.any():
        class_losses.append(squared[positive].mean())
    if negative.any():
        class_losses.append(squared[negative].mean())
    if not class_losses:
        raise ValueError('empty stop batch')
    loss = sum(class_losses) / len(class_losses)
    return loss, {'positive': int(positive.sum()), 'negative': int(negative.sum()),
                  'class_balanced': len(class_losses) == 2}


def deployed_first_action_done_loss(policy, batch):
    """Return a differentiable stop loss from the deployment zero-latent path."""
    actions = batch['action']
    padding = batch['action_is_pad']
    if actions.ndim != 3 or actions.shape[-1] != 4:
        raise ValueError('ACT carry targets must be [batch, chunk, four actions]')
    if padding.shape != actions.shape[:2] or padding[:, 0].any():
        raise ValueError('first action must be present and unpadded')
    labels = actions[:, 0, 3]
    inputs = {OBS_IMAGES: [batch[key] for key in IMAGE_KEYS],
              CONTEXT_KEY: batch[CONTEXT_KEY]}
    with deployment_mode(policy):
        # No ACTION or action_is_pad enters this forward pass. ACT's VAE uses
        # zero latent exactly as in policy.predict_action_chunk.
        raw_scores = policy.model(inputs)[0][:, 0, 3]
    loss, counts = balanced_raw_done_mse(raw_scores, labels)
    return loss, counts, raw_scores


def checkpoint_rank(metrics, objective):
    """Prefer a complete development termination pass for the new objective."""
    if objective != 'deployed_first_action':
        return metrics['selection_score']
    failures = (metrics['premature_hold_episode_fraction']
                + metrics['missed_terminal_episode_fraction'])
    return (0 if metrics['offline_termination_pass'] else 1,
            failures, metrics['selection_score'])
