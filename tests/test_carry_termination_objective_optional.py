"""Real ACT checks for the optional deployed first-action stop objective."""

import hashlib

import pytest

torch = pytest.importorskip('torch')
pytest.importorskip('lerobot')

from harness.carry_input_act import make_policy
from harness.carry_termination_objective import (
    balanced_raw_done_mse, checkpoint_rank, deployed_first_action_done_loss,
)
from harness.pair_carry_act import CONTEXT_KEY
from harness.reference_act import IMAGE_KEYS
from scripts.train_carry_act import load


def test_terminal_target_and_tail_padding_reach_chunk_zero(tmp_path):
    image = tmp_path / 'frame.jpg'
    image.write_bytes(b'provenance only; this loader does not decode the image')
    reference = {'path': image.name, 'sha256': hashlib.sha256(image.read_bytes()).hexdigest()}
    sequence = [{'id': f'{tmp_path}:r1:{i}', 'images': {'own': reference, 'top': reference},
                 'context': [0.] * 8, 'action': [0., 0., 0., float(i == 2)],
                 'done': i == 2} for i in range(3)]
    rows, target, padding = load([{'root': str(tmp_path), 'files': {},
                                   'sequences': {'r1': sequence}}])
    assert len(rows) == 3
    assert target[2, 0, 3] == 1
    assert not padding[2, 0]
    assert padding[2, 1:].all()
    assert target[1, 1, 3] == 1
    assert not padding[1, 1]


def test_balanced_loss_regresses_raw_scores_without_sigmoid():
    scores = torch.tensor([.2, .4, .8], requires_grad=True)
    targets = torch.tensor([0., 0., 1.])
    loss, counts = balanced_raw_done_mse(scores, targets)
    torch.testing.assert_close(loss, torch.tensor(.07))
    assert counts == {'positive': 1, 'negative': 2, 'class_balanced': True}
    loss.backward()
    assert scores.grad[2] < 0
    assert scores.grad[:2].gt(0).all()


def test_real_act_zero_latent_path_has_first_action_done_gradient(monkeypatch):
    torch.set_num_threads(2)
    torch.manual_seed(31)
    policy = make_policy(128, 1)
    policy.train()
    batch = {IMAGE_KEYS[0]: torch.randn(2, 3, 128, 128),
             IMAGE_KEYS[1]: torch.randn(2, 3, 128, 128),
             CONTEXT_KEY: torch.tensor([[0.] * 32, [1.] * 32])}
    batch['action'] = torch.zeros(2, 8, 4)
    batch['action'][1, 0, 3] = 1.
    batch['action_is_pad'] = torch.ones(2, 8, dtype=torch.bool)
    batch['action_is_pad'][:, 0] = False

    def forbidden_vae(*_args, **_kwargs):
        raise AssertionError('target-conditioned VAE encoder used during auxiliary loss')

    monkeypatch.setattr(policy.model.vae_encoder, 'forward', forbidden_vae)
    loss, counts, raw_scores = deployed_first_action_done_loss(policy, batch)
    assert counts['positive'] == counts['negative'] == 1
    assert policy.training and policy.model.training
    assert raw_scores.requires_grad
    grad = torch.autograd.grad(loss, policy.model.action_head.weight)[0]
    assert grad[3].norm() > 0
    assert torch.count_nonzero(grad[:3]) == 0

    # The auxiliary receives no motion/future targets and exactly matches
    # the inference branch's first action before the runtime score clamp.
    changed = {**batch, 'action': batch['action'].clone()}
    changed['action'][:, :, :3] = 100
    changed['action'][1, 1:, 3] = 100
    _, _, unchanged_scores = deployed_first_action_done_loss(policy, changed)
    torch.testing.assert_close(unchanged_scores, raw_scores)
    policy.eval()
    with torch.no_grad():
        expected = policy.predict_action_chunk({key: batch[key] for key in
                                                (*IMAGE_KEYS, CONTEXT_KEY)})[:, 0, 3]
    torch.testing.assert_close(raw_scores.detach(), expected)


def test_first_action_padding_and_checkpoint_gate():
    bad = {'action': torch.zeros(1, 8, 4),
           'action_is_pad': torch.ones(1, 8, dtype=torch.bool)}
    with pytest.raises(ValueError, match='first action'):
        deployed_first_action_done_loss(None, bad)
    failed = {'offline_termination_pass': False,
              'premature_hold_episode_fraction': 0., 'missed_terminal_episode_fraction': 1.,
              'selection_score': .1}
    passed = {'offline_termination_pass': True,
              'premature_hold_episode_fraction': 0., 'missed_terminal_episode_fraction': 0.,
              'selection_score': .2}
    assert checkpoint_rank(passed, 'deployed_first_action') < checkpoint_rank(
        failed, 'deployed_first_action')
    assert checkpoint_rank(failed, 'episode') == .1
