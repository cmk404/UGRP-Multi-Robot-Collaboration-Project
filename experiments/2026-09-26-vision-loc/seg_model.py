"""Floor/wall/self/object/background segmentation of the undistorted wrist view (torch).

Reused, not written: ``torchvision.models.segmentation.lraspp_mobilenet_v3_large``
(LR-ASPP head on MobileNetV3-Large, Howard et al. 2019; torchvision 0.26.0,
BSD-3-Clause) with the torchvision ImageNet-1K MobileNetV3-Large backbone weights
(``MobileNet_V3_Large_Weights.IMAGENET1K_V1``). Only the 5-class head is trained
from scratch; the whole network is fine-tuned on TRAIN-split teacher renders.

Training targets (AGENTS.md teacher exception): MuJoCo segmentation renders of
the same own camera at the same instant (``eval_only/labels``), written by the
teacher render (``run_vl_teacher_render.py``). The trained network's run-time
input is the own RGB frame only.
"""
from __future__ import annotations

import hashlib
import json
import math
import random
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F
import torchvision
from torchvision.models import MobileNet_V3_Large_Weights
from torchvision.models.segmentation import lraspp_mobilenet_v3_large

import vision_loc as vl

SCHEMA = 'ugrp.vision_loc.seg_model.v1'
IN_W, IN_H = 320, 240
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def device() -> torch.device:
    return torch.device('mps') if torch.backends.mps.is_available() else torch.device('cpu')


def build(pretrained_backbone: bool = True) -> torch.nn.Module:
    weights = MobileNet_V3_Large_Weights.IMAGENET1K_V1 if pretrained_backbone else None
    return lraspp_mobilenet_v3_large(weights=None, weights_backbone=weights, num_classes=len(vl.CLASSES))


def preprocess(bgr_raw: np.ndarray, size: tuple[int, int] = (IN_W, IN_H)) -> np.ndarray:
    """Raw fisheye BGR frame -> (3, H, W) float32 normalised RGB of the undistorted view (default 320 x 240)."""
    und = vl.mp.undistort(bgr_raw)
    small = cv2.resize(und, tuple(size), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB).astype(np.float32)/255.
    return ((rgb - MEAN)/STD).transpose(2, 0, 1)


def load_label(path: Path) -> np.ndarray:
    lab = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    lab = lab.copy()
    lab[~vl.VALID] = vl.IGNORE
    return lab


class FrameDataset(torch.utils.data.Dataset):
    """(image, label) pairs of rendered episodes; label at IN_W x IN_H (nearest)."""

    def __init__(self, items, augment: bool):
        self.items = list(items)
        self.augment = augment

    def __len__(self):
        return len(self.items)

    def __getitem__(self, i):
        frame, label = self.items[i]
        bgr = cv2.imread(str(frame), cv2.IMREAD_COLOR)
        x = preprocess(bgr)
        y = cv2.resize(load_label(label), (IN_W, IN_H), interpolation=cv2.INTER_NEAREST).astype(np.int64)
        if self.augment:
            # deterministic per (worker seed, item): the DataLoader generator fixes the worker seeds
            rng = np.random.default_rng((torch.initial_seed() + i*7919) % (2**32))
            if rng.random() < .5:                      # horizontal flip (exact for the labels)
                x, y = x[:, :, ::-1].copy(), y[:, ::-1].copy()
            gain = rng.uniform(.75, 1.3)               # photometric jitter (lighting robustness)
            bias = rng.uniform(-.25, .25)
            x = x*gain + bias + rng.normal(0, .03, size=x.shape).astype(np.float32)
        return torch.from_numpy(np.ascontiguousarray(x, np.float32)), torch.from_numpy(np.ascontiguousarray(y))


def episode_items(ep_dir: Path, every: int = 1):
    rows = vl.read_jsonl(Path(ep_dir)/'inputs'/'frames.jsonl')
    labels = {r['frame_id']: r['label'] for r in vl.read_jsonl(Path(ep_dir)/'eval_only'/'labels.jsonl')}
    return [(Path(ep_dir)/r['file'], Path(ep_dir)/labels[r['frame_id']]) for r in rows[::every]
            if r['frame_id'] in labels]


def confusion(pred: torch.Tensor, target: torch.Tensor, k: int) -> np.ndarray:
    ok = target != vl.IGNORE
    idx = target[ok]*k + pred[ok]
    return torch.bincount(idx, minlength=k*k).reshape(k, k).cpu().numpy()


def metrics_from_confusion(cm: np.ndarray) -> dict:
    tp = np.diag(cm).astype(float)
    iou = tp/np.maximum(cm.sum(0) + cm.sum(1) - tp, 1)
    return {'pixel_acc': round(float(tp.sum()/max(cm.sum(), 1)), 5),
            'iou': {c: round(float(v), 4) for c, v in zip(vl.CLASSES, iou)},
            'miou': round(float(iou[cm.sum(1) > 0].mean()), 4)}


def train(train_eps, val_eps, out: Path, *, epochs: int = 4, batch: int = 16, lr: float = 1e-3, every: int = 2,
          val_every: int = 10, workers: int = 3, seed: int = 0, log=print) -> dict:
    torch.manual_seed(seed)
    random.seed(seed)
    np.random.seed(seed)
    dev = device()
    items = [it for ep in train_eps for it in episode_items(ep, every)]
    val_items = [it for ep in val_eps for it in episode_items(ep, val_every)]
    dl = torch.utils.data.DataLoader(FrameDataset(items, True), batch_size=batch, shuffle=True, num_workers=workers,
                                     drop_last=True, persistent_workers=workers > 0,
                                     generator=torch.Generator().manual_seed(seed))
    vdl = torch.utils.data.DataLoader(FrameDataset(val_items, False), batch_size=batch, shuffle=False,
                                      num_workers=workers)
    model = build(True).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    steps = epochs*len(dl)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=lr, total_steps=steps, pct_start=.1)
    history = []
    step = 0
    t0 = time.time()
    for epoch in range(epochs):
        model.train()
        for x, y in dl:
            x, y = x.to(dev), y.to(dev)
            logits = model(x)['out']
            loss = F.cross_entropy(logits, y, ignore_index=vl.IGNORE)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
            step += 1
            if step % 50 == 0:
                history.append({'step': step, 'epoch': epoch, 'loss': round(float(loss), 5),
                                'lr': float(sched.get_last_lr()[0]), 'wall_s': round(time.time() - t0, 1)})
                log(json.dumps(history[-1]))
        val = evaluate(model, vdl, dev)
        history.append({'step': step, 'epoch': epoch, 'val': val, 'wall_s': round(time.time() - t0, 1),
                        'bn': 'running statistics of augmented training batches'})
        log(json.dumps(history[-1]))
    n_bn = precise_bn(model, items, dev, workers=workers)
    val = evaluate(model, vdl, dev)
    history.append({'step': step, 'epoch': epochs - 1, 'val': val, 'wall_s': round(time.time() - t0, 1),
                    'bn': f'precise BN on {n_bn} clean train frames'})
    log(json.dumps(history[-1]))
    out.mkdir(parents=True, exist_ok=True)
    ckpt = out/'seg_lraspp_mbv3.pt'
    torch.save({'schema': SCHEMA, 'state_dict': {k: v.cpu() for k, v in model.state_dict().items()},
                'classes': vl.CLASSES, 'input': {'width': IN_W, 'height': IN_H, 'mean': MEAN.tolist(),
                                                 'std': STD.tolist(), 'view': 'undistorted pinhole (K)'},
                'arch': 'torchvision.models.segmentation.lraspp_mobilenet_v3_large(num_classes=5)'}, ckpt)
    info = {'schema': SCHEMA, 'checkpoint': str(ckpt), 'sha256': hashlib.sha256(ckpt.read_bytes()).hexdigest(),
            'bytes': ckpt.stat().st_size, 'params': int(sum(p.numel() for p in model.parameters())),
            'train_episodes': [str(e) for e in train_eps], 'val_episodes': [str(e) for e in val_eps],
            'train_frames': len(items), 'val_frames': len(val_items), 'every': every, 'val_every': val_every,
            'epochs': epochs, 'batch': batch, 'lr': lr, 'seed': seed, 'device': str(dev),
            'torch': torch.__version__, 'torchvision': torchvision.__version__, 'wall_s': round(time.time() - t0, 1),
            'backbone_weights': 'MobileNet_V3_Large_Weights.IMAGENET1K_V1', 'precise_bn_frames': n_bn,
            'history': history}
    (out/'train_info.json').write_text(json.dumps(info, indent=1))
    return info


@torch.no_grad()
def precise_bn(model, items, dev, *, batch: int = 32, workers: int = 3) -> int:
    """Recompute every BatchNorm's running mean/variance on clean (un-augmented) TRAIN frames.

    Training batches carry per-sample photometric jitter, so the running
    statistics collected during training describe a wider input distribution
    than a clean frame: in eval mode walls were read as floor (dev wall IoU
    0.74 in eval mode vs 0.99 wall recall with batch statistics on the same
    frames). This is the standard "precise BN" step (cumulative average over a
    pass of the training data, e.g. fvcore ``update_bn_stats``; torch
    ``torch.optim.swa_utils.update_bn`` does the same), done here on TRAIN frames.
    """
    bns = [m for m in model.modules() if isinstance(m, torch.nn.modules.batchnorm._BatchNorm)]
    saved = [m.momentum for m in bns]
    for m in bns:
        m.reset_running_stats()
        m.momentum = None                       # cumulative moving average
    dl = torch.utils.data.DataLoader(FrameDataset(items, False), batch_size=batch, shuffle=False,
                                     num_workers=workers)
    model.train()
    n = 0
    for x, _ in dl:
        model(x.to(dev))
        n += len(x)
    for m, mom in zip(bns, saved):
        m.momentum = mom
    model.eval()
    return n


@torch.no_grad()
def evaluate(model, dl, dev) -> dict:
    model.eval()
    k = len(vl.CLASSES)
    cm = np.zeros((k, k), np.int64)
    for x, y in dl:
        pred = model(x.to(dev))['out'].argmax(1).cpu()
        cm += confusion(pred, y, k)
    return metrics_from_confusion(cm)


class Segmenter:
    """Run-time wrapper: raw own frame -> (H, W, 5) class probabilities at 640 x 480."""

    def __init__(self, ckpt: Path, dev: str | None = None, infer_size: tuple[int, int] = (IN_W, IN_H)):
        blob = torch.load(str(ckpt), map_location='cpu', weights_only=False)
        if tuple(blob['classes']) != vl.CLASSES:
            raise ValueError('checkpoint classes differ')
        self.sha256 = hashlib.sha256(Path(ckpt).read_bytes()).hexdigest()
        self.dev = torch.device(dev) if dev else device()
        self.model = build(False)
        self.model.load_state_dict(blob['state_dict'])
        self.model.eval().to(self.dev)
        # Inference input size (a dev choice; the network is fully convolutional). Its head predicts at
        # 1/8 of the input, so 480 x 360 halves the class-boundary quantisation of the 320 x 240 training size.
        self.infer_size = (int(infer_size[0]), int(infer_size[1]))

    @torch.no_grad()
    def probs(self, bgr_raw: np.ndarray) -> np.ndarray:
        x = torch.from_numpy(preprocess(bgr_raw, self.infer_size))[None].to(self.dev)
        logits = self.model(x)['out']
        up = F.interpolate(logits, size=(vl.HEIGHT, vl.WIDTH), mode='bilinear', align_corners=False)
        return torch.softmax(up, 1)[0].permute(1, 2, 0).float().cpu().numpy()
