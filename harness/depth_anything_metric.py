"""Optional Depth Anything V2 metric-depth backend for REAL MasterPi.

This module is lazy and has no hard torch/cv2 dependency at import time.  The
backend is intended for a GPU worker (Colab T4 or equivalent), not Raspberry Pi.
No model is auto-downloaded by library code.
"""
from __future__ import annotations
import io, os, sys
from pathlib import Path
from typing import Any

DEFAULT_MODEL_ROOT = Path(os.environ.get('UGRP_DEPTH_ANYTHING_ROOT','/content/Depth-Anything-V2/metric_depth'))
DEFAULT_CHECKPOINT = Path(os.environ.get('UGRP_DEPTH_ANYTHING_CHECKPOINT','/content/Depth-Anything-V2/metric_depth/checkpoints/depth_anything_v2_metric_hypersim_vits.pth'))

class DepthBackendError(RuntimeError): pass

class DepthAnythingMetricProvider:
    name='depth-anything-v2-metric-hypersim-vits'
    def __init__(self, *, model_root: str|Path=DEFAULT_MODEL_ROOT, checkpoint: str|Path=DEFAULT_CHECKPOINT, device: str|None=None, input_size:int=518, max_depth:float=20.0):
        self.model_root=Path(model_root); self.checkpoint=Path(checkpoint)
        self.device=device; self.input_size=int(input_size); self.max_depth=float(max_depth)
        self._model=None; self._cv2=None; self._np=None; self._torch=None
    def load(self):
        if self._model is not None: return self
        try:
            import cv2, numpy as np, torch
        except Exception as exc:
            raise DepthBackendError('Depth Anything metric backend requires torch, torchvision, opencv-python and numpy') from exc
        if not self.model_root.exists(): raise DepthBackendError(f'metric_depth repo missing: {self.model_root}')
        if not self.checkpoint.exists(): raise DepthBackendError(f'metric checkpoint missing: {self.checkpoint}')
        sys.path.insert(0,str(self.model_root))
        try:
            from depth_anything_v2.dpt import DepthAnythingV2
        except Exception as exc:
            raise DepthBackendError(f'cannot import DepthAnythingV2 from {self.model_root}') from exc
        config={'encoder':'vits','features':64,'out_channels':[48,96,192,384],'max_depth':self.max_depth}
        model=DepthAnythingV2(**config)
        state=torch.load(str(self.checkpoint),map_location='cpu')
        model.load_state_dict(state)
        device=self.device or ('cuda' if torch.cuda.is_available() else 'mps' if getattr(torch.backends,'mps',None) and torch.backends.mps.is_available() else 'cpu')
        self._model=model.to(device).eval(); self.device=device
        self._cv2=cv2; self._np=np; self._torch=torch
        return self
    def infer_metric_depth(self, jpeg:bytes):
        self.load()
        raw=self._cv2.imdecode(self._np.frombuffer(jpeg,dtype=self._np.uint8),self._cv2.IMREAD_COLOR)
        if raw is None: raise DepthBackendError('invalid JPEG')
        depth=self._model.infer_image(raw,input_size=self.input_size)
        if depth is None or getattr(depth,'ndim',None)!=2: raise DepthBackendError('model returned invalid depth map')
        return depth
