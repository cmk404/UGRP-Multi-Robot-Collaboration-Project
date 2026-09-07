#!/usr/bin/env bash
set -euo pipefail
ROOT="${UGRP_DEPTH_ANYTHING_REPO:-/content/Depth-Anything-V2}"
CHECKPOINT="$ROOT/metric_depth/checkpoints/depth_anything_v2_metric_hypersim_vits.pth"
if [ ! -d "$ROOT/.git" ]; then git clone --depth 1 https://github.com/DepthAnything/Depth-Anything-V2 "$ROOT"; fi
python3 -m pip install -q -r "$ROOT/metric_depth/requirements.txt"
mkdir -p "$(dirname "$CHECKPOINT")"
if [ ! -s "$CHECKPOINT" ]; then
  curl -fL --retry 3 -o "$CHECKPOINT" 'https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small/resolve/main/depth_anything_v2_metric_hypersim_vits.pth?download=true'
fi
python3 - <<'PY'
import sys,torch
from pathlib import Path
root=Path('/content/Depth-Anything-V2/metric_depth'); sys.path.insert(0,str(root))
from depth_anything_v2.dpt import DepthAnythingV2
m=DepthAnythingV2(encoder='vits',features=64,out_channels=[48,96,192,384],max_depth=20)
p=root/'checkpoints/depth_anything_v2_metric_hypersim_vits.pth'; m.load_state_dict(torch.load(p,map_location='cpu'))
print({'depth_backend':'Depth Anything V2 Metric Hypersim Small','checkpoint':str(p),'cuda':torch.cuda.is_available()})
PY
