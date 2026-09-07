"""Small deterministic perception primitives for transferable robot state."""

from __future__ import annotations

import io
import os
from pathlib import Path
from typing import Any


def _largest_compact_component(mask, *, min_pixels: int):
    """Largest compact 4-connected component; rejects JPEG/self speckles."""
    import numpy as np
    h, w = mask.shape
    visited = np.zeros_like(mask, dtype=np.uint8)
    best = None
    best_n = 0
    ys0, xs0 = np.nonzero(mask)
    for sy, sx in zip(ys0.tolist(), xs0.tolist()):
        if visited[sy, sx]:
            continue
        stack = [(sy, sx)]
        visited[sy, sx] = 1
        ys, xs = [], []
        while stack:
            y, x = stack.pop()
            ys.append(y); xs.append(x)
            if y > 0 and mask[y-1, x] and not visited[y-1, x]:
                visited[y-1, x] = 1; stack.append((y-1, x))
            if y+1 < h and mask[y+1, x] and not visited[y+1, x]:
                visited[y+1, x] = 1; stack.append((y+1, x))
            if x > 0 and mask[y, x-1] and not visited[y, x-1]:
                visited[y, x-1] = 1; stack.append((y, x-1))
            if x+1 < w and mask[y, x+1] and not visited[y, x+1]:
                visited[y, x+1] = 1; stack.append((y, x+1))
        n = len(xs)
        if n < min_pixels:
            continue
        bw = max(xs) - min(xs) + 1
        bh = max(ys) - min(ys) + 1
        aspect = bw / max(1, bh)
        fill = n / max(1, bw * bh)
        if not (0.35 <= aspect <= 2.8) or fill < 0.18:
            continue
        if n > best_n:
            best_n = n
            best = (np.asarray(ys), np.asarray(xs))
    return best


def _component_detection(mask, *, min_pixels: int) -> dict[str, Any]:
    comp = _largest_compact_component(mask, min_pixels=min_pixels)
    if comp is None:
        return {"visible": False, "pixels": 0}
    ys, xs = comp
    h, w = mask.shape
    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    # The eye-in-hand camera often sees robot/cable fragments clipped at an
    # image edge. A floor block should form a bounded component. Bottom
    # clipping remains allowed for a genuinely close block.
    if x0 <= 1 or x1 >= w - 2 or y0 <= 1:
        return {"visible": False, "pixels": 0}
    return {
        "visible": True,
        "cx": float(xs.mean() / max(1, w - 1)),
        "cy": float(ys.mean() / max(1, h - 1)),
        "bbox": [
            float(x0 / max(1, w - 1)),
            float(y0 / max(1, h - 1)),
            float(x1 / max(1, w - 1)),
            float(y1 / max(1, h - 1)),
        ],
        "area_ratio": float(xs.size / (h * w)),
        "pixels": int(xs.size),
    }


def detect_scene_rgb(rgb) -> dict[str, dict[str, Any]] | None:
    """Detect demo red/yellow/blue objects from one RGB frame.

    This is transferable camera-only perception. It deliberately does not
    infer metric XYZ on REAL hardware; metric position requires calibrated
    intrinsics/extrinsics plus a robot/base motion estimate.
    """
    try:
        import numpy as np
    except Exception:
        return None
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        return None
    r = rgb[..., 0].astype("int16")
    g = rgb[..., 1].astype("int16")
    b = rgb[..., 2].astype("int16")
    masks = {
        # Physical MasterPi lighting often lowers red saturation enough that
        # the old 3x-channel rule misses the cube entirely. Keep channel
        # dominance but use the same conservative margin as the Pi fallback.
        "red": (r > 70) & ((r - g) > 25) & ((r - b) > 25) & (r * 100 > g * 125) & (r * 100 > b * 125),
        "yellow": (r > 100) & (g > 65) & (g > 0.45 * r) & (g < 1.05 * r) & (b < 100) & ((g - b) > 35),
        "blue": (b > 70) & (b > 1.7 * r) & (b > 1.5 * g) & ((b - r) > 40) & ((b - g) > 30),
    }
    h, w = r.shape
    out: dict[str, dict[str, Any]] = {}
    for color, mask in masks.items():
        frac = 0.00035 if color == "red" else 0.00030
        min_pixels = max(18 if color == "red" else 16, int(h * w * frac))
        out[color] = _component_detection(mask, min_pixels=min_pixels)
    return out


def _detect_rgb(rgb) -> dict[str, Any] | None:
    scene = detect_scene_rgb(rgb)
    return None if scene is None else scene["red"]


def detect_scene_bytes(jpeg: bytes) -> dict[str, dict[str, Any]] | None:
    """Detect all known colored objects from one JPEG camera frame."""
    try:
        import numpy as np
        from PIL import Image
        rgb = np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB"))
    except Exception:
        return None
    return detect_scene_rgb(rgb)


def detect_red_target(image_path: str | Path) -> dict[str, Any] | None:
    """Detect the demo target from an RGB image file; never reads simulator state."""
    try:
        import numpy as np
        from PIL import Image
        rgb = np.asarray(Image.open(image_path).convert("RGB"))
    except Exception:
        return None
    return _detect_rgb(rgb)


def detect_red_target_bytes(jpeg: bytes) -> dict[str, Any] | None:
    """Detect the demo target directly from a camera JPEG frame."""
    scene = detect_scene_bytes(jpeg)
    return None if scene is None else scene["red"]

def prepare_planner_image(
    image_path: str | Path,
    *,
    mask_masterpi_self: bool = False,
    max_edge: int | None = None,
) -> str:
    """Create the shared SIM/REAL VLM image without changing sensor evidence.

    Deterministic perception always consumes the original camera frame.  Only
    the VLM copy is masked/downscaled, which keeps Groq vision-token cost below
    the 8k TPM request ceiling while preserving the same planner pixels in SIM
    and REAL.
    """
    try:
        from PIL import Image, ImageDraw
        src = Path(image_path)
        im = Image.open(src).convert("RGB")
        w, h = im.size
        if mask_masterpi_self:
            draw = ImageDraw.Draw(im)
            y0, y1 = int(round(h * 0.785)), int(round(h * 0.865))
            draw.rectangle((0, y0, w, y1), fill=(20, 20, 20))
        if max_edge is None:
            try:
                max_edge = int(os.environ.get("UGRP_PLANNER_IMAGE_MAX_EDGE", "320"))
            except ValueError:
                max_edge = 320
        max_edge = max(224, int(max_edge))
        if max(im.size) > max_edge:
            scale = max_edge / float(max(im.size))
            new_size = (
                max(1, int(round(im.width * scale))),
                max(1, int(round(im.height * scale))),
            )
            im = im.resize(new_size, Image.Resampling.LANCZOS)
        suffix = "-planner-selfmask" if mask_masterpi_self else "-planner"
        out = src.with_name(src.stem + suffix + ".jpg")
        im.save(out, format="JPEG", quality=86, optimize=True)
        return str(out)
    except Exception:
        return str(image_path)


def mask_masterpi_self_for_planner(image_path: str | Path) -> str:
    """Backward-compatible wrapper for the shared planner-image preparation."""
    return prepare_planner_image(image_path, mask_masterpi_self=True)
