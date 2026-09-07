#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TRACE_ROOT = ROOT / "outputs" / "sim_traces"


def resolve_trace(value: str) -> Path:
    if value == "latest":
        runs = [p for p in TRACE_ROOT.iterdir() if p.is_dir()] if TRACE_ROOT.exists() else []
        if not runs:
            raise SystemExit("no SIM traces")
        return max(runs, key=lambda p: p.stat().st_mtime)
    p = TRACE_ROOT / value
    if not p.is_dir():
        raise SystemExit(f"unknown trace: {value}")
    return p


def make_contact_sheet(run_dir: Path) -> Path | None:
    try:
        from PIL import Image, ImageDraw
    except Exception:
        return None
    trace = json.loads((run_dir / "trace.json").read_text(encoding="utf-8"))
    frames = trace.get("frames") or []
    if not frames:
        return None
    # At most 12 temporally distributed keyframes; always include start/end.
    n = min(12, len(frames))
    if n == 1:
        idxs = [0]
    else:
        idxs = sorted(set(round(i * (len(frames) - 1) / (n - 1)) for i in range(n)))
    tiles = []
    for idx in idxs:
        item = frames[idx]
        op = run_dir / str(item.get("observer_image") or "")
        rp = run_dir / str(item.get("robot_image") or "")
        if not op.is_file() or not rp.is_file():
            continue
        observer = Image.open(op).convert("RGB")
        robot = Image.open(rp).convert("RGB")
        observer.thumbnail((480, 270))
        robot.thumbnail((240, 180))
        tile = Image.new("RGB", (500, 470), "white")
        tile.paste(observer, ((500 - observer.width) // 2, 28))
        tile.paste(robot, ((500 - robot.width) // 2, 300))
        draw = ImageDraw.Draw(tile)
        draw.text((10, 8), f"frame {idx:03d}  3rd-person", fill="black")
        draw.text((10, 282), "robot camera", fill="black")
        tiles.append(tile)
    if not tiles:
        return None
    cols = 3
    rows = (len(tiles) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * 500, rows * 470), "white")
    for i, tile in enumerate(tiles):
        sheet.paste(tile, ((i % cols) * 500, (i // cols) * 470))
    out = run_dir / "contact_sheet.jpg"
    sheet.save(out, quality=90)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("trace", nargs="?", default="latest")
    args = ap.parse_args()
    run_dir = resolve_trace(args.trace)
    analysis = json.loads((run_dir / "analysis.json").read_text(encoding="utf-8"))
    trace = json.loads((run_dir / "trace.json").read_text(encoding="utf-8"))
    sheet = make_contact_sheet(run_dir)
    print(f"trace_id: {run_dir.name}")
    print(f"status: {analysis.get('status')}")
    print(f"action: {analysis.get('action')}")
    print(f"frames: {len(trace.get('frames') or [])}")
    for issue in analysis.get("issues") or []:
        print(f"- [{issue.get('severity')}] {issue.get('code')}: {issue.get('summary')}")
    if not (analysis.get("issues") or []):
        print("- no deterministic anomaly detected")
    if sheet:
        print(f"contact_sheet: {sheet}")
    print(f"analysis: {run_dir / 'analysis.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
