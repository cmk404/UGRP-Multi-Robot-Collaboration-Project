"""Side-by-side episode video: robot wrist camera (robot input) | TOP mosaic (evaluation only).

Reads one finished loop / M1 run directory (never modified):
  frames/*.jpg + inputs/frames.jsonl   wrist JPEGs exactly as the robot saw them (every 0.2 SIM s)
  eval_only/top/*.jpg + eval_only/top_index.jsonl   observe_top.py mosaic (every 0.5 SIM s)
Each output frame pairs one wrist frame with the latest TOP frame at or before its SIM time
(frames before the first TOP frame show a blank panel). The caption shows SIM time and the
runner's recorded phase. Encoded with ffmpeg (H.264, yuv420p).

  python make_videos.py <run dir> <out.mp4> [--fps 10] [--width 1280]
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT = '/System/Library/Fonts/Menlo.ttc'


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def make(run: Path, out: Path, fps: float = 10., width: int = 1280) -> dict:
    wrist = rows(run/'inputs'/'frames.jsonl')
    top = rows(run/'eval_only'/'top_index.jsonl') if (run/'eval_only'/'top_index.jsonl').exists() else []
    top_t = [r['t'] for r in top]
    result = json.loads((run/'result.json').read_text())
    title = f"{result.get('episode')}  outcome={result.get('outcome')}"
    half = width//2
    h = half*3//4
    bar = 44
    try:
        font = ImageFont.truetype(FONT, 15)
    except OSError:
        font = ImageFont.load_default()
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = ['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{width}x{h + bar}',
           '-r', str(fps), '-i', '-', '-c:v', 'libx264', '-preset', 'medium', '-crf', '28', '-pix_fmt', 'yuv420p',
           '-movflags', '+faststart', str(out)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    blank = Image.new('RGB', (half, h), (30, 30, 30))
    cache = {}
    for row in wrist:
        canvas = Image.new('RGB', (width, h + bar), (0, 0, 0))
        canvas.paste(Image.open(run/row['file']).convert('RGB').resize((half, h)), (0, 0))
        k = bisect.bisect_right(top_t, row['t'] + 1e-6) - 1
        if k >= 0:
            if k not in cache:
                cache.clear()
                cache[k] = Image.open(run/top[k]['file']).convert('RGB').resize((half, h))
            canvas.paste(cache[k], (half, 0))
        else:
            canvas.paste(blank, (half, 0))
        phase = row.get('phase')
        state = row.get('student_state') or row.get('skill_phase')
        draw = ImageDraw.Draw(canvas)
        draw.text((8, h + 4), f"SIM {row['t']:7.1f} s  phase={phase}  state={state}", fill=(255, 255, 255), font=font)
        draw.text((8, h + 23), f'{title}   left: wrist camera (robot input)   right: TOP (evaluation only, never a robot input)',
                  fill=(200, 200, 200), font=font)
        proc.stdin.write(np.asarray(canvas, np.uint8).tobytes())
    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError('ffmpeg failed')
    data = out.read_bytes()
    return {'video': str(out), 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data), 'frames': len(wrist),
            'fps': fps, 'sim_speed': round(fps*.2, 2), 'top_frames': len(top), 'source': str(run)}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('run', type=Path)
    p.add_argument('out', type=Path)
    p.add_argument('--fps', type=float, default=10.)
    p.add_argument('--width', type=int, default=1280)
    a = p.parse_args(argv)
    print(json.dumps(make(a.run, a.out, a.fps, a.width)))


if __name__ == '__main__':
    main()
