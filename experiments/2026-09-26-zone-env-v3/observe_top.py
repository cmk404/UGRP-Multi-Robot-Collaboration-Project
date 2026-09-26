"""Evaluation-only TOP recording around the UNCHANGED loop / M1 runners (environment v3).

The TOP cameras are evaluation-only (2026-09-25 user decision). The loop and M1
runners never render them, so this wrapper records the four TOP views every
``--top-period`` SIM seconds into ``<episode>/eval_only/top/`` for the report
video. It wraps ``MultiMasterPiProductionV2._physics_step_for`` and renders only
AFTER the physics step returned (physics lock released) with the observer
renderer (``render_team_jpeg``); nothing is written to mjData, and the student /
controller never receives these frames (they live under ``eval_only/``).

Usage (same arguments as the wrapped runner after the runner name):
  python experiments/2026-09-26-zone-env-v3/observe_top.py loop --prereg ... --only ... --output ...
  python experiments/2026-09-26-zone-env-v3/observe_top.py m1   --prereg ... --only ... --output ...
"""
from __future__ import annotations

import contextlib
import hashlib
import io
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCHEMA = 'ugrp.zone_env_v3.top_recording.v1'
# Mosaic layout: north row on top, west column on the left (x east, y north).
MOSAIC = (('cctv_top_north', 'cctv_top_north_east'), ('cctv_top', 'cctv_top_east'))
DEFAULT_PERIOD_S = .5


@contextlib.contextmanager
def record_top(out: Path, period_s: float = DEFAULT_PERIOD_S, quality: int = 80):
    import numpy as np
    from PIL import Image
    from sim.multi_masterpi_production import MultiMasterPiProductionV2 as World
    original = World._physics_step_for
    rows, state = [], {'next': 0.}

    def stepped(self, *args, **kwargs):
        result = original(self, *args, **kwargs)
        now = float(self.data.time)
        if now + 1e-9 >= state['next'] and out.exists():
            tiles = []
            for row in MOSAIC:
                tiles.append(np.concatenate([np.asarray(Image.open(io.BytesIO(
                    self.render_team_jpeg(camera=name, quality=95)))) for name in row], axis=1))
            mosaic = Image.fromarray(np.concatenate(tiles, axis=0)).resize((960, 720))
            folder = out/'eval_only'/'top'
            folder.mkdir(parents=True, exist_ok=True)
            buf = io.BytesIO()
            mosaic.save(buf, format='JPEG', quality=quality)
            name = f'{len(rows):05d}.jpg'
            (folder/name).write_bytes(buf.getvalue())
            rows.append({'index': len(rows), 't': round(now, 4), 'file': f'eval_only/top/{name}',
                         'sha256': hashlib.sha256(buf.getvalue()).hexdigest()})
            state['next'] = now + period_s
        return result

    World._physics_step_for = stepped
    try:
        yield rows
    finally:
        World._physics_step_for = original
        if out.exists():
            (out/'eval_only').mkdir(parents=True, exist_ok=True)
            (out/'eval_only'/'top_index.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
            (out/'eval_only'/'top_recording.json').write_text(json.dumps({
                'schema': SCHEMA, 'period_sim_s': period_s, 'frames': len(rows), 'mosaic': MOSAIC,
                'size_px': [960, 720], 'jpeg_quality': quality,
                'note': ('evaluation-only observer frames rendered after each physics step returned; never an '
                         'input of the student or controller'),
                'wrapper': 'experiments/2026-09-26-zone-env-v3/observe_top.py',
                'wrapper_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}, indent=2) + '\n')


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    period = DEFAULT_PERIOD_S
    if '--top-period' in argv:
        i = argv.index('--top-period')
        period = float(argv[i + 1])
        del argv[i:i + 2]
    which, rest = argv[0], argv[1:]
    if which == 'loop':
        import scripts.run_owncam_closed_loop as runner
    elif which == 'm1':
        import scripts.run_m1_owncam as runner
    else:
        raise SystemExit('first argument: loop or m1')
    original_run = runner.run

    def run(spec, out, *args, **kwargs):
        with record_top(Path(out), period):
            return original_run(spec, out, *args, **kwargs)
    runner.run = run
    runner.main(rest)


if __name__ == '__main__':
    main()
