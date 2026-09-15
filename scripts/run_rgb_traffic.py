"""Execute one fixed-source two-robot traffic trial and save exact actor input."""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import time
import traceback

from harness.rgb_traffic import RGBTrafficRuntime
from scripts.traffic_scenarios import SCENARIOS, scenario, faults

ROOT = Path(__file__).resolve().parents[1]


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def run(name, out, *, coordinated=True, max_steps=300):
    from scripts.rgb_traffic_scene import RGBTrafficScene
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip():
        raise RuntimeError('commit source/protocol before physics execution')
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    maps, setup = scenario(name)
    out.mkdir(parents=True, exist_ok=False); (out/'rgb').mkdir()
    write(out/'actor-maps.json', maps)
    write(out/'run.json', {'source_sha': source, 'scenario': name, 'coordinated': coordinated,
        'private_setup': setup, 'max_steps': max_steps,
        'input_boundary': ['own RGB', 'common top RGB', 'authored static maps', 'issued command history',
                           'reports derived from these inputs', 'monotonic issued-command clock'],
        'own_rgb_usage': 'required, decoded and archived; top RGB controls localization/navigation',
        'scope': 'two unloaded robots, centralized whole-route reservation, no LLM; no loaded-team claim',
        'environment': {'python': platform.python_version(), 'platform': platform.platform(),
                        **{p: importlib.metadata.version(p) for p in ('mujoco', 'numpy', 'opencv-python')}},
        'model_calls': 0, 'model_cost_usd': 0})
    runtime = RGBTrafficRuntime(maps, coordination=coordinated)
    scene = RGBTrafficScene(out, maps, setup)
    started = time.monotonic(); now = 0.; rows = []; statuses = {}; error = None; evaluation = None
    try:
        scene.open(); write(out/'invariants-before.json', scene.initial_invariants)
        with (out/'actor-decisions.jsonl').open('w') as ledger:
            for index in range(max_steps):
                frames, images = scene.capture(index)
                fault = faults(name, index)
                row = runtime.step(frames, index, now, missing=fault['missing'], restart=fault['restart'])
                row['images'] = images; row['fault'] = fault
                # Durable reservation checkpoint is written before any GO dispatch.
                pending = out/'coordinator-state.pending.json'
                write(pending, row['reservation_state']); pending.replace(out/'coordinator-state.json')
                ledger.write(json.dumps(row)+'\n'); ledger.flush()
                rows.append(row)
                statuses = {u: d['status'] for u, d in row['proposals'].items()}
                scene.last_status = ' '.join(u+':'+row['permissions'][u]['phase'] for u in maps)
                now += scene.execute(row['issued_actions'], drive_blocked=fault['drive_blocked'])
                if index % 20 == 0:
                    print(json.dumps({'sequence': index, 'clock_s': now, 'statuses': statuses,
                                      'permissions': {u:p['reason'] for u,p in row['permissions'].items()}}), flush=True)
                if all(s == 'arrived' for s in statuses.values()) and not runtime.coordinator.reservations:
                    break
                if any(d['done'] and d['status'] != 'arrived' for d in row['proposals'].values()):
                    break
                if name == 'blocked_exit' and index >= 70:
                    break
        evaluation = scene.evaluate(statuses)
        write(out/'invariants-after.json', scene.invariants())
        write(out/'contact-events.json', scene.contact_events)
        write(out/'traffic-events.json', runtime.coordinator.events)
    except Exception:
        error = traceback.format_exc()
    finally:
        try: scene.close()
        except Exception: error = (error or '')+'\ncleanup: '+traceback.format_exc()
    result = {'source_sha': source, 'scenario': name, 'coordinated': coordinated,
        'statuses': statuses, 'decision_rounds': len(rows), 'command_clock_s': now,
        'wall_elapsed_s': time.monotonic()-started, 'evaluation': evaluation, 'error': error,
        'model_calls': 0, 'model_cost_usd': 0}
    write(out/'result.json', result)
    write(out/'manifest.json', {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in sorted(out.rglob('*')) if p.is_file()})
    print(json.dumps(result), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=SCENARIOS, required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--independent', action='store_true')
    parser.add_argument('--max-steps', type=int, default=300)
    args = parser.parse_args()
    result = run(args.scenario, args.out_dir, coordinated=not args.independent, max_steps=args.max_steps)
    return 1 if result['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
