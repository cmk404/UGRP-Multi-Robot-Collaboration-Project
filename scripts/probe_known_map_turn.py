"""Fixed-command image diagnostic, not an autonomous navigation success test."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess

from scripts.known_map_scene import KnownMapScene
from sim.authored_navigation_map import load_map

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT).strip():
        raise RuntimeError('commit diagnostic source before running')
    source = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    args.out_dir.mkdir(parents=True, exist_ok=False)
    (args.out_dir / 'rgb').mkdir()
    data = load_map(ROOT / 'maps/navigation/open.json')
    case = dict(case_id='turn_diagnostic', robot_id='r1', start_xy_m=[-.5, -2.55], start_yaw_deg=0)
    stop = dict(kind='mecanum', forward=0., left=0., turn=0., duration_s=.4)
    actions = []
    for _ in range(3):
        actions += [{**stop, 'forward': .08, 'duration_s': .6}, stop]
    for _ in range(12):
        actions += [{**stop, 'turn': .10, 'duration_s': .6}, stop]
    scene = KnownMapScene(args.out_dir, data, 'r1', case)
    try:
        scene.open()
        records = []
        for index, action in enumerate(actions + [stop]):
            _, _, images = scene.capture(index)
            records.append(dict(frame_id=index, images=images, issued=action))
            scene.last_status = 'fixed-command diagnostic'
            scene.execute(action)
        result = {'source_sha': source, 'case': case, 'records': records,
                  'evaluation': scene.evaluate('diagnostic_only'),
                  'invariants_before': scene.initial_invariants, 'invariants_after': scene.invariants(),
                  'claim': 'fixed issued pulses; no visual feedback controller or navigation success claim'}
        (args.out_dir / 'diagnostic.json').write_text(json.dumps(result, indent=2)+'\n')
    finally:
        scene.close()
    files = {str(p.relative_to(args.out_dir)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(args.out_dir.rglob('*')) if p.is_file()}
    (args.out_dir / 'manifest.json').write_text(json.dumps(files, indent=2)+'\n')
    print(json.dumps(result['evaluation']))


if __name__ == '__main__':
    main()
