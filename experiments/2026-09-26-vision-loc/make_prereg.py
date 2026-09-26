"""Write prereg.json: frozen student (model, calibration, PF config, sources) and the door_1 gate, before test.

Run once after dev selection and BEFORE any test segmentation/localization. The
file is committed; ``vision_loc_cli.py`` refuses test episodes unless every
frozen hash still matches (``check_frozen``).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
FROZEN_FILES = ('vision_loc.py', 'vision_loc_cli.py', 'seg_model.py', 'episodes.json', 'calibration_train.json',
                'maps/zone_wide_door_walls_v3_notags.json', '../2026-09-26-markerless-probe/markerless_probe.py')


def sha_file(p) -> str:
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--checkpoint', required=True)
    ap.add_argument('--train-info', required=True)
    ap.add_argument('--config', required=True, help='selected dev config (copied into the experiment folder)')
    ap.add_argument('--dev-selection', required=True, help='dev_variants.json (selection record)')
    args = ap.parse_args(argv)
    tab = json.loads((HERE/'episodes.json').read_text())
    test_eps = [e['episode_id'] for e in tab['episodes'] if e['split'] == 'test']
    cfg = Path(args.config)
    info = json.loads(Path(args.train_info).read_text())
    prereg = {
        'schema': 'ugrp.vision_loc.prereg.v1',
        'registered': '2026-09-26, Kiro, after dev selection and before any test segmentation, localization or scoring',
        'source_sha_at_registration': subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=HERE, capture_output=True,
                                                     text=True).stdout.strip(),
        'student': {
            'model': {'checkpoint': str(args.checkpoint), 'sha256': sha_file(args.checkpoint), 'bytes': Path(args.checkpoint).stat().st_size,
                      'train_info_sha256': sha_file(args.train_info), 'arch': info.get('backbone_weights'),
                      'release': 'pending (hashed locally; GitHub Release per docs/model_artifacts.md not done)'},
            'calibration': {'file': 'calibration_train.json', 'sha256': sha_file(HERE/'calibration_train.json')},
            'config': {'file': str(cfg.relative_to(HERE)), 'sha256': sha_file(cfg), 'value': json.loads(cfg.read_text())},
            'm1_motion_model': 'M1 calibration (markerless_probe.load_m1_calibration, hash-checked), unchanged',
            'filters': {'vision': 'student', 'boundary': 'PR #210 detector baseline', 'deadreck': 'baseline',
                        'oracle': 'EVAL-ONLY diagnostic (teacher labels), never a student result'},
            'frozen_files_sha256': {f: sha_file(HERE/f) for f in FROZEN_FILES}},
        'dev_selection': {'file': str(Path(args.dev_selection).relative_to(HERE)), 'sha256': sha_file(args.dev_selection)},
        'test_episodes': test_eps,
        'scoring': {'once': True, 'metrics': 'vision_loc_cli.py score (pooled over the 6 test episodes and per episode)',
                    'groups': {'door_zone': '|x - 2.2| < 0.6 and -0.45 < y < 0.55 (GT), as PR #210',
                               'door_loaded': 'door_zone and own-command load state loaded (carrying through door_1)',
                               'lateral': '|y_est - y_gt| (door_1 is crossed along x)'},
                    'false_detections': 'learned column edges outside the teacher-label interval by > 5 px'},
        'gate': {
            'name': 'door_1 carrying clearance',
            'derivation': ('door_1 opening 0.50 m (y -0.20..0.30); loaded body envelope y +-0.15 m '
                           '(harness/owncam_drive.LOADED_ENVELOPE) -> 0.10 m clearance per side. A 3 deg heading '
                           'error swings the envelope front corner (0.20 m ahead) by 0.2*sin(3 deg) = 0.010 m; '
                           'keeping ~0.03 m for path tracking and contact margin leaves 0.06 m for the lateral '
                           'localization error.'),
            'lateral_p99_max_m': .06, 'position_p90_max_m': .05, 'yaw_p90_max_deg': 3.,
            'min_episodes': 3, 'min_door_loaded_frames_per_episode': 20,
            'rule': ('PASS iff, over the pooled test door_loaded frames of the vision filter: lateral p99 <= 0.06 m AND '
                     'position p90 <= 0.05 m AND yaw p90 <= 3.0 deg, with door_loaded frames (>= 20) in >= 3 test '
                     'episodes (else INSUFFICIENT_DATA; teacher failures that never reach the door give no door frames)'),
            'reference_not_gate': ('tag PF on the M1 test frames (tags_v2): door p90 3.6 cm; environment v3 loop test '
                                   '(tags_v3) tag PF - other environments, reported next to the result')},
        'recommendation_rule': ('PASS: wall and door-frame tags are not needed for door_1 carrying at the measured '
                                'accuracy (offline; closed loop still to verify). FAIL: keep a closed-loop check '
                                'before removing them and report the residual error and where it comes from.'),
    }
    (HERE/'prereg.json').write_text(json.dumps(prereg, indent=1) + '\n')
    print(json.dumps(prereg['student']['model'], indent=1))


if __name__ == '__main__':
    main()
