"""Sequential, frozen-source physical comparison; use the UGRP session wrapper."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mjpython', required=True)
    parser.add_argument('--out-dir', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    if subprocess.check_output(['git','status','--porcelain'], cwd=root, text=True).strip():
        raise RuntimeError('source must be committed')
    source = subprocess.check_output(['git','rev-parse','HEAD'], cwd=root, text=True).strip()
    args.out_dir.mkdir(parents=True, exist_ok=False)
    cases = [('crossing', False), ('crossing', True), ('head_on', True), ('following', True),
             ('delayed_owner', True), ('report_gap', True), ('restart', True), ('blocked_exit', True)]
    results = []
    for name, coordinated in cases:
        if subprocess.check_output(['git','rev-parse','HEAD'], cwd=root, text=True).strip() != source:
            raise RuntimeError('source changed during cohort')
        label = name + ('-reserved' if coordinated else '-independent')
        folder = args.out_dir/label
        command = [args.mjpython, '-m', 'scripts.run_rgb_traffic', '--scenario', name, '--out-dir', str(folder)]
        if not coordinated: command.append('--independent')
        started = time.monotonic()
        with (args.out_dir/(label+'.log.txt')).open('w') as output:
            process = subprocess.run(command, cwd=root, stdout=output, stderr=subprocess.STDOUT, timeout=600)
        result = json.loads((folder/'result.json').read_text()) if (folder/'result.json').exists() else {'error':'missing result'}
        result.update(label=label, process_returncode=process.returncode, wrapper_wall_s=time.monotonic()-started)
        results.append(result)
        (args.out_dir/'cohort.json').write_text(json.dumps({'source_sha':source,'results':results},indent=2)+'\n')
        print(json.dumps(result),flush=True)


if __name__ == '__main__': main()
