"""Finite GPU inventory; records evidence even when NVIDIA tooling is absent."""
import glob
import json
import os
from pathlib import Path
import shutil
import subprocess


def inventory(*, env=None):
    candidates = [shutil.which('nvidia-smi'), '/usr/local/nvidia/bin/nvidia-smi', '/usr/bin/nvidia-smi']
    executable = next((p for p in candidates if p and Path(p).is_file()), None)
    report = {'nvidia_smi': executable, 'devices': glob.glob('/dev/nvidia*'),
              'driver_version': None, 'libraries': []}
    version = Path('/proc/driver/nvidia/version')
    if version.is_file():
        report['driver_version'] = version.read_text()
    for root in ['/usr/lib/x86_64-linux-gnu', '/usr/lib64-nvidia', '/usr/local/nvidia', '/usr/lib64']:
        if Path(root).exists():
            report['libraries'].extend(str(p) for p in Path(root).rglob('libEGL_nvidia.so*'))
    if executable:
        try:
            result = subprocess.run([executable, '--query-gpu=name,driver_version', '--format=csv,noheader'],
                                    capture_output=True, text=True, timeout=20, env=env)
            report.update(nvidia_smi_exit=result.returncode, gpus=result.stdout, nvidia_smi_error=result.stderr)
        except Exception as exc:
            report['nvidia_smi_error'] = type(exc).__name__
    return report


def main():
    report = inventory()
    try:
        import torch
        report['torch'] = {'version': torch.__version__, 'cuda_version': torch.version.cuda,
                           'available': torch.cuda.is_available(), 'count': torch.cuda.device_count()}
        if torch.cuda.is_available():
            report['torch']['devices'] = [torch.cuda.get_device_name(i) for i in range(torch.cuda.device_count())]
    except Exception as exc:
        report['torch_error'] = type(exc).__name__ + ': ' + str(exc)
    Path('/kaggle/working/gpu-inventory.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
