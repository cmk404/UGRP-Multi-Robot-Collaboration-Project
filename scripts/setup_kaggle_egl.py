"""Select an existing Kaggle NVIDIA EGL vendor without modifying system drivers."""
from pathlib import Path
import json
import os
import subprocess
from scripts.setup_colab_egl import PROBE


def configure(python, output, *, search_roots=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    roots=search_roots or [Path('/usr/lib/x86_64-linux-gnu'),Path('/usr/lib64-nvidia'),
                          Path('/usr/local/nvidia'),Path('/usr/lib64')]
    libraries=sorted({str(p.resolve()) for root in roots if root.exists()
                      for p in root.rglob('libEGL_nvidia.so*') if p.is_file()})
    smi=subprocess.run(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader'],capture_output=True,text=True,timeout=20)
    report={'nvidia_smi_exit':smi.returncode,'gpus':smi.stdout.strip(),'egl_libraries':libraries}
    env={**os.environ,'MUJOCO_GL':'egl','PYOPENGL_PLATFORM':'egl'}
    if libraries:
        vendor=output/'nvidia.json';vendor.write_text(json.dumps({'file_format_version':'1.0.0','ICD':{'library_path':libraries[0]}})+'\n')
        env['__EGL_VENDOR_LIBRARY_FILENAMES']=str(vendor)
    probe=subprocess.run([str(python),'-c',PROBE],env=env,capture_output=True,text=True,timeout=45)
    report.update(probe_exit=probe.returncode,probe_stdout=probe.stdout,probe_stderr=probe.stderr)
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if probe.returncode:raise RuntimeError('NVIDIA EGL probe failed; see renderer/report.json')
    gl=json.loads(probe.stdout)
    if 'NVIDIA' not in gl['vendor']:raise RuntimeError('NVIDIA EGL unavailable; see renderer/report.json')
    return gl,{key:env[key] for key in ('MUJOCO_GL','PYOPENGL_PLATFORM','__EGL_VENDOR_LIBRARY_FILENAMES') if key in env}
