"""Select an existing Kaggle NVIDIA EGL vendor without modifying system drivers."""
from pathlib import Path
import json
import os
import subprocess
from scripts.setup_colab_egl import PROBE
from scripts.probe_kaggle_gpu import inventory


def nvidia_environment(*, search_roots=None, base_env=None):
    """Find mounted driver libraries without initializing a renderer or simulator."""
    roots=search_roots or [Path('/usr/lib/x86_64-linux-gnu'),Path('/usr/lib64-nvidia'),
                          Path('/usr/local/nvidia'),Path('/usr/lib64')]
    driver_libraries=sorted({str(p.resolve()) for root in roots if root.exists()
                      for pattern in ('libEGL_nvidia.so*','libnvidia-ml.so*','libcuda.so*')
                      for p in root.rglob(pattern) if p.is_file()})
    libraries=[p for p in driver_libraries if Path(p).name.startswith('libEGL_nvidia.so')]
    env=dict(os.environ if base_env is None else base_env)
    # The offline CPU dependency path must not hide mounted NVIDIA libraries.
    directories=list(dict.fromkeys(str(Path(path).parent) for path in driver_libraries))
    existing=env.get('LD_LIBRARY_PATH','')
    env['LD_LIBRARY_PATH']=':'.join([*directories,*([existing] if existing else [])])
    return env,libraries


def configure(python, output, *, search_roots=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    env,libraries=nvidia_environment(search_roots=search_roots)
    env.update(MUJOCO_GL='egl',PYOPENGL_PLATFORM='egl')
    report=inventory(env=env);report['egl_libraries']=libraries
    report['library_search_path']=env['LD_LIBRARY_PATH']
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if libraries:
        vendor=output/'nvidia.json';vendor.write_text(json.dumps({'file_format_version':'1.0.0','ICD':{'library_path':libraries[0]}})+'\n')
        env['__EGL_VENDOR_LIBRARY_FILENAMES']=str(vendor)
    probe=subprocess.run([str(python),'-c',PROBE],env=env,capture_output=True,text=True,timeout=45)
    report.update(probe_exit=probe.returncode,probe_stdout=probe.stdout,probe_stderr=probe.stderr)
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if probe.returncode:raise RuntimeError('NVIDIA EGL probe failed; see renderer/report.json')
    gl=json.loads(probe.stdout)
    if 'NVIDIA' not in gl['vendor']:raise RuntimeError('NVIDIA EGL unavailable; see renderer/report.json')
    return gl,{key:env[key] for key in ('MUJOCO_GL','PYOPENGL_PLATFORM','__EGL_VENDOR_LIBRARY_FILENAMES','LD_LIBRARY_PATH') if key in env}
