"""Register Colab's existing NVIDIA EGL libraries and verify a fresh GL context.

Run explicitly in the Colab runtime before launching GPU-rendered experiments.
No drivers are downloaded and no running experiment is changed.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess

PROBE = """import json,mujoco
from OpenGL import GL
context=mujoco.GLContext(64,64)
context.make_current()
print(json.dumps({k:GL.glGetString(v).decode() for k,v in [('vendor',GL.GL_VENDOR),('renderer',GL.GL_RENDERER)]}))
context.free()
"""


def register(library_dir=Path('/usr/lib64-nvidia'), etc=Path('/etc'), share=Path('/usr/share')):
    library = library_dir/'libEGL_nvidia.so.0'
    if not library.is_file():
        raise FileNotFoundError('Colab NVIDIA EGL library is unavailable')
    vendor = share/'glvnd/egl_vendor.d/10_nvidia.json'
    value = {'file_format_version': '1.0.0', 'ICD': {'library_path': str(library)}}
    if vendor.exists() and json.loads(vendor.read_text()) != value:
        raise ValueError('existing NVIDIA vendor registration differs; inspect it before changing')
    config = etc/'ld.so.conf.d/ugrp-colab-nvidia.conf'
    text = str(library_dir)+'\n'
    if config.exists() and config.read_text() != text:
        raise ValueError('existing UGRP loader configuration differs')
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(text)
    subprocess.run(['ldconfig'], check=True)
    vendor.parent.mkdir(parents=True, exist_ok=True)
    vendor.write_text(json.dumps(value)+'\n')
    return vendor


def verify(python, vendor):
    env = {**os.environ, 'MUJOCO_GL': 'egl', 'PYOPENGL_PLATFORM': 'egl',
           '__EGL_VENDOR_LIBRARY_FILENAMES': str(vendor)}
    result = subprocess.run([str(python), '-c', PROBE], env=env, text=True,
                            capture_output=True, timeout=45, check=True)
    gl = json.loads(result.stdout)
    if 'NVIDIA' not in gl['vendor']:
        raise RuntimeError('assigned GPU is not the actual OpenGL renderer')
    return {'verified': gl, 'environment': {k:env[k] for k in
            ('MUJOCO_GL', 'PYOPENGL_PLATFORM', '__EGL_VENDOR_LIBRARY_FILENAMES')}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--python', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('verification output must be new')
    result = verify(args.python, register())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
