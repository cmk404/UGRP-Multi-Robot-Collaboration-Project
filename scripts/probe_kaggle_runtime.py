"""Small read-only Kaggle runtime inventory; never records credentials/environment."""
import ctypes.util
import importlib.util
import json
from pathlib import Path
import platform
import socket
import sys

report = {
    'python': sys.version, 'platform': platform.platform(),
    'os_release': Path('/etc/os-release').read_text(),
    'libraries': {name: ctypes.util.find_library(name) for name in ('OSMesa', 'EGL', 'GL', 'glfw')},
    'modules': {name: importlib.util.find_spec(name) is not None for name in ('mujoco', 'numpy', 'cv2', 'PIL', 'pip', 'venv')},
}
try:
    socket.getaddrinfo('pypi.org', 443)
    report['pypi_dns'] = True
except OSError as error:
    report['pypi_dns'] = str(error)
output = Path('/kaggle/working/runtime-probe.json')
output.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
