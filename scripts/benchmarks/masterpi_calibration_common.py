from __future__ import annotations
import hashlib, json, math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / 'sim/masterpi_dynamics_calibration.json'
CAL_DIR = ROOT / 'calibration/masterpi'

FORBIDDEN_SOURCE_TOKENS = ('sim', 'mujoco', 'synthetic', 'mock', 'generated', 'fake')

def real_measurement_source(value: object) -> bool:
    text = str(value or '').strip().lower()
    return bool(text) and not any(token in text for token in FORBIDDEN_SOURCE_TOKENS)

def finite(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))

def load_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]

def save_jsonl(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as fh:
        for row in rows:
            fh.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + '\n')

def load_manifest(path: Path = MANIFEST) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))

def save_manifest(obj: dict, path: Path = MANIFEST) -> None:
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

def invalidate_manifest(obj: dict) -> None:
    obj['validated'] = False
    obj['validated_at'] = None
    obj['validation_provenance'] = None

def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
