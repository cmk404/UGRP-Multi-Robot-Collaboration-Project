"""Restore byte-verified archived evidence into a NEW output directory."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
def sha(data): return hashlib.sha256(data).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    root = args.destination.resolve()
    root.mkdir(parents=True, exist_ok=False)
    count = 0
    def save(name, data, digest):
        nonlocal count
        path = (root / name).resolve()
        if not path.is_relative_to(root) or sha(data) != digest:
            raise ValueError(f'unsafe path or mismatched bytes: {name}')
        if path.exists():
            if path.read_bytes() != data:
                raise ValueError(f'conflicting archive entries: {name}')
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        if sha(path.read_bytes()) != digest:
            raise ValueError(f'read-back mismatch: {name}')
        count += 1
    for name in ('fixed-metadata', 'delay-metadata', 'varied-metadata', 'development-metadata', 'teacher-training-rgb'):
        manifest = json.loads((HERE / f'{name}.manifest.json').read_text())
        archive = HERE / f'{name}.zip'
        if sha(archive.read_bytes()) != manifest['sha256']:
            raise ValueError(f'archive hash mismatch: {name}')
        with zipfile.ZipFile(archive) as z:
            for row in manifest['entries']:
                save(row['path'], z.read(row['path']), row['sha256'])
    index = json.loads((HERE / 'cohort-rgb-index.json').read_text())
    for archive in index['archives']:
        path = HERE / archive['path']
        if sha(path.read_bytes()) != archive['sha256']:
            raise ValueError(f'RGB archive mismatch: {path.name}')
        with zipfile.ZipFile(path) as z:
            for row in index['entries']:
                if row['archive'] == archive['path']:
                    save(row['path'], z.read(f"objects/{row['sha256']}.jpg"), row['sha256'])
    print(json.dumps({'restored_files': count, 'destination': str(root), 'scope': index['scope']}))

if __name__ == '__main__':
    main()
