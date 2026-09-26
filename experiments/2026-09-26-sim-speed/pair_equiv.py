"""Ad hoc equivalence of two M2 pair run dirs (scratch): every file byte-equal except volatile result.json keys."""
import hashlib, json, sys
from pathlib import Path

VOLATILE = {'wall_seconds', 'load_average_start', 'load_average_end', 'source_sha', 'dirty_source', 'threads'}


def files(root):
    return {str(p.relative_to(root)): p for p in sorted(root.rglob('*')) if p.is_file()}


def main(a, b, qa=None, qb=None):
    A, B = files(Path(a)), files(Path(b))
    out = {'only_a': sorted(set(A) - set(B)), 'only_b': sorted(set(B) - set(A)), 'differ': [], 'same': 0}
    for k in sorted(set(A) & set(B)):
        if k in ('result.json', 'hashes.json'):
            continue
        if hashlib.sha256(A[k].read_bytes()).digest() == hashlib.sha256(B[k].read_bytes()).digest():
            out['same'] += 1
        else:
            out['differ'].append(k)
    ra, rb = (json.loads(x['result.json'].read_text()) for x in (A, B))
    out['result_differing_keys'] = sorted(k for k in set(ra) | set(rb) if k not in VOLATILE and ra.get(k) != rb.get(k))
    out['result_volatile_keys_ignored'] = sorted(VOLATILE)
    ha, hb = (json.loads(x['hashes.json'].read_text()) for x in (A, B))
    out['hashes_json_differing'] = sorted(k for k in set(ha) | set(hb) if ha.get(k) != hb.get(k)) if isinstance(ha, dict) else None
    if qa and qb:
        ca = Path(qa).read_text().splitlines(); cb = Path(qb).read_text().splitlines()
        out['qpos_checkpoints'] = {'identical': ca == cb, 'n_a': len(ca), 'n_b': len(cb)}
    out['equivalent'] = (not out['only_a'] and not out['only_b'] and not out['differ'] and not out['result_differing_keys']
                         and (not qa or out['qpos_checkpoints']['identical']))
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main(*sys.argv[1:])
