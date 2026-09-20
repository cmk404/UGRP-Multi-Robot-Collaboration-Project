#!/usr/bin/env python3
"""Re-evaluate archived transit clearance without changing original verdicts."""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from harness.dispatch_evaluation import carry_clearance


def main():
    for arg in sys.argv[1:]:
        run = Path(arg)
        result = json.loads((run/'result.json').read_text())
        commands = json.loads((run/'issued-commands.json').read_text())
        samples = [json.loads(line) for line in (run/'referee-only.jsonl').read_text().splitlines()]
        cargo = carry_clearance(samples, commands, result.get('plan'))
        clear = bool(cargo) and all(v['sampled_continuous_clearance'] for v in cargo.values())
        report = {'cargo': cargo, 'sampled_continuous_clearance': clear,
                  'complete_e2e_with_clearance': bool(clear and result['physical_success']
                      and result['protocol_complete'] and not result['error'])}
        (run/'carry-clearance-audit.json').write_text(json.dumps(report, indent=2)+'\n')
        print(run.name, report)


if __name__ == '__main__':
    main()
