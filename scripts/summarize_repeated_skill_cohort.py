"""Audit-only counts for a frozen Jev/Gemini/rule protocol and result snapshot."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def summarize(root):
    root = Path(root)
    index = json.loads((root/'index.json').read_text())
    templates = json.loads((root/'phase-templates.json').read_text())
    if index['source_sha'] != templates['actor_source_sha']:
        raise ValueError('snapshot actor source mismatch')
    jobs = {}
    for phase, template in templates['phases'].items():
        for job in template['protocol']['jobs']:
            key = (phase, job['trial_id'])
            if key in jobs:
                raise ValueError('duplicate planned trial')
            jobs[key] = job
    results = {}
    for row in index['rows']:
        key = (row['phase'], row['trial_id'])
        if key not in jobs or key in results:
            raise ValueError('duplicate or unplanned result')
        path = (root/row['result_file']).resolve()
        if not path.is_relative_to(root.resolve()):
            raise ValueError('result outside snapshot')
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != row['result_sha256']:
            raise ValueError('result hash mismatch')
        result = json.loads(raw)
        if result['source_sha'] != index['source_sha']:
            raise ValueError('result actor source mismatch')
        if type(result.get('success')) is not bool:
            raise ValueError('missing explicit success verdict')
        for field in ('case', 'policy', 'arm', 'repeat', 'trial_id'):
            if result.get(field) != jobs[key][field]:
                raise ValueError('trial identity mismatch: '+field)
        if result.get('partition') != row['phase']:
            raise ValueError('phase mismatch')
        results[key] = result
    groups = []
    for phase, policy, arm in sorted({(phase, j['policy'], j['arm']) for (phase, _), j in jobs.items()}):
        keys = [key for key, j in jobs.items() if key[0] == phase and (j['policy'], j['arm']) == (policy, arm)]
        available = [results[k] for k in keys if k in results]
        failed = [r for r in available if not r['success']]
        def reason(r):
            error = r.get('error')
            message = error.get('message', '') if isinstance(error, dict) else str(error or '')
            if message in ('model_http_error', 'model_transport_error'):
                return message
            if error:
                return 'execution_error'
            return r.get('stop_reason') or 'task_incomplete'
        groups.append({'phase': phase, 'policy': policy, 'arm': arm,
            'planned': len(keys), 'attempted': len(available), 'pending': len(keys)-len(available),
            'successes': len(available)-len(failed), 'failures': len(failed),
            'failure_fraction_attempted': len(failed)/len(available) if available else None,
            'failure_reasons': dict(Counter(map(reason, failed))),
            'by_case': {case: {'planned': len(ks), 'attempted': len(rs),
                              'successes': sum(r['success'] for r in rs),
                              'failures': sum(not r['success'] for r in rs)}
                        for case in sorted({jobs[k]['case'] for k in keys})
                        for ks in [[k for k in keys if jobs[k]['case'] == case]]
                        for rs in [[results[k] for k in ks if k in results]]}})
    return {'source_sha': index['source_sha'], 'snapshot_unix': index['snapshot_unix'],
            'planned': len(jobs), 'attempted': len(results), 'pending': len(jobs)-len(results),
            'complete': len(results) == len(jobs), 'groups': groups,
            'scope': 'RGB approach/route/recovery only, not grasp/carry. Phase, policy and arm have separate denominators. All failed attempts retained; pending is not failure. Partial unmatched groups do not establish a model ranking. Repeated fixed scenes are not independent new environments.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = summarize(args.snapshot)
    with args.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps({k: report[k] for k in ('complete', 'planned', 'attempted', 'pending')}))


if __name__ == '__main__':
    main()
