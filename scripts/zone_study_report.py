#!/usr/bin/env python3
"""EVALUATION-ONLY report for the Korean-dialogue study.

Reads trial records (see ``docs/zone_study_metrics.md``), writes a Korean
markdown summary, ``metrics.json``, a TensorBoard-friendly ``scalars.json`` and,
with ``--tb-events``, real TensorBoard event files. Nothing is written back into
any robot-facing path and no server configuration is changed.

Usage::

    PYTHONPATH=. .venv-sim-worker-mac/bin/python scripts/zone_study_report.py \
      outputs/zone-study-PILOT-ID/trials \
      --output outputs/zone-study-PILOT-ID/report

Reporting rules kept here on purpose:

* failures, aborts, timeouts and budget exhaustion stay in every denominator;
* no significance test on a pilot cohort — intervals and per-pair tables only;
* the reference ceiling ``R`` is printed apart from the four main conditions.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if __name__ == '__main__':
    sys.path.insert(0, str(ROOT))

from harness import zone_study_eval as ev


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fmt(value, digits=2):
    if value is None:
        return '—'
    if isinstance(value, bool):
        return '예' if value else '아니오'
    if isinstance(value, float):
        return f'{value:.{digits}f}'
    return str(value)


def interval(ci):
    if not ci or ci.get('low') is None:
        return '—'
    return f'[{ci["low"]:.3f}, {ci["high"]:.3f}]'


def condition_table(summary):
    rows = ['| 조건 | 시행 | 성공 | 성공률 | PAR makespan(SIM초) | 성공 makespan | 배송률 | 발화 비용(초) | idle(로봇초) | 충돌 | 교착 | 재계획 | 호출 | 토큰 |',
            '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for condition, row in summary['conditions'].items():
        m = row['metrics']
        rows.append('| {label} | {t} | {s} | {sr} | {par} | {ok} | {dr} | {talk} | {idle} | {cf} | {dl} | {rp} | {mc} | {tok} |'.format(
            label=row['label_ko'], t=row['trials'], s=row['successes'],
            sr=fmt(row['success_rate'], 3), par=fmt(m['par_makespan_sim_s'], 1),
            ok=fmt(m['makespan_success_only_s'], 1), dr=fmt(row['cohort_delivery_rate'], 3),
            talk=fmt(m['talk_sim_cost_s'], 1), idle=fmt(m['idle_robot_s'], 1),
            cf=fmt(m['conflicts'], 1), dl=fmt(m['deadlocks'], 1), rp=fmt(m['replans'], 1),
            mc=fmt(m['model_calls'], 1), tok=fmt(m['tokens_total'], 0)))
    return rows


def dialogue_table(summary):
    rows = ['| 조건 | 발화 | 전달 edge | 한국어 준수 | 침묵 | 코드전환 | ID 손상 | 사실 | 거짓 | 확인불가 | 사실성 | 채널 위반 |',
            '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for condition, row in summary['conditions'].items():
        d = row['dialogue']
        rows.append('| {label} | {u} | {e} | {k} | {sil} | {cs} | {idi} | {t} | {f} | {un} | {ts} | {viol} |'.format(
            label=row['label_ko'], u=d['utterances'], e=d['delivery_edges'],
            k=fmt(d['korean_share'], 3), sil=d['silent_messages'],
            cs=d['code_switch_messages'], idi=d['id_issue_messages'],
            t=d['claims_true'], f=d['claims_false'], un=d['claims_unverifiable'],
            ts=fmt(d['truthful_share'], 3), viol=row['boundary_violation_trials']))
    return rows


def act_table(summary):
    kinds = sorted({k for row in summary['conditions'].values() for k in row['dialogue']['acts_coarse']})
    if not kinds:
        return ['행위 유형으로 분류할 발화가 없다.']
    rows = ['| 조건 | ' + ' | '.join(kinds) + ' |', '|---|' + '---:|' * len(kinds)]
    for row in summary['conditions'].values():
        counts = row['dialogue']['acts_coarse']
        rows.append('| ' + row['label_ko'] + ' | '
                    + ' | '.join(str(counts.get(k, 0)) for k in kinds) + ' |')
    return rows


def influence_table(summary):
    rows = ['| 조건 | 결정 변경 | 직전 수신 발화 있음 | 비율 | 선행 발화 행위 |',
            '|---|---:|---:|---:|---|']
    for row in summary['conditions'].values():
        d = row['dialogue']
        acts = ', '.join(f'{k}={v}' for k, v in sorted(d['preceding_acts'].items())) or '—'
        share = None if not d['decision_changes'] else d['changes_with_prior_inbound'] / d['decision_changes']
        rows.append(f'| {row["label_ko"]} | {d["decision_changes"]} | '
                    f'{d["changes_with_prior_inbound"]} | {fmt(share, 3)} | {acts} |')
    return rows


def comparison_section(comparisons):
    if not comparisons:
        return ['짝지은 seed가 없어 비교를 만들지 않았다.']
    rows = []
    order = {m: i for i, m in enumerate(ev.COMPARISON_METRICS)}
    grouped = sorted(comparisons, key=lambda c: (order.get(c['metric'], 99), c['metric'],
                                                 c['baseline'], c['variant']))
    current = None
    for c in grouped:
        if c['metric'] != current:
            current = c['metric']
            rows += ['', f'**{current}**', '',
                     '| 기준 | 비교 | 짝 | 기준 평균 | 비교 평균 | 차이 | 95% 부트스트랩 구간 | dz | rank-biserial | 비교>기준 |',
                     '|---|---|---:|---:|---:|---:|---|---:|---:|---:|']
        rows.append('| {b} | {v} | {n} | {bm} | {vm} | {d} | {ci} | {dz} | {rb} | {pos}/{n} |'.format(
            b=ev.CONDITION_LABELS_KO[c['baseline']], v=ev.CONDITION_LABELS_KO[c['variant']],
            n=c['n_pairs'], bm=fmt(c['baseline_mean'], 3), vm=fmt(c['variant_mean'], 3),
            d=fmt(c['mean_diff'], 3), ci=interval(c['diff_ci']),
            dz=fmt(c['cohens_dz'], 3), rb=fmt(c['rank_biserial'], 3),
            pos=c['pairs_variant_higher']))
    small = sorted({c['n_pairs'] for c in comparisons if c['small_sample']})
    if small:
        rows += ['', f'짝 수가 {ev.SMALL_SAMPLE_PAIRS}개 미만인 비교가 있다(짝 {small}). '
                     '유의성 검정을 하지 않고 구간과 짝별 표만 읽는다. '
                     '차이 부호는 `비교 − 기준`이며 짝별 값은 `metrics.json`에 있다.']
    return rows


def boundary_section(summary):
    rows, main, reference = [], [], []
    for row in summary['per_trial']:
        b = row['boundary']
        if not (b['input_leaks'] or b['forbidden_grounds'] or b['channel_violations']):
            continue
        target = reference if b['condition'] == ev.REFERENCE_CONDITION else main
        target.append((row['efficiency']['trial_id'], b))
    if not main:
        rows.append('주 4조건의 모든 시행에서 금지 입력 key·금지 근거 인용·채널 위반이 없었다.')
    else:
        rows.append('| 시행 | 금지 입력 key | 금지 근거 | 채널 위반 |')
        rows.append('|---|---|---|---:|')
        for trial_id, b in main:
            keys = sorted({k for r in b['input_leaks'] for k in r['forbidden_input_keys']})
            grounds = sorted({g for r in b['forbidden_grounds'] for g in r['forbidden_grounds']})
            rows.append(f'| {trial_id} | {", ".join(keys) or "—"} | '
                        f'{", ".join(grounds) or "—"} | {len(b["channel_violations"])} |')
        rows.append('')
        rows.append('**평가 자료가 로봇 입력으로 흘러간 시행이 있다. '
                    '해당 코호트를 통신 효과 근거로 쓰지 않는다.**')
    if reference:
        keys = sorted({k for _, b in reference for r in b['input_leaks']
                       for k in r['forbidden_input_keys']})
        rows.append('')
        rows.append(f'R 조건 {len(reference)}개 시행은 설계상 전지적 참조 상한이며 '
                    f'추가 입력({", ".join(keys) or "—"})을 받는다. '
                    '주 조건 경계 판정에 합산하지 않는다.')
    return rows


def markdown(summary, comparisons, sources, generated_at):
    prov = summary['provenance']
    lines = [
        '# 한국어 로봇 대화 효율 연구 — 평가 요약',
        '',
        f'- 생성: {generated_at} (평가 전용 사후 분석)',
        f'- 시행 수: {summary["trials"]}, 시나리오: {", ".join(map(str, summary["scenarios"])) or "—"}',
        f'- 실패 가중치(PAR): 비성공 시행에 horizon × {summary["penalty_factor"]}를 부과한다. '
        '빠른 실패가 느린 성공보다 좋게 보이지 않는다.',
        f'- 선행 발화 탐색 창: {summary["lookback_s"]}초 (연관이며 인과가 아니다)',
        f'- 실행 번들 단일 여부: {fmt(prov["single_bundle"])}'
        + (f' — 혼재 필드: {", ".join(prov["mixed_fields"])}' if prov['mixed_fields'] else ''),
        '',
        '이 문서는 저장된 평가 로그만 읽는다. 시뮬레이션·모델 호출을 하지 않았고, '
        '정답·TOP·심판 판정은 로봇 입력으로 되돌리지 않는다.',
        '',
        '## 1. 조건별 효율',
        '',
        *condition_table(summary),
        '',
        'PAR makespan은 실패·중단·시간 초과·예산 소진을 분모에 유지한 값이다. '
        '성공 makespan 열은 성공한 시행만의 평균이므로 단독으로 조건을 비교하지 않는다.',
        '',
        '## 2. 조건별 대화 지표',
        '',
        *dialogue_table(summary),
        '',
        '한국어 준수는 literal ID·enum을 제외한 한글 비율이 '
        f'{ev.KOREAN_RATIO_THRESHOLD} 이상인 발화의 비율이다. 침묵은 준수 성공으로 세지 않는다. '
        '사실성은 발화 시각의 평가 로그와 대조한 결과이며 확인불가는 실패로 바꾸지 않는다.',
        '',
        '### 행위 유형',
        '',
        *act_table(summary),
        '',
        'PR 172(병합)의 세부 규칙 라벨을 재사용하고 지휘 조건용 `order`만 추가했다. '
        '대응 규칙은 `docs/zone_study_metrics.md`에 있다.',
        '',
        '### 결정 변경 직전 발화',
        '',
        *influence_table(summary),
        '',
        '## 3. 짝지은 seed 비교',
        '',
        *comparison_section(comparisons),
        '',
        '## 4. 입력 경계 감사',
        '',
        *boundary_section(summary),
        '',
        '## 5. 원본',
        '',
        '| 파일 | SHA-256 |',
        '|---|---|',
    ]
    for path, digest in sources:
        lines.append(f'| `{path}` | `{digest}` |')
    lines += [
        '',
        '## 6. 남은 검증',
        '',
        '- 이 보고서는 지표 계산의 정확성만 확인한다. 조건 간 우열은 사전 고정한 코호트를 '
        '실제로 실행한 뒤에만 주장한다.',
        '- 로그 schema는 Package A(`harness/zone_study_contract.py`)가 소유한다. '
        f'`{ev.TRIAL_SCHEMA}`는 A의 호출·메시지·행동 기록을 담고 A가 검증한다. '
        f'과거 파일럿 로그를 위해 `{ev.PROVISIONAL_SCHEMA}`도 계속 읽는다.',
        '',
    ]
    return '\n'.join(lines)


def write_events(scalars, directory, at):
    """Optional: real TensorBoard event files, one run per entry. No server change."""
    from tensorboard.compat.proto.event_pb2 import Event
    from tensorboard.compat.proto.summary_pb2 import Summary
    from tensorboard.plugins.hparams import api_pb2, metadata, plugin_data_pb2
    from tensorboard.summary.writer.event_file_writer import EventFileWriter
    from tensorboard.util.tensor_util import make_tensor_proto

    directory = Path(directory)
    written = 0
    for run in scalars['runs']:
        path = directory / run['run']
        path.mkdir(parents=True, exist_ok=True)
        writer = EventFileWriter(str(path), max_queue_size=50, flush_secs=5)

        def add(summary, step=0):
            writer.add_event(Event(wall_time=at, step=step, summary=summary))

        start = plugin_data_pb2.SessionStartInfo(start_time_secs=at)
        for key, value in run['hparams'].items():
            start.hparams[key].string_value = str(value)
        experiment = api_pb2.Experiment(
            hparam_infos=[api_pb2.HParamInfo(name=k, type=api_pb2.DATA_TYPE_STRING)
                          for k in run['hparams']],
            metric_infos=[api_pb2.MetricInfo(name=api_pb2.MetricName(tag=t))
                          for t in run['scalars']],
            time_created_secs=at)
        # STATUS_SUCCESS means the export finished, not that the robots succeeded.
        end = plugin_data_pb2.SessionEndInfo(status=api_pb2.STATUS_SUCCESS, end_time_secs=at)
        for tag, data in ((metadata.EXPERIMENT_TAG, plugin_data_pb2.HParamsPluginData(experiment=experiment)),
                          (metadata.SESSION_START_INFO_TAG, plugin_data_pb2.HParamsPluginData(session_start_info=start)),
                          (metadata.SESSION_END_INFO_TAG, plugin_data_pb2.HParamsPluginData(session_end_info=end))):
            add(Summary(value=[Summary.Value(tag=tag, tensor=make_tensor_proto([], dtype='float32'),
                                             metadata=metadata.create_summary_metadata(data))]))
        for tag, value in run['scalars'].items():
            add(Summary(value=[Summary.Value(tag=tag, simple_value=float(value))]), run['step'])
            written += 1
        writer.close()
    return {'runs': len(scalars['runs']), 'scalars': written, 'logdir': str(directory)}


def build(paths, output, penalty_factor=ev.DEFAULT_PENALTY_FACTOR,
          lookback_s=ev.DEFAULT_LOOKBACK_S, resamples=10000, seed=0,
          include_reference=False, tb_events=None, now=None):
    trials = ev.load_trials(paths)
    if not trials:
        raise ev.TrialError('no trial records found')
    summary = ev.summarise(trials, penalty_factor=penalty_factor, lookback_s=lookback_s)
    comparisons = ev.compare_all(trials, include_reference=include_reference,
                                 resamples=resamples, seed=seed, penalty_factor=penalty_factor)
    scalars = ev.scalar_export(summary)
    sources = sorted((t['source_path'], sha256_file(t['source_path'])) for t in trials)
    at = now if now is not None else time.time()
    generated_at = time.strftime('%Y-%m-%dT%H:%M:%S%z', time.localtime(at))

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = markdown(summary, comparisons, sources, generated_at)
    (output / 'summary.md').write_text(report)
    (output / 'metrics.json').write_text(json.dumps(
        {'summary': summary, 'comparisons': comparisons,
         'sources': [{'path': p, 'sha256': d} for p, d in sources],
         'generated_at': generated_at},
        ensure_ascii=False, indent=2, sort_keys=False))
    (output / 'scalars.json').write_text(json.dumps(scalars, ensure_ascii=False, indent=2))
    events = None
    if tb_events:
        events = write_events(scalars, tb_events, at)
        (output / 'tensorboard.json').write_text(json.dumps(events, ensure_ascii=False, indent=2))
    return {'output': str(output), 'trials': len(trials), 'comparisons': len(comparisons),
            'runs': len(scalars['runs']), 'events': events,
            'boundary_clean_trials': sum(r['boundary']['clean'] for r in summary['per_trial'])}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('trials', nargs='+', help='trial record JSON files or directories')
    parser.add_argument('--output', required=True, help='report output directory')
    parser.add_argument('--penalty-factor', type=float, default=ev.DEFAULT_PENALTY_FACTOR,
                        help='PAR multiplier on the SIM horizon for non-success trials (>=1)')
    parser.add_argument('--lookback-s', type=float, default=ev.DEFAULT_LOOKBACK_S,
                        help='window before a decision change for preceding utterances')
    parser.add_argument('--bootstrap-resamples', type=int, default=10000)
    parser.add_argument('--bootstrap-seed', type=int, default=0,
                        help='fixed RNG seed so intervals are reproducible')
    parser.add_argument('--include-reference', action='store_true',
                        help='also compare the all-seeing reference ceiling R')
    parser.add_argument('--tb-events', help='also write TensorBoard event files here')
    args = parser.parse_args(argv)
    result = build(args.trials, args.output, penalty_factor=args.penalty_factor,
                   lookback_s=args.lookback_s, resamples=args.bootstrap_resamples,
                   seed=args.bootstrap_seed, include_reference=args.include_reference,
                   tb_events=args.tb_events)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
