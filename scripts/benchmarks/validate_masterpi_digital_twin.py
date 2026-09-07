#!/usr/bin/env python3
"""Hard promotion gate for the calibrated MasterPi task digital twin.

No SIM-generated row is admissible evidence. --promote is the only supported
path that sets manifest.validated=true, and it succeeds only after independent
physical hold-outs meet every acceptance threshold.
"""
from __future__ import annotations
import sys
import argparse,datetime as dt,json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    from scripts.benchmarks.masterpi_calibration_common import CAL_DIR,MANIFEST,finite,load_jsonl,load_manifest,real_measurement_source,save_manifest,sha256_file
except ModuleNotFoundError:
    from masterpi_calibration_common import CAL_DIR,MANIFEST,finite,load_jsonl,load_manifest,real_measurement_source,save_manifest,sha256_file
from sim.calibration_schema import REQUIRED_VALIDATED_PARAMETERS

DEFAULTS={
 'static':CAL_DIR/'static_measurements.json','chassis':CAL_DIR/'chassis_trials.jsonl',
 'servo':CAL_DIR/'servo_trials.jsonl','hand_eye':CAL_DIR/'hand_eye_trials.jsonl','task':CAL_DIR/'task_trials.jsonl',
}

def _physical_rows(path, required_numeric=(), require_bool=None):
    rows=load_jsonl(path); done=[]; bad=[]
    for r in rows:
        complete=all(finite(r.get(k)) for k in required_numeric)
        if require_bool is not None: complete=complete and isinstance(r.get(require_bool),bool)
        if not complete: continue
        if not real_measurement_source(r.get('measurement_source')): bad.append(str(r.get('trial_id'))); continue
        done.append(r)
    return done,bad

def validate(manifest_path=MANIFEST,static_path=DEFAULTS['static'],chassis_path=DEFAULTS['chassis'],servo_path=DEFAULTS['servo'],hand_eye_path=DEFAULTS['hand_eye'],task_path=DEFAULTS['task']):
    m=load_manifest(manifest_path); reasons=[]; checks={}; p=m.get('parameters',{}); res=m.get('results',{}); acc=m.get('acceptance',{})
    missing=[k for k in REQUIRED_VALIDATED_PARAMETERS if not finite(p.get(k))]
    checks['all_model_parameters_fitted']=not missing
    if missing: reasons.append('missing calibrated parameters: '+','.join(missing))

    try: static=json.loads(static_path.read_text(encoding='utf-8'))
    except Exception as e: static={}; reasons.append(f'static evidence unreadable: {e}')
    static_fields=('wheel_radius_m','wheelbase_m','track_m','block_mass_kg','block_floor_friction')
    static_ok=static.get('hardware_unit')=='ugrp1' and real_measurement_source(static.get('measurement_source')) and all(finite(static.get(k)) for k in static_fields)
    if static_ok:
        static_ok=all(abs(float(static[k])-float(p.get(k)))<1e-9 for k in static_fields)
    checks['static_physical_evidence']=static_ok
    if not static_ok: reasons.append('static geometry/object measurements are missing, non-physical, or not applied to manifest')

    ch,bad=_physical_rows(chassis_path,('dx_m','dy_m','dyaw_deg','peak_speed_mps','stop_distance_m'))
    ch_fit=[r for r in ch if r.get('split')!='holdout']; ch_hold=[r for r in ch if r.get('split')=='holdout']
    checks['chassis_evidence']=(len(ch_fit)>=72 and len(ch_hold)>=36 and not bad)
    if not checks['chassis_evidence']: reasons.append(f'chassis evidence incomplete: fit={len(ch_fit)}/72 holdout={len(ch_hold)}/36 bad_sources={len(bad)}')

    sv,bad=_physical_rows(servo_path,('measured_delta_deg','settle_time_s'))
    sv_fit=[r for r in sv if r.get('split')!='holdout']; sv_hold=[r for r in sv if r.get('split')=='holdout']
    checks['servo_evidence']=(len(sv_fit)>=40 and len(sv_hold)>=20 and not bad and {int(r['servo']) for r in sv_hold}>={3,4,5,6})
    if not checks['servo_evidence']: reasons.append(f'servo evidence incomplete: fit={len(sv_fit)}/40 holdout={len(sv_hold)}/20')

    he,bad=_physical_rows(hand_eye_path,('nx','bottom_ny','true_x_m','true_y_m'))
    he_fit=[r for r in he if r.get('split')!='holdout']; he_hold=[r for r in he if r.get('split')=='holdout']
    checks['hand_eye_evidence']=(len(he_fit)>=12 and len(he_hold)>=12 and not bad)
    if not checks['hand_eye_evidence']: reasons.append(f'hand-eye evidence incomplete: fit={len(he_fit)}/12 holdout={len(he_hold)}/12')

    task,bad=_physical_rows(task_path,('red_x_m','red_y_m','red_yaw_deg'),require_bool='physical_success')
    task_fit=[r for r in task if r.get('split')!='holdout']; task_hold=[r for r in task if r.get('split')=='holdout']
    checks['task_evidence']=(len(task_fit)>=12 and len(task_hold)>=20 and not bad)
    if not checks['task_evidence']: reasons.append(f'task evidence incomplete: fit={len(task_fit)}/12 holdout={len(task_hold)}/20')

    count_expect={'fit_trials':len(ch_fit),'held_out_trials':len(ch_hold),'servo_fit_trials':len(sv_fit),'servo_held_out_trials':len(sv_hold),'hand_eye_fit_trials':len(he_fit),'hand_eye_held_out_trials':len(he_hold),'task_fit_trials':len(task_fit),'task_held_out_trials':len(task_hold)}
    count_ok=True
    for key,value in count_expect.items():
        if int(res.get(key) or 0)!=value: count_ok=False; reasons.append(f'manifest result count {key}={res.get(key)} but evidence has {value}')
    checks['result_counts_match_evidence']=count_ok

    thresholds=(
      ('translation_endpoint_mae_m','held_out_translation_endpoint_mae_m_max'),
      ('yaw_endpoint_mae_deg','held_out_yaw_endpoint_mae_deg_max'),
      ('peak_speed_relative_error','held_out_peak_speed_relative_error_max'),
      ('stop_distance_mae_m','held_out_stop_distance_mae_m_max'),
      ('servo_endpoint_mae_deg','held_out_servo_endpoint_mae_deg_max'),
      ('servo_settle_time_relative_error','held_out_servo_settle_time_relative_error_max'),
      ('hand_eye_position_mae_m','held_out_hand_eye_position_mae_m_max'),
      ('grasp_success_rate_gap','grasp_success_rate_gap_max'),
      ('task_outcome_disagreement_rate','task_outcome_disagreement_rate_max'),
    )
    metrics_ok=True; metric_report={}
    for result_key,accept_key in thresholds:
        value=res.get(result_key); limit=acc.get(accept_key); passed=finite(value) and finite(limit) and float(value)<=float(limit)
        metric_report[result_key]={'value':value,'max':limit,'pass':passed}
        if not passed: metrics_ok=False; reasons.append(f'{result_key}={value} exceeds/misses {accept_key}={limit}')
    checks['all_held_out_metrics_pass']=metrics_ok

    paths={'static':static_path,'chassis':chassis_path,'servo':servo_path,'hand_eye':hand_eye_path,'task':task_path}
    hashes={name:sha256_file(path) for name,path in paths.items() if path.exists()}
    passed=all(checks.values()) and not reasons
    return {'pass':passed,'checks':checks,'metrics':metric_report,'evidence_counts':count_expect,'dataset_sha256':hashes,'reasons':reasons}

def promote(report,manifest_path=MANIFEST):
    if not report['pass']: raise ValueError('digital twin promotion blocked: '+'; '.join(report['reasons']))
    m=load_manifest(manifest_path); m['validated']=True; m['validated_at']=dt.datetime.now(dt.timezone.utc).isoformat()
    m['validation_provenance']={'validator':'scripts/benchmarks/validate_masterpi_digital_twin.py','dataset_sha256':report['dataset_sha256'],'evidence_counts':report['evidence_counts'],'held_out_metrics':{k:v['value'] for k,v in report['metrics'].items()}}
    save_manifest(m,manifest_path)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--manifest',type=Path,default=MANIFEST)
    for name,path in DEFAULTS.items(): ap.add_argument('--'+name.replace('_','-'),type=Path,default=path)
    ap.add_argument('--promote',action='store_true'); a=ap.parse_args()
    report=validate(a.manifest,a.static,a.chassis,a.servo,a.hand_eye,a.task)
    if a.promote:
        if not report['pass']: print(json.dumps(report,ensure_ascii=False,indent=2)); return 2
        promote(report,a.manifest); report['promoted']=True
    print(json.dumps(report,ensure_ascii=False,indent=2)); return 0 if report['pass'] else 1
if __name__=='__main__': raise SystemExit(main())
