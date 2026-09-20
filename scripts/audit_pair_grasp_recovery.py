"""Replay recovery RGB inputs and check commands against issued arm history."""
import json
from harness.pair_navigation import ROBOTS, authorize_pair
from harness.pair_grasp_spacing import PairGraspSpacing, HOLD_DT
from harness.pair_carry_sync import PairCarrySync
from scripts.audit_pair_carry_sync import _rgb,_same,_safe_file
from scripts.audit_camera_grasp_student import audit as audit_grasp
from scripts.pair_grasp_recovery import slip_requested

def audit_recoveries(root, report, models, seen):
    if not report.get('recoveries'):return []
    trace=json.loads(_safe_file(root,'execution-trace.json').read_text())
    previous=json.loads(_safe_file(root,'grasp-result.json').read_text())
    skill=json.loads((models/'student-skill.json').read_text())
    output=[];previous_trigger=-1
    for index,rec in enumerate(report.get('recoveries',[]),1):
        if index>2 or rec['index']!=index or rec['regrasp_requested']!=(index==1):raise ValueError('unbounded recovery')
        trigger=rec['trigger_index']
        if trigger<=previous_trigger or trigger>=len(report['steps']):raise ValueError('invalid recovery sequence')
        row=report['steps'][trigger];previous_trigger=trigger
        if row['permission']['phase']=='GO' or not slip_requested(row['decisions']):raise ValueError('recovery without RGB slip and common stop')
        if any(abs(a[k])>0 for a in row['issued_actions'].values() for k in ('forward','left','turn')):raise ValueError('recovery started while driving')
        chunk=trace[rec['trace_start']:rec['trace_end']]
        if len(chunk)<3 or any(r.get('recovery_index')!=index for r in chunk):raise ValueError('recovery trace ownership mismatch')
        pre={r:{ch:previous['preclose_issued_commands'][r][ch] for ch in ('3','4','5')} for r in ROBOTS}
        lifted={r:{ch:max(500,min(2500,pre[r][ch]+skill['lift_delta_pulses'][r][ch])) for ch in pre[r]} for r in ROBOTS}
        expected=[('place_lower',pre,.70,.25),('place_open',{r:{'1':2000} for r in ROBOTS},.65,.50),('place_retract',lifted,.70,.25)]
        for actual,(stage,targets,duration,settle) in zip(chunk,expected):
            if actual['stage']!=stage or not _same(actual['command'],{'targets':targets,'duration_s':duration,'settle_s':settle}):raise ValueError('recovery place changed from issued commands')
        if rec['regrasp_requested']:
            grasp=audit_grasp(root,models,report_name=rec['grasp_report'])
            if not grasp['ok']:raise ValueError('recovery grasp input audit failed')
            previous=json.loads(_safe_file(root,rec['grasp_report']).read_text())
            close=[x for x in chunk if x['stage']=='grasp_close']
            if len(close)!=1 or close[0]['command']['targets']!={r:{'1':1600} for r in ROBOTS}:raise ValueError('undeclared recovery close')
            if close[0]['command']['duration_s']!=skill['close_duration_s'] or close[0]['command']['settle_s']!=skill['close_settle_s']:raise ValueError('recovery close timing changed')
            actors={r:PairGraspSpacing(report['map'],r) for r in ROBOTS}
            sync=PairCarrySync(report['map']['map_id']+f'-recovery-{index}-grasp-spacing')
            for i,step in enumerate(rec.get('spacing_steps',[])):
                if set(step)!={'index','images','frame_ids','decisions','permission','issued_actions','sim_time_s','executed'} or step['index']!=i:raise ValueError('recovery spacing schema mismatch')
                decisions={}
                for rid in ROBOTS:
                    if step['frame_ids'][rid] in seen[rid] or set(step['images'][rid])!={'own','top'}:raise ValueError('recovery image freshness/input mismatch')
                    seen[rid].add(step['frame_ids'][rid])
                    decisions[rid]=actors[rid].decide(_rgb(root,step['images'][rid]['own']),_rgb(root,step['images'][rid]['top']))
                if not _same(decisions,step['decisions']):raise ValueError('recovery RGB decision mismatch')
                if step['images']['r1']['top']!=step['images']['r3']['top']:raise ValueError('recovery common camera mismatch')
                permission=authorize_pair(sync,decisions,step['frame_ids'],i,interval_s=HOLD_DT)
                if permission!=step['permission']:raise ValueError('recovery common permission mismatch')
                for rid in ROBOTS:
                    action=dict(decisions[rid]['action'])
                    if permission['phase']!='GO':action.update(forward=0.,left=0.,turn=0.)
                    if not _same(action,step['issued_actions'][rid]):raise ValueError('recovery issued wheel command mismatch')
            if not _same(sync.events,rec.get('spacing_sync_events',[])):raise ValueError('recovery sync event mismatch')
            if rec.get('regrasp_commands_completed') and (len(rec['spacing_steps'])!=104 or not all(a.stable_frames>=5 for a in actors.values())):raise ValueError('false regrasp completion')
            output.append({'index':index,'grasp':grasp,'spacing_rounds':len(rec['spacing_steps'])})
        else:output.append({'index':index,'place_commands_verified':True})
    return output
