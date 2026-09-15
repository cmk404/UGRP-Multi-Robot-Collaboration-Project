"""Output-only recovery scoring. Never an action or transition input."""
from scripts.evaluate_pair_navigation import evaluate_grasp_stability
from harness.pair_navigation import ROBOTS

def evaluate_recoveries(rows, report):
    results=[]
    for rec in report.get('recoveries',[]):
        segment=[r for r in rows if rec['start_sim_time_s']-.001<=r['sim_time_s']<=rec['end_sim_time_s']+.001]
        release=[r for r in segment if r['phase']=='release_hold']
        released=len(release)>=10 and all(r['payload_floor_contact'] and
            all(not r['contacts'][rid][side] for rid in ROBOTS for side in ('left','right')) for r in release[-10:])
        item={'index':rec['index'],'released_on_floor':released,'command_error':rec['error']}
        if rec['regrasp_requested']:
            item['grasp_stability']=evaluate_grasp_stability(segment)
            following=next((r for r in report['steps'] if r['index']>rec['trigger_index']),None)
            last_frames=(rec.get('spacing_steps') or [{}])[-1].get('frame_ids',{})
            fresh=bool(following and following['sim_time_s']>=rec['end_sim_time_s'] and
                following['permission']['phase']=='GO' and all(following['frame_ids'][rid]>last_frames.get(rid,float('inf')) for rid in ROBOTS))
            item['resumed_with_fresh_rgb']=fresh
            item['handled']=released and not rec['error'] and item['grasp_stability']['success'] and fresh
        else:item['handled']=released and not rec['error']
        results.append(item)
    return {'success':bool(results) and all(r['handled'] for r in results) and
            len(results)<=2 and sum(r['regrasp_requested'] for r in report.get('recoveries',[]))<=1,
            'events':results,'scope':'Recovery handling, separate from uninterrupted endurance or task success'}
