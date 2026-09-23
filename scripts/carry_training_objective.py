"""Termination-aware sampling and development selection, without test inputs."""
import math
from scripts.audit_carry_termination import audit


def sampling_groups(rows,objective='episode'):
    if objective not in ('legacy','episode'):raise ValueError('unknown termination objective')
    if objective=='legacy':
        return [3 if r['done'] else max(range(3),key=lambda k:abs(r['action'][k])) for r in rows]
    ends={}
    for row in rows:
        root,slot,index=row['id'].rsplit(':',2)
        if row['done']:ends[(root,slot)]=int(index)
    groups=[]
    for row in rows:
        root,slot,index=row['id'].rsplit(':',2)
        if row['done']:group=3
        elif objective=='episode' and 0<ends[(root,slot)]-int(index)<=8:
            # The last 1.6s (8 x 0.2s) remains NOT done. Explicitly sample
            # these hard negatives rather than dilute them in whole routes.
            group=4
        else:group=max(range(3),key=lambda k:abs(row['action'][k]))
        groups.append(group)
    return groups


def selection(rows,predictions,mean_motion_mae,legacy_score,objective='episode'):
    if len(rows)!=len(predictions):raise ValueError('prediction count mismatch')
    if objective not in ('legacy','episode'):raise ValueError('unknown termination objective')
    if objective=='legacy':
        if not math.isfinite(legacy_score):raise ValueError('nonfinite selection metric')
        return {'selection_score':legacy_score,'termination_objective':'legacy'}
    samples=[{'id':r['id'],'target':r['action'],'prediction':p} for r,p in zip(rows,predictions)]
    result=audit(samples)
    premature=result['premature_pair_hold_episodes']/result['episodes']
    missed=result['missed_terminal_episodes']/result['episodes']
    score=premature+missed+mean_motion_mae if objective=='episode' else legacy_score
    if not math.isfinite(score):raise ValueError('nonfinite selection metric')
    return {'selection_score':score,'termination_objective':objective,
            'premature_hold_episode_fraction':premature,'missed_terminal_episode_fraction':missed,
            'offline_termination_pass':result['offline_termination_pass']}
