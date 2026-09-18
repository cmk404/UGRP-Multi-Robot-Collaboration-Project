"""Audited task/issued-command context. No measured robot state belongs here."""
import math

AXES = ('forward', 'left', 'turn')
SCALES = (.15, .15, .15)
CONTEXT_SIZE = 8

def context(goal, route, slot, previous):
    if route not in ('north', 'south') or slot not in ('r1', 'r3'):
        raise ValueError('unknown authored route or model slot')
    if len(goal) != 2 or len(previous) != 3:
        raise ValueError('goal and issued command dimensions')
    value = [goal[0]/3, goal[1]/3, float(route=='north'), float(route=='south'),
             float(slot=='r3')] + [v/s for v,s in zip(previous,SCALES)]
    if not all(math.isfinite(v) and abs(v)<=2 for v in value):
        raise ValueError('finite bounded authored/issued inputs required')
    return value

def decode(values):
    if len(values)!=4 or not all(math.isfinite(v) for v in values):
        raise ValueError('four finite policy outputs required')
    actions={k: max(-s,min(s,float(v)*s)) for k,v,s in zip(AXES,values,SCALES)}
    score=max(0.,min(1.,float(values[3])))
    # This threshold is fixed before the final evaluation; no truth gate.
    done=score>=.65
    if done: actions=dict.fromkeys(AXES,0.)
    return {'action':actions,'done':done,'stop_score':score}
