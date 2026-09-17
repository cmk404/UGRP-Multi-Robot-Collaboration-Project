"""Executable-plan feedback from authored geometry and allowed current RGB."""
import copy,math
import numpy as np
from harness.camera_goal_transport import decode
from harness.dispatch_skill_binding import SkillBindings,beam_feature
from harness.dispatch_navigation_map import navigation_map,solo_gate
from harness.dispatch_pair_navigation import plan_route
from harness.known_map_navigation import pixel_to_world
from harness.three_robot_plan import digest


def inspect_routes(committed,static_map,top_rgb):
    frame=decode(top_rgb);feature=beam_feature(top_rgb,hue_upper=35)
    center=pixel_to_world(np.array(feature['center'])*feature['image_size'],frame.shape,static_map['top_camera'])
    endpoints=np.array(feature['endpoints'])*feature['image_size']
    axis=endpoints[1]-endpoints[0]
    heading=(math.atan2(-axis[1],axis[0])+math.pi/2+math.pi/2)%math.pi-math.pi/2
    routes={}
    for name in ('north','south'):
        value=copy.deepcopy(committed)
        next(t for t in value['plan']['tasks'] if t['object']=='beam')['route']=name
        value['plan_hash']=digest(value['plan'])
        bindings=SkillBindings(value,static_map)
        try:
            bindings.check_route()
            if bindings.cluttered:
                data=navigation_map(bindings,top_rgb)
                route=plan_route([*center,heading],[*data['goal']['center_m'],heading],data)
                routes[name]={'feasible':route is not None,'route':route,
                    'reason':'swept footprint path found' if route else 'no swept footprint path in authored map plus RGB obstacles',
                    'observed_barriers':[o for o in data['obstacles'] if o['id'].startswith('rgb_')]}
            else:routes[name]={'feasible':True,'reason':'open-map translation adapter; current RGB guards still required'}
        except RuntimeError as error:routes[name]={'feasible':False,'reason':str(error)}
    box_routes={}
    for name in ('north','south'):
        try:box_routes[name]={'feasible':True,'gate_m':solo_gate(static_map,name,top_rgb)}
        except RuntimeError as error:box_routes[name]={'feasible':False,'reason':str(error)}
    box_chosen=next(t['route'] for t in committed['plan']['tasks'] if t['object']=='box')
    chosen=next(t['route'] for t in committed['plan']['tasks'] if t['object']=='beam')
    return {'feasible':routes[chosen]['feasible'] and box_routes[box_chosen]['feasible'],
        'chosen_beam_route':chosen,'beam_routes':routes,'chosen_box_route':box_chosen,'box_routes':box_routes,
        'source':'authored static geometry + current TOP RGB + declared loaded footprint; not physical passage proof',
        'plan_hash':committed['plan_hash'],'input_sha256':__import__('hashlib').sha256(top_rgb).hexdigest()}


def negotiate_executable(team,frames,history,task,static_map,sim_time,*,max_rounds=8,max_replans=2,max_tokens=1000000):
    """Reject unsupported commits, supply evidence, then require NEW unanimous ACKs.

    Called before motors are authorized. Never repairs routes or role assignments.
    The caller has already held every endpoint; no occupied resource is freed.
    """
    from scripts.three_robot_runtime import write
    for attempt in range(max_replans+1):
        for _ in range(max_rounds):
            if sum((c.get('usage') or {}).get('prompt_tokens',0) for c in team.calls)>=max_tokens:
                raise RuntimeError('planning input token budget exhausted')
            turn=team.agreement.last_turn+1
            committed=team.negotiate(frames,history,turn,sim_time)
            team.rounds[-1]['task_snapshot']=copy.deepcopy(task);team.save()
            if committed:break
        if not team.agreement.committed:raise RuntimeError('no valid unanimous dispatch plan')
        report=inspect_routes(team.agreement.committed,static_map,frames['r1']['top_bytes'])
        write(team.output.parent/f'plan-feasibility-{attempt}.json',report)
        if report['feasible']:return report
        rejected=copy.deepcopy(team.agreement.committed)
        team.event('REPLAN_REQUIRED',sim_time,reason=report,stopped_before_execution=True)
        team.agreement.invalidate('RGB/map feasibility rejected '+report['chosen_beam_route'])
        team.save()
        task['execution_feedback']={'rejected_plan':rejected,'capability_result':report,
            'instruction':'Propose a feasible route and get a NEW exact unanimous agreement. Assignments remain your decision. A map path is not physical success.'}
    raise RuntimeError('no executable unanimously agreed plan within replan budget')
