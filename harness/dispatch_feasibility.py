"""Executable-plan feedback from authored geometry and allowed current RGB."""
import copy,math
import numpy as np
from harness.camera_goal_transport import decode
from harness.dispatch_skill_binding import SkillBindings,beam_feature
from harness.dispatch_navigation_map import navigation_map,solo_gate
from harness.dispatch_plan_guidance import LEGACY_FEEDBACK
from harness.dispatch_pair_navigation import plan_placement_route
from harness.known_map_navigation import pixel_to_world
from harness.three_robot_plan import digest


def inspect_pickup_approach(bindings,top_rgb,identity,reference_top):
    """The existing coarse skill aligns laterally, then drives forward.

    It cannot park or route around a robot in that approach. Reject that
    allocation before executing; do not silently substitute participants.
    """
    from harness.dispatch_skill_binding import PairCoarsePixels
    observer=PairCoarsePixels(identity,bindings,reference_top)
    frame=decode(top_rgb);h,w=frame.shape[:2]
    radius=.28  # Combined conservative half widths of two chassis.
    occupied={rid:pixel_to_world(np.array(value['claim']['center'])*[w,h],frame.shape,
                                bindings.static_map['top_camera']) for rid,value in identity.items()}
    checks={}
    for slot,rid in bindings.pair.items():
        d=observer.decide(top_rgb,slot)
        if not d['ok']:
            checks[rid]={'feasible':False,'reason':'initial coarse RGB pose unresolved'};continue
        start=np.array(d['wheel_center_px'])
        error=np.array(d['image_error'])*[w,h]
        elbow=start+np.array([0.,error[1]])
        end=elbow+np.array([max(0.,error[0]-.065*w),0.])
        points=[np.array(pixel_to_world(p,frame.shape,bindings.static_map['top_camera']))
                for p in (start,elbow,end)]
        blocked=[]
        for other,point in occupied.items():
            if other==rid:continue
            for a,b in zip(points,points[1:]):
                direction=b-a
                t=float(np.clip(np.dot(np.array(point)-a,direction)/max(1e-12,np.dot(direction,direction)),0.,1.))
                if np.linalg.norm(np.array(point)-(a+t*direction))<radius:
                    blocked.append(other);break
        checks[rid]={'feasible':not blocked,'blocking_robots':blocked,
            'coarse_path_m':[p.tolist() for p in points],
            'reason':'existing coarse path clear' if not blocked else 'coarse skill cannot pass or relocate an occupied robot'}
    return {'feasible':all(r['feasible'] for r in checks.values()),'robots':checks,
        'source':'current TOP RGB, prior own-motion identity and authored chassis clearance; no live poses',
        'scope':'conservative start admission; later vacating an approach is not an implemented pickup skill'}


def inspect_routes(committed,static_map,top_rgb,identity=None,reference_top=None):
    frame=decode(top_rgb);feature=beam_feature(top_rgb,hue_upper=35)
    center=pixel_to_world(np.array(feature['center'])*feature['image_size'],frame.shape,static_map['top_camera'])
    endpoints=np.array(feature['endpoints'])*feature['image_size']
    axis=endpoints[1]-endpoints[0]
    heading=(math.atan2(-axis[1],axis[0])+math.pi/2+math.pi/2)%math.pi-math.pi/2
    bindings=SkillBindings(committed,static_map)
    pickup=(inspect_pickup_approach(bindings,top_rgb,identity,reference_top)
            if identity is not None and reference_top is not None else None)
    box_first=bindings.tasks['box']['id'] in bindings.tasks['beam']['after']
    other=None
    if bindings.cluttered and identity is not None:
        from harness.dispatch_yield import WheelObserver
        claim=identity[bindings.solo]['claim']
        if not claim['valid']:raise RuntimeError('solo identity unresolved for occupied-space planning')
        hint=np.array(claim['center'])*[frame.shape[1]-1,frame.shape[0]-1]
        other=WheelObserver(static_map,hint).observe(top_rgb)
    routes={}
    for name in ('north','south'):
        value=copy.deepcopy(committed)
        next(t for t in value['plan']['tasks'] if t['object']=='beam')['route']=name
        value['plan_hash']=digest(value['plan'])
        bindings=SkillBindings(value,static_map)
        try:
            bindings.check_route()
            if bindings.cluttered:
                data=navigation_map(bindings,top_rgb,other_robot_center_px=other['center_px'] if other else None,
                    planned_box_completion=box_first)
                route=plan_placement_route([*center,heading],[*data['goal']['center_m'],heading],data)
                future=navigation_map(bindings,top_rgb,planned_box_completion=True)
                future_route=plan_placement_route([*center,heading],[*future['goal']['center_m'],heading],future)
                routes[name]={'feasible':route is not None,'route':route,
                    'reason':'swept footprint path found' if route else 'no swept footprint path in authored map plus RGB obstacles',
                    'observed_barriers':[o for o in data['obstacles'] if o['id'].startswith('rgb_')],
                    'after_box_delivery_and_yield_feasible':future_route is not None,
                    'conditional_on_box_delivery_and_yield':box_first}
            else:routes[name]={'feasible':True,'reason':'open-map translation adapter; current RGB guards still required'}
        except RuntimeError as error:routes[name]={'feasible':False,'reason':str(error)}
    box_routes={}
    for name in ('north','south'):
        try:box_routes[name]={'feasible':True,'gate_m':solo_gate(static_map,name,top_rgb)}
        except RuntimeError as error:box_routes[name]={'feasible':False,'reason':str(error)}
    box_chosen=next(t['route'] for t in committed['plan']['tasks'] if t['object']=='box')
    chosen=next(t['route'] for t in committed['plan']['tasks'] if t['object']=='beam')
    return {'feasible':routes[chosen]['feasible'] and box_routes[box_chosen]['feasible'] and (pickup is None or pickup['feasible']),
        'chosen_beam_route':chosen,'beam_routes':routes,'chosen_box_route':box_chosen,'box_routes':box_routes,
        'other_robot_rgb_observation':other,
        'pickup_approach':pickup,
        'source':'authored static geometry + current TOP RGB + declared loaded footprint; not physical passage proof',
        'plan_hash':committed['plan_hash'],'input_sha256':__import__('hashlib').sha256(top_rgb).hexdigest()}


def negotiate_executable(team,frames,history,task,static_map,sim_time,*,max_rounds=8,max_replans=2,max_tokens=1000000,live_replan=False,identity=None,reference_top=None,feedback_instruction=None):
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
        report=inspect_routes(team.agreement.committed,static_map,frames['r1']['top_bytes'],identity,reference_top)
        write(team.output.parent/f'plan-feasibility-{attempt}.json',report)
        if report['feasible']:return report
        rejected=copy.deepcopy(team.agreement.committed)
        team.event('REPLAN_REQUIRED',sim_time,reason=report,stopped_before_execution=True)
        team.agreement.invalidate('RGB/map capability rejected the committed plan')
        team.save()
        if live_replan and team.mode=='fixture':
            team.mode='llm';team.plan_fixture=None
            team.event('FIXTURE_TO_LIVE_REPLAN_DIAGNOSTIC',sim_time)
        task['execution_feedback']={'rejected_plan':rejected,'capability_result':report,
            'instruction':feedback_instruction or LEGACY_FEEDBACK}
    raise RuntimeError('no executable unanimously agreed plan within replan budget')
