"""Privileged recovery demonstrator and separate post-run scoring helpers."""
import math
AXES=('forward','left','turn')
LIMITS={'forward':(-.05,.15),'left':(-.10,.10),'turn':(-.15,.15)}

def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi

def errors(state,goals,rid):
    x,y=state['bases'][rid][:2];gx,gy=goals[rid][:2]
    return {'x':gx-x,'y':gy-y,'yaw':wrap(-state['base_yaw_rad'][rid])}

def bounded(value,maximum,minimum=0.):
    return math.copysign(min(maximum,max(minimum,abs(value))),value) if value else 0.

def command(error):
    """Align heading, then lateral position, then approach/back up; no teleport."""
    ex,ey,angle=(error[k] for k in ('x','y','yaw'))
    if not all(math.isfinite(v) for v in (ex,ey,angle)):raise ValueError('finite errors required')
    a=dict.fromkeys(AXES,0.)
    ready=abs(ex)<=.002 and abs(ey)<=.002 and abs(angle)<=math.radians(.5)
    if abs(angle)>math.radians(.5):a['turn']=bounded(.65*angle,.10,.006)
    elif abs(ey)>.002:
        # World y error transformed into body axes at the actual teacher yaw.
        yaw=-angle;a['forward']=max(-.05,min(.15,math.sin(yaw)*.35*ey))
        a['left']=bounded(math.cos(yaw)*.35*ey,.04,.012)
    elif abs(ex)>.002:a['forward']=bounded(.3*ex,.12 if ex>0 else .045,.005)
    return {'ok':True,'ready':ready,**a,'stop_score':float(ready),'reason':'privileged_recovery_teacher'}

def heldout_region(error):
    """Conservative combination-region screen applied only to training provenance."""
    return abs(error['yaw'])>math.radians(3) and (abs(error['y'])>.008 or error['x']>.33 or error['x']<-.003)

def score_alignment(samples,goals,end_time):
    window=[s for s in samples if end_time-.401<=s['sim_time_s']<=end_time+.0001 and s['phase'].startswith('approach')]
    good=len(window)>=4 and window[-1]['sim_time_s']-window[0]['sim_time_s']>=.299
    maximum={'x':0.,'y':0.,'yaw':0.,'linear_speed':0.,'angular_speed':0.}
    for s in window:
        for rid in goals:
            e=errors(s,goals,rid)
            for k in ('x','y','yaw'):maximum[k]=max(maximum[k],abs(e[k]))
    for a,b in zip(window,window[1:]):
        dt=b['sim_time_s']-a['sim_time_s']
        for rid in goals:
            speed=math.hypot(*(b['bases'][rid][i]-a['bases'][rid][i] for i in (0,1)))/dt
            omega=abs(wrap(b['base_yaw_rad'][rid]-a['base_yaw_rad'][rid]))/dt
            maximum['linear_speed']=max(maximum['linear_speed'],speed);maximum['angular_speed']=max(maximum['angular_speed'],omega)
    good=good and maximum['x']<=.005 and maximum['y']<=.005 and maximum['yaw']<=math.radians(1.) and maximum['linear_speed']<=.005 and maximum['angular_speed']<=.02
    return {'success':bool(good),'samples':len(window),'max_errors':maximum,'tolerance':{'x_m':.005,'y_m':.005,'yaw_deg':1.,'linear_speed_mps':.005,'angular_speed_radps':.02,'window_s':.4}}
