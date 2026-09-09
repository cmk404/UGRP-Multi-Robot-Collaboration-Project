"""One or three independent RGB actors in one physical arena; truth is referee-only."""
from pathlib import Path
import argparse
import base64
import hashlib
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor

from harness.llm_transport_skill import LLMTransportSkill
from harness.gemini_transport_policy import GeminiTransportPlanner
from harness.gemini_proxy import GeminiProxyCompleter, GeminiProxyError
from harness.inference_recovery import InferenceRecovery, transport_error
from harness.camera_message_bus import CameraMessageBus
from harness.visual_drive_guard import validate_visual_drive
from harness.visual_placement import inspect_placement
from harness.visual_macro_runtime import VisualMacroExecutor
from harness.navigation_events import NavigationEvents, InferenceInputBudget
from harness.inference_accounting import settle_pending, settle_completed


def inspect_actor_placement(actor, wrist, nav):
    """Use the same release continuity for planning and fresh action checks."""
    released = actor.state == 'released'
    release_confirmed = released and actor.box.reason == 'VISUAL_RELEASE_CONFIRMED'
    return inspect_placement(
        wrist, nav, cargo_id=actor.cargo_id, destination_zone=actor.destination_zone,
        stage='released' if released else 'before_release',
        held_identity_confirmed=actor.held or release_confirmed,
        release_commanded=release_confirmed,
    )


def footprint_inside(position, yaw, dimensions, center, half_extents):
    """Offline referee only: include the rotated box's full floor footprint."""
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    radii = ((c*dimensions[0]+s*dimensions[1])/2,
             (s*dimensions[0]+c*dimensions[1])/2)
    return all(abs(position[i]-center[i])+radii[i] <= half_extents[i] for i in (0,1))


def record_command(row, *, commands, current_decision, execution_feedback):
    """Persist one executor event and expose its completed macro to the planner."""
    row = dict(row)
    row['decision_id'] = current_decision.get(row.get('robot_id'))
    execution = row.get('execution')
    if execution is not None:
        execution_feedback[row['robot_id']]['last_macro'] = {
            'decision_id': row['decision_id'], **execution}
    commands.write(json.dumps(row) + '\n')
    commands.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', required=True)
    ap.add_argument('--seed', type=int, default=41)
    ap.add_argument('--robots', type=int, choices=(1,3), default=1)
    ap.add_argument('--seconds', type=float, default=360)
    ap.add_argument('--impratio', type=float, default=10)
    ap.add_argument('--noslip-iterations', type=int, default=0)
    ap.add_argument('--record', action='store_true')
    ap.add_argument('--max-calls', type=int, default=40)
    ap.add_argument('--max-input-tokens', type=int, default=120000,
                    help='reported input limit; conservative estimated preflight is not a hard provider cap')
    ap.add_argument('--input-request-estimate', type=int, default=6000)
    ap.add_argument('--model', default='gemini-3.8-flash')
    ap.add_argument('--reasoning-effort', choices=('none','low','medium','high'), default='none')
    ap.add_argument('--communication', choices=('none','status','natural'), default='none')
    ap.add_argument('--request-timeout', type=float, default=30)
    ap.add_argument('--max-transient-failures', type=int, default=5)
    args = ap.parse_args()
    if (args.max_calls <= 0 or args.max_input_tokens <= 0 or args.input_request_estimate <= 0
            or not math.isfinite(args.seconds) or args.seconds <= 0):
        ap.error('call, input-token and finite simulation budgets must be positive')
    import mujoco
    from sim.multi_masterpi_production import MultiMasterPiProductionV2
    from sim.camera_robot_port import CameraRobotPort
    out=Path(args.output); out.mkdir(parents=True, exist_ok=False)
    source={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for base in ('sim','harness','scripts','calibration')
            for p in sorted(Path(base).rglob('*')) if p.is_file() and p.suffix in ('.py','.xml','.json','.png','.yaml','.yml') and '__pycache__' not in p.parts}
    (out/'source-manifest.json').write_text(json.dumps(source,indent=2))
    (out/'run-config.json').write_text(json.dumps(vars(args),indent=2))
    source_hash=hashlib.sha256(json.dumps(source,sort_keys=True).encode()).hexdigest()
    active=['r1'] if args.robots==1 else ['r1','r2','r3']
    cargo_ids=tuple(f'small_box_0{i+1}' for i in range(args.robots))
    world=MultiMasterPiProductionV2(warehouse_layout='camera_team',seed=args.seed,render=True,
                                   warehouse_cargo_ids=cargo_ids)
    world.model.opt.impratio=args.impratio; world.model.opt.noslip_iterations=args.noslip_iterations
    for zone in 'abc':
        gid=mujoco.mj_name2id(world.model,mujoco.mjtObj.mjOBJ_GEOM,'warehouse_zone_'+zone)
        world.model.geom_group[gid]=0
    initial=world.warehouse_state()
    # This is an operator task assignment of object identity and named region.
    # Geometric goal positions remain exclusively in the referee.
    goals={rid: min(world.warehouse_zones, key=lambda z: math.dist(world.warehouse_spec_by_id[cid].goal_xyz[:2], world.warehouse_zones[z].center_xy)) for rid,cid in zip(active,cargo_ids)}
    ports={rid:CameraRobotPort(world,rid) for rid in active}
    actors={rid:LLMTransportSkill(rid,cid,goals[rid]) for rid,cid in zip(active,cargo_ids)}
    commands=(out/'commands.jsonl').open('w'); control=(out/'control.jsonl').open('w'); truth=(out/'evaluation-only.jsonl').open('w')
    current_decision={rid:None for rid in active}
    execution_feedback={rid:{} for rid in active}
    def command_log(row):
        record_command(row, commands=commands, current_decision=current_decision,
                       execution_feedback=execution_feedback)
    def stop_actor(rid, reason):
        executors[rid].cancel(float(world.data.time), reason)
        ports[rid].stop()
        command_log({'event':'raw_action','robot_id':rid,'time':float(world.data.time),
                     'raw_action':{'kind':'wait'},'stop_reason':reason})
    navigation_events={rid:NavigationEvents() for rid in active}
    event_log=(out/'navigation-events.jsonl').open('w')
    event_steps={rid:0 for rid in active}
    def drive_guard(rid, action, now):
        nav=ports[rid].capture(camera='nav_cam')
        event_steps[rid]+=1
        path=out/'inputs'/rid/f'nav-event-{event_steps[rid]:06d}.jpg'
        path.write_bytes(base64.b64decode(nav['image']))
        check=navigation_events[rid].inspect(nav,action,now,bus.inbox(rid,now),
            executed_drive_s=executors[rid].total_drive_control_s)
        event_log.write(json.dumps({'robot_id':rid,'time':now,'decision_id':current_decision[rid],
            'path':str(path.relative_to(out)),'sha256':nav['sha256'],'result':check})+'\n');event_log.flush()
        return check
    executors={rid:VisualMacroExecutor(ports[rid],log_callback=command_log,
                drive_guard=lambda action,now,rid=rid:drive_guard(rid,action,now)) for rid in active}
    for rid in active: (out/'inputs'/rid).mkdir(parents=True)
    steps={rid:0 for rid in active}; reasons={rid:'TIME_BUDGET' for rid in active}; done=set()
    start=float(world.data.time); deadline=start+args.seconds; next_truth=start
    max_lifts={cid:0. for cid in cargo_ids}; peer_contact_s=0.; obstacle_contact_s=0.; concurrent_motion_s=0.; concurrent_carry_s=0.
    previous_cargo=None; previous_time=None; samples=[]; video=None; error=None
    planners={rid:GeminiTransportPlanner(rid,GeminiProxyCompleter(model=args.model,max_tokens=800,
                timeout=args.request_timeout,reasoning_effort=args.reasoning_effort), communication_mode=args.communication) for rid in active}
    pool=ThreadPoolExecutor(max_workers=len(active),thread_name_prefix='gemini-robot')
    bus=CameraMessageBus(active,args.communication)
    message_log=(out/'messages.jsonl').open('w')
    recovery={rid:InferenceRecovery(args.max_transient_failures) for rid in active}
    input_budgets={rid:InferenceInputBudget(args.max_input_tokens,
        initial_request_estimate=args.input_request_estimate) for rid in active}
    pending={};memory={rid:[] for rid in active};calls={rid:0 for rid in active};failures={rid:0 for rid in active}
    state_versions={rid:0 for rid in active}
    state_fingerprints={rid:(actors[rid].state,actors[rid].operation,actors[rid].phase) for rid in active}
    def state_version(rid):
        fingerprint=(actors[rid].state,actors[rid].operation,actors[rid].phase)
        if fingerprint != state_fingerprints[rid]:
            state_versions[rid]+=1;state_fingerprints[rid]=fingerprint
        return state_versions[rid]
    llm_log=(out/'llm-decisions.jsonl').open('w')
    next_monitor={rid:start for rid in active}
    monitor_log=(out/'visual-guard.jsonl').open('w')
    wall_start=time.monotonic(); last_wall=time.monotonic(); max_frame_age=8.
    interrupted=False
    try:
        if args.record:
            from scripts.record_visual_team import VisualTeamVideo
            video=VisualTeamVideo(world,out/'motion-1x.mp4',actors)
            video.capture(force=True)
        while world.data.time < deadline and len(done)<len(active):
            now=float(world.data.time)
            for rid in active:
                ex=executors[rid];ex.tick(now)
                if rid in done:
                    if rid in pending:
                        pending[rid]['future'].cancel()
                        row=settle_completed(rid,pending[rid],planners[rid],input_budgets[rid],now=now)
                        if row is not None:
                            llm_log.write(json.dumps(row,ensure_ascii=False)+'\n');llm_log.flush()
                            pending.pop(rid)
                    continue
                if not ex.idle:continue
                actor=actors[rid]
                if ex.last_interruption is not None:
                    memory[rid].append({'feedback':'NAVIGATION_INTERRUPTED:'+ex.last_interruption['reason'],
                                        'skill_state':actor.state})
                    ex.last_interruption=None
                if rid in pending:
                    item=pending[rid];future=item['future'];wrist=item['wrist'];nav=item['nav'];requested_at=item['requested_at']
                    if not future.done():continue
                    pending.pop(rid)
                    decision=None;fresh_guard=None;fresh_placement=None;inference_error=None
                    try:
                        decision=future.result()
                        recovery[rid].success()
                        audit=getattr(planners[rid],'last_audit',{})
                        input_budgets[rid].record(item['call_id'],audit.get('usage'))
                        audit['wall_latency_ms']=getattr(planners[rid].completer,'last_latency_ms',None)
                        audit['requested_reasoning_effort']=planners[rid].completer.reasoning_effort
                        decision_id=item['call_id']+'-decision'
                        if state_version(rid)!=item['state_version']:
                            llm_log.write(json.dumps({'event':'llm_result','robot_id':rid,'time':now,
                                'requested_at':requested_at,'call_id':item['call_id'],'decision_id':decision_id,
                                'decision':decision,'audit':audit,'disposition':'discarded_state_changed',
                                'requested_state_version':item['state_version'],'current_state_version':state_versions[rid]},ensure_ascii=False)+'\n');llm_log.flush()
                            execution_feedback[rid]['last_decision']={
                                'call_id':item['call_id'],'action':decision['action'],
                                'disposition':'discarded_state_changed'}
                            memory[rid].append({'feedback':'STATE_CHANGED_REPLAN','skill_state':actor.state})
                            memory[rid]=memory[rid][-16:]
                            continue
                        if now-requested_at>max_frame_age:raise ValueError('STALE_LLM_FRAME')
                        if decision['action']['kind']=='drive':
                            fresh_nav=ports[rid].capture(camera='nav_cam')
                            guard_path=out/'inputs'/rid/f'guard-call-{item["call_number"]:04d}.jpg'
                            guard_path.write_bytes(base64.b64decode(fresh_nav['image']))
                            guard=validate_visual_drive(fresh_nav,decision['action'])
                            fresh_guard={'path':str(guard_path.relative_to(out)),'sha256':fresh_nav['sha256'],
                                         'frame_id':fresh_nav['frame_id'],'result':guard}
                        else:
                            guard=validate_visual_drive(nav,decision['action'])
                        if not guard['allowed']:raise ValueError('OWN_RGB_DRIVE_BLOCKED:'+guard['reason'])
                        if decision['action']['kind'] in ('release','finish'):
                            current_wrist=ports[rid].capture();current_nav=ports[rid].capture(camera='nav_cam')
                            for label,obs in [('wrist',current_wrist),('nav',current_nav)]:
                                (out/'inputs'/rid/f'placement-call-{item["call_number"]:04d}-{label}.jpg').write_bytes(base64.b64decode(obs['image']))
                            fresh_placement=inspect_actor_placement(actor,current_wrist,current_nav)
                        action=actor.request(decision['action'],placement_evidence=fresh_placement)
                        state_version(rid)
                        current_decision[rid]=decision_id
                        navigation_events[rid].begin_command(decision['action'],item['known_message_ids'])
                        execution_feedback[rid]['last_decision']={
                            'call_id':item['call_id'],'action':decision['action'],'disposition':'accepted'}
                        if action:
                            if action['kind']=='finish':done.add(rid);reasons[rid]=action['reason'];stop_actor(rid,'runner_stop')
                            else:ex.submit(action,wrist,actor.phase,now)
                        actor.last_llm_reason=decision['reason']
                        memory[rid].append({'action':decision['action'],'reason':decision['reason'],'feedback':'accepted','skill_state':actor.state})
                        llm_log.write(json.dumps({'event':'llm_result','robot_id':rid,'time':now,'requested_at':requested_at,
                            'call_id':item['call_id'],'decision_id':decision_id,'decision':decision,'audit':audit,
                            'fresh_drive_guard':fresh_guard,'fresh_placement':fresh_placement,'disposition':'accepted'},ensure_ascii=False)+'\n');llm_log.flush()
                        if decision.get('outgoing_message') is not None:
                            try:
                                sent=bus.publish(rid,decision['outgoing_message'],now=now,observed_at=requested_at,decision_id=decision_id)
                                if sent:
                                    message_log.write(json.dumps(sent,ensure_ascii=False)+'\n');message_log.flush()
                                    actors[rid].last_message=sent
                            except ValueError as message_exc:
                                message_log.write(json.dumps({'event':'rejected_message','robot_id':rid,'decision_id':decision_id,'error':str(message_exc)})+'\n');message_log.flush()
                        print(rid,'GEMINI',calls[rid],actor.state,decision,flush=True)
                        failures[rid]=0
                    except Exception as exc:
                        failure_audit=getattr(planners[rid],'last_audit',{})
                        failure_audit['wall_latency_ms']=getattr(planners[rid].completer,'last_latency_ms',None)
                        failure_audit['requested_reasoning_effort']=planners[rid].completer.reasoning_effort
                        if decision is None:
                            input_budgets[rid].record(item['call_id'],failure_audit.get('usage'))
                        stop_actor(rid,'runner_stop')
                        # A camera veto is valid feedback, not a broken model/API call.
                        expected_veto=str(exc).startswith(('OWN_RGB_DRIVE_BLOCKED:', 'SAFE_INSIDE_', 'CAMERA_INSIDE_', 'STALE_LLM_FRAME'))
                        original_error=transport_error(exc)
                        if original_error is not None:
                            inference_error=recovery[rid].failure(original_error,time.monotonic())
                            inference_error['latency_ms']=original_error.latency_ms
                            memory[rid].append({'inference_recovery':inference_error})
                        failures[rid]=0 if expected_veto else failures[rid]+1
                        if fresh_placement is not None:
                            memory[rid].append({'placement_evidence':fresh_placement})
                        memory[rid].append({'feedback':str(exc)[:300],'skill_state':actor.state})
                        message=str(exc)
                        disposition=('rejected_stale' if message=='STALE_LLM_FRAME' else
                                     'rejected_guard' if message.startswith('OWN_RGB_DRIVE_BLOCKED:') else
                                     'inference_error' if inference_error is not None else
                                     'rejected_state')
                        execution_feedback[rid]['last_decision']={
                            'call_id':item['call_id'],'action':decision['action'] if decision else None,
                            'disposition':disposition,'reason':message[:240]}
                        llm_log.write(json.dumps({'event':'llm_result','robot_id':rid,'time':now,'call_id':item['call_id'],
                            'decision_id':item['call_id']+'-decision','decision':decision,'error':message,
                            'disposition':disposition,'fresh_drive_guard':fresh_guard,'fresh_placement':fresh_placement,
                            'audit':failure_audit,'inference_error':inference_error},ensure_ascii=False)+'\n');llm_log.flush()
                        print(rid,'LLM_ERROR',str(exc),flush=True)
                        if inference_error is not None:
                            if not inference_error['retry_scheduled']:
                                done.add(rid);reasons[rid]='LLM_ERROR:'+inference_error['error_kind']
                        elif failures[rid]>=3:
                            done.add(rid);reasons[rid]='LLM_ERROR:'+str(exc)
                    memory[rid]=memory[rid][-16:]
                    continue
                if not recovery[rid].ready(time.monotonic()):continue
                wrist=ports[rid].capture();nav=ports[rid].capture(camera='nav_cam')
                if video:video.update_inputs(rid,wrist,nav,now)
                index=steps[rid];steps[rid]+=1
                for name,obs in [('wrist',wrist),('nav',nav)]:
                    (out/'inputs'/rid/f'{index:04d}-{name}.jpg').write_bytes(base64.b64decode(obs['image']))
                before=actor.phase
                action=actor.advance(wrist)
                state_version(rid)
                control.write(json.dumps({'robot_id':rid,'step':index,'time':now,'phase':before,
                    'skill_state':actor.state,'wrist_sha256':wrist['sha256'],'nav_sha256':nav['sha256'],
                    'own_pose_commands':wrist['actuator_state']['servo_pulses'],
                    'estimated_target':actor.box.last_target,
                    'estimated_target_provenance':actor.box.last_target_provenance,
                    'face_alignment':actor.box.last_face_alignment,
                    'attachment':actor.box.last_attachment,
                    'action':action or {'kind':'llm_request'}},ensure_ascii=False)+'\n');control.flush()
                if action:
                    if action['kind']=='finish':done.add(rid);reasons[rid]=action['reason'];stop_actor(rid,'runner_stop')
                    else:ex.submit(action,wrist,before,now)
                elif calls[rid]>=args.max_calls:
                    done.add(rid);reasons[rid]='LLM_CALL_BUDGET';stop_actor(rid,'runner_stop')
                elif not input_budgets[rid].can_reserve:
                    done.add(rid);reasons[rid]='LLM_INPUT_TOKEN_BUDGET_PREFLIGHT';stop_actor(rid,'runner_stop')
                else:
                    current_decision[rid]=None
                    calls[rid]+=1
                    if actor.last_guard_reason:
                        memory[rid].append({'visual_feedback':actor.last_guard_reason,'skill_state':actor.state})
                        actor.last_guard_reason=None
                    if actor.state in ('carrying','released'):
                        placement=inspect_actor_placement(actor,wrist,nav)
                        actor.observe_placement(placement)
                        memory[rid].append({'placement_evidence':placement})
                        memory[rid]=memory[rid][-16:]
                    call_id=f'{rid}-call-{calls[rid]:04d}'
                    if not input_budgets[rid].reserve(call_id):
                        raise RuntimeError('INPUT_RESERVATION_RACE')
                    budget_context=input_budgets[rid].snapshot(calls_used=calls[rid],
                        max_calls=args.max_calls,remaining_sim_seconds=deadline-now)
                    execution_context=json.loads(json.dumps(execution_feedback[rid]))
                    wrist_path=f'inputs/{rid}/{index:04d}-wrist.jpg';nav_path=f'inputs/{rid}/{index:04d}-nav.jpg'
                    inbox=bus.inbox(rid,now)
                    request={'event':'llm_request','robot_id':rid,'time':now,'call_id':call_id,
                        'requested_reasoning_effort':planners[rid].completer.reasoning_effort,
                        'budget':budget_context,'execution_feedback':execution_context,
                        'task':{'cargo_id':actor.cargo_id,'destination_zone':actor.destination_zone},
                        'skill_state':actor.state,'state_version':state_version(rid),'memory':list(memory[rid]),'received_messages':inbox,
                        'images':[{'camera':'wrist','path':wrist_path,'sha256':wrist['sha256']},
                                  {'camera':'nav','path':nav_path,'sha256':nav['sha256']}]}
                    llm_log.write(json.dumps(request,ensure_ascii=False)+'\n');llm_log.flush()
                    pending[rid]={'future':pool.submit(planners[rid].decide,wrist,nav,list(memory[rid]),
                        cargo_id=actor.cargo_id,destination_zone=actor.destination_zone,skill_state=actor.state,messages=inbox,
                        budget=budget_context,execution_feedback=execution_context),
                        'wrist':wrist,'nav':nav,'requested_at':now,'state_version':state_versions[rid],
                        'known_message_ids':[m['message_id'] for m in inbox],
                        'call_id':call_id,'call_number':calls[rid]}
            now=float(world.data.time)
            for rid in active:
                actor=actors[rid]
                if rid in done or actor.state!='carrying' or now<next_monitor[rid]:continue
                monitor=ports[rid].capture(); next_monitor[rid]=now+.5
                path=out/'inputs'/rid/f'monitor-{monitor["frame_id"]:06d}.jpg'
                path.write_bytes(base64.b64decode(monitor['image']))
                stop=actor.advance(monitor)
                state_version(rid)
                monitor_log.write(json.dumps({'robot_id':rid,'time':now,'frame_id':monitor['frame_id'],
                    'sha256':monitor['sha256'],'skill_state':actor.state,
                    'estimated_target':actor.box.last_target,
                    'estimated_target_provenance':actor.box.last_target_provenance,
                    'face_alignment':actor.box.last_face_alignment,
                    'guard_reason':actor.last_guard_reason,
                    'attachment':actor.box.last_attachment})+'\n')
                if actor.state!='carrying':
                    stop_actor(rid,'runner_stop')
                    if stop and stop['kind']=='finish':done.add(rid);reasons[rid]=stop['reason']
            if now>=next_truth:
                state=world.warehouse_state();samples.append(state)
                truth.write(json.dumps({'time':now,'state':state,
                    'robot_positions':{rid:world.robot(rid).base_xyz().tolist() for rid in active},
                    'grip_positions':{rid:world.robot(rid).site_xyz('grip_site').tolist() for rid in active}})+'\n');truth.flush()
                for cid in cargo_ids:
                    z=state['cargo'][cid]['position'][2]-initial['cargo'][cid]['position'][2]
                    max_lifts[cid]=max(max_lifts[cid],z)
                if previous_cargo is not None:
                    dt=now-previous_time
                    moving=sum(math.dist(state['cargo'][cid]['position'][:2],previous_cargo[cid]['position'][:2])>0.002 for cid in cargo_ids)
                    if moving>=2:concurrent_carry_s+=dt
                previous_cargo=state['cargo'];previous_time=now;next_truth=now+.25
            dt=float(world.model.opt.timestep)
            if sum(any(abs(v)>1e-5 for v in ports[r]._motor_commands) for r in active)>=2:concurrent_motion_s+=dt
            peer=False;obstacle=False
            for contact in world.data.contact:
                if contact.dist>=-.002:continue
                a=mujoco.mj_id2name(world.model,mujoco.mjtObj.mjOBJ_GEOM,int(contact.geom1)) or ''
                b=mujoco.mj_id2name(world.model,mujoco.mjtObj.mjOBJ_GEOM,int(contact.geom2)) or ''
                ra=next((r for r in world.robot_ids if a.startswith(r+'__')),None)
                rb=next((r for r in world.robot_ids if b.startswith(r+'__')),None)
                if ra and rb and ra!=rb:peer=True
                if (ra and 'barrier' in b) or (rb and 'barrier' in a):obstacle=True
            peer_contact_s+=dt*peer;obstacle_contact_s+=dt*obstacle
            world._physics_step_for(world.robot('r1'))
            if video and world.data.time>=video.next_frame:video.capture()
            # Inference latency advances the physical world without fast-forwarding stale images.
            if any(r not in done for r in pending) or any(r not in done and not recovery[r].ready(time.monotonic()) for r in active):
                delay=float(world.model.opt.timestep)-(time.monotonic()-last_wall)
                if delay>0:time.sleep(delay)
            last_wall=time.monotonic()
    except KeyboardInterrupt:
        interrupted=True;error='KeyboardInterrupt: interrupted by operator'
        for rid in active:
            if rid not in done: reasons[rid]='INTERRUPTED'
        print(error,flush=True)
    except Exception as exc:
        error=f'{type(exc).__name__}: {exc}'
        for rid in active:
            if rid not in done: reasons[rid]='EXECUTION_ERROR:'+error
        print(error,flush=True)
        raise
    finally:
        for rid in active:stop_actor(rid,'episode_end')
        decision_elapsed_sim_s=float(world.data.time)-start
        def emit_late(row):
            llm_log.write(json.dumps(row,ensure_ascii=False)+'\n');llm_log.flush()
        settle_pending(pending,planners,input_budgets,pool,
                       now=float(world.data.time),emit=emit_late)
        until=float(world.data.time)+1
        while world.data.time<until:
            for port in ports.values():port.tick(float(world.data.time))
            world._physics_step_for(world.robot('r1'))
            if video and world.data.time>=video.next_frame:video.capture()
        final=world.warehouse_state(); outcomes={}
        within_sim_budget=decision_elapsed_sim_s <= args.seconds+float(world.model.opt.timestep)+1e-9
        for rid,cid in zip(active,cargo_ids):
            box=final['cargo'][cid];zone=world.warehouse_zones[goals[rid]]
            p=box['position']; half=world.warehouse_spec_by_id[cid].dimensions_m
            inside=footprint_inside(p,box.get('yaw',0),half,zone.center_xy,zone.half_extents_xy)
            constrained=any(any(s['cargo'][cid].get('constraints_active',{}).values()) for s in [initial,*samples,final])
            gates={'lift':max_lifts[cid]>=.04,'inside_destination':inside,'stable':box['stable'],
                   'no_attachment_constraint':not constrained,'visual_release':reasons[rid]=='VISUAL_RELEASE_CONFIRMED'}
            physical_success=all(gates.values())
            gates.update({'input_budget_verified':input_budgets[rid].verified_within_limit,
                          'call_budget':calls[rid]<=args.max_calls,'simulation_budget':within_sim_budget})
            outcomes[rid]={'cargo_id':cid,'destination_zone':goals[rid],'reason':reasons[rid],'physical_success':physical_success,'success':all(gates.values()),'gates':gates,'max_lift_m':max_lifts[cid],'decisions':steps[rid],'recovery_attempts':actors[rid].recovery_attempts}
        result={'seed':args.seed,'active_robots':active,'outcomes':outcomes,'success':all(o['success'] for o in outcomes.values()),
                'elapsed_wall_s':time.monotonic()-wall_start,
                'episode_start_sim_time':start,
                'episode_stop_sim_time':start+decision_elapsed_sim_s,
                'decision_elapsed_sim_s':decision_elapsed_sim_s,
                'passive_settle_s':float(world.data.time)-start-decision_elapsed_sim_s,
                'elapsed_sim_s':float(world.data.time)-start,'peer_penetration_gt2mm_s':peer_contact_s,
                'obstacle_penetration_gt2mm_s':obstacle_contact_s,'concurrent_drive_s':concurrent_motion_s,
                'concurrent_cargo_motion_s':concurrent_carry_s,'initial':initial,'final':final,'source_hash':source_hash,
                'physics':{'impratio':args.impratio,'noslip_iterations':args.noslip_iterations},
                'budgets':{'sim_seconds':args.seconds,'max_calls_per_robot':args.max_calls,
                           'max_input_tokens_per_robot':args.max_input_tokens,
                           'input_request_estimate':args.input_request_estimate,
                           'budget_mode':'estimated_preflight'},
                'input_usage':{rid:{'reported_prompt_tokens':input_budgets[rid].tokens,
                                   'calls_without_usage':input_budgets[rid].calls_without_usage,
                                   'estimated_unreported_tokens':input_budgets[rid].estimated_unreported_tokens,
                                   'unsettled_reservations':len(input_budgets[rid].reservations),
                                   'verified_within_limit':input_budgets[rid].verified_within_limit} for rid in active},
                'controller':'gemini_RGB_decisions_with_visual_manipulation_skills','llm_calls':calls,
                'model':args.model,'reasoning_effort':args.reasoning_effort,
                'messages':len(bus.sent),'communication':args.communication,
                'inference_errors':{rid:recovery[rid].total_errors for rid in active},
                'inference_unresolved':{rid:recovery[rid].consecutive for rid in active},'error':error}
        (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:result[k] for k in ('success','outcomes','concurrent_drive_s','concurrent_cargo_motion_s')}),flush=True)
        monitor_log.close();llm_log.close();commands.close();control.close();truth.close();message_log.close();event_log.close()
        if video:video.close()
        world.close()

if __name__=='__main__':main()
