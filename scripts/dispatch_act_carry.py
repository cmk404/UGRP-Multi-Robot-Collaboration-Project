"""ACT replaces only loaded motion. Existing task grants and RGB guards remain."""
import json
import time
import copy
import math
from pathlib import Path
from collections import deque
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout
from threading import Lock
from harness.pair_carry_act_contract import context,AXES
from harness.pair_carry_act_client import CarryClient
from harness.dispatch_own_hold import OwnHoldContinuity
from harness.pair_carry_sync import PairCarrySync
from harness.dispatch_pair_navigation import authorize_pair

MIN_ACTUATION_WINDOW_S=.02
ACT_ABORT_JOIN_TIMEOUT_S=60.


class PredictionAudit:
    """Keep completed slot receipts readable after the physics owner aborts."""
    def __init__(self,slots):
        self.slots=tuple(slots);self.lock=Lock();self.completed={}

    def record(self,slot,inp,decision,failure):
        with self.lock:self.completed[slot]=(copy.deepcopy(inp),copy.deepcopy(decision),failure)

    def snapshot(self):
        with self.lock: completed=copy.deepcopy(self.completed)
        inputs={};decisions={};failures={}
        for slot,(inp,decision,failure) in completed.items():
            wire=inp.get('wire_sha256')
            if not isinstance(wire,str) or len(wire)!=64:
                failures[slot]=failure or 'wire status unconfirmed'
                continue
            inputs[slot]=inp
            if failure:failures[slot]=failure
            elif decision is not None:decisions[slot]=decision
        return inputs,decisions,failures,[slot for slot in self.slots if slot not in inputs]


def compute_act_visual(pair,work,tag,frames,attempt,audit=None,abort_clients=None):
    """Retain the ACT future so an owner-side failure cannot erase requests."""
    workers=getattr(pair.io,'_decision_workers',None)
    if workers is None:return pair.io.compute_visual(work)
    future=workers.submit(work)
    try:return pair.io.await_visual(future)
    except BaseException as owner_error:
        began=time.monotonic();result=None;completion='completed';worker_error=None
        aborted_workers=[];abort_errors=[]
        worker_exception=None
        try:result=future.result(timeout=ACT_ABORT_JOIN_TIMEOUT_S)
        except FutureTimeout:
            completion='timeout'
            future.cancel()  # A queued job must not start after its clients close.
            unresolved=(audit.snapshot()[3] if audit is not None
                        else tuple(abort_clients or {}))
            for slot in unresolved:
                client=(abort_clients or {}).get(slot)
                if not hasattr(client,'abort'):continue
                try:client.abort();aborted_workers.append(slot)
                except Exception as error:
                    abort_errors.append(slot+': '+type(error).__name__+': '+str(error))
        except BaseException as error:
            worker_exception=error
            completion='raised';worker_error=type(error).__name__+': '+str(error)
        index=tag.rsplit('-',1)[-1]
        if tag.startswith('act-carry-') and index.isdigit():
            slots=tuple(pair.bindings.pair)
            inputs,decisions,failures,unconfirmed=(audit.snapshot() if audit is not None
                else ({},{},{},list(slots)))
            if isinstance(result,dict):
                for slot,inp in result.get('inputs',{}).items():
                    wire=inp.get('wire_sha256')
                    if isinstance(wire,str) and len(wire)==64:
                        inputs[slot]=inp
                        if slot in result.get('decisions',{}):
                            decisions[slot]=result['decisions'][slot]
                unconfirmed=[slot for slot in slots if slot not in inputs]
            first=frames[next(iter(pair.bindings.pair.values()))]
            discard_reason=('prediction_raised' if worker_exception is owner_error
                            else 'owner_aborted')
            pair.calls.append({'kind':'act_inference_error','tag':tag,'attempt':attempt,
                'index':int(index),'frame_id':first['frame_id'],
                'observed_at_s':first['observed_at_s'],'received_at_s':pair.time(),
                'inputs':inputs,'decisions':decisions,'unconfirmed_slots':unconfirmed,
                'slot_failures':failures,'discard_reason':discard_reason,
                'worker_completion':completion,'worker_error':worker_error,
                'aborted_workers':aborted_workers,'abort_errors':abort_errors,
                'worker_join_wall_s':time.monotonic()-began,
                'owner_error':type(owner_error).__name__+': '+str(owner_error),
                'error':('ACT prediction raised before adoption' if worker_exception is owner_error
                         else 'ACT owner aborted before prediction adoption')})
        raise


def observe_act(pair, tag, predict, *, not_before=None, predict_with_audit=False,
                abort_clients=None):
    """Keep owner physics running; accept only a coherent, fresh raw RGB batch."""
    robots=tuple(pair.bindings.pair.values())
    if not getattr(pair.io,'realtime_control',False):
        frames=pair.io.capture(tag,own_robots=robots,overview=False)
        return frames,predict(frames)
    from sim.snapshot_contract import SnapshotBackpressure
    for attempt in range(3):
        # Image records are immutable; each reobservation needs a fresh path.
        capture_tag=tag+'-attempt-'+str(attempt)
        deadline=pair.time()+2.
        while True:
            try:
                future=pair.io.capture_async(capture_tag,own_robots=robots,overview=False)
                if future is not None:break
            except SnapshotBackpressure:
                pass
            if pair.time()>=deadline:
                pair._hold_pair()
                raise RuntimeError('ACT capture backpressure exceeded 2 SIM seconds')
            pair.tick(.02)
        frames=pair.io.await_visual(future)
        first=frames[robots[0]]
        observed=float(first['observed_at_s'])
        if (not math.isfinite(observed) or observed>pair.time()+1e-9
                or any(f['frame_id']!=first['frame_id']
                       or float(f['observed_at_s'])!=observed for f in frames.values())):
            pair._hold_pair()
            raise ValueError('ACT requires coherent capture timestamps and frame IDs')
        if pair.time()-observed>.6:
            pair._hold_pair()
            pair.calls.append({'kind':'act_stale_capture','tag':tag,
                'frame_id':first['frame_id'],'observed_at_s':observed,
                'received_at_s':pair.time()})
            continue
        audit=PredictionAudit(tuple(pair.bindings.pair)) if predict_with_audit else None
        work=(lambda:predict(frames,audit)) if predict_with_audit else (lambda:predict(frames))
        prediction=compute_act_visual(pair,work,tag,frames,attempt,audit,abort_clients)
        if isinstance(prediction,dict) and prediction.get('error'):
            pair._hold_pair()
            pair.calls.append({'kind':'act_inference_error','tag':tag,
                'index':prediction['index'],'frame_id':first['frame_id'],
                'observed_at_s':observed,'received_at_s':pair.time(),
                'inputs':prediction['inputs'],'decisions':prediction['decisions'],
                'error':prediction['error']})
            raise RuntimeError('ACT inference failed: '+prediction['error'])
        while not_before is not None and pair.time()<not_before-1e-9:
            pair.tick(min(.02,not_before-pair.time()))
        remaining=observed+.6-pair.time()
        if remaining>=MIN_ACTUATION_WINDOW_S:
            return frames,prediction
        pair._hold_pair()
        # Preserve even unused model requests; sub-tick leases are not motion.
        record={'kind':'act_stale_prediction','tag':tag,'attempt':attempt,
            'frame_id':first['frame_id'],'observed_at_s':observed,
            'received_at_s':pair.time(),
            'discard_reason':('insufficient_actuation_window' if remaining>0
                              else 'expired_rgb'),
            'remaining_actuation_s':max(0.,remaining)}
        if isinstance(prediction,dict):
            record.update(index=prediction.get('index'),inputs=prediction.get('inputs'),
                          decisions=prediction.get('decisions'))
        pair.calls.append(record)
    raise RuntimeError('ACT RGB/prediction remained stale after bounded reobservation')


def issue_act(pair, frames, actions, index):
    if not getattr(pair.io,'realtime_control',False):
        pair.drive_mecanum(actions,.2)
        return None
    frame=frames[next(iter(pair.bindings.pair.values()))]
    now=pair.time()
    if frame['observed_at_s']+.6-now<MIN_ACTUATION_WINDOW_S:
        pair._hold_pair()
        raise RuntimeError('ACT accepted RGB expired before command: insufficient actuation window')
    duration=pair.issue_mecanum_bounded(actions,.25,observed_at_s=frame['observed_at_s'])
    if duration<=0:
        raise RuntimeError('ACT accepted RGB expired before command commit')
    pair.calls.append({'kind':'act_issue','index':index,'issued_at_s':now,
        'observed_at_s':frame['observed_at_s'],'valid_until_s':now+duration,
        'source_frame_ids':{s:frames[r]['frame_id'] for s,r in pair.bindings.pair.items()},
        'plan_hash':pair.bindings.committed['plan_hash'],'duration_s':duration,
        'actions':copy.deepcopy(actions)})
    moving=any(abs(v)>1e-9 for action in actions.values() for v in action.values())
    pair.tick(.02 if moving else min(.2,duration))
    return now+.2

def track_release_rgb(pair, frames):
    """Maintain RGB shaft identity for the shared release stage, not ACT inputs."""
    frame=frames[next(iter(pair.bindings.pair.values()))]
    feature=pair.carried_beam.observe(frame['top_bytes'])
    return {'frame_id':frame['frame_id'],'image':frame['shared_top_rgb'],
            'center':feature['center'],'purpose':'release continuity only'}


def start_parallel_temporal_clients(python, model_dir, slots):
    """Start both owned workers, closing successes if either startup fails."""
    from harness.carry_input_client import InputCarryClient
    with ThreadPoolExecutor(max_workers=len(slots)) as pool:
        futures={slot:pool.submit(InputCarryClient,python,model_dir,cpu_threads=1) for slot in slots}
        clients={};errors=[]
        for slot in slots:
            try:clients[slot]=futures[slot].result()
            except Exception as error:errors.append(slot+': '+type(error).__name__+': '+str(error))
    if clients and len({client.history for client in clients.values()})!=1:
        errors.append('worker history mismatch')
    if len({id(client) for client in clients.values()})!=len(clients):
        errors.append('worker clients must be distinct')
    if errors:
        errors.extend(close_parallel_clients(clients))
        raise RuntimeError('ACT parallel worker startup failed: '+'; '.join(errors))
    return clients


def close_parallel_clients(clients):
    errors=[];closed=set()
    for slot,client in clients.items():
        if id(client) in closed:continue
        closed.add(id(client))
        try:client.close()
        except Exception as error:errors.append(slot+': close '+type(error).__name__+': '+str(error))
    return errors


class ParallelClientOwnership:
    """Transfer workers atomically; late startup closes itself after owner exit."""
    def __init__(self,python,model_dir,slots):
        self.python=python;self.model_dir=model_dir;self.slots=slots
        self.lock=Lock();self.abandoned=False;self.clients={}

    def start(self):
        with self.lock:
            if self.abandoned:raise RuntimeError('ACT owner abandoned worker startup')
        started=start_parallel_temporal_clients(self.python,self.model_dir,self.slots)
        with self.lock:
            adopted=not self.abandoned and not self.clients
            if adopted:self.clients=started
        if not adopted:
            errors=close_parallel_clients(started)
            raise RuntimeError('ACT owner abandoned worker startup'+
                               ('; '+'; '.join(errors) if errors else ''))
        return started

    def abandon_and_close(self):
        with self.lock:
            self.abandoned=True
            owned=self.clients
            self.clients={}
        return close_parallel_clients(owned)


def _timed_client_predict(client, history, slot=None, audit=None, input_meta=None):
    began=time.monotonic()
    try:decision=client.predict(history);failure=None
    except Exception as error:
        decision=None;failure=type(error).__name__+': '+str(error)
    wall_s=time.monotonic()-began
    if audit is not None:
        inp={**input_meta,'wire_sha256':getattr(client,'last_request_sha256',None),
             'inference_wall_s':wall_s,'response_received':failure is None}
        audit.record(slot,inp,decision,failure)
    return decision,failure,wall_s


def parallel_temporal_decisions(pair,frames,clients,executor,candidate_histories,guards,
                                goal,route,previous,call_time,plan_hash,audit=None):
    """Collect both private worker results before accepting any candidate state."""
    slots=tuple(pair.bindings.pair)
    if len({id(clients[slot]) for slot in slots})!=len(slots):
        raise ValueError('parallel ACT requires one private client per slot')
    actor_inputs={};requests={}
    for slot,rid in pair.bindings.pair.items():
        frame=frames[rid];ctx=context(goal,route,slot,previous[slot])
        current={'own_rgb':frame['own_bytes'],'top_rgb':frame['top_bytes'],'context':ctx}
        record={'images':{'own':frame['own_rgb'],'top':frame['shared_top_rgb']},
                'context':ctx,'sim_time_s':frame.get('observed_at_s',call_time),
                'frame_id':frame['frame_id']}
        history=candidate_histories[slot]
        history.append((current,record))
        padded=[history[0]]*(clients[slot].history-len(history))+list(history)
        requests[slot]=[item[0] for item in padded]
        actor_inputs[slot]={'images':record['images'],'context':ctx,'wire_sha256':None,
            'physical_robot_id':rid,'frame_id':frame['frame_id'],
            'observed_at_s':frame.get('observed_at_s',call_time),
            'history':[item[1] for item in padded]}
    futures={slot:executor.submit(_timed_client_predict,clients[slot],requests[slot],
                                  slot,audit,actor_inputs[slot]) for slot in slots}
    decisions={};errors=[]
    for slot in slots:
        decision,failure,wall_s=futures[slot].result()
        actor_inputs[slot].update(wire_sha256=getattr(clients[slot],'last_request_sha256',None),
                                  inference_wall_s=wall_s,response_received=failure is None)
        if failure:
            errors.append(slot+': '+failure)
            continue
        decisions[slot]=decision
        try:held=guards[slot].observe(frames[pair.bindings.pair[slot]]['own_bytes'])
        except Exception as error:
            errors.append(slot+': RGB attachment validation: '+type(error).__name__+': '+str(error))
            continue
        decisions[slot]={**decision,'ready':held['held_estimate'],
                         'own_attachment':held,'plan_hash':plan_hash}
    return {'inputs':actor_inputs,'decisions':decisions,'error':'; '.join(errors) if errors else None}

def carry(pair,python,model_dir,max_steps=900,stop_mode='rgb_guarded'):
    if stop_mode not in ('learned', 'rgb_guarded', 'rgb_refined'):
        raise ValueError('unknown ACT stop mode')
    pair.phase='TRANSIT';pair.transport_started=True
    if getattr(pair.io,'realtime_control',False):pair._hold_pair()
    # ACT still receives raw fixed cameras. The shared release stage needs a
    # continuous RGB identity; its estimate never enters the model or steering.
    anchor,_=observe_act(pair,'act-carry-anchor',lambda _frames:None)
    track_release_rgb(pair,anchor)
    own_guards={slot:OwnHoldContinuity(anchor[rid]['own_bytes']) for slot,rid in pair.bindings.pair.items()}
    goal=pair.bindings.static_map['docks'][pair.bindings.plan['dock']]['slots']['beam']['center_m']
    route=pair.bindings.tasks['beam']['route'];previous={r:[0.,0.,0.] for r in pair.bindings.pair}
    sync=PairCarrySync('act-'+pair.bindings.committed['plan_hash']);ready_count=0
    adapter=json.loads((Path(model_dir)/'adapter.json').read_text())
    temporal=adapter.get('kind')=='carry_input_ablation'
    realtime=getattr(pair.io,'realtime_control',False)
    parallel=temporal and realtime
    client=None;clients={};executor=None
    startup=ParallelClientOwnership(python,model_dir,tuple(pair.bindings.pair)) if parallel else None
    try:
        if parallel:
            clients=pair.io.compute_visual(startup.start)
            executor=ThreadPoolExecutor(max_workers=len(clients),thread_name_prefix='act-slot')
            history_length=next(iter(clients.values())).history
        else:
            if temporal:
                from harness.carry_input_client import InputCarryClient
                factory=lambda:InputCarryClient(python,model_dir)
            else:factory=lambda:CarryClient(python,model_dir)
            client=(pair.io.compute_visual(factory) if realtime else factory())
            history_length=client.history if temporal else 1
        histories={s:deque(maxlen=history_length) for s in pair.bindings.pair}
        not_before=None
        for index in range(max_steps):
            plan_hash=pair.bindings.committed['plan_hash']
            call_time=pair.time()
            def predict(frames,audit=None):
                # Workers update private image/history state only. The owner
                # adopts it after freshness and plan authority are checked.
                tracker=copy.deepcopy(pair.carried_beam)
                guards=copy.deepcopy(own_guards)
                candidate_histories=copy.deepcopy(histories)
                raw=frames[next(iter(pair.bindings.pair.values()))]
                feature=tracker.observe(raw['top_bytes'])
                tracking={'frame_id':raw['frame_id'],'image':raw['shared_top_rgb'],
                          'center':feature['center'],'purpose':'release continuity only'}
                if parallel:
                    result=parallel_temporal_decisions(
                        pair,frames,clients,executor,candidate_histories,guards,
                        goal,route,previous,call_time,plan_hash,audit)
                    return {'index':index,**result,'tracker':tracker,'guards':guards,
                            'histories':candidate_histories,'tracking':tracking}
                decisions={};actor_inputs={}
                for slot,rid in pair.bindings.pair.items():
                    f=frames[rid];ctx=context(goal,route,slot,previous[slot])
                    began=time.monotonic()
                    if temporal:
                        current={'own_rgb':f['own_bytes'],'top_rgb':f['top_bytes'],'context':ctx}
                        record={'images':{'own':f['own_rgb'],'top':f['shared_top_rgb']},
                                'context':ctx,'sim_time_s':f.get('observed_at_s',call_time),
                                'frame_id':f['frame_id']}
                        candidate_histories[slot].append((current,record))
                        padded=([candidate_histories[slot][0]]*(client.history-len(candidate_histories[slot]))
                                +list(candidate_histories[slot]))
                    failure=None
                    try:
                        d=(client.predict([v[0] for v in padded]) if temporal else
                           client.predict(f['own_bytes'],f['top_bytes'],ctx))
                    except Exception as error:
                        failure=type(error).__name__+': '+str(error)
                    inference_wall_s=time.monotonic()-began
                    actor_inputs[slot]={'images':{'own':f['own_rgb'],'top':f['shared_top_rgb']},
                        'context':ctx,'wire_sha256':getattr(client,'last_request_sha256',None),'physical_robot_id':rid,
                        'frame_id':f['frame_id'],'observed_at_s':f.get('observed_at_s',call_time),
                        'inference_wall_s':inference_wall_s,'response_received':failure is None}
                    if temporal:actor_inputs[slot].update(history=[v[1] for v in padded])
                    if failure:
                        return {'index':index,'inputs':actor_inputs,'decisions':decisions,'error':failure}
                    decisions[slot]=d
                    try:
                        held=guards[slot].observe(f['own_bytes'])
                    except Exception as error:
                        return {'index':index,'inputs':actor_inputs,'decisions':decisions,
                                'error':'RGB attachment validation: '+type(error).__name__+': '+str(error)}
                    decisions[slot]={**d,'ready':held['held_estimate'],'own_attachment':held,'plan_hash':plan_hash}
                return {'index':index,'decisions':decisions,'inputs':actor_inputs,'tracker':tracker,
                        'guards':guards,'histories':candidate_histories,'tracking':tracking}
            frames,prediction=observe_act(pair,'act-carry-'+str(index),predict,
                                          not_before=not_before,predict_with_audit=parallel,
                                          abort_clients=clients if parallel else None)
            if prediction.get('error'):
                first=frames[next(iter(pair.bindings.pair.values()))]
                pair.calls.append({'kind':'act_inference_error','index':index,
                    'frame_id':first['frame_id'],'observed_at_s':first.get('observed_at_s',call_time),
                    'received_at_s':pair.time(),'sim_time_s':pair.time(),'inputs':prediction['inputs'],
                    'decisions':prediction['decisions'],'error':prediction['error']})
                raise RuntimeError('ACT inference failed: '+prediction['error'])
            if pair.bindings.committed['plan_hash']!=plan_hash:
                if realtime:
                    first=frames[next(iter(pair.bindings.pair.values()))]
                    pair.calls.append({'kind':'act_stale_prediction','tag':'act-carry-'+str(index),
                        'index':index,'frame_id':first['frame_id'],
                        'observed_at_s':first['observed_at_s'],'received_at_s':pair.time(),
                        'inputs':prediction['inputs'],'decisions':prediction['decisions'],
                        'discard_reason':'plan_changed'})
                raise RuntimeError('ACT plan changed during inference')
            pair.carried_beam=prediction['tracker'];own_guards=prediction['guards']
            histories=prediction['histories'];release_tracking=prediction['tracking']
            decisions=prediction['decisions'];actor_inputs=prediction['inputs']
            permission=authorize_pair(sync,decisions,{s:frames[r]['frame_id'] for s,r in pair.bindings.pair.items()},index)
            ready_count=ready_count+1 if all(d['done'] for d in decisions.values()) else 0
            # Hold BOTH if either proposes arrival. Never let the partner drag it.
            pause=any(d['done'] for d in decisions.values())
            actions={s:dict.fromkeys(AXES,0.) if pause else d['action'] for s,d in decisions.items()}
            pair.calls.append({'kind':'act_carry','index':index,'sim_time_s':pair.time(),'inputs':actor_inputs,'decisions':decisions,'permission':permission,'actions':actions,'ready_count':ready_count,'release_tracking':release_tracking})
            if index%50==0:print(json.dumps({'act_carry_step':index,'sim_time_s':pair.time(),'stop_scores':{r:d['stop_score'] for r,d in decisions.items()},'actions':actions}),flush=True)
            if permission['phase']!='GO':raise RuntimeError('ACT carry RGB attachment guard stopped')
            if stop_mode=='learned' and ready_count>=3:return
            if pause and stop_mode!='learned':
                from harness.carry_arrival import arrival_evidence, PrematureCarryStop
                evidence=arrival_evidence(pair.carried_beam.previous,pair.bindings.static_map,pair.bindings.plan['dock'])
                pair.calls.append({'kind':'act_stop_admission','mode':stop_mode,
                                   'model_requested_stop':True,
                                   'requesting_slots':[s for s,d in decisions.items() if d['done']],
                                   'ready_count':ready_count,'rgb':evidence})
                if evidence['arrived']:
                    if stop_mode=='rgb_refined' or ready_count>=3:return
                    # The strict condition still requires three joint votes.
                    # A local stop outside the goal is rejected immediately.
                    not_before=issue_act(pair,frames,actions,index)
                    previous={s:[a[k] for k in AXES] for s,a in actions.items()}
                    continue
                if stop_mode=='rgb_refined':
                    # Explicit hybrid condition: ACT transit plus the existing
                    # RGB final approach, reported separately from ACT alone.
                    from harness.dispatch_skill_binding import ImageRoute
                    navigator=ImageRoute(pair.bindings,'beam')
                    navigator.observe(frames[next(iter(pair.bindings.pair.values()))]['top_bytes'])
                    # Only final alignment in the destination neighbourhood.
                    # Do not silently repair an arbitrary incorrect route.
                    if max(abs(v) for v in evidence['error_px']) > 80:
                        raise PrematureCarryStop('ACT stop too far from destination for RGB refinement')
                    navigator.index=len(navigator.points)-1
                    navigator.confirmations=0
                    pair.calls.append({'kind':'act_rgb_refinement','controller':'existing RGB final waypoint',
                                       'counts_as_pure_act':False})
                    pair.carry(navigator,max_steps=min(max_steps,160))
                    return
                raise PrematureCarryStop('ACT premature stop rejected by RGB arrival guard')
            not_before=issue_act(pair,frames,actions,index)
            previous={s:[a[k] for k in AXES] for s,a in actions.items()}
        raise RuntimeError('ACT carry decision budget exhausted')
    finally:
        try:
            if executor is not None:executor.shutdown(wait=True,cancel_futures=True)
        finally:
            try:
                if realtime:pair._hold_pair()
            finally:
                if client is not None:client.close()
                close_errors=startup.abandon_and_close() if startup is not None else []
                if close_errors:raise RuntimeError('; '.join(close_errors))
