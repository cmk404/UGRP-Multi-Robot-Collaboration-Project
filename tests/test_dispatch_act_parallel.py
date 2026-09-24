"""Bounded two-worker ACT inference keeps each robot's request and state private."""

from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
import hashlib
import json
from threading import Barrier, Event, Lock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from scripts.dispatch_act_carry import (
    ParallelClientOwnership, carry, issue_act, observe_act,
    parallel_temporal_decisions, start_parallel_temporal_clients,
)


def frames():
    return {robot: {'own_bytes': robot.encode(), 'top_bytes': b'top',
                    'own_rgb': {'path': robot + '.jpg'},
                    'shared_top_rgb': {'path': 'top.jpg'},
                    'frame_id': 17, 'observed_at_s': 0.}
            for robot in ('physical-left', 'physical-right')}


class Guard:
    def __init__(self): self.observed = []
    def observe(self, raw):
        self.observed.append(raw)
        return {'held_estimate': True}


def decision_pair():
    return SimpleNamespace(bindings=SimpleNamespace(pair={
        'r1': 'physical-right', 'r3': 'physical-left'}))


def run_parallel(clients):
    pair=decision_pair(); guards={s:Guard() for s in pair.bindings.pair}
    histories={s:deque(maxlen=4) for s in pair.bindings.pair}
    previous={s:[0.,0.,0.] for s in pair.bindings.pair}
    with ThreadPoolExecutor(max_workers=2) as pool:
        result=parallel_temporal_decisions(
            pair,frames(),clients,pool,histories,guards,[1.,1.],'north',previous,0.,'plan')
    return result,guards,histories


def test_parallel_workers_meet_barrier_and_keep_history_hashes_isolated():
    barrier=Barrier(2)
    class Client:
        history=4
        def __init__(self, expected): self.expected=expected
        def predict(self, history):
            assert len(history)==4
            assert all(frame['own_rgb']==self.expected for frame in history)
            assert all(frame['context'][4]==float(self.expected==b'physical-left') for frame in history)
            self.last_request_sha256=hashlib.sha256(self.expected).hexdigest()
            barrier.wait(timeout=2)
            return {'action':{'forward':.01,'left':.02,'turn':0.},
                    'done':False,'stop_score':.1}
    clients={'r1':Client(b'physical-right'),'r3':Client(b'physical-left')}
    result,guards,histories=run_parallel(clients)
    assert result['error'] is None
    assert set(result['inputs'])==set(result['decisions'])=={'r1','r3'}
    assert result['inputs']['r1']['physical_robot_id']=='physical-right'
    assert result['inputs']['r3']['physical_robot_id']=='physical-left'
    assert result['inputs']['r1']['wire_sha256']!=result['inputs']['r3']['wire_sha256']
    assert all(len(value['history'])==4 and value['response_received'] for value in result['inputs'].values())
    assert guards['r1'].observed==[b'physical-right']
    assert guards['r3'].observed==[b'physical-left']
    assert all(len(history)==1 for history in histories.values())


def test_one_worker_failure_waits_for_other_and_preserves_both_requests():
    barrier=Barrier(2)
    class Client:
        history=4
        def __init__(self, slot): self.slot=slot
        def predict(self, history):
            self.last_request_sha256='wire-'+self.slot
            barrier.wait(timeout=2)
            if self.slot=='r1':raise ValueError('broken reply')
            return {'action':{'forward':.01,'left':0.,'turn':0.},
                    'done':False,'stop_score':.2}
    result,guards,_=run_parallel({s:Client(s) for s in ('r1','r3')})
    assert result['error']=='r1: ValueError: broken reply'
    assert result['inputs']['r1']['wire_sha256']=='wire-r1'
    assert result['inputs']['r3']['wire_sha256']=='wire-r3'
    assert result['inputs']['r1']['response_received'] is False
    assert result['inputs']['r3']['response_received'] is True
    assert set(result['decisions'])=={'r3'}
    assert guards['r1'].observed==[] and guards['r3'].observed==[b'physical-left']


def test_parallel_request_refuses_shared_client_instance():
    client=SimpleNamespace(history=4)
    with pytest.raises(ValueError,match='private client'):
        run_parallel({'r1':client,'r3':client})


def test_owned_client_abort_kills_only_live_worker(monkeypatch):
    from harness.carry_input_client import InputCarryClient
    from harness.recovery_act_client import RecoveryClient
    client=InputCarryClient.__new__(InputCarryClient)
    client.process=Mock()
    client.process.poll.return_value=None
    client.abort()
    client.process.kill.assert_called_once_with()
    client.process.poll.return_value=0
    client.abort()
    client.process.kill.assert_called_once_with()
    def broken_pipe(_self):raise BrokenPipeError('killed during flush')
    monkeypatch.setattr(RecoveryClient,'close',broken_pipe)
    client.close()
    client.process.stdout.close.assert_called_once_with()


def test_startup_failure_closes_successful_worker_after_both_attempts(monkeypatch):
    import harness.carry_input_client as module
    barrier=Barrier(2);lock=Lock();count=[0];created=[]
    class Client:
        def __init__(self,*args,**kwargs):
            assert kwargs=={'cpu_threads':1}
            with lock:
                index=count[0];count[0]+=1
            barrier.wait(timeout=2)
            if index==1:raise RuntimeError('startup failed')
            self.history=4;self.closed=False;created.append(self)
        def close(self):self.closed=True
    monkeypatch.setattr(module,'InputCarryClient',Client)
    with pytest.raises(RuntimeError,match='startup failed'):
        start_parallel_temporal_clients('python','model',('r1','r3'))
    assert count==[2] and len(created)==1 and created[0].closed


def test_owner_aborts_before_startup_finishes_and_late_workers_close(monkeypatch):
    import scripts.dispatch_act_carry as runtime
    entered=Barrier(2);release=Barrier(2)
    class Client:
        def __init__(self):self.closed=False
        def close(self):self.closed=True
    created={'r1':Client(),'r3':Client()}
    def delayed_start(*args):
        entered.wait(timeout=2)
        release.wait(timeout=2)
        return created
    monkeypatch.setattr(runtime,'start_parallel_temporal_clients',delayed_start)
    ownership=ParallelClientOwnership('python','model',('r1','r3'))
    with ThreadPoolExecutor(max_workers=1) as pool:
        future=pool.submit(ownership.start)
        entered.wait(timeout=2)
        assert ownership.abandon_and_close()==[]
        release.wait(timeout=2)
        with pytest.raises(RuntimeError,match='owner abandoned'):
            future.result(timeout=2)
    assert all(client.closed for client in created.values())


class RealtimePair:
    def __init__(self):
        self.now=0.;self.calls=[];self.holds=0
        self.bindings=SimpleNamespace(pair={'r1':'physical-right','r3':'physical-left'})
        self.io=SimpleNamespace(realtime_control=True,capture_async=self.capture,
                                await_visual=self.await_visual,compute_visual=lambda work:work())
    def time(self):return self.now
    def tick(self,dt):self.now+=dt
    def _hold_pair(self):self.holds+=1
    def capture(self,*args,**kwargs):
        result=Future();result.set_result(frames());return result
    def await_visual(self,future):
        self.now+=.02
        return future.result()


def test_partial_parallel_failure_produces_one_owner_error_row():
    pair=RealtimePair();barrier=Barrier(2)
    class Client:
        history=4
        def __init__(self,slot):self.slot=slot
        def predict(self,_history):
            self.last_request_sha256='request-'+self.slot
            barrier.wait(timeout=2)
            if self.slot=='r1':raise RuntimeError('worker failed')
            return {'action':{'forward':0.,'left':.02,'turn':0.},'done':False,'stop_score':0.}
    clients={s:Client(s) for s in pair.bindings.pair}
    with ThreadPoolExecutor(max_workers=2) as pool:
        def predict(batch):
            result=parallel_temporal_decisions(
                pair,batch,clients,pool,{s:deque(maxlen=4) for s in clients},
                {s:Guard() for s in clients},[1.,1.],'north',
                {s:[0.,0.,0.] for s in clients},pair.time(),'plan')
            return {'index':0,**result}
        with pytest.raises(RuntimeError,match='ACT inference failed'):
            observe_act(pair,'act-carry-0',predict)
    assert len(pair.calls)==1 and pair.calls[0]['kind']=='act_inference_error'
    assert set(pair.calls[0]['inputs'])=={'r1','r3'}
    assert set(pair.calls[0]['decisions'])=={'r3'}
    assert pair.holds==1


@pytest.mark.parametrize('failed_slot',(None,'r1'))
def test_owner_abort_joins_both_real_requests_and_preserves_original_error(failed_slot):
    pair=RealtimePair();barrier=Barrier(3)
    class OwnerAbort(RuntimeError):pass
    original=OwnerAbort('solo physics failed')
    class Client:
        history=4
        def __init__(self,slot):self.slot=slot
        def predict(self,_history):
            self.last_request_sha256=hashlib.sha256(self.slot.encode()).hexdigest()
            barrier.wait(timeout=2)
            if self.slot==failed_slot:raise ValueError('worker failed after wire')
            return {'action':{'forward':.01,'left':0.,'turn':0.},
                    'done':False,'stop_score':0.}
    clients={slot:Client(slot) for slot in pair.bindings.pair}
    calls=[0]
    def await_visual(future):
        calls[0]+=1
        if calls[0]==1:
            pair.now=.02
            return future.result()
        barrier.wait(timeout=2)
        raise original
    pair.io.await_visual=await_visual
    def predict(batch,audit):
        result=parallel_temporal_decisions(
            pair,batch,clients,inner,{s:deque(maxlen=4) for s in clients},
            {s:Guard() for s in clients},[1.,1.],'north',
            {s:[0.,0.,0.] for s in clients},pair.time(),'plan',audit)
        return {'index':0,**result}
    with ThreadPoolExecutor(max_workers=2) as inner, ThreadPoolExecutor(max_workers=1) as outer:
        pair.io._decision_workers=outer
        with pytest.raises(OwnerAbort) as raised:
            observe_act(pair,'act-carry-0',predict,predict_with_audit=True)
    assert raised.value is original
    assert len(pair.calls)==1
    row=pair.calls[0]
    assert row['kind']=='act_inference_error' and row['discard_reason']=='owner_aborted'
    assert row['worker_completion']=='completed' and row['unconfirmed_slots']==[]
    assert set(row['inputs'])=={'r1','r3'}
    assert set(row['decisions'])==({'r1','r3'} if failed_slot is None else {'r3'})
    assert set(row['slot_failures'])==({failed_slot} if failed_slot else set())
    assert all(len(row['inputs'][s]['wire_sha256'])==64 for s in clients)


def test_owner_abort_timeout_marks_unconfirmed_slot_without_claiming_request(monkeypatch):
    import scripts.dispatch_act_carry as runtime
    monkeypatch.setattr(runtime,'ACT_ABORT_JOIN_TIMEOUT_S',.02)
    pair=RealtimePair();blocked=Event();release=Event();known=Event()
    original_record=runtime.PredictionAudit.record
    def record(self,slot,*args):
        original_record(self,slot,*args)
        if slot=='r1':known.set()
    monkeypatch.setattr(runtime.PredictionAudit,'record',record)
    class OwnerAbort(RuntimeError):pass
    original=OwnerAbort('owner stopped')
    class Client:
        history=4
        def __init__(self,slot):self.slot=slot
        def abort(self):self.aborted=True
        def predict(self,_history):
            self.last_request_sha256=hashlib.sha256(self.slot.encode()).hexdigest()
            if self.slot=='r3':
                blocked.set()
                assert release.wait(timeout=2)
            return {'action':{'forward':.01,'left':0.,'turn':0.},
                    'done':False,'stop_score':0.}
    clients={slot:Client(slot) for slot in pair.bindings.pair}
    calls=[0]
    def await_visual(future):
        calls[0]+=1
        if calls[0]==1:
            pair.now=.02
            return future.result()
        assert blocked.wait(timeout=2) and known.wait(timeout=2)
        raise original
    pair.io.await_visual=await_visual
    def predict(batch,audit):
        result=parallel_temporal_decisions(
            pair,batch,clients,inner,{s:deque(maxlen=4) for s in clients},
            {s:Guard() for s in clients},[1.,1.],'north',
            {s:[0.,0.,0.] for s in clients},pair.time(),'plan',audit)
        return {'index':0,**result}
    with ThreadPoolExecutor(max_workers=2) as inner, ThreadPoolExecutor(max_workers=1) as outer:
        pair.io._decision_workers=outer
        with pytest.raises(OwnerAbort) as raised:
            observe_act(pair,'act-carry-0',predict,predict_with_audit=True,
                        abort_clients=clients)
        release.set()
    assert raised.value is original
    assert len(pair.calls)==1
    row=pair.calls[0]
    assert row['worker_completion']=='timeout'
    assert set(row['inputs'])==set(row['decisions'])=={'r1'}
    assert row['unconfirmed_slots']==['r3']
    assert row['inputs']['r1']['response_received'] is True
    assert row['aborted_workers']==['r3']
    assert clients['r3'].aborted and not getattr(clients['r1'],'aborted',False)


def test_sub_tick_actuation_headroom_discards_and_reobserves_full_request():
    pair=RealtimePair();capture_index=[0];predict_index=[0]
    def capture(*args,**kwargs):
        capture_index[0]+=1
        current={rid:{**frame,'frame_id':capture_index[0],
                      'observed_at_s':pair.time()} for rid,frame in frames().items()}
        future=Future();future.set_result(current);return future
    def compute(work):
        pair.now+=(.57 if predict_index[0]==0 else .04)
        predict_index[0]+=1
        return work()
    pair.io.capture_async=capture;pair.io.compute_visual=compute
    def predict(batch):
        return {'index':predict_index[0],
                'inputs':{'r1':{'frame_id':batch['physical-right']['frame_id']}},
                'decisions':{'r1':{'done':False}}}
    accepted,result=observe_act(pair,'act-carry-4',predict)
    assert capture_index==[2] and predict_index==[2]
    assert accepted['physical-right']['frame_id']==2
    assert result['inputs']['r1']['frame_id']==2
    assert len(pair.calls)==1
    assert pair.calls[0]['kind']=='act_stale_prediction'
    assert pair.calls[0]['discard_reason']=='insufficient_actuation_window'
    assert pair.calls[0]['inputs']['r1']['frame_id']==1
    assert 0<pair.calls[0]['remaining_actuation_s']<.02


def test_sub_tick_actuation_lease_never_reaches_motor_issue():
    pair=RealtimePair();pair.now=.59975
    pair.issue_mecanum_bounded=Mock()
    pair.bindings.committed={'plan_hash':'plan'}
    actions={slot:{'forward':.02,'left':0.,'turn':0.} for slot in pair.bindings.pair}
    with pytest.raises(RuntimeError,match='insufficient actuation window'):
        issue_act(pair,frames(),actions,11)
    pair.issue_mecanum_bounded.assert_not_called()
    assert pair.holds==1


@pytest.mark.parametrize('discard',('stale','plan_change'))
def test_discarded_parallel_prediction_never_adopts_private_tracker_or_issues(
        tmp_path,monkeypatch,discard):
    import harness.carry_input_client as module
    import scripts.dispatch_act_carry as runtime
    (tmp_path/'adapter.json').write_text(json.dumps({'kind':'carry_input_ablation'}))
    clients=[]
    class Client:
        history=4
        def __init__(self,*args,**kwargs):clients.append(self);self.closed=False
        def predict(self,history):
            self.last_request_sha256='fixture'
            return {'action':{'forward':.02,'left':0.,'turn':0.},'done':False,'stop_score':0.}
        def close(self):self.closed=True
    class Tracker:
        def __init__(self):self.observations=0
        def observe(self,raw):
            self.observations+=1
            return {'center':[.5,.5]}
    monkeypatch.setattr(module,'InputCarryClient',Client)
    monkeypatch.setattr(runtime,'OwnHoldContinuity',lambda *_:Guard())
    pair=RealtimePair()
    pair.bindings.static_map={'docks':{'dock':{'slots':{'beam':{'center_m':[1.,1.]}}}}}
    pair.bindings.plan={'dock':'dock'}
    pair.bindings.tasks={'beam':{'route':'north'}}
    pair.bindings.committed={'plan_hash':'original'}
    pair.carried_beam=Tracker()
    pair.issue_mecanum_bounded=lambda *args,**kwargs:pytest.fail('discarded prediction issued')
    def observe(owner,tag,predict,**kwargs):
        if tag=='act-carry-anchor':return frames(),None
        result=predict(frames())
        assert result['error'] is None
        if discard=='stale':raise RuntimeError('stale prediction discarded')
        owner.bindings.committed['plan_hash']='changed'
        owner.now=.2
        return frames(),result
    monkeypatch.setattr(runtime,'observe_act',observe)
    with pytest.raises(RuntimeError,match='stale prediction|plan changed'):
        carry(pair,'python',tmp_path,max_steps=1)
    assert len(clients)==2 and all(client.closed for client in clients)
    assert pair.carried_beam.observations==1
    assert not [entry for entry in pair.calls if entry['kind']=='act_carry']
    if discard=='plan_change':
        assert len(pair.calls)==1
        assert pair.calls[0]['discard_reason']=='plan_changed'
        assert set(pair.calls[0]['inputs'])=={'r1','r3'}
