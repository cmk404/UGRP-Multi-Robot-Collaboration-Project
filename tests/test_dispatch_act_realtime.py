"""ACT inference cannot freeze physics or issue from stale camera evidence."""
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from scripts.dispatch_act_carry import observe_act, issue_act
from scripts.dispatch_pair_skill import BoundPairSkill


class ObservationPair:
    def __init__(self, delays=(.04,), backpressure=False, incoherent=False):
        self.now=0.;self.calls=[];self.capture_count=0;self.prediction_count=0
        self.holds=0;self.ticks=[];self.delays=delays;self.capture_tags=[]
        self.backpressure=backpressure;self.incoherent=incoherent
        self.bindings=SimpleNamespace(pair={'r1':'r2','r3':'r1'})
        self.io=SimpleNamespace(realtime_control=True,capture_async=self.capture,
                                await_visual=self.await_visual,compute_visual=self.compute)

    def time(self):return self.now
    def tick(self,seconds):self.now+=seconds;self.ticks.append(seconds)
    def _hold_pair(self):self.holds+=1

    def capture(self,tag,**kwargs):
        assert kwargs=={'own_robots':('r2','r1'),'overview':False}
        self.capture_count+=1
        if self.backpressure and self.capture_count==1:
            from sim.snapshot_contract import SnapshotBackpressure
            raise SnapshotBackpressure('busy')
        if tag in self.capture_tags:raise FileExistsError(tag)
        self.capture_tags.append(tag)
        batch={rid:{'frame_id':self.capture_count,'observed_at_s':self.now,
                    'own_bytes':rid.encode(),'top_bytes':b'raw TOP'} for rid in ('r2','r1')}
        if self.incoherent:batch['r1']['frame_id']+=1
        future=Future();future.set_result(batch);return future

    def await_visual(self,future):self.tick(.02);return future.result()

    def compute(self,predict):
        self.tick(self.delays[min(self.prediction_count,len(self.delays)-1)])
        self.prediction_count+=1
        return predict()


def prediction(frames):
    assert frames['r2']['own_bytes']==b'r2' and frames['r1']['own_bytes']==b'r1'
    assert all(f['top_bytes']==b'raw TOP' for f in frames.values())
    return {'index':7,'inputs':{'r1':{'frame_id':frames['r2']['frame_id']}},
            'decisions':{'r1':{'done':False}}}


def test_owner_advances_through_raw_capture_compute_and_command_cadence():
    pair=ObservationPair(backpressure=True)
    frames,value=observe_act(pair,'act-carry-7',prediction,not_before=.2)
    assert pair.capture_count==2 and pair.prediction_count==1
    assert pair.time()==pytest.approx(.2) and pair.holds==0
    assert value['inputs']['r1']['frame_id']==frames['r2']['frame_id']
    assert max(pair.ticks)<=.04


def test_delayed_prediction_is_preserved_but_reobserved_before_acceptance():
    pair=ObservationPair(delays=(.61,.04))
    frames,_=observe_act(pair,'act-carry-7',prediction)
    assert frames['r2']['frame_id']==2 and pair.holds==1
    assert len(pair.calls)==1 and pair.calls[0]['kind']=='act_stale_prediction'
    assert pair.calls[0]['inputs']['r1']['frame_id']==1
    assert pair.calls[0]['index']==7 and pair.prediction_count==2
    assert pair.capture_tags==['act-carry-7-attempt-0','act-carry-7-attempt-1']


def test_persistently_stale_model_stops_after_three_requests():
    pair=ObservationPair(delays=(.7,))
    with pytest.raises(RuntimeError,match='remained stale'):
        observe_act(pair,'act-carry-7',prediction)
    assert pair.holds==pair.prediction_count==len(pair.calls)==3
    assert all(row['kind']=='act_stale_prediction' for row in pair.calls)


def test_incoherent_batch_stops_before_model_request():
    pair=ObservationPair(incoherent=True)
    with pytest.raises(ValueError,match='coherent capture'):
        observe_act(pair,'act-carry-7',prediction)
    assert pair.holds==1 and pair.prediction_count==0


def test_partial_model_failure_records_requests_and_stops_without_retry():
    pair=ObservationPair()
    def failing(frames):
        result=prediction(frames)
        result['error']='TimeoutError: second robot response'
        return result
    with pytest.raises(RuntimeError,match='ACT inference failed'):
        observe_act(pair,'act-carry-7',failing)
    assert pair.holds==1 and pair.prediction_count==1
    assert pair.calls[0]['kind']=='act_inference_error'
    assert pair.calls[0]['inputs']['r1']['frame_id']==1


def test_capture_expiring_during_cadence_wait_is_reobserved():
    pair=ObservationPair()
    frames,_=observe_act(pair,'act-carry-7',prediction,not_before=.7)
    assert frames['r2']['frame_id']==2 and pair.holds==1
    assert pair.time()-frames['r2']['observed_at_s']<.6


def motor_pair(now):
    pair=BoundPairSkill.__new__(BoundPairSkill)
    pair.calls=[];pair.phase='TRANSIT';pair.transport_started=True
    pair.bindings=SimpleNamespace(pair={'r1':'r2','r3':'r1'},
        committed={'plan_hash':'frozen-plan'},reserve_beam_apron=Mock())
    pair.carried_beam=SimpleNamespace(previous={'center':[1.,2.]})
    pair.io=SimpleNamespace(realtime_control=True,pair_issue_bounded=Mock())
    pair.time=lambda:now;pair.tick=Mock();pair._hold_pair=Mock()
    return pair


def test_act_issue_uses_original_capture_deadline_and_physical_robot_mapping():
    pair=motor_pair(.55)
    frames={r:{'frame_id':4,'observed_at_s':0.} for r in ('r2','r1')}
    actions={s:{'forward':.12,'left':0.,'turn':0.} for s in ('r1','r3')}
    assert issue_act(pair,frames,actions,2)==pytest.approx(.75)
    commands,duration,stage=pair.io.pair_issue_bounded.call_args.args
    assert set(commands)=={'r2','r1'} and stage=='TRANSIT'
    assert duration==pytest.approx(.05) and pair.calls[-1]['valid_until_s']==pytest.approx(.6)
    assert pair.calls[-1]['source_frame_ids']=={'r1':4,'r3':4}
    pair.tick.assert_called_once_with(.02)


def test_expired_act_input_cannot_revive_a_motor_lease():
    pair=motor_pair(.61)
    frames={r:{'frame_id':4,'observed_at_s':0.} for r in ('r2','r1')}
    actions={s:{'forward':.12,'left':0.,'turn':0.} for s in ('r1','r3')}
    with pytest.raises(RuntimeError,match='expired before command'):
        issue_act(pair,frames,actions,2)
    pair.io.pair_issue_bounded.assert_not_called()
    pair._hold_pair.assert_called_once()


def test_stationary_act_confirmation_keeps_bounded_dwell():
    pair=motor_pair(.1)
    frames={r:{'frame_id':4,'observed_at_s':.1} for r in ('r2','r1')}
    actions={s:{'forward':0.,'left':0.,'turn':0.} for s in ('r1','r3')}
    issue_act(pair,frames,actions,2)
    pair.tick.assert_called_once_with(.2)


def test_persistent_capture_backpressure_is_bounded_without_model_requests():
    from sim.snapshot_contract import SnapshotBackpressure
    pair=ObservationPair()
    pair.io.capture_async=Mock(side_effect=SnapshotBackpressure('busy'))
    with pytest.raises(RuntimeError,match='backpressure exceeded'):
        observe_act(pair,'busy',prediction)
    assert pair.time()==pytest.approx(2.)
    assert pair.holds==1 and pair.prediction_count==0
