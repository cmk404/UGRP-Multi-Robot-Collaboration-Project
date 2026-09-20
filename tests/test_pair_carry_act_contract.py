import base64
import itertools
import pytest
from harness.pair_carry_act_contract import context,decode
from scripts.pair_carry_act_worker import decode_request
from harness.dispatch_execution import validate_action

def test_only_task_and_issued_context():
    v=context([1.5,-1.3],'north','r3',[.03,-.06,0.])
    assert len(v)==8 and v[2:5]==[1.,0.,1.]
    assert v[-3:]==pytest.approx([.2,-.4,0])
    with pytest.raises(ValueError):context([float('nan'),0],'north','r1',[0,0,0])

def test_wire_rejects_measured_state():
    r={'own_rgb':base64.b64encode(b'own').decode(),'top_rgb':base64.b64encode(b'top').decode(),'context':[0]*8}
    assert decode_request(r)==(b'own',b'top',[0]*8)
    for field in ('qpos','robot_pose','contact','success','teacher_action'):
        with pytest.raises(ValueError):decode_request({**r,field:0})
    with pytest.raises(ValueError):decode_request({**r,'context':[0]*7})

def test_stop_and_signed_action_bounds():
    assert decode([-2,2,-2,.1])['action']=={'forward':-.05,'left':.1,'turn':-.15}
    assert decode([1,1,1,.65])['action']==dict.fromkeys(('forward','left','turn'),0.)
    with pytest.raises(ValueError):decode([0,0,float('inf'),0])

def test_decoded_actions_satisfy_actual_transit_executor():
    for values in itertools.product((-100.,-1.,0.,1.,100.),repeat=3):
        action={'kind':'mecanum',**decode([*values,.1])['action'],'duration_s':.2}
        assert validate_action(action,'TRANSIT')==action
