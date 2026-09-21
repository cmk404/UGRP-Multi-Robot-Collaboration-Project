from harness.dispatch_evaluation import carry_clearance
from harness.dispatch_evaluation import concurrent_transport
import copy
import pytest


def test_placement_does_not_hide_a_drop_and_release_is_excluded():
    plan = {'tasks': [{'object': 'beam', 'participants': ['r1', 'r3']}]}
    history = {'r1': [{'stage': 'TRANSIT', 'issued_at_s': 1.,
                      'action': {'duration_s': .4}}]}
    def sample(t, contact):
        return {'sim_time_s': t, 'cargo': {'beam': {'floor_contact': contact}}}
    rows = [sample(0., True)] + [sample(t, False) for t in [1., 1.1, 1.2, 1.3, 1.4]]
    rows += [sample(1.5, True)]  # Commanded lowering after transport is allowed.
    assert carry_clearance(rows, history, plan)['beam']['sampled_continuous_clearance']
    rows[3]['cargo']['beam']['floor_contact'] = True
    result = carry_clearance(rows, history, plan)['beam']
    assert not result['sampled_continuous_clearance']
    assert result['first_floor_contact_s'] == 1.2


def test_missing_transport_or_sparse_evaluation_cannot_pass():
    plan = {'tasks': [{'object': 'beam', 'participants': ['r1', 'r3']}]}
    assert not carry_clearance([], {}, plan)['beam']['sampled_continuous_clearance']
    history = {'r3': [{'stage': 'TRANSIT', 'issued_at_s': 1., 'duration_s': 1.}]}
    rows = [{'sim_time_s': t, 'cargo': {'beam': {'floor_contact': False}}}
            for t in [1., 2.]]
    assert not carry_clearance(rows, history, plan)['beam']['sampled_continuous_clearance']


def test_concurrency_requires_both_loaded_objects_and_all_three_carriers_to_move():
    plan={'tasks':[{'object':'beam','participants':['r1','r3']},{'object':'box','participants':['r2']}]}
    history={r:[{'stage':'TRANSIT','issued_at_s':1.,'action':{'duration_s':1.}}] for r in ('r1','r2','r3')}
    rows=[{'sim_time_s':t,'weld':False,'robots':{r:[t*.05,0,0] for r in history},
           'cargo':{obj:{'position':[t*.05,0,.1],'floor_contact':False,'robot_contact':True} for obj in ('beam','box')}}
          for t in (1.,1.1,1.2)]
    assert concurrent_transport(rows,history,plan)['simultaneous_loaded_motion_s']==pytest.approx(.2)
    for field in ('stationary_box','stationary_carrier','ground_box','not_held','weld','serial_commands','missing_samples'):
        samples=copy.deepcopy(rows);commands=copy.deepcopy(history)
        if field=='stationary_box':
            for s in samples:s['cargo']['box']['position']=[0,0,.1]
        elif field=='stationary_carrier':
            for s in samples:s['robots']['r2']=[0,0,0]
        elif field=='ground_box':
            for s in samples:s['cargo']['box']['floor_contact']=True
        elif field=='not_held':
            for s in samples:s['cargo']['box']['robot_contact']=False
        elif field=='weld':
            for s in samples:s['weld']=True
        elif field=='serial_commands':commands['r2'][0]['issued_at_s']=2.
        else:samples.pop(1)
        assert concurrent_transport(samples,commands,plan)['simultaneous_loaded_motion_s']==0,field
