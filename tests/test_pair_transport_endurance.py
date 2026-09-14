import copy,json
from pathlib import Path
import pytest
from harness.pair_grasp_endurance import EnduranceActor
from scripts.run_pair_grasp_endurance import evaluate_endurance
ROOT=Path(__file__).parents[1];FIX=ROOT/'tests/fixtures/pair_grasp_spacing'
def data():return json.loads((ROOT/'maps/pair_navigation/narrow-door.json').read_text())
def rgb(n):return (FIX/n).read_bytes()

def test_missing_own_rgb_holds_even_with_valid_top():
    r=EnduranceActor(data(),'r1','shuttle').decide(b'bad',rgb('anchor-top.jpg'))
    assert not r['ready'] and all(r['action'][k]==0 for k in ('forward','left','turn'))

def test_shuttle_rejects_corridor_that_only_blocks_its_far_end():
    m=data();m['obstacles'].append({'id':'shuttle_end','center_m':[.9,-2.],'half_extents_m':[.02,.1],'height_m':.3})
    r=EnduranceActor(m,'r1','shuttle').decide(rgb('anchor-r1-own.jpg'),rgb('anchor-top.jpg'))
    assert not r['ready'] and 'corridor' in r['error']

def test_repeated_unchanged_images_cannot_fake_a_completed_shuttle():
    a=EnduranceActor(data(),'r1','shuttle')
    for _ in range(120):
        r=a.decide(rgb('anchor-r1-own.jpg'),rgb('anchor-top.jpg'))
        assert -.05<=r['action']['forward']<=.05
        if not r['ready']:break
    assert not r['ready'] and 'tracking envelope' in r['error']

def record(duration=300,drop=0.):
    rows=json.loads((FIX/'baseline-evaluation-only.json').read_text())
    anchor=copy.deepcopy(next(r['bases'] for r in rows if r['phase']=='grasp_close'))
    for r in rows:
        r['bases']=copy.deepcopy(anchor);r['height_above_start_m']=.075
        for c in r['contacts'].values():c['bilateral']=True
    for i in range(round(duration*10)+1):
        rows.append({'phase':'carry_stop','sim_time_s':25+i*.1,'bases':copy.deepcopy(anchor),
            'height_above_start_m':.075-drop*i/(duration*10),'position_m':[.58,-2,.095-drop*i/(duration*10)],
            'contacts':{r:{'bilateral':True} for r in ('r1','r3')},'tilt_deg':0.,'within_authored_bounds':True,
            'finger_centers_m':{r:[0,0,.095] for r in ('r1','r3')},'beam_minus_finger_z_m':{r:-drop*i/(duration*10) for r in ('r1','r3')}})
    report={'duration_s':300,'endurance_start_s':25,'wall_contact_ticks':0,'unexpected_contact_ticks':0,'weld_active_ticks':0,'invariants_initial':{},'invariants_final':{}}
    return rows,report

def test_three_minutes_cannot_be_reported_as_five():
    r=evaluate_endurance(*record(180))
    assert r['checkpoints']['180']['passed']
    assert not r['checkpoints']['300']['gates']['duration_completed']
    assert not r['success']

def test_sliding_beam_fails_even_when_contacts_and_lift_remain():
    r=evaluate_endurance(*record(300,.02))
    assert r['checkpoints']['300']['gates']['bilateral_every_sample']
    assert r['checkpoints']['300']['gates']['lifted_every_sample']
    assert not r['checkpoints']['300']['gates']['height_retained']
    assert r['checkpoints']['300']['finger_height_change_m']['r1']==0
    assert r['checkpoints']['300']['beam_relative_to_finger_change_m']['r1']==pytest.approx(-.02)

def test_endurance_pass_does_not_imply_release():
    rows,report=record();report['release_check_required']=True
    result=evaluate_endurance(rows,report)
    assert result['checkpoints']['300']['passed']
    assert not result['success'] and not result['released_on_floor']
    release={'phase':'release_hold','payload_floor_contact':True,'contacts':{r:{'left':False,'right':False} for r in ('r1','r3')}}
    rows.extend(copy.deepcopy(release) for _ in range(10))
    assert evaluate_endurance(rows,report)['success']
    rows[-1]['contacts']['r3']['left']=True
    assert not evaluate_endurance(rows,report)['success']

@pytest.mark.parametrize('kind',['spacing','endurance'])
def test_integral_resists_persistent_rgb_error_but_stays_bounded_and_stops(monkeypatch,kind):
    import numpy as np
    from harness.pair_grasp_spacing import PairGraspSpacing
    a=PairGraspSpacing(data(),'r1',integral_gain=2.) if kind=='spacing' else EnduranceActor(data(),'r1','stationary',integral_gain=2.)
    initial=a.decide(rgb('anchor-r1-own.jpg'),rgb('anchor-top.jpg'))
    obs=copy.deepcopy(initial['observations'])
    for value in obs.values():value['xy_m'][1]+=.005
    monkeypatch.setattr(a.vision,'observe',lambda *args:copy.deepcopy(obs))
    first=a.decide(b'',b'')['action']['left']
    for _ in range(300):result=a.decide(b'',b'')
    assert result['ready'] and abs(result['action']['left'])>abs(first)
    assert np.max(np.abs(a.integral_effort))<=.08
    assert -.1<=result['action']['left']<=.1
    saved=a.integral_effort.copy()
    def lost(*args):raise ValueError('own camera missing')
    monkeypatch.setattr(a.vision,'observe',lost)
    result=a.decide(b'',b'')
    assert not result['ready'] and all(result['action'][k]==0 for k in ('forward','left','turn'))
    assert np.array_equal(saved,a.integral_effort)
