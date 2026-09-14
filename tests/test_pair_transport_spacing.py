"""Preserve the pre-lift RGB anchor and reject the recorded pre-drive collapse."""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from harness.pair_grasp_spacing import PairGraspSpacing, HOLD_DT
from harness.pair_navigation import ROBOTS, authorize_pair
from harness.pair_carry_sync import PairCarrySync
from scripts.evaluate_pair_navigation import evaluate_grasp_stability

FIX = Path(__file__).parent/'fixtures/pair_grasp_spacing'
MAP = Path(__file__).parents[1]/'maps/pair_navigation/narrow-door.json'


def read(name):
    raw=(FIX/name).read_bytes()
    assert hashlib.sha256(raw).hexdigest()==json.loads((FIX/'manifest.json').read_text())['files'][name]
    return raw


@pytest.mark.parametrize('rid',ROBOTS)
def test_real_11cm_contraction_cannot_be_adopted_as_a_new_anchor(rid):
    actor=PairGraspSpacing(json.loads(MAP.read_text()),rid)
    initial=actor.decide(read(f'anchor-{rid}-own.jpg'),read('anchor-top.jpg'))
    assert initial['ready'] and initial['status']=='spacing_anchor'
    anchor=copy.deepcopy(actor.anchor)
    result=actor.decide(read(f'collapsed-{rid}-own.jpg'),read('collapsed-top.jpg'))
    assert not result['ready'] and result['status']=='spacing_stop'
    assert all(result['action'][k]==0 for k in ('forward','left','turn'))
    assert all(np.array_equal(anchor[r],actor.anchor[r]) for r in ROBOTS)
    assert actor.decide(read(f'anchor-{rid}-own.jpg'),read('anchor-top.jpg'))['status']=='spacing_stop'


def test_small_inward_drift_produces_opposing_outward_commands(monkeypatch):
    data=json.loads(MAP.read_text());actors={r:PairGraspSpacing(data,r) for r in ROBOTS}
    for rid,actor in actors.items():
        initial=actor.decide(read(f'anchor-{rid}-own.jpg'),read('anchor-top.jpg'))
        obs=copy.deepcopy(initial['observations'])
        obs['r1']['xy_m'][1]+=.004
        obs['r3']['xy_m'][1]-=.004
        monkeypatch.setattr(actor.vision,'observe',lambda a,b,obs=obs:copy.deepcopy(obs))
    decisions={r:a.decide(b'',b'') for r,a in actors.items()}
    assert decisions['r1']['action']['left']<0<decisions['r3']['action']['left']
    sync=PairCarrySync('hold')
    assert authorize_pair(sync,decisions,{r:1 for r in ROBOTS},0,interval_s=HOLD_DT)['phase']=='GO'
    decisions['r3']['ready']=False
    assert authorize_pair(sync,decisions,{r:2 for r in ROBOTS},1,interval_s=HOLD_DT)['phase']=='HOLD'


def test_invalid_owned_camera_stops_even_if_top_is_valid():
    actor=PairGraspSpacing(json.loads(MAP.read_text()),'r1')
    d=actor.decide(b'invalid RGB',read('anchor-top.jpg'))
    assert not d['ready'] and d['status']=='spacing_stop'


def test_original_transport_success_fails_whole_grasp_stability():
    rows=json.loads(read('baseline-evaluation-only.json'))
    result=evaluate_grasp_stability(rows)
    assert not result['success']
    assert .107<result['max_spacing_change_m']<.110
    assert result['nonbilateral_samples']==3
    assert not result['gates']['grasp_spacing_preserved']
    assert not result['gates']['grasp_bilateral_through_lift_hold']


def test_stability_evaluation_requires_full_hold_and_not_just_its_last_two_seconds():
    rows=json.loads(read('baseline-evaluation-only.json'))
    rows=[r for r in rows if r['phase']!='grasp_hold' or r['sim_time_s']>22.5]
    assert not evaluate_grasp_stability(rows)['gates']['full_grasp_samples']
