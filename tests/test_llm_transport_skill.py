from unittest.mock import Mock
import pytest
from harness.llm_transport_skill import LLMTransportSkill


def test_navigation_and_release_require_planner_choice():
    skill=LLMTransportSkill('r1','small_box_01','B')
    skill.state='carrying';skill.box.phase='carry'
    skill.box.decide=Mock(return_value={'kind':'wait','duration':.05})
    assert skill.advance({'own':'frame'}) is None
    assert skill.operation is None
    assert skill.request({'kind':'drive','fwd':.1,'turn':-.1,'duration':.8})['turn']==-.1
    assert skill.box.phase=='carry'
    assert skill.request({'kind':'wait','duration':.5})['kind']=='wait'
    skill.request({'kind':'release'}, {'status':'inside','stage':'before_release','source':'own_rgb'})
    assert skill.operation=='release' and skill.box.phase=='release'


def test_approach_stops_before_grasp_until_model_pick():
    skill=LLMTransportSkill('r1','small_box_01','B')
    skill.request({'kind':'approach'})
    hover={'kind':'pose','pulses':{1:2000,3:700}}
    def observe(_):
        skill.box.phase='lower'
        return hover
    skill.box.decide=observe
    assert skill.advance({}) is None
    assert skill.state=='ready_to_pick' and skill.operation is None
    assert skill.request({'kind':'pick'})==hover
    assert skill.operation=='pick'


def test_invalid_phase_cannot_finish_or_pick_and_guard_cannot_be_ignored():
    skill=LLMTransportSkill('r2','small_box_02','A')
    with pytest.raises(ValueError):skill.request({'kind':'finish'})
    with pytest.raises(ValueError):skill.request({'kind':'pick'})
    skill.state='carrying'
    skill.box.decide=Mock(return_value={'kind':'finish','reason':'VISUAL_LOAD_DROPPED'})
    assert skill.advance({})['reason']=='VISUAL_LOAD_DROPPED'
    assert skill.state=='failure'


def test_release_and_finish_require_fresh_camera_inside_evidence():
    skill=LLMTransportSkill('r1','small_box_01','B')
    skill.state='carrying';skill.box.phase='carry'
    with pytest.raises(ValueError,match='SAFE_INSIDE'):skill.request({'kind':'release'})
    with pytest.raises(ValueError,match='SAFE_INSIDE'):
        skill.request({'kind':'release'},{'status':'outside','stage':'before_release','source':'own_rgb'})
    skill.request({'kind':'release'},{'status':'inside','stage':'before_release','source':'own_rgb'})
    skill.state='released';skill.operation=None;skill.placement_evidence=None
    with pytest.raises(ValueError,match='CAMERA_INSIDE'):skill.request({'kind':'finish'})
    with pytest.raises(ValueError,match='CAMERA_INSIDE'):
        skill.request({'kind':'finish'},{'status':'uncertain','stage':'released','source':'own_rgb'})
    result=skill.request({'kind':'finish'},{'status':'inside','stage':'released','source':'own_rgb'})
    assert result['reason']=='VISUAL_RELEASE_CONFIRMED'


def test_explicit_post_release_approach_rebuilds_stale_visual_box_only_then():
    skill=LLMTransportSkill('r1','small_box_01','B')
    skill.state='released';old=skill.box
    old.phase='carry';old.last_attachment={'stale':True};skill.pending_hover={'stale':True}
    skill.observe_placement({'status':'outside','stage':'released','source':'own_rgb'})
    # Evidence alone never starts a grasp recovery.
    assert skill.operation is None and skill.box is old
    skill.request({'kind':'approach'})
    assert skill.operation=='approach' and skill.state=='approaching'
    assert skill.box is not old and skill.box.last_attachment is None
    assert skill.box.near_field_reacquisition is True
    assert skill.pending_hover is None and skill.recovery_attempts==1
    with pytest.raises(ValueError,match='PICK_REQUIRES'):
        skill.request({'kind':'pick'})


def test_post_release_recovery_is_bounded_and_rejects_truth_fields():
    skill=LLMTransportSkill('r1','small_box_01','B')
    with pytest.raises(ValueError,match='TRUTH'):
        skill.observe_placement({'status':'outside','stage':'released','position':[1,2]})
    for attempt in range(3):
        skill.state='released';skill.operation=None
        skill.request({'kind':'approach'},{'status':'uncertain','stage':'released','source':'own_rgb'})
    skill.state='released';skill.operation=None
    with pytest.raises(ValueError,match='EXHAUSTED'):
        skill.request({'kind':'approach'},{'status':'outside','stage':'released','source':'own_rgb'})


def test_normal_initial_approach_does_not_enable_near_field_reacquisition():
    skill=LLMTransportSkill('r1','small_box_01','B')
    skill.request({'kind':'approach'})
    assert skill.box.near_field_reacquisition is False
