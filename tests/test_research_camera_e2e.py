import copy
import json

import pytest

from harness.research_camera_actor import agreed_roles, build_request, preparation_ready, validate_reply
from scripts.run_research_camera_e2e import evaluate_trial


def reply(**changes):
    return dict(reason="visual evidence",message="",status="READY",confidence=.9,
                checks=["at_grasp_pose","stopped"],command_id=None,action={"kind":"wait"},**changes)


def test_role_disagreement_never_gets_centrally_repaired():
    a={"roles":{"r1":"top_end","r3":"bottom_end"},"accept":True}
    b={"roles":{"r1":"bottom_end","r3":"top_end"},"accept":True}
    assert agreed_roles({"r1":a,"r3":b}) is None
    b=copy.deepcopy(a);b["accept"]=False
    assert agreed_roles({"r1":a,"r3":b}) is None
    assert agreed_roles({"r1":a,"r3":a})==a["roles"]


@pytest.mark.parametrize("change",[
    {"checks":["contact_truth"]},{"position":[1,2,3]},{"confidence":float("nan")},
    {"action":{"kind":"arm","servo_id":1,"pulse":1500}},
    {"action":{"kind":"drive","forward":.1,"turn":0,"duration_s":1}},
])
def test_rejects_truth_fields_and_unsafe_preparation(change):
    value=reply();value.update(change)
    with pytest.raises(ValueError):validate_reply(json.dumps(value),"PREPARE")


def test_readiness_is_not_proximity_or_command_issuance():
    value=reply();assert preparation_ready(value)
    for change in ({"checks":["stopped"]},{"confidence":.79},
                   {"action":{"kind":"arm","servo_id":1,"pulse":2000}},
                   {"status":"UNCERTAIN"}):
        candidate={**value,**change};assert not preparation_ready(candidate)


def test_request_whitelists_camera_context_and_communication_condition():
    camera={"images":{k:{"jpeg_base64":k} for k in ("own_rgb","top_rgb")},
            "truth_pose":"FORBIDDEN_TRUTH","plan":{"state":"FORBIDDEN_PLAN"},"stage":"GRASP"}
    kwargs=dict(roles=None,own_history=[{"command_id":"own"}],inbox=[{"text":"peer claim"}],previous=camera)
    request=build_request("r1","GRASP",camera,communication="none",**kwargs)
    assert "FORBIDDEN" not in json.dumps(request)
    context=json.loads(request["messages"][1]["content"])
    assert context["received_peer_claims"]==[]
    assert context["own_issued_commands"]==kwargs["own_history"]
    assert len(request["images"])==4
    request=build_request("r1","GRASP",camera,communication="natural",**kwargs)
    assert json.loads(request["messages"][1]["content"])["received_peer_claims"]==kwargs["inbox"]


def samples():
    rows=[]
    for i in range(40):
        grasp=i<25
        rows.append({"sim_time_s":i*.1,"height_above_start_m":.04 if grasp else 0,
            "contacts":{r:{"bilateral":grasp,"left":grasp,"right":grasp} for r in ("r1","r3")},
            "constraints_active":{"r1":False,"r3":False},"goal":{"success":not grasp}})
    return rows


def test_success_requires_lift_hold_goal_settling_and_both_fingers_released():
    assert evaluate_trial(samples())["physical_success"]
    for mutate in (lambda s:s[-1]["contacts"]["r1"].update(left=True),
                   lambda s:s[-5]["goal"].update(success=False),
                   lambda s:s[10]["constraints_active"].update(r1=True)):
        rows=samples();mutate(rows);assert not evaluate_trial(rows)["physical_success"]
    rows=samples()
    for row in rows:row["height_above_start_m"]=0
    assert not evaluate_trial(rows)["physical_success"]
    assert not evaluate_trial([])["physical_success"]
