"""Pure input-boundary tests for exact pixel-servo continuation."""
import hashlib
import json
from pathlib import Path

import pytest

from harness.camera_pixel_resume import (
    canonical, digest, load_resume_parent, replay_recorded_call,
    verify_live_last_input,
)


STARTUP = [{"kind":"arm","servo_id":1,"pulse":2000}]
BOUNDARY = "own/top RGB and command history only"


class FakeController:
    def __init__(self, *, wrong=False):
        self.history=canonical(STARTUP)
        self.last_observation={}
        self.last_decision={}
        self.wrong=wrong

    def step(self, own, top, active=True):
        action={"kind":"wait" if not self.wrong else "bogus"}
        self.last_observation={"own":own.decode(),"top":top.decode()}
        self.last_decision={"active":active}
        self.history.append(action)
        return action


def parent_run(root: Path):
    root.mkdir()
    (root/'r1').mkdir()
    own=b'own-rgb';top=b'top-rgb'
    (root/'r1/000-own.jpg').write_bytes(own)
    (root/'r1/000-top.jpg').write_bytes(top)
    images={
        "own":{"path":"r1/000-own.jpg","sha256":hashlib.sha256(own).hexdigest()},
        "top":{"path":"r1/000-top.jpg","sha256":hashlib.sha256(top).hexdigest()},
    }
    controller=FakeController();before=canonical(controller.history)
    action=canonical(controller.step(own,top,active=True))
    call={"round":0,"robot_id":"r1","active":True,"images":images,
          "input_sha256":digest({"images":images,"history":before,"active":True}),
          "action":action,"observation":canonical(controller.last_observation),
          "decision":canonical(controller.last_decision),"history":canonical(controller.history)}
    config={"out_dir":str(root),"rounds":1,"seed":11,"active_robot":"r1",
            "render_width":1280,"render_height":960,"settle_seconds":1.0,
            "weld":False,"policy":"pixel-servo"}
    report={"source_sha":"parent-sha","config":config,"config_sha256":digest(config),
            "input_boundary":BOUNDARY,
            "input_boundary_sha256":hashlib.sha256(BOUNDARY.encode()).hexdigest(),
            "startup_commands":canonical(STARTUP),"startup_history":{"r1":canonical(STARTUP)},
            "calls":[call],"rounds_completed":1,"error":None}
    (root/'result.json').write_text(json.dumps(report))
    # A resume helper must never consult or copy output-only evaluation data.
    (root/'evaluation-only.jsonl').write_text('not json and not policy input')
    return report,call


def load(root: Path, total=2):
    return load_resume_parent(root,startup_commands=STARTUP,seed=11,
                              active_robot='r1',render_width=1280,
                              render_height=960,total_rounds=total,
                              input_boundary=BOUNDARY)


def test_parent_validation_and_exact_replay_materialize_auditable_prefix(tmp_path):
    parent=tmp_path/'parent';report,call=parent_run(parent)
    loaded,groups=load(parent)
    output=tmp_path/'output';output.mkdir()

    action=replay_recorded_call(FakeController(),groups[0][0],parent,output)

    assert loaded==report and action=={"kind":"wait"}
    for record in call['images'].values():
        copied=output/record['path']
        assert copied.is_file() and not copied.is_symlink()
        assert hashlib.sha256(copied.read_bytes()).hexdigest()==record['sha256']


def test_parent_image_tampering_fails_closed(tmp_path):
    parent=tmp_path/'parent';parent_run(parent)
    (parent/'r1/000-top.jpg').write_bytes(b'tampered')

    with pytest.raises(ValueError,match='parent image hash mismatch'):
        load(parent)


def test_policy_replay_mismatch_fails_before_continuation(tmp_path):
    parent=tmp_path/'parent';_,call=parent_run(parent)
    output=tmp_path/'output';output.mkdir()

    with pytest.raises(ValueError,match='parent action replay mismatch'):
        replay_recorded_call(FakeController(wrong=True),call,parent,output)
    assert list(output.rglob('*.jpg'))==[]


def test_parent_config_schedule_and_total_budget_are_strict(tmp_path):
    parent=tmp_path/'parent';report,_=parent_run(parent)
    with pytest.raises(ValueError,match='total rounds must exceed'):
        load(parent,total=1)

    report['config']['seed']=12
    report['config_sha256']=digest(report['config'])
    (parent/'result.json').write_text(json.dumps(report))
    with pytest.raises(ValueError,match='config mismatch: seed'):
        load(parent)


@pytest.mark.parametrize("mutation,match", [
    (lambda report: report.update(input_boundary="changed"), "input boundary mismatch"),
    (lambda report: report.update(input_boundary_sha256="0" * 64),
     "input boundary hash mismatch"),
    (lambda report: report["calls"][0].update(active=False), "active schedule mismatch"),
    (lambda report: report["calls"].append(canonical(report["calls"][0])),
     "call coverage is invalid"),
    (lambda report: report["calls"][0]["images"].update(extra={}),
     "image keys mismatch"),
])
def test_parent_boundary_schedule_duplicates_and_image_schema_fail_closed(
        tmp_path, mutation, match):
    parent=tmp_path/'parent';report,_=parent_run(parent)
    mutation(report)
    (parent/'result.json').write_text(json.dumps(report))

    with pytest.raises(ValueError,match=match):
        load(parent)


def test_live_last_input_hash_gate_accepts_exact_bytes_and_rejects_each_camera(tmp_path):
    parent=tmp_path/'parent';_,call=parent_run(parent)
    own=(parent/call['images']['own']['path']).read_bytes()
    top=(parent/call['images']['top']['path']).read_bytes()

    verified=verify_live_last_input({'r1':own},top,[call],0)
    assert verified == {
        'round':0,
        'top_sha256':call['images']['top']['sha256'],
        'own_sha256':{'r1':call['images']['own']['sha256']},
    }
    with pytest.raises(ValueError,match='live own image mismatch'):
        verify_live_last_input({'r1':b'wrong'},top,[call],0)
    with pytest.raises(ValueError,match='live top image mismatch'):
        verify_live_last_input({'r1':own},b'wrong',[call],0)
