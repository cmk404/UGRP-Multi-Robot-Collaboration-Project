"""Keep the local UI wired to the original agreed-plan executor."""
from functools import partial
import json
from types import SimpleNamespace

import pytest

from harness.dispatch_plan import build_dispatch_request, fixture_plan, validate_dispatch_reply
from harness.dispatch_skill_binding import SkillBindings
from harness.three_robot_plan import TeamAgreement, ROBOTS
from sim.research_dispatch_arena import actor_task, authored_map
from scripts import sim_dispatch
from scripts.three_robot_runtime import ThreeRobotRuntime


def test_cli_dispatch_reuses_skills_entry_with_operator_goal(monkeypatch, tmp_path):
    from scripts.sim_cli import main
    monkeypatch.setattr('sim.workflow_manager.RECORDS', tmp_path / 'fixture-records')
    invoked = []
    monkeypatch.setattr('scripts.run_dispatch_e2e.main', lambda argv: invoked.append(argv) or 0)
    monkeypatch.setattr(sim_dispatch, 'bundled_models', lambda: (tmp_path/'grasp', tmp_path/'stages'))
    assert main(['dispatch', '--headless', '--task', 'dock_b로 함께 옮겨',
                 '--required-dock', 'dock_b', '--variant', 'open', '--model', 'test-model']) == 0
    args = invoked[0]
    assert args[args.index('--executor')+1] == 'skills'
    assert args[args.index('--task')+1] == 'dock_b로 함께 옮겨'
    assert args[args.index('--model')+1] == 'test-model'
    assert '--viewer' not in args
    assert '--efficient-capture' in args
    assert '--auto-route-overlap' in args
    assert args[-4:] == ['--required-dock', 'dock_b', '--variant', 'open']


def test_standard_dispatch_can_restore_serial_route_and_full_capture(monkeypatch, tmp_path):
    invoked=[]
    monkeypatch.setattr('scripts.run_dispatch_e2e.main', lambda argv: invoked.append(argv) or 0)
    assert sim_dispatch.main(['--headless','--plan-replay','saved.json',
                              '--grasp-model-dir',str(tmp_path/'grasp'),
                              '--stage-model-dir',str(tmp_path/'stage'),
                              '--full-capture','--serial-route'])==0
    assert '--efficient-capture' not in invoked[0]
    assert '--auto-route-overlap' not in invoked[0]
    assert '--route-overlap' not in invoked[0]


@pytest.mark.parametrize('args', [
    ['--serial-route','--route-overlap'],
    ['--serial-route','--auto-route-overlap'],
    ['--full-capture','--efficient-capture'],
])
def test_standard_dispatch_rejects_conflicting_performance_options(monkeypatch, args):
    monkeypatch.setattr(sim_dispatch.sys.stdin, 'isatty', lambda: False)
    with pytest.raises(SystemExit) as error:
        sim_dispatch.main(['--headless']+args)
    assert error.value.code==2


def test_research_runner_defaults_and_explicit_overlap_flags(monkeypatch, tmp_path):
    from scripts import run_dispatch_e2e
    seen=[]
    monkeypatch.setattr('scripts.run_dispatch_skills.run', lambda args: seen.append(args) or 0)
    base=['--executor','skills','--output',str(tmp_path/'one'),
          '--grasp-model-dir',str(tmp_path/'grasp'),
          '--stage-model-dir',str(tmp_path/'stage')]
    assert run_dispatch_e2e.main(base)==0
    assert not seen[-1].efficient_capture and not seen[-1].route_overlap and not seen[-1].auto_route_overlap
    assert run_dispatch_e2e.main([*base[:3],str(tmp_path/'two'),*base[4:],
                                  '--route-overlap','--overlap-start','grasp'])==0
    assert seen[-1].route_overlap and not seen[-1].auto_route_overlap
    assert seen[-1].overlap_start=='grasp'


@pytest.mark.parametrize('args', [
    ['--executor', 'raw'], ['--executor=raw'],
    ['--task', '새 작업', '--plan-replay', 'plan.json'],
    ['--grasp-model-dir', 'incomplete'],
])
def test_launcher_rejects_silent_protocol_switches(args, monkeypatch):
    monkeypatch.setattr(sim_dispatch.sys.stdin, 'isatty', lambda: False)
    with pytest.raises(SystemExit) as error:
        sim_dispatch.main(['--headless'] + args)
    assert error.value.code == 2


def test_operator_instruction_reaches_all_peers_and_original_programs_follow_plan(tmp_path):
    static_map = authored_map('open')
    task = actor_task(static_map)
    task['operator_instruction'] = '서로 역할과 순서를 합의해'
    plan = fixture_plan(solo='r1', dock='dock_b')
    frames = {r: dict(own_bytes=b'own-'+r.encode(), top_bytes=b'top',
                     own_rgb={'path':r+'.jpg'}, shared_top_rgb={'path':'top.jpg'}, frame_id=1)
              for r in ROBOTS}
    polls = []
    team = ThreeRobotRuntime(tmp_path, run_id='local-ui', mode='fixture',
        plan_fixture=plan, agreement=TeamAgreement('local-ui', plan_validator=lambda p:p),
        request_builder=partial(build_dispatch_request, task=task, execution_pilot=True),
        reply_validator=validate_dispatch_reply, roles_fixed_by_skill=False,
        idle_callback=lambda: polls.append(True))
    try:
        for turn in range(4):
            if team.negotiate(frames, {r:[] for r in ROBOTS}, turn, 0):
                break
        assert team.agreement.committed
        bound = SkillBindings(team.agreement.committed, static_map)
        assert bound.solo == 'r1'
        assert set(bound.pair.values()) == {'r2', 'r3'}
        assert bound.programs['r1'][0]['object'] == 'box'
        assert all(row['execution_adapter'] == 'existing_visual_box_v1' for row in bound.programs['r1'])
        for rid in ROBOTS:
            requests = list((tmp_path/rid).glob('*-request.json'))
            assert requests
            request = json.loads(requests[0].read_text())
            context = json.loads(request['messages'][1]['content'])
            assert context['mission']['operator_instruction'] == task['operator_instruction']
        assert polls and all(c['model'] == 'scripted-fixture-not-llm' for c in team.calls)
    finally:
        team.close(0)


def test_bundled_models_do_not_overwrite_changes(tmp_path, monkeypatch):
    import zipfile
    monkeypatch.setattr(sim_dispatch, 'ROOT', tmp_path)
    bundle = tmp_path/'experiments/dispatch-skill-integration-20260917/models.zip'
    bundle.parent.mkdir(parents=True)
    with zipfile.ZipFile(bundle, 'w') as archive:
        archive.writestr('models/grasp/student-skill.json', '{}')
        archive.writestr('models/varied/varied-start-skill.json', '{}')
    grasp, stage = sim_dispatch.bundled_models()
    assert sim_dispatch.bundled_models() == (grasp, stage)
    (grasp/'student-skill.json').write_text('user edit')
    with pytest.raises(ValueError, match='changed'):
        sim_dispatch.bundled_models()
    assert (grasp/'student-skill.json').read_text() == 'user edit'


def test_native_observer_cannot_mutate_research_physics(monkeypatch):
    mujoco = pytest.importorskip('mujoco')
    import mujoco.viewer
    import contextlib
    from scripts.dispatch_native_view import DispatchNativeView
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>')
    data = mujoco.MjData(model)
    original_qpos = data.qpos.copy()
    original_gravity = model.opt.gravity.copy()
    class Viewer:
        cam = SimpleNamespace(lookat=[0.,0.,0.])
        def lock(self): return contextlib.nullcontext()
        def is_running(self): return True
        def sync(self):
            # Simulate native reset/physics/actuator panel writes.
            self.data.qpos[:] = 2
            self.model.opt.gravity[:] = 0
        def close(self): pass
        def _sim(self): return None
    def launch(model, data, **kwargs):
        viewer = Viewer(); viewer.model = model; viewer.data = data
        return viewer
    monkeypatch.setattr(mujoco.viewer, 'launch_passive', launch)
    scene = SimpleNamespace(world=SimpleNamespace(model=model, data=data), time=lambda:float(data.time), deadline=None)
    view = DispatchNativeView(scene)
    try:
        assert (data.qpos == original_qpos).all()
        assert (model.opt.gravity == original_gravity).all()
        view.keys.put(81)
        with pytest.raises(KeyboardInterrupt, match='quit'):
            view.poll()
    finally:
        view.close()


def _clocked_native_view(monkeypatch, *, factor=1., oversleep=0.):
    pytest.importorskip('mujoco')
    import queue
    from scripts import dispatch_native_view

    class Clock:
        now = 0.
        sleeps = []
        def monotonic(self): return self.now
        def sleep(self, seconds):
            self.sleeps.append(seconds)
            self.now += seconds + oversleep

    class Viewer:
        running = True
        polls = 0
        def is_running(self):
            self.polls += 1
            return self.running

    clock = Clock()
    monkeypatch.setattr(dispatch_native_view, 'time', clock)
    sim = [0.]
    scene = SimpleNamespace(time=lambda: sim[0], deadline=None)
    view = dispatch_native_view.DispatchNativeView.__new__(dispatch_native_view.DispatchNativeView)
    view.scene, view.viewer, view.keys = scene, Viewer(), queue.SimpleQueue()
    view.factor, view.paused = factor, False
    view.pace_sim = view.pace_wall = view.last_tick_wall = 0.
    view.rebase_pace, view.next_poll, view.next_sync = False, 0., float('inf')
    return view, clock, sim


@pytest.mark.parametrize('factor,expected_wall', [(1.,1.), (2.,.5)])
def test_native_pacing_batches_poll_and_sleep_without_oversleep_drift(monkeypatch, factor, expected_wall):
    view, clock, sim = _clocked_native_view(monkeypatch, factor=factor, oversleep=.002)
    for _ in range(4000):
        view.tick()
        sim[0] += .00025
        clock.now += .00001  # Other work in the unchanged fine physics step.
    assert abs(clock.now - expected_wall) < .012
    assert len(clock.sleeps) < 220
    assert all(.005 <= delay <= .01 for delay in clock.sleeps)
    assert view.viewer.polls < 130  # A wall-time cadence, not 4000 GUI calls.


def test_native_pacing_rebases_after_long_inference_and_operator_pause(monkeypatch):
    view, clock, sim = _clocked_native_view(monkeypatch)
    for _ in range(400):
        view.tick(); sim[0] += .00025; clock.now += .00001
    clock.now += 10.  # Physics was stopped during an external model call.
    view.tick()
    resumed = clock.now
    for _ in range(400):
        sim[0] += .00025; view.tick(); clock.now += .00001
    assert .09 <= clock.now - resumed <= .11
    view.paused = True
    view.keys.put(32)  # Space resumes even when the pause is brief.
    clock.now += .05
    view.tick()
    assert not view.paused and not view.rebase_pace
    assert view.pace_wall >= resumed


def test_native_pacing_checks_budget_every_step_and_polls_quit_promptly(monkeypatch):
    view, clock, sim = _clocked_native_view(monkeypatch)
    view.next_poll = 1.
    view.scene.deadline = .001
    clock.now = .001
    with pytest.raises(RuntimeError, match='wall budget'):
        view.tick()
    view.scene.deadline = None
    clock.now = 0.
    view.next_poll = .01
    view.keys.put(81)
    with pytest.raises(KeyboardInterrupt, match='quit'):
        for _ in range(100):
            view.tick(); sim[0] += .00025; clock.now += .0001
    assert clock.now < .02


def test_real_native_observer_lifecycle():
    import os
    if os.environ.get('UGRP_TEST_NATIVE_VIEWER') != '1':
        pytest.skip('explicit native display probe only')
    import mujoco
    from scripts.dispatch_native_view import DispatchNativeView
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body pos="0 0 1"><freejoint/><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>')
    data = mujoco.MjData(model)
    scene = SimpleNamespace(world=SimpleNamespace(model=model, data=data), time=lambda:float(data.time), deadline=None)
    view = DispatchNativeView(scene)
    try:
        for _ in range(50):
            view.tick()
            mujoco.mj_step(model, data)
        assert view.viewer.is_running()
        assert data.time > 0
        assert view.model is not model and view.data is not data
    finally:
        view.close()
