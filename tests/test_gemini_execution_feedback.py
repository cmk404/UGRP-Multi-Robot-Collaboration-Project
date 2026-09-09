"""Offline integration checks for executor feedback wiring; no model calls."""
import io
import json

from harness.visual_macro_runtime import VisualMacroExecutor
from harness.gemini_transport_policy import GeminiTransportPlanner
from scripts.evaluate_gemini_team import record_command
from tests.test_budget_repair_regressions import OfflineCompleter, actor_obs


class Port:
    robot_id = 'r1'

    def __init__(self):
        self.applied = []
        self.stops = 0

    def tick(self, now):
        pass

    def stop(self):
        self.stops += 1

    def apply(self, action, now):
        self.applied.append((now, dict(action)))


def test_completed_executor_macro_reaches_next_planner_feedback():
    stream = io.StringIO()
    current_decision = {'r1': 'r1-call-0001-decision'}
    execution_feedback = {'r1': {}}

    def callback(row):
        record_command(row, commands=stream, current_decision=current_decision,
                       execution_feedback=execution_feedback)

    executor = VisualMacroExecutor(Port(), log_callback=callback)
    executor.submit({'kind': 'drive', 'fwd': .1, 'turn': 0, 'duration': .25},
                    {'sha256': 'offline-frame', 'actuator_state': {'servo_pulses': {}}},
                    'carrying', 0.0)
    executor.tick(.5)

    feedback = execution_feedback['r1']['last_macro']
    assert feedback['decision_id'] == 'r1-call-0001-decision'
    assert feedback['macro']['kind'] == 'drive'
    assert feedback['status'] == 'completed'
    assert feedback['elapsed_drive_control_s'] == .25

    rows = [json.loads(line) for line in stream.getvalue().splitlines()]
    finished = next(row for row in rows if row['event'] == 'macro_finished')
    assert finished['decision_id'] == 'r1-call-0001-decision'
    assert finished['execution'] == {key: value for key, value in feedback.items()
                                     if key != 'decision_id'}

    completer = OfflineCompleter()
    planner = GeminiTransportPlanner('r1', completer)
    planner.decide(actor_obs('robot_cam'), actor_obs('nav_cam'), [],
                   cargo_id='small_box_01', destination_zone='A', skill_state='carrying',
                   execution_feedback=execution_feedback['r1'])
    transmitted = json.loads(completer.messages[-1]['content'])
    assert transmitted['execution_feedback']['last_macro'] == feedback
    assert planner.last_audit['model_context']['execution_feedback']['last_macro'] == feedback
