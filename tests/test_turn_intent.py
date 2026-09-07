import unittest
from pathlib import Path
from harness.web import explicit_empty_gripper_statement, spatial_context_statement, turn_intent
from harness.loop import ReplayCompleter, run_loop
from harness.catalog import default_registry
from harness.protocol import system_prompt

class TurnIntentTests(unittest.TestCase):
    def test_plain_chat_is_conversation_only(self):
        self.assertEqual(turn_intent('안녕. 응답 돼?'), (False, False))
        self.assertEqual(turn_intent('오늘 상태 어때?'), (False, False))
    def test_visual_question_uses_camera_without_motion(self):
        self.assertEqual(turn_intent('지금 화면에 뭐 보여?'), (False, True))
    def test_location_statement_gets_grounded_ack_without_scene_invention(self):
        self.assertEqual(
            spatial_context_statement('빨간 블록이 오른쪽에 있어'),
            '알겠어. 오른쪽에 있다는 위치 힌트로 기억할게.',
        )
        self.assertIsNone(spatial_context_statement('오른쪽에 뭐 있어?'))

    def test_location_statements_do_not_move_robot(self):
        self.assertEqual(turn_intent('오른쪽에 있어'), (False, True))
        self.assertEqual(turn_intent('빨간 블록이 오른쪽에 있어'), (False, True))
        self.assertEqual(turn_intent('빨간 블록이 보여'), (False, True))

    def test_directional_commands_still_move_robot(self):
        self.assertEqual(turn_intent('오른쪽으로 가'), (True, True))
        self.assertEqual(turn_intent('왼쪽으로 이동해줘'), (True, True))
        self.assertEqual(turn_intent('우회전해'), (True, True))

    def test_robot_requests_use_agent(self):
        self.assertEqual(turn_intent('빨간 블럭 찾아줘'), (True, True))
        self.assertEqual(turn_intent('빨간 블럭을 파란 블럭 위에 올려줘'), (True, True))

    def test_explicit_empty_gripper_correction_is_not_an_action(self):
        self.assertTrue(explicit_empty_gripper_statement('너 손에 빨간 블럭 없는데'))
        self.assertTrue(explicit_empty_gripper_statement('집게에 아무것도 없어'))
        self.assertTrue(explicit_empty_gripper_statement('집게 비었어'))
        self.assertFalse(explicit_empty_gripper_statement('손에 빨간 블럭 없어?'))
        self.assertFalse(explicit_empty_gripper_statement('빨간 블럭이 오른쪽에 없어'))
    def test_conversation_prompt_has_no_tools(self):
        p=system_prompt('- pick', conversation_only=True)
        self.assertIn('conversation-only',p)
        self.assertNotIn('Available tools',p)
    def test_conversation_mode_rejects_tool_then_final(self):
        seen=[]
        r=run_loop(ReplayCompleter([{'tool':'approach'},{'final':'안녕'}]), default_registry(runner=lambda n: seen.append(n)), '안녕', conversation_only=True, execute=True)
        self.assertEqual(seen,[])
        self.assertEqual(r.final,'안녕')
        self.assertIn('conversation-only',r.steps[0].error)

if __name__=='__main__': unittest.main()

class ConversationPlainTextFallbackTests(unittest.TestCase):
    def test_plain_text_model_output_becomes_final_without_error(self):
        r=run_loop(ReplayCompleter(['그냥 평문 답변']), default_registry(), '뭐 보여?', conversation_only=True, execute=False)
        self.assertEqual(r.final, '그냥 평문 답변')
        self.assertEqual(r.stopped, 'final')
        self.assertFalse(any(step.error for step in r.steps))

    def test_compound_workspace_commands_are_agent_turns(self):
        for text in (
            '세 블럭 한 곳에 모아봐',
            '블록들 정리해줘',
            '저쪽에 배치해봐',
            '앞에 있는 거 치워줘',
            '파란 블럭 가져와',
        ):
            with self.subTest(text=text):
                self.assertEqual(turn_intent(text), (True, True))
