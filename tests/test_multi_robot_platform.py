import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from harness import team_bus
from harness import web as harness_web
from sim.bridge import BRIDGE_ALLOWED_ACTIONS, BRIDGE_COLOR_ACTIONS, BRIDGE_DESTINATION_ACTIONS, BridgeState
from scripts import sim_actions


class TeamBusTests(unittest.TestCase):
    def _start_consensus_round(self):
        posted = team_bus.post_operator_chat("세 로봇이 각자 블럭을 찾아 동시에 이동해", namespace="sim")
        consensus = posted["consensus_round"]
        self.assertEqual(consensus["status"], "collecting")
        self.assertEqual(consensus["targets"], ["r1", "r2", "r3"])
        return posted, consensus["id"]

    def _submit_consensus_proposal(self, round_id, robot_id, action="approach"):
        return team_bus.submit_consensus_proposal(
            round_id,
            robot_id,
            {
                "action": action,
                "target_color": {"r1": "red", "r2": "blue", "r3": "yellow"}[robot_id],
            },
            namespace="sim",
        )

    def test_operator_team_chat_creates_three_robot_consensus_round(self):
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            posted, round_id = self._start_consensus_round()

            self.assertEqual(posted["targets"], ["r1", "r2", "r3"])
            proposal_wakes = posted["wakeups"]
            self.assertEqual([w["robot_id"] for w in proposal_wakes], ["r1", "r2", "r3"])
            self.assertTrue(all(w["reason"] == "consensus_proposal" for w in proposal_wakes))
            self.assertTrue(all(w["consensus_round_id"] == round_id for w in proposal_wakes))
            self.assertFalse(any(w.get("execution_ready") for w in proposal_wakes))

    def test_consensus_does_not_commit_before_all_three_proposals(self):
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            _, round_id = self._start_consensus_round()

            first = self._submit_consensus_proposal(round_id, "r1")
            second = self._submit_consensus_proposal(round_id, "r2")

            self.assertFalse(first["committed"])
            self.assertFalse(second["committed"])
            snap = team_bus.snapshot(namespace="sim")
            round_state = next(r for r in snap["consensus_rounds"] if r["id"] == round_id)
            self.assertEqual(round_state["status"], "collecting")
            self.assertEqual(set(round_state["proposals"]), {"r1", "r2"})
            execution_wakes = [
                w for w in snap["wakeups"]
                if w.get("consensus_round_id") == round_id and w.get("reason") == "consensus_execute"
            ]
            self.assertEqual(execution_wakes, [], "no robot may act before the third proposal arrives")

    def test_third_proposal_commits_and_releases_three_execution_wakeups_together(self):
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            _, round_id = self._start_consensus_round()
            self._submit_consensus_proposal(round_id, "r1")
            self._submit_consensus_proposal(round_id, "r2")

            committed = self._submit_consensus_proposal(round_id, "r3")

            self.assertTrue(committed["committed"])
            self.assertEqual(committed["consensus_round"]["status"], "committed")
            execution_wakes = committed["execution_wakeups"]
            self.assertEqual([w["robot_id"] for w in execution_wakes], ["r1", "r2", "r3"])
            self.assertTrue(all(w["reason"] == "consensus_execute" for w in execution_wakes))
            self.assertTrue(all(w["execution_ready"] for w in execution_wakes))
            self.assertTrue(all(w["consensus_round_id"] == round_id for w in execution_wakes))
            self.assertEqual(
                {w["available_at"] for w in execution_wakes},
                {execution_wakes[0]["available_at"]},
                "consensus commit must release all three robot executions simultaneously",
            )

    def test_goal_peer_message_and_ack_are_shared_without_central_planner(self):
        with tempfile.TemporaryDirectory() as td, \
             patch.object(team_bus, "BUS_PATH", Path(td) / "bus.json"), \
             patch.object(team_bus, "LOCK_PATH", Path(td) / "bus.lock"):
            team_bus.reset()
            goal = team_bus.set_goal("세 로봇이 협력해서 블록을 옮긴다", source="test")
            msg = team_bus.send_message(
                "r1", "r2", "오른쪽을 확인해줘", proposed_action="observe_scene"
            )
            acked = team_bus.acknowledge("r2", msg["id"], True, "확인")
            ctx = team_bus.context_for("r2")

            self.assertEqual(goal["text"], "세 로봇이 협력해서 블록을 옮긴다")
            self.assertEqual(ctx["messages"][0]["id"], msg["id"])
            self.assertTrue(acked["ack"]["r2"]["accepted"])

    def test_team_chat_mentions_and_plain_group_text_create_expected_wakeups(self):
        with tempfile.TemporaryDirectory() as td, \
             patch.object(team_bus, "BUS_PATH", Path(td) / "bus.json"), \
             patch.object(team_bus, "LOCK_PATH", Path(td) / "bus.lock"):
            team_bus.reset()
            direct = team_bus.post_operator_chat("@R2 상태만 알려줘")
            self.assertEqual(direct["targets"], ["r2"])
            self.assertEqual(direct["chat"]["author"], "operator")
            self.assertEqual(direct["chat"]["targets"], ["r2"])
            self.assertEqual([w["robot_id"] for w in direct["wakeups"]], ["r2"])

            group = team_bus.post_operator_chat("현재 상황 공유해줘")
            self.assertEqual(group["targets"], ["r1", "r2", "r3"])
            self.assertEqual([w["robot_id"] for w in group["wakeups"]], ["r1", "r2", "r3"])
            self.assertLess(group["wakeups"][0]["available_at"], group["wakeups"][1]["available_at"])
            self.assertLess(group["wakeups"][1]["available_at"], group["wakeups"][2]["available_at"])

            everyone = team_bus.post_operator_chat("@TEAM 모두 확인")
            self.assertEqual(everyone["targets"], ["r1", "r2", "r3"])

    def test_peer_handoff_is_visible_in_room_and_wakes_only_recipient(self):
        with tempfile.TemporaryDirectory() as td, \
             patch.object(team_bus, "BUS_PATH", Path(td) / "bus.json"), \
             patch.object(team_bus, "LOCK_PATH", Path(td) / "bus.lock"):
            team_bus.reset()
            msg = team_bus.send_message("r1", "r3", "이 위치를 확인해줘", proposed_action="observe_scene")
            snap = team_bus.snapshot()
            self.assertEqual(snap["chat"][-1]["role"], "peer")
            self.assertEqual(snap["chat"][-1]["author"], "r1")
            self.assertEqual(snap["chat"][-1]["targets"], ["r3"])
            pending = [w for w in snap["wakeups"] if w["status"] == "pending"]
            self.assertEqual([(w["robot_id"], w["reason"]) for w in pending], [("r3", "peer_message")])
            self.assertEqual(msg["wakeup_ids"], [pending[0]["id"]])

    def test_dispatcher_owned_wakeup_is_not_absorbed_by_its_own_team_context(self):
        with tempfile.TemporaryDirectory() as td, \
             patch.object(team_bus, "BUS_PATH", Path(td) / "bus.json"), \
             patch.object(team_bus, "LOCK_PATH", Path(td) / "bus.lock"):
            team_bus.reset()
            posted = team_bus.post_operator_chat("@R1 블록 모아봐")
            wake = team_bus.claim_wakeup()
            self.assertEqual(wake["status"], "claimed")
            team_bus.context_for("r1", exclude_wakeup_id=wake["id"])
            snap = team_bus.snapshot()
            self.assertEqual(snap["wakeups"][0]["status"], "claimed")
            released = team_bus.release_wakeup(wake["id"], delay_s=1.0, error="TPM")
            self.assertEqual(released["status"], "pending")
            self.assertEqual(released["attempts"], 1)
            self.assertEqual(posted["chat"]["id"], released["source_chat_id"])

    def test_active_planner_context_absorbs_pending_wakeup_without_resurrection(self):
        with tempfile.TemporaryDirectory() as td, \
             patch.object(team_bus, "BUS_PATH", Path(td) / "bus.json"), \
             patch.object(team_bus, "LOCK_PATH", Path(td) / "bus.lock"):
            team_bus.reset()
            posted = team_bus.post_operator_chat("@R2 새 정보야")
            wake = team_bus.claim_wakeup()
            self.assertEqual(wake["robot_id"], "r2")
            self.assertEqual(wake["status"], "claimed")

            ctx = team_bus.context_for("r2")
            self.assertEqual(ctx["chat"][-1]["id"], posted["chat"]["id"])
            absorbed = team_bus.snapshot()["wakeups"][-1]
            self.assertEqual(absorbed["status"], "absorbed")

            # Dispatcher may receive HTTP 409 after the already-running turn
            # consumes the message. release_wakeup must not requeue it.
            released = team_bus.release_wakeup(wake["id"], delay_s=0.01)
            self.assertEqual(released["status"], "absorbed")
            self.assertIsNone(team_bus.claim_wakeup())

    def test_team_chat_mentions_route_only_target_and_plain_room_text_wakes_all(self):
        with tempfile.TemporaryDirectory() as td, \
             patch.object(team_bus, "BUS_PATH", Path(td) / "bus.json"), \
             patch.object(team_bus, "LOCK_PATH", Path(td) / "bus.lock"):
            team_bus.reset()
            directed = team_bus.post_operator_chat("@R2 이 메시지만 확인해줘")
            self.assertEqual(directed["targets"], ["r2"])
            self.assertEqual([w["robot_id"] for w in directed["wakeups"]], ["r2"])
            self.assertEqual(directed["chat"]["sender_id"], "operator")
            self.assertEqual(directed["chat"]["targets"], ["r2"])

            team_bus.reset()
            room = team_bus.post_operator_chat("지금 각자 상황 한 줄씩 알려줘")
            self.assertEqual(room["targets"], ["r1", "r2", "r3"])
            self.assertEqual([w["robot_id"] for w in room["wakeups"]], ["r1", "r2", "r3"])
            self.assertEqual(team_bus.mention_targets("@TEAM 상태 공유"), ["r1", "r2", "r3"])

    def test_peer_message_is_mirrored_to_room_and_wakes_recipient_but_plain_reply_does_not(self):
        with tempfile.TemporaryDirectory() as td, \
             patch.object(team_bus, "BUS_PATH", Path(td) / "bus.json"), \
             patch.object(team_bus, "LOCK_PATH", Path(td) / "bus.lock"):
            team_bus.reset()
            msg = team_bus.send_message("r1", "r3", "오른쪽 구역 확인 가능해?")
            snap = team_bus.snapshot()
            self.assertEqual(snap["chat"][-1]["kind"], "peer")
            self.assertEqual(snap["chat"][-1]["sender_id"], "r1")
            self.assertEqual(snap["chat"][-1]["targets"], ["r3"])
            self.assertEqual([w["robot_id"] for w in snap["wakeups"]], ["r3"])
            self.assertEqual(msg["wakeup_ids"], [snap["wakeups"][0]["id"]])

            before = len(snap["wakeups"])
            team_bus.post_agent_chat("r3", "확인할게", reply_to=snap["chat"][-1]["id"])
            after = team_bus.snapshot()
            self.assertEqual(len(after["wakeups"]), before)
            self.assertEqual(after["chat"][-1]["kind"], "agent")

    def test_claimed_team_wakeup_can_be_absorbed_by_existing_robot_turn(self):
        with tempfile.TemporaryDirectory() as td, \
             patch.object(team_bus, "BUS_PATH", Path(td) / "bus.json"), \
             patch.object(team_bus, "LOCK_PATH", Path(td) / "bus.lock"):
            team_bus.reset()
            posted = team_bus.post_operator_chat("@R1 같이 보고 있어")
            claimed = team_bus.claim_wakeup()
            self.assertEqual(claimed["robot_id"], "r1")
            ctx = team_bus.context_for("r1")
            self.assertEqual(ctx["chat"][-1]["id"], posted["chat"]["id"])
            wake = team_bus.snapshot()["wakeups"][0]
            self.assertEqual(wake["status"], "absorbed")

    def test_sim_and_real_peer_evidence_are_isolated_and_ack_is_addressed(self):
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
            "UGRP_TEAM_BUS_REAL_PATH": str(Path(td) / "real.json"),
            "UGRP_TEAM_BUS_REAL_LOCK": str(Path(td) / "real.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            team_bus.reset(namespace="real")
            sim_msg = team_bus.send_message("r1", "r2", "SIM only", namespace="sim")
            real_msg = team_bus.send_message("r3", "r1", "REAL only", namespace="real")

            self.assertEqual([m["id"] for m in team_bus.snapshot(namespace="sim")["messages"]], [sim_msg["id"]])
            self.assertEqual([m["id"] for m in team_bus.snapshot(namespace="real")["messages"]], [real_msg["id"]])
            with self.assertRaisesRegex(ValueError, "not addressed"):
                team_bus.acknowledge("r3", sim_msg["id"], True, namespace="sim")
            with self.assertRaisesRegex(ValueError, "own peer message"):
                team_bus.acknowledge("r1", sim_msg["id"], True, namespace="sim")
            acked = team_bus.acknowledge("r2", sim_msg["id"], True, namespace="sim")
            self.assertTrue(acked["ack"]["r2"]["accepted"])


class TeamWakeDeliveryTests(unittest.TestCase):
    def test_temporary_team_retry_delay_accepts_seconds_and_milliseconds(self):
        from harness.web import _temporary_team_retry_delay
        self.assertAlmostEqual(_temporary_team_retry_delay("TPM Please try again in 157.5ms."), 0.5075)
        self.assertAlmostEqual(_temporary_team_retry_delay("TPM retry in 2.25s"), 2.60)
        self.assertIsNone(_temporary_team_retry_delay("ordinary planner error"))
        self.assertAlmostEqual(_temporary_team_retry_delay("Groq 한도에 걸렸습니다."), 12.35)
        self.assertAlmostEqual(
            _temporary_team_retry_delay(
                'Gemini 프록시 HTTP 503: {"error":{"message":"No capacity available for model gemini-3.7-flash-tiered"}}'
            ),
            2.35,
        )
        self.assertAlmostEqual(
            _temporary_team_retry_delay("Groq 모델 fallback까지 실패했습니다. Groq 요청이 모두 실패했습니다."),
            12.35,
        )

    class _State:
        password = None

    class _Response:
        def __init__(self, body):
            self.body = body
        def __enter__(self):
            return self
        def __exit__(self, *args):
            return False
        def read(self):
            return json.dumps(self.body).encode()

    def test_team_role_color_is_personalized_before_goal_inference(self):
        from harness.goals import infer_goal
        from harness.web import _personalize_team_turn
        text = "R1은 빨간 블럭, R2는 파란 블럭, R3는 노란 블럭을 각각 찾아봐"
        self.assertEqual(infer_goal(_personalize_team_turn(text, "r1")).subject, "RED_BLOCK")
        self.assertEqual(infer_goal(_personalize_team_turn(text, "r2")).subject, "BLUE_BLOCK")
        self.assertEqual(infer_goal(_personalize_team_turn(text, "r3")).subject, "YELLOW_BLOCK")

    def test_find_then_pick_room_command_keeps_pick_as_the_local_mission(self):
        from harness.goals import infer_goal
        from harness.web import _personalize_team_turn
        text = "R1은 빨간 블럭, R2는 파란 블럭, R3는 노란 블럭을 각각 찾아서 집어봐"
        for rid, subject in (("r1", "RED_BLOCK"), ("r2", "BLUE_BLOCK"), ("r3", "YELLOW_BLOCK")):
            mission = _personalize_team_turn(text, rid)
            self.assertIn("집어", mission, mission)
            goal = infer_goal(mission)
            self.assertEqual(goal.subject, subject)
            self.assertEqual(goal.kind, "GRASP", goal)

    def test_three_robot_tower_command_personalizes_sequential_local_missions(self):
        from harness.web import _personalize_team_turn, _team_tower_intent
        text = "세 로봇이 협력해서 3개의 블럭을 1열로 쌓아 3단 탑을 만들어"
        self.assertTrue(_team_tower_intent(text))
        # The mission fixes the physical order and the handoff target, but it no
        # longer dictates a tool name: tool choice belongs to the planner.
        r3, r1, r2 = (_personalize_team_turn(text, rid) for rid in ("r3", "r1", "r2"))
        self.assertIn("1/3", r3); self.assertIn("노란", r3); self.assertIn("R1", r3); self.assertIn("send_peer_message", r3)
        self.assertIn("2/3", r1); self.assertIn("파란", r1); self.assertIn("노란", r1); self.assertIn("R2", r1)
        self.assertIn("3/3", r2); self.assertIn("빨간", r2); self.assertIn("파란", r2)
        for mission in (r3, r1, r2):
            self.assertNotIn("stage_base", mission)
            self.assertNotIn("stack_on", mission)

    def test_generic_team_stack_phrase_maps_to_tower_without_expanding_color_specific_request(self):
        from harness.web import _team_tower_intent

        self.assertTrue(_team_tower_intent("블럭 쌓아봐"))
        self.assertTrue(_team_tower_intent("블록 좀 쌓아줘"))
        self.assertFalse(_team_tower_intent("빨간 블럭 쌓아봐"))

    def test_cooperative_beam_mission_personalizes_three_required_roles(self):
        from harness.web import _personalize_team_turn, _team_beam_intent

        text = "세 로봇이 협업해서 긴 빔을 출발 구역 A에서 도착 구역 B로 운반해"
        self.assertTrue(_team_beam_intent(text))
        missions = {rid: _personalize_team_turn(text, rid) for rid in ("r1", "r2", "r3")}
        self.assertIn("role='carrier_left'", missions["r1"])
        self.assertIn("role='scout'", missions["r2"])
        self.assertIn("role='carrier_right'", missions["r3"])
        for mission in missions.values():
            self.assertIn("team_beam_transport", mission)
            self.assertIn("beam_transport_v1", mission)

    def test_cooperative_beam_action_is_sim_only_and_quorum_typed(self):
        self.assertIn("team_beam_transport", BRIDGE_ALLOWED_ACTIONS)
        self.assertIn("team_beam_transport", sim_actions.ACTIONS)
        self.assertEqual(
            set(sim_actions.ACTION_PARAMETERS["team_beam_transport"]),
            {"mission_id", "role"},
        )
        clean = sim_actions._validate_params(
            "team_beam_transport",
            {"mission_id": "beam_transport_v1", "role": "carrier_left"},
        )
        self.assertEqual(clean["role"], "carrier_left")

    def test_tower_wake_chain_advances_only_after_verified_stage(self):
        message = "세 로봇이 협력해서 3개의 블럭을 1열로 쌓아 3단 탑을 만들어"
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            posted = team_bus.post_operator_chat(message, namespace="sim", targets_override=["r3"])
            self.assertEqual(posted["targets"], ["r3"])

            def fake_urlopen(req, timeout=0):
                return self._Response({"final": "done", "text": "done", "tools": [], "stopped": "final"})

            # LLM-first: when the planner turn succeeds the direct primitive is
            # never called. (It would also post to the live bridge on :8091 and
            # move the production simulator from inside the test suite.)
            with patch.object(harness_web, "urlopen", fake_urlopen), \
                 patch.object(harness_web, "_run_sim_tower_fallback", side_effect=AssertionError("planner succeeded; direct primitive must not run")), \
                 patch.object(harness_web, "_sim_team_tower_stage_complete", return_value=True):
                first = team_bus.claim_wakeup(namespace="sim")
                self.assertEqual(first["robot_id"], "r3")
                harness_web._deliver_team_wakeup(self._State(), "sim", first)
                second = team_bus.claim_wakeup(namespace="sim")
                self.assertEqual(second["robot_id"], "r1")
                self.assertIn("TOWER_STEP_2", second["message"])
                harness_web._deliver_team_wakeup(self._State(), "sim", second)
                third = team_bus.claim_wakeup(namespace="sim")
                self.assertEqual(third["robot_id"], "r2")
                self.assertIn("TOWER_STEP_3", third["message"])
                harness_web._deliver_team_wakeup(self._State(), "sim", third)

            snap = team_bus.snapshot(namespace="sim")
            self.assertFalse(any(w["status"] == "pending" for w in snap["wakeups"]))
            self.assertTrue(any(e.get("kind") == "tower_complete" for e in snap["events"]))
            handoffs = [e for e in snap["events"] if e.get("kind") == "tower_handoff"]
            self.assertEqual([h.get("source") for h in handoffs], ["router_fallback", "router_fallback"])

    def test_tower_planner_handoff_suppresses_router_fallback_message(self):
        message = "세 로봇이 협력해서 3개의 블럭을 1열로 쌓아 3단 탑을 만들어"
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            team_bus.set_goal(message, namespace="sim")
            team_bus.post_operator_chat(message, namespace="sim", targets_override=["r3"])
            wake = team_bus.claim_wakeup(namespace="sim")

            def planner_that_hands_off(req, timeout=0):
                # The R3 planner itself hands the next step to R1 during its turn.
                team_bus.send_message("r3", "r1", "노란 블럭 바닥층 완료. 파란 블럭을 노란 위에 올려 줘.", namespace="sim")
                return self._Response({"final": "1단계 끝", "text": "", "tools": [], "stopped": "final"})

            with patch.object(harness_web, "urlopen", planner_that_hands_off), \
                 patch.object(harness_web, "_run_sim_tower_fallback", side_effect=AssertionError("must not run")), \
                 patch.object(harness_web, "_sim_team_tower_stage_complete", return_value=True):
                harness_web._deliver_team_wakeup(self._State(), "sim", wake)

            snap = team_bus.snapshot(namespace="sim")
            r3_to_r1 = [m for m in snap["messages"] if m["sender_id"] == "r3" and m["recipient"] == "r1"]
            self.assertEqual(len(r3_to_r1), 1, "router must not duplicate the planner's own handoff")
            self.assertNotIn("TOWER_STEP_2", r3_to_r1[0]["message"])
            handoff = [e for e in snap["events"] if e.get("kind") == "tower_handoff"]
            self.assertEqual(handoff[0].get("source"), "planner")
            # The untagged planner handoff still enters the stage-2 referee for R1
            # because the tower is the active team goal.
            second = team_bus.claim_wakeup(namespace="sim")
            self.assertEqual(second["robot_id"], "r1")
            with patch.object(harness_web, "urlopen", lambda req, timeout=0: self._Response({"final": "ok", "text": "", "tools": [], "stopped": "final"})), \
                 patch.object(harness_web, "_run_sim_tower_fallback", side_effect=AssertionError("must not run")), \
                 patch.object(harness_web, "_sim_team_tower_stage_complete", return_value=False) as check:
                harness_web._deliver_team_wakeup(self._State(), "sim", second)
            check.assert_called_with(2)
            snap = team_bus.snapshot(namespace="sim")
            self.assertTrue(any(e.get("kind") == "tower_stage_failed" and e.get("stage") == 2 for e in snap["events"]))

    def test_tower_planner_error_uses_verified_public_action_fallback(self):
        message = "세 로봇이 협력해서 3개의 블럭을 1열로 쌓아 3단 탑을 만들어"
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            team_bus.post_operator_chat(message, namespace="sim", targets_override=["r3"])
            wake = team_bus.claim_wakeup(namespace="sim")

            def fake_urlopen(req, timeout=0):
                return self._Response({"error": "Groq 모델 fallback까지 실패했습니다. Groq 요청이 모두 실패했습니다."})

            planner_fallback = {
                "ok": True, "skill": "stage_base", "outcome_status": "ACHIEVED",
                "reason": "staged",
            }
            with patch.object(harness_web, "urlopen", fake_urlopen), \
                 patch.object(harness_web, "_run_sim_tower_fallback", return_value=planner_fallback) as run_fallback, \
                 patch.object(harness_web, "_sim_team_tower_stage_complete", return_value=True):
                harness_web._deliver_team_wakeup(self._State(), "sim", wake)

            # Only after the planner provider failed, and only once.
            run_fallback.assert_called_once_with("r3", 1)
            snap = team_bus.snapshot(namespace="sim")
            self.assertEqual(snap["wakeups"][0]["status"], "done")
            self.assertFalse(any(e.get("kind") == "tower_direct_action" for e in snap["events"]))
            self.assertTrue(any(e.get("kind") == "tower_planner_fallback" for e in snap["events"]))
            next_wake = team_bus.claim_wakeup(namespace="sim")
            self.assertEqual(next_wake["robot_id"], "r1")

    def test_tower_step_asks_planner_first_and_never_preempts_it(self):
        message = "세 로봇이 협력해서 3개의 블럭을 1열로 쌓아 3단 탑을 만들어"
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            team_bus.post_operator_chat(message, namespace="sim", targets_override=["r3"])
            wake = team_bus.claim_wakeup(namespace="sim")
            captured = {}

            def fake_urlopen(req, timeout=0):
                captured.update(json.loads(req.data.decode()))
                return self._Response({"final": "노란 블럭 바닥층 완료", "text": "", "tools": [{"skill": "stage_base", "ok": True}], "stopped": "final"})

            with patch.object(harness_web, "_run_sim_tower_fallback", side_effect=AssertionError("direct primitive must not pre-empt the planner")), \
                 patch.object(harness_web, "_sim_team_tower_stage_complete", return_value=True), \
                 patch.object(harness_web, "urlopen", fake_urlopen):
                harness_web._deliver_team_wakeup(self._State(), "sim", wake)

            self.assertTrue(captured["execute"])
            self.assertIn("1/3", captured["message"])
            self.assertNotIn("stage_base", captured["message"])
            snap = team_bus.snapshot(namespace="sim")
            self.assertEqual(snap["wakeups"][0]["status"], "done")
            self.assertEqual(snap["chat"][-1]["author"], "r3")
            self.assertEqual(team_bus.claim_wakeup(namespace="sim")["robot_id"], "r1")

    def test_consensus_tower_execution_waits_for_predecessor_stage(self):
        message = "세 로봇이 협력해서 3개의 블럭을 1열로 쌓아 3단 탑을 만들어"
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            posted = team_bus.post_operator_chat(message, namespace="sim")
            execution = []
            for rid in team_bus.ROBOT_IDS:
                committed = team_bus.submit_consensus_proposal(
                    posted["consensus_round"]["id"], rid, {"plan": rid}, namespace="sim"
                )
                execution = committed.get("execution_wakeups") or execution
            stage_two = next(w for w in execution if w["robot_id"] == "r1")

            with patch.object(harness_web, "_wait_sim_team_tower_dependency", return_value=False) as wait_dependency, \
                 patch.object(harness_web, "_run_sim_tower_fallback") as run_fallback:
                harness_web._deliver_team_wakeup(self._State(), "sim", stage_two)

            wait_dependency.assert_called_once_with(2)
            run_fallback.assert_not_called()
            wake = next(
                w for w in team_bus.snapshot(namespace="sim")["wakeups"]
                if w["id"] == stage_two["id"]
            )
            self.assertEqual(wake["status"], "error")
            self.assertIn("dependency stage 1", wake["error"])

    def test_consensus_tower_marks_complete_only_after_stage_three(self):
        message = "세 로봇이 협력해서 3개의 블럭을 1열로 쌓아 3단 탑을 만들어"
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            "UGRP_TEAM_BUS_SIM_PATH": str(Path(td) / "sim.json"),
            "UGRP_TEAM_BUS_SIM_LOCK": str(Path(td) / "sim.lock"),
        }, clear=False):
            team_bus.reset(namespace="sim")
            posted = team_bus.post_operator_chat(message, namespace="sim")
            execution = []
            for rid in team_bus.ROBOT_IDS:
                committed = team_bus.submit_consensus_proposal(
                    posted["consensus_round"]["id"], rid, {"plan": rid}, namespace="sim"
                )
                execution = committed.get("execution_wakeups") or execution
            stage_one = next(w for w in execution if w["robot_id"] == "r3")

            with patch.object(harness_web, "_wait_sim_team_tower_dependency", return_value=True), \
                 patch.object(harness_web, "_run_sim_tower_fallback", return_value={"ok": True, "skill": "stage_base"}), \
                 patch.object(harness_web, "_sim_team_tower_stage_complete", return_value=True):
                harness_web._deliver_team_wakeup(self._State(), "sim", stage_one)

            events = team_bus.snapshot(namespace="sim")["events"]
            self.assertFalse(any(e.get("kind") == "tower_complete" for e in events))

    def test_force_release_can_retry_dispatcher_wakeup_after_context_absorption(self):
        with tempfile.TemporaryDirectory() as td, \
             patch.object(team_bus, "BUS_PATH", Path(td) / "bus.json"), \
             patch.object(team_bus, "LOCK_PATH", Path(td) / "bus.lock"):
            team_bus.reset()
            team_bus.post_operator_chat("@R1 빨간 블럭 찾아봐")
            wake = team_bus.claim_wakeup()
            team_bus.context_for("r1")  # simulates accidental absorption
            self.assertEqual(team_bus.snapshot()["wakeups"][0]["status"], "absorbed")
            retried = team_bus.release_wakeup(wake["id"], delay_s=.5, error="TPM", force=True)
            self.assertEqual(retried["status"], "pending")


    def _exercise(self, mode: str):
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ, {
            f"UGRP_TEAM_BUS_{mode.upper()}_PATH": str(Path(td) / f"{mode}.json"),
            f"UGRP_TEAM_BUS_{mode.upper()}_LOCK": str(Path(td) / f"{mode}.lock"),
        }, clear=False):
            team_bus.reset(namespace=mode)
            posted = team_bus.post_operator_chat("@R2 연결 테스트", namespace=mode)
            wake = team_bus.claim_wakeup(namespace=mode)
            self.assertEqual(wake["robot_id"], "r2")
            captured = {}

            def fake_urlopen(req, timeout=0):
                captured.update(json.loads(req.data.decode()))
                return self._Response({"final": "R2 응답", "text": "R2 응답", "tools": [], "stopped": "final"})

            with patch.object(harness_web, "urlopen", fake_urlopen):
                harness_web._deliver_team_wakeup(self._State(), mode, wake)

            snap = team_bus.snapshot(namespace=mode)
            self.assertEqual(snap["wakeups"][-1]["status"], "done")
            self.assertEqual(snap["chat"][-1]["author"], "r2")
            self.assertEqual(snap["chat"][-1]["message"], "R2 응답")
            self.assertEqual(snap["chat"][-1]["reply_to"], posted["chat"]["id"])
            return captured

    def test_sim_team_wakeup_can_execute_independent_agent_turn(self):
        payload = self._exercise("sim")
        self.assertTrue(payload["execute"])
        self.assertFalse(payload["chat_only"])

    def test_real_team_wakeup_is_chat_only_and_cannot_auto_actuate(self):
        payload = self._exercise("real")
        self.assertFalse(payload["execute"])
        self.assertTrue(payload["chat_only"])


class PeerEvidenceTests(unittest.TestCase):
    def test_peer_summary_reports_other_robots_last_tool_and_color_only(self):
        with tempfile.TemporaryDirectory() as td, \
             patch.object(team_bus, "BUS_PATH", Path(td) / "bus.json"), \
             patch.object(team_bus, "LOCK_PATH", Path(td) / "bus.lock"):
            team_bus.reset()
            team_bus.record_event("r1", "tool", tool="search", ok=True, target_color="red")
            team_bus.record_event("r2", "tool", tool="search", ok=True, target_color="blue")
            team_bus.record_event("r2", "tool", tool="approach", ok=False, target_color="blue", reason="x")
            team_bus.record_event("r3", "team_wake_start")
            peers = team_bus.peer_summary("r1")
            self.assertEqual(set(peers), {"r2"})
            self.assertEqual(peers["r2"]["last_tool"], "approach")
            self.assertEqual(peers["r2"]["target_color"], "blue")
            self.assertFalse(peers["r2"]["ok"])
            self.assertNotIn("r1", peers)

    def test_peer_too_close_is_a_wait_recovery_for_the_planner(self):
        code, required, recovery = sim_actions._failure(
            "approach", "PEER_TOO_CLOSE: r2 approach refused, peer r1 is 0.18 m away (< 0.24 m); wait for it to clear"
        )
        self.assertEqual(code, "PEER_TOO_CLOSE")
        self.assertEqual(required, "peer.clearance=SAFE")
        self.assertEqual(recovery, "wait")


class SharedWorldRefereeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from sim.multi_masterpi_production import MultiMasterPiProductionV2
        except Exception as exc:  # pragma: no cover - mujoco missing locally
            raise unittest.SkipTest(f"mujoco world unavailable: {exc}")
        cls.world = MultiMasterPiProductionV2(seed=3, render=False)

    @classmethod
    def tearDownClass(cls):
        cls.world.close()

    def test_chassis_action_is_refused_when_a_peer_is_inside_soft_radius(self):
        from sim import multi_masterpi_production as mmp
        world = self.world
        world.reset(3)
        r1, r2 = world.controllers["r1"], world.controllers["r2"]
        x2, y2, z2 = (float(v) for v in r2.base_xyz())
        # Side by side, 0.20 m apart: the two 0.104 m half-widths overlap.
        r1.set_base_pose_for_test((x2, y2 + 0.20, z2), 0.0)
        peer, gap = world.nearest_peer(r2)
        self.assertEqual(peer, "r1")
        self.assertLess(gap, mmp.TEAM_PEER_CLEARANCE_M)
        result = world.act("r2", "approach", target_color="red")
        self.assertFalse(result.ok)
        self.assertIn("PEER_TOO_CLOSE", result.reason)
        self.assertIn("r1", result.reason)
        # Camera-only actions are never blocked by the referee.
        for name in ("observe_scene", "search", "search_destination"):
            self.assertNotIn(name, mmp.TEAM_CHASSIS_ACTIONS)
        # Back at its own spawn lane the peer is clear and the pre-check passes.
        sx, sy, sz, _ = mmp.DEFAULT_SPAWNS["r1"]
        r1.set_base_pose_for_test((sx, sy, sz), 0.0)
        _, gap = world.nearest_peer(r2)
        self.assertGreater(gap, mmp.TEAM_PEER_CLEARANCE_M)
        world._refuse_if_peer_close(r2, "approach", mmp.TEAM_PEER_CLEARANCE_M)

    def test_extended_arm_counts_toward_peer_clearance(self):
        from sim import multi_masterpi_production as mmp
        world = self.world
        world.reset(3)
        r1, r2 = world.controllers["r1"], world.controllers["r2"]
        x2, y2, z2 = (float(v) for v in r2.base_xyz())
        # r1 stands 0.30 m behind r2's tail, facing it. Folded arm: clear.
        r1.set_base_pose_for_test((x2 - 0.30, y2, z2), 0.0)
        _, folded_gap = world.nearest_peer(r1)
        precision = world._precision_module()
        r1.move_servos_timed({1: precision.GRIPPER_OPEN, 6: 1500, **world._hover_pose()}, 0.6)
        _, hover_gap = world.nearest_peer(r1)
        self.assertGreater(folded_gap, hover_gap + 0.05)
        self.assertLess(hover_gap, mmp.TEAM_PEER_CLEARANCE_M)
        world.reset(3)

    def test_reset_layout_keeps_blocks_far_enough_apart_for_two_standoffs(self):
        from sim import multi_masterpi_production as mmp
        for seed in (1, 7, 42, 1114274011):
            self.world.reset(seed)
            objects = self.world.state()["shared_objects"]
            colors = list(objects)
            for i, a in enumerate(colors):
                for b in colors[i + 1:]:
                    dx = objects[a][0] - objects[b][0]
                    dy = objects[a][1] - objects[b][1]
                    self.assertGreaterEqual((dx * dx + dy * dy) ** 0.5, mmp.TEAM_BLOCK_MIN_SEPARATION_M - 0.005, (seed, a, b))
                    # Own Y row per block: the robot heading for the far block
                    # never has to pass a peer standing at the nearer block.
                    self.assertGreaterEqual(abs(dy), mmp.TEAM_BLOCK_MIN_ROW_SEPARATION_M - 0.005, (seed, a, b))
            self.assertNotEqual(
                {c: tuple(round(v, 3) for v in objects[c][:2]) for c in colors},
                {c: tuple(round(v, 3) for v in mmp.TEAM_DETERMINISTIC_LAYOUT[c][:2]) for c in colors},
                f"seed {seed} fell back to the fixed layout",
            )
        self.world.reset(3)

    def test_three_cube_tower_does_not_creep_or_topple_at_rest(self):
        # With MuJoCo's default soft impedance a 5 g cube on a cube crept ~1 mm
        # per minute and a three-cube tower fell over on its own within ~90 s.
        import mujoco
        from sim import multi_masterpi_production as mmp
        world = self.world
        world.reset(3)
        r1 = world.controllers["r1"]
        half, side = mmp.TARGET_BLOCK_HALF_M, mmp.TARGET_BLOCK_SIDE_M
        r1.set_free_body_pose_for_reset("yellow_block", (0.72, 0.42, half), 0.0)
        r1.set_free_body_pose_for_reset("blue_block", (0.723, 0.42, half + side + 0.0005), 0.0)
        r1.set_free_body_pose_for_reset("red_block", (0.722, 0.421, half + 2 * side + 0.001), 0.0)
        mujoco.mj_forward(world.model, world.data)
        before = {c: [float(v) for v in r1.body_xyz(f"{c}_block")] for c in ("red", "blue", "yellow")}
        r1.step(motor_commands=mmp.STOP, duration_s=60.0)
        for color, start in before.items():
            now = [float(v) for v in r1.body_xyz(f"{color}_block")]
            self.assertLess(((now[0] - start[0]) ** 2 + (now[1] - start[1]) ** 2) ** 0.5, 0.002, color)
            self.assertLess(abs(now[2] - start[2]), 0.004, color)
        world.reset(3)

    def test_transit_corridor_clears_parked_and_standoff_peers(self):
        from sim import multi_masterpi_production as mmp
        side_half_width, folded_front, rear = 0.104, 0.121, 0.107
        lane_near_edge = mmp.TEAM_SAFE_CORRIDOR_X - side_half_width
        lane_far_edge = mmp.TEAM_SAFE_CORRIDOR_X + side_half_width
        parked_front = mmp.TEAM_PARK_X + folded_front
        for rid, (sx, _sy, _sz, _yaw) in mmp.DEFAULT_SPAWNS.items():
            self.assertLessEqual(sx, mmp.TEAM_PARK_X + 1e-9, rid)
        self.assertGreater(lane_near_edge - parked_front, mmp.TEAM_PEER_HARD_CLEARANCE_M)
        nearest_standoff_rear = mmp.TEAM_BLOCK_X_RANGE[0] - mmp.TEAM_APPROACH_STANDOFF_M - rear
        self.assertGreater(nearest_standoff_rear - lane_far_edge, mmp.TEAM_PEER_HARD_CLEARANCE_M)


class TeamApproachCameraContractTests(unittest.TestCase):
    """SIM-only atomic team geometry keeps its own camera-visible handoff.

    Public ``approach``/``pick`` no longer use this path; they delegate to the
    shared REAL controller and are covered by the SIM/REAL parity tests.
    """

    def test_three_parallel_approaches_end_with_target_visible_then_pick(self):
        import sys
        if sys.platform.startswith("linux") and os.environ.get("MUJOCO_GL", "").lower() not in {"egl", "osmesa"}:
            # A GLFW offscreen context on a headless box aborts the interpreter
            # (not an exception), so only render where a software GL is selected.
            raise unittest.SkipTest("headless Linux needs MUJOCO_GL=egl for offscreen rendering")
        try:
            from sim.multi_masterpi_production import MultiMasterPiProductionV2
            world = MultiMasterPiProductionV2(seed=2026, render=True)
        except Exception as exc:  # pragma: no cover - mujoco/GL missing locally
            raise unittest.SkipTest(f"rendering multi world unavailable: {exc}")
        try:
            world.set_speed_multiplier(3.0)
            plan = {"r1": "red", "r2": "blue", "r3": "yellow"}
            for rid, color in plan.items():
                reason = world._geometric_approach(world.controllers[rid], color)
                self.assertIn("geometric approach aligned", reason)
                det = world.scene_detections(rid)[color]
                self.assertTrue(det.get("visible"), (rid, color, det))
                self.assertGreater(float(det.get("area_ratio") or det.get("area") or 0.0), 0.02, (rid, det))
            for rid, color in plan.items():
                reason = world._geometric_pick(world.controllers[rid], color)
                self.assertIn("physical grasp confirmed", reason)
                self.assertEqual(world.controllers[rid].grasp_color, color)
        finally:
            world.close()


class BridgeRobotNamespaceTests(unittest.TestCase):
    def test_robot_camera_frame_does_not_replace_shared_world_state(self):
        bridge = BridgeState("token")
        shared = {
            "model": "masterpi_multi_v2_shared_world",
            "robots": {
                "r1": {"phase": "idle"},
                "r2": {"phase": "idle"},
                "r3": {"phase": "idle"},
            },
        }
        self.assertTrue(bridge.push_state(shared, state_seq=10))
        self.assertTrue(
            bridge.push_frame(b"r2-jpeg", {"phase": "moving"}, state_seq=11, robot_id="r2")
        )

        self.assertEqual(bridge.state["model"], "masterpi_multi_v2_shared_world")
        self.assertEqual(bridge.state["robots"]["r2"]["phase"], "idle")
        self.assertEqual(bridge.robot_states["r2"]["phase"], "moving")
        self.assertEqual(bridge.robot_frames["r2"], b"r2-jpeg")
        self.assertIsNone(bridge.frame)  # legacy alias belongs to R1 only


class SimRoutingTests(unittest.TestCase):
    def test_strafe_is_not_a_public_sim_or_real_action(self):
        for name in ("strafe_left", "strafe_right"):
            self.assertNotIn(name, sim_actions.ACTIONS)
            self.assertNotIn(name, BRIDGE_ALLOWED_ACTIONS)

    def test_stack_on_is_atomic_sim_team_action(self):
        self.assertIn("stack_on", sim_actions.ACTIONS)
        self.assertEqual(sim_actions.CONTRACTS["stack_on"]["preconditions"], [])
        self.assertEqual(
            sim_actions._validate_params(
                "stack_on", {"target_color": "blue", "destination_color": "yellow"}
            ),
            {"target_color": "blue", "destination_color": "yellow"},
        )
        self.assertIn("stack_on", BRIDGE_ALLOWED_ACTIONS)
        self.assertIn("stack_on", BRIDGE_COLOR_ACTIONS)
        self.assertIn("stack_on", BRIDGE_DESTINATION_ACTIONS)

    def test_sim_transport_includes_selected_robot_id(self):
        captured = {}

        class Response:
            def __enter__(self):
                return self
            def __exit__(self, *args):
                return False
            def read(self):
                return b'{"ok":true}'

        def fake_urlopen(req, timeout=0):
            captured.update(json.loads(req.data.decode()))
            return Response()

        with patch.dict(os.environ, {"UGRP_ROBOT_ID": "r3"}), \
             patch.object(sim_actions, "urlopen", fake_urlopen):
            out = sim_actions._post_bridge_action("move_forward", {"speed": 35, "duration": 0.2})

        self.assertTrue(out["ok"])
        self.assertEqual(captured["robot_id"], "r3")
        self.assertEqual(captured["action"], "move_forward")

    def test_verified_stage_base_satisfies_the_place_goal(self):
        # Live trace 20260902: R3 staged yellow, then the PLACE goal stayed
        # UNKNOWN because stage_base carried no place_verified evidence. The
        # planner burned all 16 tool steps on refused finals and never reached
        # send_peer_message, so the tower handoff fell back to the router.
        from harness.goals import goal_status, infer_goal
        from harness.state import StateEstimator

        with patch.dict(os.environ, {"UGRP_ROBOT_ID": "r3"}), \
             patch.object(sim_actions, "_post_bridge_action", return_value={"ok": True, "reason": "stage_base ACHIEVED"}):
            out = sim_actions.run("stage_base", target_color="yellow")
        self.assertEqual(out["outcome_status"], "ACHIEVED")
        self.assertTrue(out["place_verified"])

        est = StateEstimator()
        est.state.grasp.state = "HELD"
        est.update_tool_result({"result": out})
        self.assertEqual(est.state.task.phase, "PLACED")
        self.assertEqual(est.state.grasp.state, "EMPTY")
        goal = infer_goal("노란 블럭을 공용 스택 지점의 바닥층으로 옮겨 놔.")
        self.assertEqual(goal.kind, "PLACE")
        self.assertEqual(goal_status(goal, est.state), "ACHIEVED")

    def test_tower_mission_goal_ignores_the_delegated_next_step(self):
        from harness.goals import infer_goal
        from harness.web import _personalize_team_turn

        r3 = infer_goal(_personalize_team_turn("세 블럭 쌓아줘", "r3"))
        self.assertEqual((r3.kind, r3.subject), ("PLACE", "YELLOW_BLOCK"))
        r1 = infer_goal(_personalize_team_turn("세 블럭 쌓아줘", "r1"))
        self.assertEqual((r1.kind, r1.subject, r1.target), ("PLACE", "BLUE_BLOCK", "YELLOW_BLOCK"))
        r2 = infer_goal(_personalize_team_turn("세 블럭 쌓아줘", "r2"))
        self.assertEqual((r2.kind, r2.subject, r2.target), ("PLACE", "RED_BLOCK", "BLUE_BLOCK"))

    def test_collapsed_tool_shape_is_parsed_as_a_tool_call(self):
        # Live R3 trace 20260902T073341Z: the model answered
        # {"send_peer_message": {...}} three times and the turn died on protocol errors.
        from harness.protocol import ProtocolError, parse_action

        action = parse_action('{"send_peer_message": {"recipient": "r1", "message": "다음 단계 진행해 줘"}}')
        self.assertEqual(action.name, "send_peer_message")
        self.assertEqual(action.args["recipient"], "r1")
        with self.assertRaises(ProtocolError):
            parse_action('{"Send Peer": {"recipient": "r1"}}')
        with self.assertRaises(ProtocolError):
            parse_action('{"send_peer_message": "not an args object"}')

    def test_success_claim_before_any_tool_is_pushed_back_once(self):
        from harness.catalog import default_registry
        from harness.loop import ReplayCompleter, run_loop
        from harness.web import _personalize_team_turn

        seen = []

        def runner(name, **kwargs):
            seen.append(name)
            return {"ok": True, "skill": name, **kwargs, "command_status": "ACCEPTED",
                    "execution_status": "COMPLETED", "outcome_status": "ACHIEVED", "place_verified": True}

        replies = [
            {"final": "노란 블록을 바닥층으로 배치 완료했습니다. R1에게 넘겼습니다."},
            {"tool": "stage_base", "args": {"target_color": "yellow"}},
            {"final": "done"},
        ]
        with patch.dict(os.environ, {"UGRP_ROBOT_ID": "r3"}):
            result = run_loop(
                ReplayCompleter(replies),
                default_registry(actions_path="scripts/sim_actions.py", runner=runner),
                _personalize_team_turn("세 블럭 쌓아줘", "r3"),
                execute=True, max_steps=6,
                planner_context={"self_id": "r3"},
            )
        self.assertEqual(seen, ["stage_base"])
        # After the handoff grace step the planner may close with its own final.
        self.assertIn(result.stopped, {"final", "goal_achieved"})
        errors = [step.error for step in result.steps if step.error]
        self.assertTrue(any("before any tool" in err for err in errors), errors)

    def test_team_turn_gives_the_planner_one_handoff_step_after_goal_achieved(self):
        # The sensor-verified goal used to close the turn right after the last
        # skill, so the LLM never got to call send_peer_message and every tower
        # handoff was the router fallback. A TEAM turn now gets exactly one more
        # planner step; a solo turn still finishes without another provider call.
        from harness.catalog import default_registry
        from harness.loop import ReplayCompleter, run_loop
        from harness.web import _personalize_team_turn

        seen = []

        def runner(name, **kwargs):
            seen.append((name, kwargs))
            base = {"ok": True, "skill": name, **kwargs, "command_status": "ACCEPTED",
                    "execution_status": "COMPLETED", "outcome_status": "ACHIEVED"}
            if name == "stage_base":
                base["place_verified"] = True
            return base

        replies = [
            {"tool": "stage_base", "args": {"target_color": "yellow"}},
            {"tool": "send_peer_message", "args": {"recipient": "r1", "message": "노란 블럭 바닥층 완료. 파란 블럭을 노란 블럭 위에 올려 줘."}},
            {"final": "should not be needed"},
        ]
        with patch.dict(os.environ, {"UGRP_ROBOT_ID": "r3"}):
            result = run_loop(
                ReplayCompleter(replies),
                default_registry(actions_path="scripts/sim_actions.py", runner=runner),
                _personalize_team_turn("세 블럭 쌓아줘", "r3"),
                execute=True, max_steps=6,
                planner_context={"self_id": "r3", "team_goal": "3단 탑"},
            )
        self.assertEqual([name for name, _ in seen], ["stage_base", "send_peer_message"])
        self.assertEqual(result.stopped, "goal_achieved")
        self.assertIn("노란", result.final)
        self.assertIn("바닥층", result.final)

        seen.clear()
        solo = run_loop(
            ReplayCompleter(list(replies)),
            default_registry(actions_path="scripts/sim_actions.py", runner=runner),
            "노란 블럭을 공용 스택 지점의 바닥층으로 옮겨.",
            execute=True, max_steps=6,
        )
        self.assertEqual([name for name, _ in seen], ["stage_base"])
        self.assertEqual(solo.stopped, "goal_achieved")


if __name__ == "__main__":
    unittest.main()
