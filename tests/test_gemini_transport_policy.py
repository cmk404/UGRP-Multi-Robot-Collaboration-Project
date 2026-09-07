import base64
import hashlib
import json
import unittest

import cv2
import numpy as np

from harness.coela_modules import PlanningError
from harness.gemini_transport_policy import GeminiTransportPlanner


def obs(camera, color, *, rid="r1", frame=1):
    image = np.full((48, 64, 3), color, np.uint8)
    ok, encoded = cv2.imencode(".jpg", image); assert ok
    jpeg = encoded.tobytes()
    return {"robot_id": rid, "frame_id": frame, "sim_time": float(frame),
            "image": base64.b64encode(jpeg).decode(), "sha256": hashlib.sha256(jpeg).hexdigest(),
            "camera": camera, "actuator_state": {"servo_pulses": {"1": 1500}}}


class Completer:
    model_name = "gemini-3.7-flash"
    last_model = "gemini-3.7-flash-20260901"
    last_usage = {"prompt_tokens": 20, "completion_tokens": 10, "total_tokens": 30}
    def __init__(self, reply): self.reply, self.calls = reply, []
    def complete(self, messages, *, images):
        self.calls.append((messages, images)); return json.dumps(self.reply, ensure_ascii=False)


class GeminiTransportPlannerTest(unittest.TestCase):
    def call(self, reply, *, skill_state="idle"):
        c = Completer(reply); p = GeminiTransportPlanner("r1", c)
        result = p.decide(obs("robot_cam", (0, 0, 255)), obs("nav_cam", (255, 0, 0), frame=2),
                          [{"own_action": "look"}], cargo_id="small_box_01",
                          destination_zone="A", skill_state=skill_state)
        return p, c, result

    def test_both_actual_images_reach_one_labeled_multimodal_payload(self):
        p, c, result = self.call({"action": {"kind": "approach"}, "reason": "손목 화면에서 상자를 확인함"})
        messages, images = c.calls[0]
        self.assertEqual(result["action"], {"kind": "approach"})
        self.assertNotIn(obs("robot_cam", (0, 0, 255))["image"], json.dumps(messages))
        self.assertEqual([item["label"].split()[0] for item in images], ["CURRENT_WRIST", "CURRENT_NAV"])
        decoded = [cv2.imdecode(np.frombuffer(base64.b64decode(item["image"].split(",", 1)[1]), np.uint8), cv2.IMREAD_COLOR)
                   for item in images]
        self.assertGreater(decoded[0][:, :, 2].mean(), 200)
        self.assertGreater(decoded[1][:, :, 0].mean(), 200)
        self.assertEqual([f["camera"] for f in p.last_audit["input_frames"]], ["robot_cam", "nav_cam"])
        self.assertTrue(all(f["sha256"] == f["transmitted_sha256"] for f in p.last_audit["input_frames"]))

    def test_model_response_genuinely_selects_different_actions(self):
        replies = [
            ({"action": {"kind": "pick"}, "reason": "집기 준비 상태가 화면에서 확인됨"}, "ready_to_pick"),
            ({"action": {"kind": "drive", "fwd": .04, "turn": -.2, "duration": .5}, "reason": "주황 장애물이 왼쪽에 보여 오른쪽으로 회피함"}, "carrying"),
            ({"action": {"kind": "wait", "duration": .7}, "reason": "자홍 동료가 앞에 보여 기다림"}, "carrying"),
            ({"action": {"kind": "release"}, "reason": "파란 목적지 안에 들어와 내려놓음"}, "carrying"),
        ]
        self.assertEqual([self.call(r, skill_state=s)[2]["action"]["kind"] for r, s in replies],
                         ["pick", "drive", "wait", "release"])

    def test_audit_exposes_raw_text_and_usage(self):
        reply = {"action": {"kind": "finish"}, "reason": "임무가 완료됨"}
        p, _c, result = self.call(reply, skill_state="released")
        self.assertEqual(json.loads(p.last_audit["raw_text"]), reply)
        self.assertEqual(p.last_audit["usage"]["total_tokens"], 30)
        self.assertEqual(p.last_audit["requested_model"], "gemini-3.7-flash")
        self.assertEqual(p.last_audit["response_model"], "gemini-3.7-flash-20260901")
        self.assertEqual(result["reason"], "임무가 완료됨")

    def test_rejects_foreign_camera_hash_and_forbidden_memory_before_call(self):
        c = Completer({"action": {"kind": "wait", "duration": .5}, "reason": "잠시 기다림"})
        p = GeminiTransportPlanner("r1", c)
        wrist, nav = obs("robot_cam", (1, 2, 3)), obs("nav_cam", (3, 2, 1), frame=2)
        cases = [(obs("robot_cam", (1, 2, 3), rid="r2"), nav, {}),
                 (wrist, {**nav, "sha256": "0" * 64}, {}),
                 (wrist, nav, {"peer_positions": {"r2": [1, 2]}})]
        for a, b, memory in cases:
            with self.subTest(memory=memory), self.assertRaises(ValueError):
                p.decide(a, b, memory, cargo_id="small_box_01", destination_zone="A", skill_state="idle")
        self.assertEqual(c.calls, [])

    def test_invalid_model_schema_bounds_and_reason_raise_without_fallback(self):
        bad = [
            {"action": {"kind": "drive", "fwd": .16, "turn": 0, "duration": .5}, "reason": "전진함"},
            {"action": {"kind": "wait"}, "reason": "기다림"},
            {"action": {"kind": "pick", "target": "box"}, "reason": "집기함"},
            {"action": {"kind": "approach"}, "reason": "english only"},
            {"action": {"kind": "drive", "fwd": float("nan"), "turn": 0, "duration": .5}, "reason": "전진함"},
        ]
        for reply in bad:
            with self.subTest(reply=reply):
                c = Completer(reply); p = GeminiTransportPlanner("r1", c)
                with self.assertRaises(PlanningError) as caught:
                    p.decide(obs("robot_cam", (0, 0, 0)), obs("nav_cam", (0, 0, 0), frame=2), [],
                             cargo_id="small_box_01", destination_zone="A", skill_state="idle")
                self.assertIsNotNone(caught.exception.raw)
                self.assertIsNone(p.last_audit["parsed"])

    def test_phase_incompatible_model_choice_is_rejected(self):
        cases = [({"kind": "pick"}, "idle"), ({"kind": "release"}, "ready_to_pick"),
                 ({"kind": "finish"}, "carrying"), ({"kind": "approach"}, "carrying")]
        for action, state in cases:
            with self.subTest(action=action, state=state), self.assertRaises(PlanningError):
                self.call({"action": action, "reason": "현재 상태와 맞지 않는 선택임"}, skill_state=state)

    def test_grip_uncertainty_requires_explicit_llm_check_before_motion_or_release(self):
        check = {"action": {"kind": "check_grip"},
                 "reason": "긴 운반 뒤 그립이 불확실해 팔 프로브로 다시 확인함"}
        self.assertEqual(self.call(check, skill_state="grip_uncertain")[2]["action"],
                         {"kind": "check_grip"})
        wait = {"action": {"kind": "wait", "duration": .4},
                "reason": "화면 가림이 사라질 때까지 잠시 기다림"}
        self.assertEqual(self.call(wait, skill_state="grip_uncertain")[2]["action"]["kind"], "wait")
        forbidden = [
            {"kind": "drive", "fwd": .03, "turn": 0, "duration": .4},
            {"kind": "release"}, {"kind": "approach"}, {"kind": "pick"}, {"kind": "finish"},
        ]
        for action in forbidden:
            with self.subTest(action=action), self.assertRaises(PlanningError):
                self.call({"action": action, "reason": "확인하지 않고 다음 행동을 선택함"},
                          skill_state="grip_uncertain")

    def test_check_grip_is_not_available_outside_uncertain_state(self):
        reply = {"action": {"kind": "check_grip"}, "reason": "그립을 다시 확인함"}
        for state in ("idle", "ready_to_pick", "carrying", "released"):
            with self.subTest(state=state), self.assertRaises(PlanningError):
                self.call(reply, skill_state=state)

    def test_released_state_allows_model_to_choose_reapproach_and_carries_previous_action(self):
        first = {"action": {"kind": "approach"}, "reason": "놓인 위치가 불확실해 다시 접근함"}
        c=Completer(first);p=GeminiTransportPlanner("r1",c)
        args=(obs("robot_cam",(0,0,0)),obs("nav_cam",(0,0,0),frame=2),
              [{"placement":{"status":"outside","stage":"released","source":"own_rgb"}}])
        assert p.decide(*args,cargo_id="small_box_01",destination_zone="A",skill_state="released")["action"]["kind"]=="approach"
        c.reply={"action":{"kind":"wait","duration":.3},"reason":"새 화면을 기다림"}
        p.decide(*args,cargo_id="small_box_01",destination_zone="A",skill_state="approaching")
        context=json.loads(c.calls[-1][0][-1]["content"])
        assert context["previous_model_action"]=={"kind":"approach"}

    def test_previous_nav_is_only_sent_for_pixel_stagnation(self):
        c=Completer({"action":{"kind":"drive","fwd":.04,"turn":.1,"duration":.5},
                     "reason":"현재 화면을 보고 짧게 이동함"})
        p=GeminiTransportPlanner("r1",c)
        wrist=obs("robot_cam",(0,0,255));nav1=obs("nav_cam",(255,0,0),frame=2)
        p.decide(wrist,nav1,[],cargo_id="small_box_01",destination_zone="A",skill_state="carrying")
        self.assertEqual(len(c.calls[0][1]),2)
        nav2=obs("nav_cam",(0,255,0),frame=3)
        p.decide(obs("robot_cam",(0,0,200),frame=3),nav2,[],cargo_id="small_box_01",destination_zone="A",skill_state="carrying")
        labels=[item["label"].split()[0] for item in c.calls[1][1]]
        self.assertEqual(labels,["CURRENT_WRIST","CURRENT_NAV"])
        nav3=obs("nav_cam",(0,255,0),frame=4)
        p.decide(obs("robot_cam",(0,0,180),frame=4),nav3,[],cargo_id="small_box_01",destination_zone="A",skill_state="carrying")
        labels=[item["label"].split()[0] for item in c.calls[2][1]]
        self.assertEqual(labels,["CURRENT_WRIST","CURRENT_NAV","PREVIOUS_NAV"])
        previous_bytes=base64.b64decode(c.calls[2][1][2]["image"].split(",",1)[1])
        self.assertEqual(hashlib.sha256(previous_bytes).hexdigest(),nav2["sha256"])
        self.assertEqual(p.last_audit["input_frames"][2]["frame_id"],3)
        self.assertEqual(p.last_audit["image_count"],3)

    def test_failed_reply_does_not_replace_previous_nav_or_action(self):
        good={"action":{"kind":"wait","duration":.2},"reason":"화면을 잠시 더 확인함"}
        c=Completer(good);p=GeminiTransportPlanner("r1",c)
        p.decide(obs("robot_cam",(0,0,0)),obs("nav_cam",(10,0,0),frame=2),[],
                 cargo_id="small_box_01",destination_zone="A",skill_state="carrying")
        c.reply={"bad":"reply"}
        with self.assertRaises(PlanningError):
            p.decide(obs("robot_cam",(0,0,0),frame=3),obs("nav_cam",(20,0,0),frame=3),[],
                     cargo_id="small_box_01",destination_zone="A",skill_state="carrying")
        self.assertEqual(p._previous_nav["frame_id"],2)
        self.assertEqual(p._previous_model_action,{"kind":"wait","duration":.2})
        self.assertEqual(p.last_audit["input_frames"][2]["frame_id"],2)

    def test_compacts_memory_and_audits_actual_request_size(self):
        c=Completer({"action":{"kind":"wait","duration":.2},"reason":"카메라를 더 확인함"})
        p=GeminiTransportPlanner("r1",c)
        memory=[]
        for index in range(20):
            memory.append({"action":{"kind":"drive","fwd":.03,"turn":0,"duration":.5},
                           "frame_geometry":{"pixels":list(range(200))},
                           "sha256":"f"*64,
                           "placement":{"status":"uncertain","reason":"가림", "stage":"carrying",
                                        "source":"own_rgb"}})
        memory.append({"feedback":"OWN_RGB_DRIVE_BLOCKED", "error":{"code":"DRIVE_FAILED","reason":"앞 위험"},
                       "placement":{"status":"inside","reason":"세 변 확인", "stage":"released",
                                    "source":"own_rgb"}})
        p.decide(obs("robot_cam",(0,0,0)),obs("nav_cam",(0,0,0),frame=2),memory,
                 cargo_id="small_box_01",destination_zone="A",skill_state="carrying")
        context=json.loads(c.calls[0][0][-1]["content"])
        compact=context["own_local_memory"]
        self.assertEqual(compact["latest_placement"]["status"],"inside")
        self.assertLessEqual(len(compact["last_actions"]),3)
        self.assertNotIn("frame_geometry",json.dumps(compact))
        self.assertNotIn("sha256",json.dumps(compact))
        self.assertLessEqual(p.last_audit["serialized_context_chars"],6000)
        self.assertEqual(p.last_audit["image_count"],2)

    def test_drive_duration_can_be_four_seconds_but_wait_remains_one(self):
        action={"action":{"kind":"drive","fwd":.05,"turn":0,"duration":4.0},
                "reason":"보이는 빈 통로로 제한 시간 동안 이동함"}
        self.assertEqual(self.call(action,skill_state="carrying")[2]["action"]["duration"],4.0)
        with self.assertRaises(PlanningError):
            self.call({"action":{"kind":"wait","duration":1.1},"reason":"화면을 더 기다림"},
                      skill_state="carrying")

    def test_six_long_inbox_messages_are_trimmed_to_context_cap_without_failure(self):
        reply={"action":{"kind":"wait","duration":.2},"reason":"새 메시지와 화면을 확인함"}
        c=Completer(reply);p=GeminiTransportPlanner("r1",c,communication_mode="natural")
        messages=[]
        for index in range(6):
            messages.append({"sender":f"r{index}", "request_time":float(index),
                             "call_id":"c"+"x"*200, "model_decision_id":"d"+"y"*200,
                             "age_s":index, "message":"동료 관찰 "+("긴내용"*100)})
        p.decide(obs("robot_cam",(0,0,0)),obs("nav_cam",(0,0,0),frame=2),
                 [{"feedback":"NAVIGATION_INTERRUPTED:OWN_RGB_STAGNATION:"+("위험"*500)}],
                 cargo_id="small_box_01",destination_zone="A",skill_state="carrying",
                 messages=messages)
        context=json.loads(c.calls[0][0][-1]["content"])
        self.assertLessEqual(p.last_audit["serialized_context_chars"],6000)
        self.assertLessEqual(len(context["incoming_messages"]),6)
        self.assertTrue(all("call_id" not in item and "model_decision_id" not in item
                            for item in context["incoming_messages"]))
        self.assertIn("OWN_RGB_STAGNATION",json.dumps(context["own_local_memory"]))

    def test_visual_progress_is_camera_only_bounded_and_does_not_choose_action(self):
        reply={"action":{"kind":"drive","fwd":.04,"turn":.1,"duration":.5},
               "reason":"현재 화면 근거로 같은 이동을 직접 선택함"}
        c=Completer(reply);p=GeminiTransportPlanner("r1",c)
        for frame in range(2,10):
            p.decide(obs("robot_cam",(0,0,0),frame=frame),
                     obs("nav_cam",(20,30,40),frame=frame),[],cargo_id="small_box_01",
                     destination_zone="A",skill_state="carrying")
        context=json.loads(c.calls[-1][0][-1]["content"])
        evidence=context["visual_progress"]
        self.assertTrue(evidence["nav_nearly_unchanged"])
        self.assertGreaterEqual(evidence["stagnation_streak"],2)
        self.assertGreaterEqual(evidence["same_action_streak"],2)
        self.assertLessEqual(len(context["recent_visual_history"]),6)
        serialized=json.dumps(context)
        self.assertNotIn("recommended",serialized)
        self.assertNotIn("truth",serialized)
        self.assertEqual(p._previous_model_action,reply["action"])
        c.reply={"action":{"kind":"wait","duration":.2},"reason":"화면 변화 뒤 다시 관찰함"}
        p.decide(obs("robot_cam",(0,0,0),frame=10),obs("nav_cam",(255,255,255),frame=10),[],
                 cargo_id="small_box_01",destination_zone="A",skill_state="carrying")
        changed=json.loads(c.calls[-1][0][-1]["content"])["visual_progress"]
        self.assertFalse(changed["nav_nearly_unchanged"])
        self.assertEqual(changed["stagnation_streak"],0)
        self.assertEqual(p.last_audit["visual_progress"],changed)
        self.assertEqual(p.last_audit["communication_mode"],"none")
        self.assertIsNone(p.last_audit["incoming_messages"])

    def test_none_mode_omits_messages_from_input_and_rejects_output_message(self):
        c=Completer({"action":{"kind":"wait","duration":.2},"reason":"카메라를 더 확인함"})
        p=GeminiTransportPlanner("r1",c)
        p.decide(obs("robot_cam",(0,0,0)),obs("nav_cam",(0,0,0),frame=2),[],
                 cargo_id="small_box_01",destination_zone="A",skill_state="carrying",
                 messages=[{"observed":"동료 메시지"}])
        context=json.loads(c.calls[0][0][-1]["content"])
        self.assertNotIn("incoming_messages",context)
        self.assertNotIn("outgoing_message",c.calls[0][0][0]["content"])
        c.reply={"action":{"kind":"wait","duration":.2},"reason":"카메라를 더 확인함",
                 "outgoing_message":"앞이 보입니다"}
        with self.assertRaises(PlanningError):
            p.decide(obs("robot_cam",(0,0,0),frame=3),obs("nav_cam",(0,0,0),frame=3),[],
                     cargo_id="small_box_01",destination_zone="A",skill_state="carrying")

    def test_status_and_natural_communication_are_opt_in_and_private_reason_is_separate(self):
        envelope={"sender":"r2","request_time":1.0,"call_id":"c1","model_decision_id":"d1",
                  "message":{"observed":"앞에 장애물이 보임","intent":"wait","request":"clear_path"}}
        status_reply={"action":{"kind":"wait","duration":.2},"reason":"내 화면의 장애물을 확인함",
                      "outgoing_message":{"observed":"주황 장애물이 보임","intent":"wait",
                                          "request":"clear_path"}}
        c=Completer(status_reply);p=GeminiTransportPlanner("r1",c,communication_mode="status")
        result=p.decide(obs("robot_cam",(0,0,0)),obs("nav_cam",(0,0,0),frame=2),[],
                        cargo_id="small_box_01",destination_zone="A",skill_state="carrying",
                        messages=[envelope])
        context=json.loads(c.calls[0][0][-1]["content"])
        system=c.calls[0][0][0]["content"]
        self.assertEqual(context["incoming_messages"],[envelope])
        self.assertIn("첫 idle 호출",system)
        self.assertIn("고정 할당",system)
        self.assertIn("틀릴 수 있는 진술",system)
        self.assertIn("물리적 화물 인계나 공동 운반 기능은 없",system)
        self.assertIn("시스템이 자동 생성하지 않는다",system)
        self.assertEqual(result["reason"],"내 화면의 장애물을 확인함")
        self.assertEqual(result["outgoing_message"],status_reply["outgoing_message"])
        natural={"action":{"kind":"wait","duration":.2},"reason":"카메라를 더 확인함",
                 "outgoing_message":"앞에 주황 장애물이 보여 잠시 기다립니다"}
        self.assertEqual(self._communicating_result("natural",natural)["outgoing_message"],
                         natural["outgoing_message"])

    def _communicating_result(self, mode, reply):
        c=Completer(reply);p=GeminiTransportPlanner("r1",c,communication_mode=mode)
        return p.decide(obs("robot_cam",(0,0,0)),obs("nav_cam",(0,0,0),frame=2),[],
                        cargo_id="small_box_01",destination_zone="A",skill_state="carrying")

    def test_invalid_outgoing_is_dropped_without_losing_valid_movement(self):
        bad={"action":{"kind":"drive","fwd":.03,"turn":0,"duration":.3},
             "reason":"보이는 통로로 짧게 이동함",
             "outgoing_message":{"observed":"앞이 보임","intent":"teleport","request":"none"}}
        result=self._communicating_result("status",bad)
        self.assertEqual(result["action"],bad["action"])
        self.assertNotIn("outgoing_message",result)

    def test_incoming_messages_reject_spatial_oracle_fields_before_model_call(self):
        c=Completer({"action":{"kind":"wait","duration":.2},"reason":"잠시 기다림"})
        p=GeminiTransportPlanner("r1",c,communication_mode="status")
        with self.assertRaises(ValueError):
            p.decide(obs("robot_cam",(0,0,0)),obs("nav_cam",(0,0,0),frame=2),[],
                     cargo_id="small_box_01",destination_zone="A",skill_state="carrying",
                     messages=[{"sender":"r2","coordinates":[1,2]}])
        self.assertEqual(c.calls,[])

    def test_none_mode_prompt_is_unchanged_by_shared_communication_guidance(self):
        c=Completer({"action":{"kind":"wait","duration":.2},"reason":"현재 화면을 더 확인함"})
        p=GeminiTransportPlanner("r1",c)
        p.decide(obs("robot_cam",(0,0,0)),obs("nav_cam",(0,0,0),frame=2),[],
                 cargo_id="small_box_01",destination_zone="A",skill_state="idle")
        system=c.calls[0][0][0]["content"]
        self.assertEqual(system, __import__("harness.gemini_transport_policy", fromlist=["PROMPT"]).PROMPT)
        self.assertNotIn("첫 idle 호출",system)
        self.assertNotIn("incoming_messages",system)

    def test_both_shared_modes_request_model_authored_initial_announcement(self):
        replies={
            "status":{"action":{"kind":"approach"},"reason":"화물 접근을 시작함",
                      "outgoing_message":{"observed":"small_box_01을 A로 운반 시작",
                                          "intent":"approach","request":"none"}},
            "natural":{"action":{"kind":"approach"},"reason":"화물 접근을 시작함",
                       "outgoing_message":"small_box_01을 A 목적지로 운반 시작합니다"},
        }
        for mode,reply in replies.items():
            with self.subTest(mode=mode):
                c=Completer(reply);p=GeminiTransportPlanner("r1",c,communication_mode=mode)
                result=p.decide(obs("robot_cam",(0,0,0)),obs("nav_cam",(0,0,0),frame=2),[],
                                cargo_id="small_box_01",destination_zone="A",skill_state="idle")
                system=c.calls[0][0][0]["content"]
                self.assertIn("첫 idle 호출",system)
                self.assertIn("네가 맡은 화물과 목적지, 시작 의도",system)
                self.assertIn("메시지는 네가 직접 작성",system)
                self.assertIn("outgoing_message",result)


def test_real_message_envelope_keeps_content_when_compacted():
    from harness.gemini_transport_policy import _compact_incoming
    source = {"message_id":"m1","sender":"r2","content":"앞에 장애물이 있습니다",
              "age_s":1.2,"decision_id":"unused-long-metadata"}
    compact = _compact_incoming(source)
    assert compact['content'] == source['content']
    assert compact['message_id'] == 'm1'
    assert 'decision_id' not in compact


if __name__ == "__main__": unittest.main()
