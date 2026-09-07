import pathlib
import unittest
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]

class SimUiConsistencyTests(unittest.TestCase):
    def test_browser_workcell_is_loaded_from_canonical_mujoco_xml(self):
        js=(ROOT/"harness/static/sim3d.js").read_text()
        self.assertIn("fetch('/sim-scene.xml'", js)
        self.assertIn("world.children", js)
        self.assertNotIn("box(scene,[.44,.012,.36],[.72,.006,.82],mat.yellow)", js)
        self.assertNotIn("box(scene,[.44,.012,.36],[.72,.006,-.82],mat.blue)", js)

    def test_canonical_scene_has_expected_delivery_signs_and_walls(self):
        root=ET.parse(ROOT/"sim/masterpi_scene.xml").getroot()
        geoms={g.get("name"):g for g in root.find("worldbody").findall("geom") if g.get("name")}
        self.assertEqual(geoms["delivery_blue"].get("pos"), ".72 -.82 .006")
        self.assertEqual(geoms["delivery_yellow"].get("pos"), ".72 .82 .006")
        self.assertEqual(geoms["wall_back"].get("pos"), "1.8 0 .65")
        self.assertIn("crate_blue", geoms)
        self.assertIn("crate_yellow", geoms)

    def test_lightning_package_has_remote_worker_supervisor(self):
        src=(ROOT/"scripts/lightning_worker_recover.py").read_text()
        self.assertIn("supervise_worker.sh", src)
        self.assertIn("UGRP_LIGHTNING_MACHINES", src)
        self.assertIn("pkill -f '[s]upervise_worker.sh'", src)

    def test_remote_worker_bundles_share_one_v2_manifest(self):
        manifest=(ROOT/"scripts/gpu_worker_bundle.py").read_text()
        self.assertIn('"sim/masterpi_scene.xml"', manifest)
        self.assertIn('"sim/masterpi_dynamics_v2.py"', manifest)
        self.assertIn('"sim/masterpi_dynamics_calibration.json"', manifest)
        self.assertIn('"scripts/red_block/recorder.py"', manifest)
        for rel in ("scripts/lightning_worker_recover.py","scripts/colab_worker_recover.py"):
            provider=(ROOT/rel).read_text()
            self.assertIn("gpu_worker_bundle import BUNDLE_FILES", provider)
        # Remote production imports PRIMITIVE_MOTIONS from this module at
        # action time; it must travel with every GPU worker bundle.
        from scripts.gpu_worker_bundle import BUNDLE_FILES
        self.assertIn("scripts/robot_actions.py", BUNDLE_FILES)

    def test_dashboard_has_no_obsolete_manual_robot_control_panel(self):
        html=(ROOT/"harness/static/index.html").read_text()
        for obsolete in (
            'id="robot-status"', 'id="robot-note"', 'data-drive=',
            'id="motors"', 'id="arm"', 'id="stop"',
            'function renderRobot(', 'function loadRobot(',
            'async function command(', '모터 / 팔',
        ):
            self.assertNotIn(obsolete, html)

    def test_sim_dashboard_prioritizes_main_views_and_collapses_tools(self):
        html=(ROOT/"harness/static/index.html").read_text()
        self.assertIn('<body data-mode="sim">', html)
        self.assertIn('document.body.dataset.mode = activeMode;', html)
        self.assertIn('grid-template-columns: repeat(2, minmax(0, 1fr))', html)
        self.assertIn('body[data-mode="sim"] .views { flex: 1 1 auto;', html)
        self.assertIn('grid-template-rows: minmax(0, 1fr)', html)
        self.assertIn('<details class="card tools-card">', html)
        self.assertNotIn('<details class="card tools-card" open>', html)
        self.assertIn('/sim3d.js?v=20260901wake1', html)

    def test_gpu_offline_exposes_manual_wake_button(self):
        html=(ROOT/"harness/static/index.html").read_text()
        js=(ROOT/"harness/static/sim3d.js").read_text()
        web=(ROOT/"harness/web.py").read_text()
        self.assertIn('id="sim-wake"', html)
        self.assertIn('>SIM 켜기</button>', html)
        self.assertIn('/api/sim/wake', html)
        self.assertIn('function renderSimPower(online, recovery = {})', html)
        self.assertIn('simWakeButton.hidden = true', html)
        self.assertIn('simWakeButton.textContent = recovering ? "켜는 중…" : "SIM 켜기"', html)
        self.assertIn("window.renderSimPower(online,recovery)", js)
        self.assertIn('if path == "/api/sim/wake":', web)
        self.assertIn('ugrp-sim-gpu-recover.service', web)

    def test_gpu_offline_uses_explicit_static_preview_not_fake_live_video(self):
        html=(ROOT/"harness/static/index.html").read_text()
        js=(ROOT/"harness/static/sim3d.js").read_text()
        self.assertIn('OFFLINE PREVIEW', html)
        self.assertIn('/offline/cctv_front_left.jpg', html)
        self.assertIn('/offline/robot.jpg', html)
        self.assertIn("physicsBadge.textContent=!online?'GPU OFFLINE'", js)
        self.assertIn('setSimOfflineVisuals(!online)', js)

    def test_sim_exposes_physical_grasp_telemetry(self):
        html=(ROOT/"harness/static/index.html").read_text()
        js=(ROOT/"harness/static/sim3d.js").read_text()
        self.assertIn('id="grasp-hud"', html)
        self.assertIn('BILATERAL', js)
        self.assertIn('left_normal_N', js)
        self.assertIn('right_normal_N', js)
        self.assertIn('gripper_qpos', js)

    def test_sim_dashboard_has_exactly_two_live_camera_views(self):
        html=(ROOT/"harness/static/index.html").read_text()
        self.assertEqual(html.count('class="camera robot-view"'), 1)
        self.assertEqual(html.count('class="camera observer"'), 1)
        self.assertIn('preview.src = api("/api/camera")', html)
        self.assertIn('/api/observer/cctv_front_left/snapshot', html)
        self.assertIn('3인칭 · 전면 추적 카메라 · MuJoCo RGB', html)
        for removed in ("cctv_front_right", "cctv_rear_left", "cctv_top", 'id="semantic-map-panel"', 'id="semantic-radar"'):
            self.assertNotIn(removed, html)

    def test_multi_robot_tabs_keep_independent_inflight_sessions(self):
        html=(ROOT/"harness/static/index.html").read_text()
        for rid in ("r1", "r2", "r3"):
            self.assertIn(f'data-robot="{rid}"', html)
        self.assertIn('data-robot="team"', html)
        self.assertIn("stashRobotSession(activeMode, activeRobot)", html)
        self.assertIn("state = { root: document.createElement('div'), started: false, sending: false, abortCtl: null }", html)
        self.assertIn('const requestPrefix = robotApiPrefix(requestMode, requestRobot);', html)
        self.assertIn('fetch(requestPrefix + "/api/turn"', html)
        self.assertIn('signal: session.abortCtl.signal', html)
        self.assertIn('fetch(requestPrefix + "/api/cancel"', html)
        self.assertIn('function requestRunCancel(requestPrefix)', html)
        self.assertIn('연결이 끊겼습니다. 안전을 위해 로봇 실행 중단을 요청했습니다.', html)
        self.assertIn('.robot-tab.running::after', html)
        self.assertIn('button.classList.toggle("running", running)', html)
        # Switching views is not an execution cancel operation. Only the Stop
        # button may target the active robot's /api/cancel endpoint.
        switch_start=html.index('async function switchRobot(robot)')
        switch_end=html.index('function teamStateSummary', switch_start)
        self.assertNotIn('/api/cancel', html[switch_start:switch_end])
        self.assertIn("const running = ['r1','r2','r3'].filter", html)
        self.assertIn("shared world 초기화는 실행 종료 후 가능합니다", html)
        self.assertIn("state.root.replaceChildren()", html)
        self.assertIn("body:JSON.stringify({goal, mode:activeMode})", html)

    def test_team_is_shared_bot_chat_with_mentions_and_wake_status(self):
        html=(ROOT/"harness/static/index.html").read_text()
        self.assertIn('<h1>TEAM chat</h1>', html)
        self.assertIn('id="team-cam-overview"', html)
        self.assertIn('/api/team/camera/sim/overview', html)
        self.assertIn('[hidden] { display: none !important; }', html)
        self.assertIn('id="team-chat-form"', html)
        self.assertIn('id="team-chat-log"', html)
        self.assertIn("fetch('/api/team/chat'", html)
        self.assertIn('@R1/@R2/@R3', html)
        self.assertIn("badge.textContent = 'THINKING'", html)
        self.assertIn("badge.textContent = 'QUEUED'", html)
        self.assertIn('REAL TEAM 자동 호출은 대화만', html)

    def test_inactive_mode_streams_and_rendering_are_suspended(self):
        html=(ROOT/"harness/static/index.html").read_text()
        js=(ROOT/"harness/static/sim3d.js").read_text()
        self.assertIn('data-snapshot-src="/api/observer/cctv_front_left/snapshot"', html)
        self.assertNotIn('class="cctv-native" src="/api/observer/cctv_front_left/snapshot"', html)
        self.assertIn('setObserverStreams(activeMode === "sim" && document.visibilityState === "visible")', html)
        self.assertIn('document.addEventListener("visibilitychange"', html)
        self.assertIn('cameraTimer = setTimeout(refresh, 250)', html)
        self.assertIn("document.visibilityState!=='visible'", js)
        self.assertIn("currentMode==='real'?1000", js)
        self.assertIn("GPU OFFLINE')?1000:250", js)
        self.assertIn("currentMode==='real'?66:33", js)
        self.assertIn("currentMode==='real' && v.kind!=='radar'", js)


class PlannerInputParityTests(unittest.TestCase):
    def test_sim_preserves_real_tool_schema_plus_calibrated_team_actions(self):
        from harness.catalog import default_registry
        real = default_registry(actions_path="scripts/robot_actions.py")
        sim = default_registry(actions_path="scripts/sim_actions.py")
        self.assertEqual(
            set(sim.names()) - set(real.names()),
            {
                "stage_base", "stack_on", "team_tower", "team_beam_transport",
                "team_zone_transfer",
            },
        )
        self.assertEqual(set(real.names()) - set(sim.names()), set())
        schema = sim.prompt_schema()
        self.assertIn("stage_base", schema)
        self.assertIn("stack_on", schema)
        self.assertIn("target_color", schema)
        self.assertIn("destination_color", schema)

    def test_live_launchers_share_planner_policy_defaults(self):
        real = (ROOT / "scripts/serve_real.sh").read_text()
        sim = (ROOT / "scripts/serve_sim_coworker.sh").read_text()
        for value in (
            'UGRP_STRUCTURED_RED_FASTPATH="${UGRP_STRUCTURED_RED_FASTPATH:-0}"',
            'GROQ_RATE_LIMIT_RETRIES="${GROQ_RATE_LIMIT_RETRIES:-0}"',
            'GROQ_REQUEST_TIMEOUT="${GROQ_REQUEST_TIMEOUT:-8}"',
        ):
            self.assertIn(value, real)
            self.assertIn(value, sim)
        self.assertIn('UGRP_CAMERA_SELF_MASK="${UGRP_CAMERA_SELF_MASK:-masterpi_eye_in_hand}"', real)
        self.assertIn('UGRP_CAMERA_SELF_MASK="masterpi_eye_in_hand"', sim)
        self.assertIn('UGRP_PLANNER_IMAGE_MAX_EDGE="${UGRP_PLANNER_IMAGE_MAX_EDGE:-320}"', real)
        self.assertIn('UGRP_PLANNER_IMAGE_MAX_EDGE="${UGRP_PLANNER_IMAGE_MAX_EDGE:-320}"', sim)

    def test_planner_tool_feedback_removes_environment_specific_geometry(self):
        from harness.loop import _planner_tool_result
        common = {
            "ok": True, "tool": "track",
            "result": {
                "skill": "track", "command_status": "ACCEPTED",
                "execution_status": "COMPLETED", "outcome_status": "ACHIEVED",
                "target_color": "red", "center_verified": True,
            },
        }
        real = {**common, "result": {**common["result"],
            "target_vision": {"visible": True},
            "verification_source": "pi_multiframe_centered_track",
        }}
        sim = {**common, "result": {**common["result"],
            "target_vision": {"visible": True, "cx": .12, "cy": .81, "area_ratio": .09, "bbox": [0,0,1,1]},
            "camera_red": {"visible": True, "cx": .12},
            "verification_source": "sim_internal_detail",
            "spatial_memory": {"red": {"position_xy": [1.2, 3.4]}},
        }}
        self.assertEqual(_planner_tool_result(real), _planner_tool_result(sim))
        text = repr(_planner_tool_result(sim))
        self.assertNotIn("position_xy", text)
        self.assertNotIn("verification_source", text)
        self.assertNotIn("bbox", text)
        self.assertNotIn("area_ratio", text)

    def test_planner_world_state_hides_metric_simulator_geometry(self):
        from harness.loop import _planner_state_public
        from harness.state import StateEstimator
        est = StateEstimator()
        est.state.spatial_memory = {
            "red": {
                "visible": True, "image_xy": [.4, .6],
                "metric_position_available": True, "position_xy": [1.2, 3.4],
                "relation": "GROUND", "distance_m": .12, "bearing_deg": 9.0,
            }
        }
        landmark = _planner_state_public(est.state)["landmarks"]["red"]
        self.assertEqual(landmark, {"visible": True, "image_xy": [.4, .6]})

    def test_passive_camera_paths_do_not_mutate_planner_state(self):
        web = (ROOT / "harness/web.py").read_text()
        self.assertIn('self.remember_frame(frames[-1], perceive=False)', web)
        self.assertIn('self.state.remember_frame(frames[-1], perceive=False)', web)
        self.assertIn('state.remember_frame(jpeg, perceive=False)', web)
        self.assertIn('self._async_real_perception = False', web)

    def test_sim_action_result_does_not_surface_mujoco_geometry(self):
        from unittest.mock import patch
        import scripts.sim_actions as sim_actions
        raw = {
            "ok": True, "reason": "done",
            "vision": {"visible": True, "cx": .2, "cy": .7, "area_ratio": .1},
            "blue_vision": {"visible": True, "cx": .8},
            "spatial_memory": {"red": {"position_xy": [9.0, 9.0], "distance_m": .1}},
        }
        with patch.object(sim_actions, "_bridge_health", return_value={}), \
             patch.object(sim_actions, "_post_bridge_action", return_value=raw):
            result = sim_actions.run("search", target_color="red")
        self.assertEqual(result["target_vision"], {"visible": True})
        self.assertEqual(result["camera_red"], {"visible": True})
        self.assertNotIn("camera_blue", result)
        self.assertNotIn("spatial_memory", result)


if __name__ == "__main__": unittest.main()


class SimPickContractParityTests(unittest.TestCase):
    def test_sim_chat_state_uses_strict_precision_pick_preconditions(self):
        from unittest.mock import patch
        from harness.loop import ReplayCompleter
        from harness.web import ChatState
        with patch.dict("os.environ", {"UGRP_UI_MODE": "sim"}, clear=False):
            state = ChatState(
                completer=ReplayCompleter([]),
                actions_path="scripts/sim_actions.py",
            )
        self.assertTrue(state.executive.strict_pick_preconditions)
