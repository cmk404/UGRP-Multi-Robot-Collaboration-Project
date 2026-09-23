"""Incremental RGB skills and the single-owner MuJoCo backend.

No controller receives a world, evaluator, peer camera, or committed full plan.
The scheduler owns physics; image-only workers return macros, never call ports.
This is an experimental adapter, not a claim of transport or map generalization.
"""
from __future__ import annotations

import base64
import copy
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
from statistics import median
import threading
import time
from types import SimpleNamespace
from typing import Callable
import xml.etree.ElementTree as ET

from harness.rgb_execution_bundle import (load_bundle, require_effective, baseline_diff,
                                          source_identity, environment_fingerprint)
from harness.rgb_execution_contract import SkillCapability, finite_number
from harness.rgb_execution_port import RGBExecutionPort, _serialized

ROOT = Path(__file__).resolve().parents[1]
BACKEND_ID = "rgb_incremental_dispatch_v1"
SCHEMA = "ugrp.rgb_skill_backend.v2"
MAX_TICK_S = .05
FRAME_JPEG_QUALITY = 95
ADAPTER_CONTACT_PROFILE = "local_contact_fine"
SOLO_MAX_DECISIONS = 1200
PAIR_COARSE_BASE_DECISIONS = 170
PAIR_COARSE_MAX_DECISIONS = 300
PAIR_COARSE_PROGRESS_WINDOW = 24
PAIR_COARSE_MIN_WINDOW_PROGRESS_PX = .5
INITIAL_COMMANDS = {"1": 2000, "3": 740, "4": 2320, "5": 1320, "6": 1500}
SKILLS = {f"{kind}_transport_{goal}": {
    "object_id": "box" if kind == "solo" else "beam",
    "team_size": 1 if kind == "solo" else 2,
    "roles": ["solo"] if kind == "solo" else ["lower", "upper"],
    "dock": "dock_" + goal.lower(), "route": "south" if kind == "solo" else "north",
    "resources": ["box", "south_gate"] if kind == "solo" else ["beam", "north_gate"],
} for kind in ("solo", "pair") for goal in ("A", "B")}


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _pose_schedule_probe():
    """Measure the solo adapter's actual pan samples and completion deadline."""
    macro = MacroQueue(lambda _action, _duration: None, dict(INITIAL_COMMANDS),
                       solo_pose_parity=True)
    macro.submit({"kind": "pose", "pulses": {"6": 1560}}, 0., phase="attachment_left")
    return {"pan_samples": [action["pan_pulse"] for _, action in macro.events],
            "completion_s": round(macro.until, 6)}


def _pair_pose_schedule_probe():
    macro = MacroQueue(lambda _action, _duration: None, dict(INITIAL_COMMANDS))
    macro.submit({"kind": "pose", "pulses": {"6": 1560}}, 0.)
    return {"pan_samples": [action["pan_pulse"] for _, action in macro.events],
            "completion_s": round(macro.until, 6)}


def _execution_contract():
    from harness.dispatch_skill_binding import (SOLO_GOAL_CONTROL_INSET_PX,
        PAIR_OWN_BOUNDS_PAD_PX, PAIR_MAX_CENTER_STEP_PX, PAIR_MAX_BOUNDS_STEP_PX)
    return {"owner": "RGBSkillExecutionPort", "macro": "MacroQueue",
            "owner_max_tick_s": MAX_TICK_S, "pose_schedule_probe": _pose_schedule_probe(),
            "pair_pose_schedule_probe": _pair_pose_schedule_probe(),
            "solo_max_decisions": SOLO_MAX_DECISIONS,
            "solo_goal_control_inset_px": SOLO_GOAL_CONTROL_INSET_PX,
            "pair_heading_prior_bounds": {"pad_px": PAIR_OWN_BOUNDS_PAD_PX,
                "max_center_step_px": PAIR_MAX_CENTER_STEP_PX,
                "max_bounds_step_px": PAIR_MAX_BOUNDS_STEP_PX,
                "reject_clipped_prior_mask": True},
            "pair_heading_component_fallback": {
                "membership": "centroid_in_prior_own_bounds",
                "reject_clipped_support": True,
                "requires_four_corners": True},
            "pair_role_binding": "own_motion_claim_and_four_corner_top_bbox_same_beam_side_as_saved_slot",
            "pair_coarse_progress_budget": {
                "base_decisions": PAIR_COARSE_BASE_DECISIONS,
                "max_decisions": PAIR_COARSE_MAX_DECISIONS,
                "window_decisions": PAIR_COARSE_PROGRESS_WINDOW,
                "min_window_progress_px": PAIR_COARSE_MIN_WINDOW_PROGRESS_PX,
                "ready_wait_within_max": True},
            "study_tick_period_s": .05, "study_poll_period_s": .05,
            "worker_image_max_age_s": 1., "worker_wall_limit_s": 2.,
            "worker_wall_scope": "submission_to_consumption"}


def _applied_contract(scene):
    """Read live model/XML and camera dimensions after construction, before actor work."""
    xml = ET.fromstring(scene.xml)
    pairs = xml.findall("contact/pair")
    model_pairs = int(scene.world.model.npair)
    if len(pairs) != model_pairs:
        raise ValueError("XML/model explicit contact pair count differs")
    xml_step = float(xml.find("option").get("timestep"))
    if xml_step != float(scene.world.model.opt.timestep):
        raise ValueError("XML/model timestep differs")
    return {"physics": {"contact_solver_profile": scene.manifest.get("contact_solver_profile"),
                        "timestep_s": float(scene.world.model.opt.timestep),
                        "explicit_contact_pairs": model_pairs,
                        "weld_active": bool(scene.world.data.eq_active.any())},
            "camera": {"own_raw_size": [int(scene.world.width), int(scene.world.height)],
                       "top_raw_size": [int(scene.world.observer_width), int(scene.world.observer_height)],
                       "raw_jpeg_quality": FRAME_JPEG_QUALITY,
                       "solo_skill_size": [640, 480], "solo_skill_jpeg_quality": 95,
                       "solo_resize": "cv2.resize"},
            "execution": _execution_contract()}


def _read_models(root, manifest, *, staged=False):
    root = Path(root).resolve()
    data = json.loads((root / manifest).read_text())
    if set(data["models"]) != {"r1", "r3"}:
        raise ValueError("two independent saved model slots required")
    loaded, hashes = {}, {str(root / manifest): hashlib.sha256((root / manifest).read_bytes()).hexdigest()}
    for slot, records in data["models"].items():
        if staged and set(records) != {"yaw", "lateral", "forward"}:
            raise ValueError("three saved alignment stages required")
        models = {}
        for stage, record in (records.items() if staged else [("grasp", records)]):
            path = (root / record["path"]).resolve()
            if not path.is_relative_to(root):
                raise ValueError("model outside declared root")
            content = path.read_bytes()
            if hashlib.sha256(content).hexdigest() != record["sha256"]:
                raise ValueError("saved model hash mismatch")
            value = json.loads(content)
            # Recovery-kernel artifacts bind robot identity in the hashed
            # manifest, unlike alignment artifacts which also embed robot_id.
            if (("robot_id" in value and value["robot_id"] != slot)
                    or (staged and (value.get("robot_id") != slot or value.get("stage") != stage))):
                raise ValueError("saved model identity mismatch")
            models[stage] = value
            hashes[str(path)] = record["sha256"]
        loaded[slot] = models if staged else models["grasp"]
    return data, loaded, hashes


def map_support_matrix():
    """Reuse the existing suite/split validator; never remap unsupported maps."""
    from sim.act_map_suite import load_suite, layout_digest
    _, cases = load_suite()
    return [{"map_id": "dispatch_open", "scene_supported": True, "skill_supported": True,
             "scope": "development integration only; fixed right-facing lane convention"}] + [
        {"case_id": case["id"], "map_id": case["map"]["map_id"], "split": case["split"],
         "map_sha256": _digest(case["map"]), "layout_sha256": layout_digest(case["map"]),
         "scene_supported": True, "skill_supported": False,
         "reason": "preview scene only; local_goal/heading and RGB loaded-route contract not connected"}
        for case in cases]


def backend_descriptor(config):
    """Read-only preflight: no renderer, world, model inference, or process."""
    required = {"schema", "map_id", "seed", "output_dir", "max_sim_s", "max_commands",
                "grasp_model_dir", "stage_model_dir", "reference_top", "execution_bundle_id"}
    if not isinstance(config, dict) or set(config) != required or config["schema"] != SCHEMA:
        raise ValueError("exact RGB backend config required")
    execution_bundle, bundle_sha = load_bundle(config["execution_bundle_id"])
    if execution_bundle["effective"]["execution"] != _execution_contract():
        raise ValueError("RGB execution macro schedule or worker contract changed")
    if execution_bundle["effective"]["physics"]["contact_solver_profile"] != ADAPTER_CONTACT_PROFILE:
        raise ValueError("RGB execution contact profile changed")
    if config["map_id"] != "dispatch_open":
        raise ValueError("unsupported map: no open/north/south substitution; consult map_support_matrix")
    if type(config["seed"]) is not int or config["seed"] < 0:
        raise ValueError("nonnegative reset seed required")
    if not finite_number(config["max_sim_s"], minimum=.05) or config["max_sim_s"] > 300:
        raise ValueError("SIM cap must be .05..300 seconds")
    if type(config["max_commands"]) is not int or not 1 <= config["max_commands"] <= 10000:
        raise ValueError("command cap must be 1..10000")
    _, _, grasp_hashes = _read_models(config["grasp_model_dir"], "student-skill.json")
    _, _, stage_hashes = _read_models(config["stage_model_dir"], "varied-start-skill.json", staged=True)
    reference = Path(config["reference_top"]).read_bytes()
    if not reference.startswith(b"\xff\xd8") or not reference.endswith(b"\xff\xd9"):
        raise ValueError("original reference JPEG required")
    from sim.research_dispatch_arena import episode
    scene_config = episode("open", config["seed"])
    static_map = scene_config["static_map"]
    return {"schema": "ugrp.rgb_skill_backend_descriptor.v1", "backend_id": BACKEND_ID,
            "execution_bundle_id": execution_bundle["id"],
            "execution_bundle_sha256": bundle_sha,
            "execution_bundle_status": execution_bundle["status"],
            "execution_contract": execution_bundle["effective"]["execution"],
            "synthetic": False, "weld": False, "camera_fov_changed": False,
            "clock_owner": "single_simulator", "clock_domain": "sim", "max_tick_s": MAX_TICK_S,
            "supervisor_clock_schema": "ugrp.execution_clock.v1",
            "skill_image_max_age_s": 1., "skill_worker_wall_limit_s": 2.,
            "skill_worker_wall_scope": "submission_to_consumption",
            "capabilities": copy.deepcopy(SKILLS), "config_sha256": _digest(config),
            "map_sha256": _digest(scene_config["static_map"]),
            "map_id": static_map["map_id"], "map_version": str(static_map["version"]),
            "map_instance_sha256": _digest(static_map),
            "map_group_sha256": _digest({k: static_map[k] for k in ("bounds_m", "obstacles", "top_camera")}),
            "map_to_scene": {"builder": "sim.research_dispatch_arena.build_scene_xml",
                             "source_map_sha256": _digest(static_map), "scene_xml_sha256": None},
            "camera_sha256": _digest(scene_config["static_map"]["top_camera"]),
            "goal_frame": scene_config["static_map"]["frame"],
            "reset_sha256": _digest(scene_config["setup_only"]), "seed": config["seed"],
            "input_hashes": {**grasp_hashes, **stage_hashes,
                str(Path(config["reference_top"]).resolve()): hashlib.sha256(reference).hexdigest()},
            "physical_validation": "not established by preflight",
            "unsupported": ["generalized suite transport", "loaded rotation", "terrain",
                            "arbitrary heading/identity", "automatic partner or route selection"]}


def public_static_context(config):
    """Exact public task for D/C manifests without initializing physics."""
    descriptor = backend_descriptor(config)
    from sim.research_dispatch_arena import episode
    return _public_static_context(episode("open", config["seed"])["static_map"], descriptor)


def _public_static_context(static_map, descriptor):
    return {"schema": "ugrp.rgb_static_context.v1",
        "task": {"description": json.dumps({"instruction": "Choose an object, independent participants and roles, and a declared skill. "
                    "Skills encode immutable goal/route; unknown maps/headings fail closed. Own claims are unverified.",
                    "skills": SKILLS, "authored_map": static_map}, sort_keys=True),
                 "object_ids": ["box", "beam"], "allowed_skills": list(SKILLS)},
        "static_map": {"id": static_map["map_id"], "version": str(static_map["version"]),
            "sha256": descriptor["map_sha256"], "description": json.dumps(static_map, sort_keys=True)},
        "camera_calibration": {"id": "fixed_top_and_production_own", "version": "1",
            "sha256": descriptor["camera_sha256"], "description": json.dumps(static_map["top_camera"], sort_keys=True)}}


class MacroQueue:
    """A cancellable sequence of <=.1s raw commands; never steps physics."""
    def __init__(self, issue: Callable, commands: dict, *, solo_pose_parity: bool = False):
        self.issue, self.commands = issue, commands
        self.solo_pose_parity = solo_pose_parity
        self.events = []
        self.until = 0.

    def idle(self, now):
        return not self.events and now + 1e-9 >= self.until

    def cancel(self, now):
        self.events.clear()
        self.until = now

    def submit(self, action, now, *, phase=None):
        if not self.idle(now):
            raise ValueError("macro already active")
        action = copy.deepcopy(action)
        kind = action.pop("kind")
        if kind == "pose":
            target = action.pop("pulses")
            if self.solo_pose_parity and ("duration" in action or "settle" in action):
                raise ValueError("solo visual pose schedule does not accept overrides")
            settle = action.pop("settle", .3)
            requested = action.pop("duration", 0.)
            if action or not isinstance(target, dict) or not target:
                raise ValueError("invalid pose macro")
            target = {str(k): v for k, v in target.items()}
            if any(k not in self.commands or type(v) is not int or not 500 <= v <= 2500
                   for k, v in target.items()):
                raise ValueError("invalid issued pose targets")
            delta = max(abs(v-self.commands[k]) for k, v in target.items())
            duration = (max(.25, delta/600) if self.solo_pose_parity else
                        max(float(requested), delta/600, .05))
            if self.solo_pose_parity:
                settle = (.5 if phase in {"verify_lift", "attachment_left", "attachment_right", "attachment_home"}
                          else .3 if phase == "approach" else .08)
            if not finite_number(settle, minimum=0.) or not math.isfinite(duration) or duration+settle > 10:
                raise ValueError("pose duration outside finite macro budget")
            steps = max(5, math.ceil(duration/.05)) if self.solo_pose_parity else math.ceil(duration/.05)
            interval = duration/steps
            before = dict(self.commands)
            for index in range(1, steps+1):
                for channel, value in target.items():
                    u = index/steps
                    if self.solo_pose_parity:
                        u = u*u*(3-2*u)
                    pulse = round(before[channel]+(value-before[channel])*u)
                    raw = ({"kind": "look", "pan_pulse": pulse} if channel == "6" else
                           {"kind": "arm", "servo_id": int(channel), "pulse": pulse})
                    scheduled = (now+index*interval if self.solo_pose_parity else
                                 now+duration*index/steps)
                    self.events.append((scheduled, raw))
            self.until = now+duration+settle
        elif kind in {"drive", "mecanum", "wait"}:
            duration = action.pop("duration_s", action.pop("duration", .2))
            if not finite_number(duration, minimum=.001) or duration > 10:
                raise ValueError("duration outside finite macro budget")
            if kind == "drive":
                forward = action.pop("fwd") if "fwd" in action else action.pop("forward")
                raw = {"kind": kind, "forward": forward, "turn": action.pop("turn")}
            elif kind == "mecanum":
                raw = {"kind": kind, **{k: action.pop(k) for k in ("forward", "left", "turn")}}
            else:
                raw = {"kind": "wait"}
            if action:
                raise ValueError("unexpected macro fields")
            elapsed = 0.
            while elapsed < duration-1e-9:
                length = min(.1, duration-elapsed)
                self.events.append((now+elapsed, {**raw, **({"duration_s": length} if kind != "wait" else {})}))
                elapsed += length
            self.until = now+duration
        else:
            raise ValueError("unsupported macro kind")

    def tick(self, now):
        # At most one 50ms pose slice (five servos) at an owner tick. A delayed
        # caller cannot burst old motion: the adapter bounds clock increments.
        while self.events and self.events[0][0] <= now+1e-9:
            _, action = self.events.pop(0)
            self.issue(action, min(.1, action.get("duration_s", .1)))
            if action["kind"] == "arm":
                self.commands[str(action["servo_id"])] = action["pulse"]
            elif action["kind"] == "look":
                self.commands["6"] = action["pan_pulse"]


class SoloActorSkill:
    def __init__(self, robot_id, static_map, spec):
        from harness.solo_box_transport import SoloBoxTransport
        self.skill = SoloBoxTransport(robot_id=robot_id, navigator=_route(static_map, spec),
                                      attachment_min_saturation=150, release_refine_ground_fit=True,
                                      max_decisions=SOLO_MAX_DECISIONS)

    def decide(self, own, top):
        from harness.solo_box_transport import normalize_own_rgb
        skill_own, transform = normalize_own_rgb(own)
        action, evidence = self.skill.decide(skill_own, top)
        return {"action": action, "phase": self.skill.phase, "ready": False,
                "evidence": {**evidence, "own_rgb_transform": transform}}

    def advance(self):
        raise ValueError("solo phases belong to the existing own-RGB skill")

    def resume(self):
        # The existing skill updates phase when emitting a macro, so resuming a
        # cancelled grasp/open sequence could silently skip it. Fail closed.
        raise ValueError("solo mid-macro resume requires a new RGB recovery task")


def _route(static_map, spec):
    from harness.dispatch_skill_binding import ImageRoute
    # A narrow compatibility value for one actor-selected object/skill. No
    # SkillBindings, full plan, partner assignment, dependencies or completion.
    return ImageRoute(SimpleNamespace(static_map=copy.deepcopy(static_map),
        tasks={spec["object_id"]: {"route": spec["route"]}},
        plan={"dock": spec["dock"]}), spec["object_id"])


class PairCoarseProgressBudget:
    """Bound a coarse phase using this actor's accepted TOP pixel predictions.

    The original 170 decisions remain unconditional. Beyond that point, each
    24-decision extension needs a median decrease in residual image error.
    A visually ready actor may wait at the phase barrier, but only up to the
    same absolute limit. Pixel/role rejection happens before this check.
    """
    def __init__(self):
        self.errors_px = []

    def observe(self, prediction):
        if len(self.errors_px) >= PAIR_COARSE_MAX_DECISIONS:
            raise RGBSkillUnsupported("coarse_budget_exhausted", "coarse", {
                "observations": len(self.errors_px),
                "max_decisions": PAIR_COARSE_MAX_DECISIONS})
        gap_x, gap_y = prediction["image_error"]
        if not all(math.isfinite(float(value)) for value in (gap_x, gap_y)):
            raise RGBSkillUnsupported("coarse_progress_unresolved", "coarse", {
                "image_error": prediction["image_error"]})
        # The fixed TOP contract is 960x720. Count only error outside the
        # existing forward/lateral acceptance gates, in image pixels.
        error_px = max(0., (float(gap_x) - .065) * 960) + max(
            0., (abs(float(gap_y)) - .003) * 720)
        self.errors_px.append(error_px)
        count = len(self.errors_px)
        support = {"observations": count, "residual_error_px": error_px,
                   "max_decisions": PAIR_COARSE_MAX_DECISIONS}
        if count <= PAIR_COARSE_BASE_DECISIONS or prediction["ready"]:
            return support
        if (count - PAIR_COARSE_BASE_DECISIONS - 1) % PAIR_COARSE_PROGRESS_WINDOW:
            return support
        previous = median(self.errors_px[-2*PAIR_COARSE_PROGRESS_WINDOW:-PAIR_COARSE_PROGRESS_WINDOW])
        current = median(self.errors_px[-PAIR_COARSE_PROGRESS_WINDOW:])
        progress = previous - current
        support["window_progress_px"] = progress
        if progress < PAIR_COARSE_MIN_WINDOW_PROGRESS_PX:
            raise RGBSkillUnsupported("coarse_rgb_stalled", "coarse", support)
        return support


class PairActorSkill:
    """One actor's saved RGB alignment/grasp and visual transit policy.

    Only a phase barrier is joint. Predictions, memory and frames stay local.
    Slots r1/r3 mean lower/upper image conventions chosen by independent actors.
    """
    def __init__(self, robot_id, role, static_map, spec, assets):
        self.robot_id = robot_id
        self.slot = {"lower": "r1", "upper": "r3"}[role]
        skill, grasp, stages, reference = assets
        self.saved = copy.deepcopy(skill)
        self.grasp = copy.deepcopy(grasp[self.slot])
        self.stages = copy.deepcopy(stages[self.slot])
        self.reference = reference
        self.route = _route(static_map, spec)
        self.static_map, self.spec = copy.deepcopy(static_map), dict(spec)
        # Each probe is a separate pair barrier. Only its named actor moves;
        # the other waits, so TOP motion can be attributed to this actor alone.
        self.phases = ["identify_lower", "identify_upper", "coarse", "yaw", "lateral", "forward", "dock"] + [
            f"initialize_{i}" for i in range(1, len(skill["initialization_replay"]))] + [
            f"correct_{i}" for i in range(16)] + ["preclose", "close", "lift", "settle",
                "carry", "lower", "open", "retract", "verify"]
        self.index, self.count, self.confirmations = 0, 0, 0
        self.issued = False
        self.preclose = self.lifted = self.anchor = self.previous_center = None
        self.grasp_translation = None
        self.identity_tracker = None
        self.identity_claim = None
        self.probe_step = "baseline"
        self.coarse = None
        self.coarse_budget = PairCoarseProgressBudget()

    @property
    def phase(self):
        return self.phases[self.index]

    def advance(self):
        self.index += 1
        self.count, self.confirmations, self.issued = 0, 0, False
        self.coarse_budget = PairCoarseProgressBudget()

    def resume(self):
        if self.phase not in {"coarse", "yaw", "lateral", "forward", "dock", "carry", "verify"}:
            raise ValueError("interrupted manipulation requires explicit recovery task")
        self.confirmations = 0

    def decide(self, own, top):
        from harness.camera_goal_transport import dock_command, lane_features, own_payload
        from harness.camera_varied_start_student import predict_stage
        from harness.grasp_student_inference import predict_student
        from harness.dispatch_skill_binding import PairCoarsePixels, canonical_pair_top, beam_feature, pixel_from_map
        import numpy as np
        jpeg = base64.b64decode(own["image"], validate=True)
        commands = own["actuator_state"]["servo_pulses"]
        phase = self.phase
        self.count += 1
        if phase == "coarse" and self.count > PAIR_COARSE_MAX_DECISIONS:
            raise RGBSkillUnsupported("coarse_budget_exhausted", phase, {
                "observations": self.count, "max_decisions": PAIR_COARSE_MAX_DECISIONS})
        if phase != "coarse" and self.count > (900 if phase == "carry" else 170):
            raise ValueError("bounded RGB phase exhausted")
        action, ready, evidence = {"kind": "wait", "duration": .2}, False, {}
        if phase in {"identify_lower", "identify_upper"}:
            active_slot = {"identify_lower": "r1", "identify_upper": "r3"}[phase]
            if self.slot != active_slot:
                ready = True
                evidence = {"identity_probe": "peer_probe_barrier", "own_motion": False}
            elif self.probe_step == "baseline":
                from harness.camera_motion_identity import ImageMotionIdentity
                self.identity_tracker = ImageMotionIdentity()
                self.identity_tracker.update(top, None)
                self.probe_step = "driven"
                action = {"kind": "drive", "fwd": .10, "turn": 0., "duration": .6}
                evidence = {"identity_probe": "own_drive_issued", "own_motion": True}
            elif self.probe_step == "driven":
                self.probe_step = "settled"
                action = {"kind": "wait", "duration": .3}
                evidence = {"identity_probe": "own_drive_settling", "own_motion": True}
            else:
                claim = self.identity_tracker.update(top,
                    {"kind": "drive", "forward": .10, "turn": 0., "duration_s": .6})
                evidence = {"identity_probe": "own_motion_claim", "claim": claim,
                            "own_motion": True}
                if not claim["valid"] or not claim["fresh"]:
                    raise RGBSkillUnsupported("own_motion_identity_unresolved", phase, evidence)
                self.identity_claim = claim
                ready = True
        elif phase in {"coarse", "yaw", "lateral", "forward", "dock"}:
            try:
                canonical, transform = canonical_pair_top(top, self.reference)
            except ValueError as error:
                raise RGBSkillUnsupported("canonical_top_unresolved", phase, {
                    "detail": str(error), "raw_top_sha256": hashlib.sha256(top).hexdigest(),
                    "reference_sha256": hashlib.sha256(self.reference).hexdigest()}) from error
            if phase == "coarse":
                if self.identity_claim is None:
                    raise RGBSkillUnsupported("own_motion_identity_missing", phase, {"image_transform": transform})
                if self.coarse is None:
                    self.coarse = PairCoarsePixels({self.robot_id: {"claim": self.identity_claim}},
                        SimpleNamespace(pair={self.slot: self.robot_id}), self.reference)
                try:
                    prediction = self.coarse.decide(top, self.slot)
                except ValueError as error:
                    raise RGBSkillUnsupported("coarse_rgb_unresolved", phase, {
                        "detail": str(error), "raw_top_sha256": hashlib.sha256(top).hexdigest(),
                        "image_transform": transform}) from error
                if prediction["ok"]:
                    # The saved lower/upper slots are on opposite sides of
                    # the visible shaft. Bind the declared slot to this
                    # actor's own-motion anchor and complete wheel silhouette
                    # before issuing any coarse command. This comparison is
                    # translation-invariant across the fixed TOP views.
                    ref = lane_features(self.reference, self.slot)
                    if ref is None:
                        self.coarse = None
                        raise RGBSkillUnsupported("pair_role_reference_unresolved", phase, {
                            "declared_slot": self.slot, "prediction": prediction,
                            "image_transform": transform})
                    beam_y = float(transform["observed_beam"]["center"][1]) * 720
                    claim_y = float(self.identity_claim["center"][1]) * 720
                    wheel_ylo, wheel_yhi = prediction["mask"]["wheel_bounds_px"][2:]
                    expected_side = ("upper" if ref["robot_y"] < ref["beam_y"] else
                                     "lower" if ref["robot_y"] > ref["beam_y"] else "ambiguous")
                    claim_side = "upper" if claim_y < beam_y else "lower" if claim_y > beam_y else "ambiguous"
                    wheel_side = "upper" if wheel_yhi < beam_y else "lower" if wheel_ylo > beam_y else "ambiguous"
                    role_support = {"declared_slot": self.slot, "expected_side": expected_side,
                                    "own_claim_side": claim_side, "own_wheel_side": wheel_side,
                                    "own_claim_y_px": claim_y,
                                    "own_wheel_y_bounds_px": [wheel_ylo, wheel_yhi],
                                    "observed_beam_y_px": beam_y,
                                    "reference_beam_minus_robot_y_px":
                                        float(ref["beam_y"] - ref["robot_y"]) * 720}
                    if expected_side == "ambiguous" or claim_side != expected_side or wheel_side != expected_side:
                        self.coarse = None
                        raise RGBSkillUnsupported("pair_role_image_side_mismatch", phase, {
                            "role_support": role_support, "prediction": prediction,
                            "image_transform": transform})
                    prediction["role_image_support"] = role_support
                motion = {k: prediction[k] for k in ("forward", "left", "turn")}
            elif phase == "dock":
                checks = {s: predict_stage(m, jpeg, canonical) for s, m in self.stages.items()}
                if not all(d["ok"] and d.get("precision") == "fine" for d in checks.values()):
                    raise ValueError("alignment outside saved fine support")
                prediction = dock_command(checks["forward"])
                prediction["ready"] &= all(checks[s].get("stationary_ready", checks[s]["ready"])
                                            for s in ("yaw", "lateral"))
                motion = {"forward": prediction["forward"], "left": 0., "turn": 0.}
            else:
                prediction = predict_stage(self.stages[phase], jpeg, canonical)
                motion = {"forward": 0., "left": 0., "turn": 0.}
                if not prediction["ready"]:
                    motion[{"yaw": "turn", "lateral": "left", "forward": "forward"}[phase]] = prediction["command"]
            if not prediction["ok"]:
                raise RGBSkillUnsupported(prediction.get("reason", "RGB outside saved approach support"),
                                          phase, {"prediction": prediction, "image_transform": transform})
            if phase == "coarse":
                prediction["coarse_budget"] = self.coarse_budget.observe(prediction)
            observed_ready = prediction.get("stationary_ready", prediction["ready"])
            self.confirmations = self.confirmations+1 if observed_ready else 0
            ready = self.confirmations >= 2
            if not observed_ready:
                action = {"kind": "mecanum", **motion, "duration_s": .2}
            evidence = {"prediction": prediction, "image_transform": transform}
        elif phase.startswith("initialize_"):
            if self.grasp_translation is None:
                _, transform = canonical_pair_top(top, self.reference)
                self.grasp_translation = transform["translation_px"]
            record = self.saved["initialization_replay"][int(phase.split("_")[1])]
            action, ready = self._once({"kind": "pose", "pulses": {int(k): int(v) for k, v in record["targets"][self.slot].items()},
                "duration": record["duration_s"], "settle": record.get("settle_s", .1)})
        elif phase.startswith("correct_") or phase == "preclose":
            canonical, transform = canonical_pair_top(top, self.reference, translation_px=self.grasp_translation)
            prediction = predict_student(self.grasp, jpeg, canonical, max_step=25)
            if prediction.get("observable") is not True or prediction.get("confidence", 0.) < .8:
                raise ValueError("RGB outside saved grasp support")
            evidence = {"prediction": prediction, "image_transform": transform}
            if phase == "preclose":
                self.preclose = {str(k): commands[str(k)] for k in (3, 4, 5)}
                ready = True
            else:
                targets = {int(k): max(500, min(2500, commands[str(k)]+int(delta)))
                           for k, delta in zip(self.grasp["channels"], prediction["delta_pulses"])}
                action, ready = self._once({"kind": "pose", "pulses": targets, "duration": .35, "settle": .1})
        elif phase == "close":
            action, ready = self._once({"kind": "pose", "pulses": {1: int(self.saved["close_pulses"][self.slot])},
                "duration": self.saved["close_duration_s"], "settle": self.saved["close_settle_s"]})
        elif phase == "lift":
            if self.lifted is None:
                self.lifted = {str(k): max(500, min(2500, commands[str(k)]+int(v)))
                               for k, v in self.saved["lift_delta_pulses"][self.slot].items()}
            action, ready = self._once({"kind": "pose", "pulses": self.lifted,
                "duration": self.saved["lift_duration_s"], "settle": self.saved["lift_settle_s"]})
        elif phase == "settle":
            action, ready = self._once({"kind": "wait", "duration": float(self.saved["hold_s"])})
            if ready:
                self.anchor = own_payload(jpeg, hue_upper=35)
                if self.anchor is None:
                    raise ValueError("own RGB attachment anchor unresolved")
        elif phase == "carry":
            current = own_payload(jpeg, hue_upper=35)
            if current is None or self.anchor is None or not (.25 <= current[0]/self.anchor[0] <= 4
                    and math.dist(current[1:], self.anchor[1:]) <= .15):
                raise ValueError("own RGB attachment continuity lost")
            action, evidence = self.route.observe(top)
            ready = evidence["done"]
        elif phase in {"lower", "open", "retract"}:
            target = self.preclose if phase == "lower" else {"1": 2000} if phase == "open" else self.lifted
            action, ready = self._once({"kind": "pose", "pulses": target, "duration": .7, "settle": .5})
        elif phase == "verify":
            beam = beam_feature(top, hue_upper=35)
            w, h = beam["image_size"]
            corners = np.array(beam["corners4"])*[w, h]
            slot = self.static_map["docks"][self.spec["dock"]]["slots"]["beam"]
            a = pixel_from_map(np.array(slot["center_m"])-slot["half_extents_m"], self.static_map, (h, w))
            b = pixel_from_map(np.array(slot["center_m"])+slot["half_extents_m"], self.static_map, (h, w))
            inside = bool(np.all(corners >= np.minimum(a, b)) and np.all(corners <= np.maximum(a, b)))
            stable = self.previous_center is not None and math.dist(self.previous_center, beam["center"]) < .003
            self.previous_center = beam["center"]
            self.confirmations = self.confirmations+1 if inside and stable else 0
            if self.confirmations >= 2:
                action = {"kind": "finish", "reason": "RGB_PLACEMENT_CLAIM_UNVERIFIED"}
            evidence = {"inside_visible_slot": inside, "image_stable": stable}
        return {"phase": phase, "action": action, "ready": ready, "evidence": evidence}

    def _once(self, action):
        if self.issued:
            return {"kind": "wait", "duration": .2}, True
        self.issued = True
        return action, False


@dataclass
class _WorkerTiming:
    _started: float | None = None
    _completed: float | None = None
    lock: object = field(default_factory=threading.Lock)

    def start(self, now):
        with self.lock:
            if self._started is not None or not finite_number(now, minimum=0.):
                raise ValueError("invalid worker start timestamp")
            self._started = now

    def complete(self, now):
        with self.lock:
            if (self._completed is not None or self._started is None
                    or not finite_number(now, minimum=0.) or now < self._started):
                raise ValueError("invalid worker completion timestamp")
            self._completed = now

    def snapshot(self, submitted):
        with self.lock:
            return {"worker_started_wall_s": self._started, "worker_completed_wall_s": self._completed,
                "worker_wall_duration_s": (None if self._completed is None else self._completed-self._started),
                "worker_queue_wall_s": None if self._started is None else self._started-submitted}


def _write_supervisor(record, lock, row):
    if record is not None:
        with lock:
            record({"schema": "ugrp.rgb_worker_timing.v1", **row})


class RGBSkillUnsupported(ValueError):
    """A stopped RGB decision with evidence for the separate raw audit log."""
    def __init__(self, reason, phase, diagnostics):
        super().__init__(f"{phase}: {reason}")
        self.reason, self.phase, self.diagnostics = reason, phase, diagnostics


def _timed_rgb_decision(controller, own, top, timing, submitted, metadata, record, lock):
    """Only the controller's image inputs and a raw-log sink, no port handle."""
    timing.start(time.monotonic())
    outcome = "error"
    try:
        decision = controller.decide(own, top)
        outcome = "returned"
        return decision
    except RGBSkillUnsupported as error:
        _write_supervisor(record, lock, {"event": "RGB_DECISION_REJECTED", **metadata,
            "phase": error.phase, "reason": error.reason, "diagnostics": error.diagnostics})
        raise
    finally:
        timing.complete(time.monotonic())
        _write_supervisor(record, lock, {"event": "RGB_WORKER_COMPLETED", **metadata,
            "worker_outcome": outcome, "submitted_wall_s": submitted, **timing.snapshot(submitted)})


@dataclass
class _Runner:
    controller: object
    macro: MacroQueue
    future: object = None
    started_wall: float = 0.
    observation: dict = field(default_factory=dict)
    phase: str = "START"
    sequence: int = 0
    was_paused: bool = False
    timing: _WorkerTiming | None = None


@dataclass(frozen=True)
class _CapturedRGB:
    data: bytes
    capture_id: str
    sim_time_s: float
    absolute_time_s: float
    started_wall_s: float
    completed_wall_s: float
    sha256: str

    def metadata(self):
        return {key: value for key, value in self.__dict__.items() if key != "data"}


class _RGBFrameCache:
    """Owner-local immutable bytes, NOT cached observations/commands/status.

    One cache per backend episode. A frame-producing state change at the same
    clock requires invalidate(); the factory does so for every apply/hold and
    physics advance. No reset/viewer/camera mutation API is exposed by B.
    """
    def __init__(self, own, top, *, record):
        self.own, self.top, self.record = own, top, record
        self._frames, self._key = {}, None
        self._epoch = self._sequence = 0
        self._last_absolute = None
        self._identity = None

    def invalidate(self):
        self._epoch += 1
        self._frames.clear()
        self._key = None

    def read(self, rid, *, sim_time_s, absolute_time_s, episode, camera_identity, observation_id):
        if (not finite_number(absolute_time_s, minimum=0.)
                or not finite_number(sim_time_s, minimum=0.)):
            self.invalidate()
            raise ValueError("invalid RGB capture clock")
        identity = (episode, tuple(camera_identity))
        if self._identity != identity:
            if self._identity is None or self._identity[0] != episode:
                self._last_absolute = None
            self.invalidate()
            self._identity = identity
        if self._last_absolute is not None and absolute_time_s < self._last_absolute:
            self.invalidate()
            raise ValueError("RGB capture clock moved backwards")
        self._last_absolute = absolute_time_s
        key = (identity, self._epoch, sim_time_s, absolute_time_s)
        if self._key != key:
            self._frames.clear()
            self._key = key
        records, hits = {}, {}
        for name, owner in (("top_rgb", None), ("own_rgb", rid)):
            slot = (name, owner)
            hits[name] = slot in self._frames
            if slot not in self._frames:
                start = time.monotonic()
                data = self.top() if owner is None else self.own(owner)
                ended = time.monotonic()
                if not isinstance(data, bytes) or not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
                    self.invalidate()
                    raise ValueError("original JPEG bytes required")
                self._sequence += 1
                self._frames[slot] = _CapturedRGB(data, f"rgb-{self._sequence:08d}",
                    sim_time_s, absolute_time_s, start, ended, hashlib.sha256(data).hexdigest())
            records[name] = self._frames[slot]
        self.record({"event": "RGB_FRAME_READ", "robot_id": rid, "observation_id": observation_id,
            "timestamp_s": sim_time_s, "read_wall_s": time.monotonic(), "generation": self._epoch,
            "camera_identity": list(camera_identity), "cache_hit": hits,
            "captures": {name: item.metadata() for name, item in records.items()}})
        return {name: item.data for name, item in records.items()}


class _SimulationClockError(ValueError):
    def __init__(self, code):
        super().__init__(code)
        self.error_code = code


class _SimulationClock:
    """Supervisor-only clock bridge; never reads evaluation or robot state.

    Endpoints use the accumulated absolute clock exclusively. The requested
    relative clock is checked, not forwarded. For n physical additions, at
    most (n+2) ULPs at the largest operand bound accumulation/subtraction/
    addition rounding. This bound ONLY compares the two representations:
    actual clock reversals, even one ULP, are always errors.
    """
    def __init__(self, read_absolute, *, origin, timestep, max_sim_s):
        if (not finite_number(origin, minimum=0.)
                or not finite_number(timestep, minimum=0.) or timestep == 0
                or not finite_number(max_sim_s, minimum=0.) or max_sim_s <= 0):
            raise _SimulationClockError("SIM_CLOCK_INVALID")
        self._read_absolute = read_absolute
        self.origin, self.timestep, self.max_sim_s = origin, timestep, max_sim_s
        self._last_absolute = origin
        self._invalid = self._frozen = False

    def roundoff_bound(self, relative):
        return (math.ceil(relative/self.timestep)+2)*math.ulp(self.origin+relative)

    def absolute(self):
        if self._invalid:
            raise _SimulationClockError("SIM_CLOCK_INVALID")
        if self._frozen:
            return self._last_absolute
        try:
            value = self._read_absolute()
            if not finite_number(value, minimum=0.):
                raise _SimulationClockError("SIM_CLOCK_INVALID")
            if value < self._last_absolute:
                raise _SimulationClockError("SIM_CLOCK_REVERSED")
            if value-self.origin > self.max_sim_s+self.roundoff_bound(self.max_sim_s):
                raise _SimulationClockError("SIM_CLOCK_MISMATCH")
        except Exception:
            self._invalid = True
            raise
        self._last_absolute = value
        return value

    def target(self, relative):
        if not finite_number(relative, minimum=0.) or relative > self.max_sim_s:
            raise _SimulationClockError("SIM_CLOCK_INVALID")
        actual = self.absolute()
        elapsed = actual-self.origin
        if abs(elapsed-relative) > self.roundoff_bound(max(relative, elapsed)):
            raise _SimulationClockError("SIM_CLOCK_MISMATCH")
        return actual

    def relative_or_none(self):
        try:
            return self.absolute()-self.origin
        except Exception:
            return None

    def freeze(self):
        self.relative_or_none()
        self._frozen = True


class RGBSkillExecutionPort(RGBExecutionPort):
    """Single scheduler thread; independent image workers have no motion handles."""
    def __init__(self, endpoints, *, frame_source, static_context, skill_factory,
                 advance_physics, command_states, max_sim_s=180., max_commands=10000,
                 record_observation=None, evaluation_source=None, simulation_clock=None,
                 record_supervisor=None):
        capabilities = [SkillCapability(name, frozenset({"RUN"}), frozenset({spec["team_size"]}),
            frozenset({"drive", "mecanum", "arm", "look", "wait"}), .1,
            distinct_roles=spec["team_size"] == 2) for name, spec in SKILLS.items()]
        super().__init__(endpoints, frame_source=frame_source, static_context=static_context,
                         capabilities=capabilities, clock_domain="sim", strict_revisions=True,
                         evaluation_source=evaluation_source)
        self._skill_factory, self._advance_physics = skill_factory, advance_physics
        self._command_states = copy.deepcopy(command_states)
        self._max_sim_s, self._max_commands = max_sim_s, max_commands
        self._record_observation = record_observation
        self._simulation_clock = simulation_clock
        self._last_acknowledged_time_s = None
        self._record_supervisor = record_supervisor
        self._supervisor_lock = threading.Lock()
        self._pool = ThreadPoolExecutor(max_workers=len(endpoints), thread_name_prefix="rgb-skill")
        self._runners = {}
        self._all_futures = []
        self._raw_count = 0
        self._task_records = {}
        self._last_skill_status = {rid: {} for rid in endpoints}
        self._probe_isolation_task_id = None
        self._probe_quiet_since = None

    def _submit_task(self, rid, payload):
        spec = SKILLS.get(payload.get("skill"))
        if (spec is None or payload.get("object_id") != spec["object_id"]
                or payload.get("own_role") not in spec["roles"]
                or not isinstance(payload.get("resources"), list)
                or not set(spec["resources"]).issubset(payload["resources"])):
            raise ValueError("unsupported object, role or explicit skill resources")
        if payload.get("expires_at_s", 0) > self._max_sim_s:
            raise ValueError("task exceeds SIM budget")
        result = super()._submit_task(rid, payload)
        if payload["task_id"] in self._active:
            lease = self._active[payload["task_id"]]
            self._task_records[lease.lease_id] = {"task_id": lease.task_id, "object": lease.object_id,
                "participants": list(lease.participants), "dock": spec["dock"], "skill": lease.skill}
        return result

    def _submit_command(self, rid, payload):
        raise ValueError("automatic RGB skill owns motion; interrupt before another task")

    def _hold_participants(self, participants, reason):
        participants = tuple(participants)
        revoked = []
        for rid in participants:
            runner = self._runners.get(rid) if hasattr(self, "_runners") else None
            if runner is not None:
                runner.macro.cancel(self._now_s)
                if runner.future is not None:
                    runner.future.cancel()
                    # A cancelled worker may still compute, but has no endpoint.
                    # Never reuse its mutated controller or accept its result.
                    runner.was_paused = True
                    revoked.append({"event": "RGB_WORKER_REVOKED", "robot_id": rid,
                        "observation_id": runner.observation["observation_id"], "timestamp_s": self._now_s,
                        "revoked_wall_s": time.monotonic(), "reason": reason,
                        "future_cancelled": runner.future.cancelled(), "future_done": runner.future.done(),
                        "submitted_wall_s": runner.started_wall,
                        **(runner.timing.snapshot(runner.started_wall) if runner.timing else {})})
                if reason == "task_paused":
                    runner.was_paused = True
        errors = super()._hold_participants(participants, reason)
        for rid in participants:
            getter = getattr(self._endpoints[rid], "issued_servo_commands", None)
            if getter is not None:
                self._command_states[rid].update(getter())
        for row in revoked:
            self._supervisor_event(row)
        return errors

    def _release_lease(self, lease, reason):
        for rid in lease.participants:
            runner = self._runners.pop(rid, None)
            if runner:
                runner.macro.cancel(self._now_s)
                if runner.future:
                    runner.future.cancel()
        super()._release_lease(lease, reason)

    @_serialized
    def local_status(self, rid):
        value = super().local_status(rid)
        value["own_skill_status"] = copy.deepcopy(self._last_skill_status[rid])
        return value

    def _issue(self, lease, rid, action, duration):
        if lease.task_id not in self._active or lease.paused or self._now_s >= lease.expires_at_s:
            raise ValueError("revoked skill command")
        if self._raw_count >= self._max_commands:
            raise ValueError("raw command cap reached")
        duration = min(duration, lease.expires_at_s-self._now_s)
        if action["kind"] in {"drive", "mecanum"}:
            action = {**action, "duration_s": duration}
        endpoint = self._endpoints[rid]
        endpoint.validate_bounded(action, duration)
        endpoint.apply_bounded(action, self._now_s, duration)
        self._raw_count += 1
        runner = self._runners[rid]
        consent = lease.consent_evidence[rid]
        row = {"task_id": lease.task_id, "lease_id": lease.lease_id,
            "command_id": f"{lease.lease_id}:{rid}:{self._raw_count}",
            "stage": runner.phase, "action": copy.deepcopy(action), "duration_s": duration,
            "issued_at_s": self._now_s, "observation_id": runner.observation["observation_id"],
            "decision_id": consent["decision_id"], "skill_decision_id": f"{lease.lease_id}:{rid}:{runner.sequence}",
            "consent_observation_id": consent["observation_id"],
            "meaning": "issued command, not measured state or success"}
        self._own_commands[rid].append(row)
        self._record(rid, "LOCAL_COMMAND", **row)
        # Do not invalidate high-level consent on every interpolated servo pulse.
        # Runtime separately fingerprints the actual own command history.

    def _new_runner(self, lease, rid):
        # Only own role and actor-selected immutable capability enter the worker.
        controller = self._skill_factory(rid, lease.roles[rid], lease.skill)
        return _Runner(controller, MacroQueue(lambda a, d: self._issue(lease, rid, a, d),
                                               self._command_states[rid],
                                               solo_pose_parity=lease.skill.startswith("solo_")))

    def _fail(self, lease, error):
        for rid in lease.participants:
            self._last_skill_status[rid] = {"task_id": lease.task_id, "state": "STOPPED",
                "reason": "rgb_skill_unavailable", "meaning": "no physical success claim"}
            self._record(rid, "SKILL_FAILED", task_id=lease.task_id,
                         error_type=type(error).__name__, detail=str(error))
        self._hold_participants(lease.participants, "rgb_skill_unavailable")
        self._release_lease(lease, "rgb_skill_unavailable")

    def _supervisor_event(self, row):
        # Separate raw stream: no timing telemetry enters own_events/status or
        # controller input. Worker completion can be logged after revocation.
        _write_supervisor(self._record_supervisor, self._supervisor_lock, row)

    def _worker_poll_rows(self, lease, runners, polled, guard, event):
        rows = []
        for rid, runner in zip(lease.participants, runners):
            if runner.future is None:
                continue
            age = self._now_s-runner.observation["observed_at_s"]
            wall = polled-runner.started_wall
            timing = runner.timing.snapshot(runner.started_wall) if runner.timing else {}
            completed = timing.get("worker_completed_wall_s")
            rows.append({"event": event, "robot_id": rid,
                "task_id": lease.task_id, "observation_id": runner.observation["observation_id"],
                "timestamp_s": self._now_s, "observed_at_s": runner.observation["observed_at_s"],
                "image_age_s": age, "submitted_wall_s": runner.started_wall, "polled_wall_s": polled,
                "submission_to_poll_wall_s": wall, "future_done": runner.future.done(),
                "guard": guard, "sim_age_exceeded": age > 1., "wall_cap_exceeded": wall > 2.,
                "sim_age_cap_s": 1., "wall_cap_s": 2.,
                "submission_to_completion_wall_s": None if completed is None else completed-runner.started_wall,
                "completion_to_poll_wall_s": (None if completed is None or completed > polled else polled-completed),
                "wall_scope": "submission_to_consumption", **timing})
        return rows

    def _record_worker_rejection(self, lease, runners, polled, guard):
        for row in self._worker_poll_rows(lease, runners, polled, guard, "RGB_WORKER_REJECTED"):
            self._supervisor_event(row)

    def _identity_probe_lease(self, leases):
        """Reserve TOP motion for one pair's own-command identity probes."""
        for lease in leases:
            if lease.paused or len(lease.participants) != 2 or lease.skill not in SKILLS:
                continue
            if any(rid in self._runners and isinstance(self._runners[rid].controller, PairActorSkill)
                   and self._runners[rid].controller.phase.startswith("identify_")
                   for rid in lease.participants):
                return lease
        return None

    def _service(self):
        # Poll existing work before blocking on new RGB capture/input I/O.
        # This does NOT accept any result beyond the original total wall cap.
        rank = {rid: index for index, rid in enumerate(self.robot_ids)}
        leases = sorted(list(self._active.values()), key=lambda item: min(rank[r] for r in item.participants))
        probe_lease = self._identity_probe_lease(leases)
        probe_id = None if probe_lease is None else probe_lease.task_id
        if probe_id != self._probe_isolation_task_id:
            if self._probe_isolation_task_id is not None:
                self._supervisor_event({"event": "RGB_PROBE_ISOLATION_ENDED",
                    "task_id": self._probe_isolation_task_id, "timestamp_s": self._now_s})
            self._probe_isolation_task_id, self._probe_quiet_since = probe_id, None
            if probe_id is not None:
                self._supervisor_event({"event": "RGB_PROBE_ISOLATION_STARTED",
                    "task_id": probe_id, "timestamp_s": self._now_s,
                    "scope": "other leases finish issued motion, then hold new decisions"})
        new_inputs = []
        for lease in leases:
            if lease.paused:
                continue
            if probe_lease is not None and lease is not probe_lease:
                # Let already issued actions and in-flight decisions finish,
                # then hold this lease at an idle command boundary. It cannot
                # produce unrelated TOP motion during either isolated probe.
                if all(rid in self._runners for rid in lease.participants):
                    runners = [self._runners[rid] for rid in lease.participants]
                    if all(r.future is None and r.macro.idle(self._now_s) for r in runners):
                        continue
                else:
                    continue
            if lease is probe_lease:
                foreign_leases = [other for other in leases if other is not lease]
                foreign = [self._runners[rid] for other in foreign_leases if not other.paused
                           for rid in other.participants if rid in self._runners]
                if any(r.future is not None or not r.macro.idle(self._now_s) for r in foreign):
                    self._probe_quiet_since = None
                    continue
                if foreign_leases:
                    if self._probe_quiet_since is None:
                        self._probe_quiet_since = self._now_s
                    if self._now_s-self._probe_quiet_since < .3-1e-9:
                        continue
            if any(rid not in self._runners for rid in lease.participants):
                new_inputs.extend((lease, rid, self._runners.get(rid)) for rid in lease.participants)
                continue
            try:
                for rid in lease.participants:
                    runner = self._runners[rid]
                    if runner.was_paused:
                        if runner.future is not None:
                            raise ValueError("interrupted image decision requires new task")
                        runner.controller.resume()
                        runner.was_paused = False
                    runner.macro.tick(self._now_s)
                runners = [self._runners[rid] for rid in lease.participants]
                if not all(r.macro.idle(self._now_s) for r in runners):
                    continue
                if any(r.future is not None and not r.future.done() for r in runners):
                    polled = time.monotonic()
                    if any(polled-r.started_wall > 2. for r in runners if r.future is not None):
                        self._record_worker_rejection(lease, runners, polled, "in_flight_wall")
                        raise TimeoutError("RGB decision exceeded 2-second wall cap")
                    continue
                if all(r.future is not None for r in runners):
                    polled = time.monotonic()
                    if any(self._now_s-r.observation["observed_at_s"] > 1.
                           or polled-r.started_wall > 2. for r in runners):
                        self._record_worker_rejection(lease, runners, polled, "completed_staleness")
                        raise ValueError("stale RGB worker result")
                    if any(r.observation["own_revision"] != self._own_revision[rid]
                           for rid, r in zip(lease.participants, runners)):
                        raise ValueError("revoked RGB consent")
                    if any(r.timing is None or r.timing.snapshot(r.started_wall)["worker_completed_wall_s"] is None
                           for r in runners):
                        raise ValueError("worker completion timing unavailable")
                    consumed = self._worker_poll_rows(lease, runners, polled,
                        "completed_fresh", "RGB_WORKER_CONSUMED")
                    decisions = [r.future.result() for r in runners]
                    for runner in runners:
                        runner.future = None
                    if len(runners) == 2 and len({d["phase"] for d in decisions}) != 1:
                        raise ValueError("independent skill phase mismatch")
                    finished = [d["action"].get("kind") == "finish" for d in decisions]
                    if any(finished):
                        if not all(finished):
                            raise ValueError("independent RGB completion disagreement")
                        for rid, runner, decision in zip(lease.participants, runners, decisions):
                            reason = decision["action"]["reason"]
                            self._last_skill_status[rid] = {"task_id": lease.task_id, "state": "FINISHED_UNVERIFIED",
                                "reason": reason, "meaning": "RGB skill claim; evaluator result is separate"}
                            self._record(rid, "SKILL_FINISH_CLAIM", task_id=lease.task_id,
                                observation_id=runner.observation["observation_id"],
                                decision_id=lease.consent_evidence[rid]["decision_id"], reason=reason)
                        self._hold_participants(lease.participants, "rgb_finish_claim")
                        self._release_lease(lease, "rgb_finish_claim")
                        for row in consumed:
                            self._supervisor_event(row)
                        continue
                    if len(runners) == 2 and all(d["ready"] for d in decisions):
                        for runner in runners:
                            runner.controller.advance()
                    for rid, runner, decision in zip(lease.participants, runners, decisions):
                        runner.phase = decision["phase"]
                        runner.sequence += 1
                        self._last_skill_status[rid] = {"task_id": lease.task_id, "state": "RUNNING",
                            "phase": runner.phase, "meaning": "own RGB controller phase, not measured success"}
                        self._record(rid, "SKILL_DECISION", task_id=lease.task_id,
                            observation_id=runner.observation["observation_id"],
                            decision_id=lease.consent_evidence[rid]["decision_id"],
                            skill_decision_id=f"{lease.lease_id}:{rid}:{runner.sequence}", decision=decision)
                        runner.macro.submit(decision["action"], self._now_s, phase=runner.phase)
                        runner.macro.tick(self._now_s)
                    for row in consumed:
                        self._supervisor_event(row)
                    continue
                if probe_lease is not None and lease is not probe_lease:
                    continue
                new_inputs.extend((lease, rid, runner) for rid, runner in zip(lease.participants, runners))
            except Exception as error:
                if lease.task_id in self._active:
                    self._fail(lease, error)

        # Fixed owner robot order in every communication condition. Capture and
        # persist every new input BEFORE starting any new submission deadline;
        # no worker/completion/global-planner barrier is introduced.
        prepared = []
        for lease, rid, runner in sorted(new_inputs, key=lambda item: rank[item[1]]):
            if self._active.get(lease.task_id) is not lease or lease.paused:
                continue
            try:
                preparation_started = time.monotonic()
                if runner is None:
                    # Do not pile work behind an uninterruptible revoked worker.
                    if sum(not f.done() for f in self._all_futures) >= len(self.robot_ids):
                        raise ValueError("image worker capacity unavailable")
                    runner = self._runners[rid] = self._new_runner(lease, rid)
                controller_ready = time.monotonic()
                observation = self.observe(rid)
                captured = time.monotonic()
                runner.observation = observation
                if self._record_observation:
                    self._record_observation(observation)
                saved = time.monotonic()
                own = {"robot_id": rid, "frame_id": self._observation_sequence[rid],
                    "sim_time": self._now_s, "camera": "robot_cam",
                    "image": observation["images"]["own_rgb"]["jpeg_base64"],
                    "sha256": observation["images"]["own_rgb"]["sha256"],
                    "actuator_state": {"motor_commands": [], "servo_pulses": copy.deepcopy(self._command_states[rid])}}
                top = base64.b64decode(observation["images"]["top_rgb"]["jpeg_base64"], validate=True)
                self._supervisor_event({"event": "RGB_INPUT_PREPARED", "robot_id": rid,
                    "observation_id": observation["observation_id"], "timestamp_s": self._now_s,
                    "controller_setup_wall_s": controller_ready-preparation_started,
                    "capture_and_pack_wall_s": captured-controller_ready,
                    "input_persistence_wall_s": saved-captured,
                    "request_pack_wall_s": time.monotonic()-saved})
                prepared.append((lease, rid, runner, own, top))
            except Exception as error:
                if lease.task_id in self._active:
                    self._fail(lease, error)

        # A pair actor may have been constructed while preparing this tick.
        # Apply isolation before any worker is submitted, including a solo
        # observation that was staged earlier in the fixed robot order.
        probe_lease = self._identity_probe_lease(leases)
        newly_discovered_probe = (probe_lease is not None
                                  and self._probe_isolation_task_id != probe_lease.task_id)
        for lease, rid, runner, own, top in prepared:
            if (self._active.get(lease.task_id) is not lease or lease.paused
                    or self._runners.get(rid) is not runner):
                continue
            if probe_lease is not None:
                if lease is not probe_lease:
                    continue
                # Start the quiet interval in the next service pass. A solo
                # macro may have become idle just before this construction.
                if newly_discovered_probe:
                    continue
                foreign = [self._runners[other_rid] for other in leases
                           if other is not lease and not other.paused
                           for other_rid in other.participants if other_rid in self._runners]
                if any(r.future is not None or not r.macro.idle(self._now_s) for r in foreign):
                    continue
            try:
                self._all_futures = [f for f in self._all_futures if not f.done()]
                if len(self._all_futures) >= len(self.robot_ids):
                    raise ValueError("image worker capacity unavailable")
                if (self._now_s >= lease.expires_at_s or
                        runner.observation["own_revision"] != self._own_revision[rid]):
                    raise ValueError("revoked RGB consent")
                runner.started_wall = time.monotonic()
                runner.timing = _WorkerTiming()
                runner.future = self._pool.submit(_timed_rgb_decision, runner.controller, own, top,
                    runner.timing, runner.started_wall, {"robot_id": rid, "task_id": lease.task_id,
                        "observation_id": runner.observation["observation_id"], "observed_at_s": self._now_s},
                    self._record_supervisor, self._supervisor_lock)
                self._all_futures.append(runner.future)
            except Exception as error:
                if lease.task_id in self._active:
                    self._fail(lease, error)

    @_serialized
    def tick(self, now_s):
        if self._closed:
            raise RuntimeError("execution port is closed")
        if (not finite_number(now_s, minimum=0.) or now_s < self._now_s
                or now_s-self._now_s > MAX_TICK_S+1e-8 or now_s > self._max_sim_s+1e-8):
            raise ValueError("single-owner tick must advance 0..0.05s within the SIM cap")
        delta = now_s-self._now_s
        # Expiries/watchdogs execute at every physical substep in the owner.
        try:
            if delta:
                self._advance_physics(delta)
            if self._simulation_clock is not None:
                self._simulation_clock.target(now_s)
        except Exception as error:
            failure = RuntimeError("physics owner unavailable")
            failure.error_code = "PHYSICS_OWNER_UNAVAILABLE"
            try:
                self.close(None)
            except Exception as close_error:
                # Preserve the original failure chain even if safety cleanup
                # itself fails; never turn that into a successful tick.
                failure.close_error_type = type(close_error).__name__
            raise failure from error
        result = super().tick(now_s)
        self._service()
        if self._simulation_clock is not None:
            self._last_acknowledged_time_s = self._simulation_clock.relative_or_none()
        return result

    @_serialized
    def clock_snapshot(self):
        """Clock-only supervisor capability, deliberately absent from actor facade."""
        return {"schema": "ugrp.execution_clock.v1", "clock_domain": "sim",
            "last_acknowledged_time_s": self._last_acknowledged_time_s,
            "actual_time_s": (self._simulation_clock.relative_or_none()
                              if self._simulation_clock is not None else None)}

    @_serialized
    def close(self, now_s):
        if not self._closed:
            if self._simulation_clock is not None:
                if now_s is not None:
                    # C passes the raw actual elapsed value, which can be a
                    # few accumulated ULPs above max_sim_s. This is a read-only
                    # equality check, NOT a new requested tick or cap waiver.
                    if not finite_number(now_s, minimum=0.):
                        raise _SimulationClockError("SIM_CLOCK_INVALID")
                    actual = self._simulation_clock.absolute()-self._simulation_clock.origin
                    if abs(now_s-actual) > self._simulation_clock.roundoff_bound(actual):
                        raise _SimulationClockError("SIM_CLOCK_MISMATCH")
                # Closing never acknowledges or advances an attempted tick.
                now_s = self._now_s
            elif now_s is None:
                now_s = self._now_s
            held = {rid for lease in self._active.values() for rid in lease.participants}
            held.update(rid for group in self._pending.values() for rid in group)
            try:
                super().close(now_s)
            finally:
                try:
                    self._hold_participants(set(self.robot_ids)-held, "execution_port_closed")
                finally:
                    self._pool.shutdown(wait=False, cancel_futures=True)


class _RelativeEndpoint:
    def __init__(self, endpoint, origin, *, clock=None, invalidate_frames=None):
        self.endpoint, self.origin = endpoint, origin
        self.robot_id = endpoint.robot_id
        self.clock = clock
        self.invalidate_frames = invalidate_frames
        self._last_requested = 0.

    def _requested(self, now_s):
        if not finite_number(now_s, minimum=0.):
            raise _SimulationClockError("SIM_CLOCK_INVALID")
        if now_s < self._last_requested:
            raise _SimulationClockError("SIM_CLOCK_REVERSED")

    def _absolute(self, now_s):
        self._requested(now_s)
        absolute = self.clock.target(now_s) if self.clock is not None else self.origin+now_s
        self._last_requested = now_s
        return absolute

    def validate_bounded(self, action, duration_s):
        self.endpoint.validate_bounded(action, duration_s)

    def apply_bounded(self, action, now_s, duration_s):
        if self.invalidate_frames is not None:
            self.invalidate_frames()
        self.endpoint.apply_bounded(action, self._absolute(now_s), duration_s)

    def hold(self, now_s):
        if self.invalidate_frames is not None:
            self.invalidate_frames()
        self._requested(now_s)
        if self.clock is None:
            absolute = self.origin+now_s
        else:
            try:
                absolute = self.clock.absolute()
            except Exception:
                # Unknown physical time: cancel at the last issued-command
                # clock WITHOUT interpolation or claiming this is actual time.
                absolute = self.endpoint._servo_tick_time
        self._last_requested = now_s
        self.endpoint.hold(absolute)

    def tick(self, now_s):
        self.endpoint.tick(self._absolute(now_s))

    def issued_servo_commands(self):
        return self.endpoint._actuator_state()["servo_pulses"]


class _ActorFacade:
    """Public method surface deliberately excludes evaluator and world."""
    def __init__(self, port):
        self.__port = port

    @property
    def clock_domain(self):
        return self.__port.clock_domain

    def observe(self, rid):
        return self.__port.observe(rid)

    def local_status(self, rid):
        return self.__port.local_status(rid)

    def submit(self, rid, payload):
        return self.__port.submit(rid, payload)

    def tick(self, now_s):
        return self.__port.tick(now_s)

    def close(self, now_s):
        return self.__port.close(now_s)


@dataclass
class RGBBackendBundle:
    actor_port: object
    evaluation_snapshot: Callable
    capabilities: dict
    provenance: dict
    close: Callable
    clock_snapshot: Callable | None = None


def build_rgb_skill_backend(config):
    """D-only real execution entry point. Never run from a metadata probe."""
    descriptor = backend_descriptor(config)
    skill, grasp, _ = _read_models(config["grasp_model_dir"], "student-skill.json")
    _, stages, _ = _read_models(config["stage_model_dir"], "varied-start-skill.json", staged=True)
    reference = Path(config["reference_top"]).read_bytes()
    from sim.research_dispatch_arena import episode
    from scripts.research_dispatch_scene import DispatchScene
    from scripts.run_dispatch_e2e import Referee
    from scripts.probe_dual_grasp_sync import Video
    execution_bundle, bundle_sha = load_bundle(config["execution_bundle_id"])
    scene_config = episode("open", config["seed"])
    scene_config["contact_solver_profile"] = ADAPTER_CONTACT_PROFILE
    scene = DispatchScene(scene_config, config["output_dir"])
    video = referee = port = None
    try:
        scene.open()
        applied_contract = _applied_contract(scene)
        require_effective(execution_bundle, applied_contract)
        if scene.invariants()["weld_active"]:
            raise ValueError("weld must remain OFF")
        origin = float(scene.world.data.time)
        clock = _SimulationClock(lambda: scene.world.data.time, origin=origin,
            timestep=float(scene.world.model.opt.timestep), max_sim_s=config["max_sim_s"])
        video = Video(scene.world, scene.out / "execution.mp4", 4)
        referee = Referee(scene)
        referee.sample()
        video.capture(force=True)
        static_map = scene.config["static_map"]
        static_context = _public_static_context(static_map, descriptor)
        frame_episode = object()  # never shared across reset/new backend instances
        frame_cache = _RGBFrameCache(
            lambda rid: scene.world.render_jpeg(robot_id=rid, camera="robot_cam", quality=FRAME_JPEG_QUALITY),
            lambda: scene.world.render_team_jpeg(camera="cctv_top", quality=FRAME_JPEG_QUALITY),
            record=lambda row: port._supervisor_event(row))
        renderer_state = {"identity": None, "generation": 0}
        def frames(rid, now):
            world = scene.world
            state = tuple(id(item) for item in (world, world.model, world.data,
                getattr(world, "renderer", None), getattr(world, "observer_renderer", None),
                getattr(world, "_render_executor", None)))
            if renderer_state["identity"] != state:
                frame_cache.invalidate()
                renderer_state.update(identity=state, generation=renderer_state["generation"]+1)
            identity = ("robot_cam", "cctv_top", FRAME_JPEG_QUALITY, scene.manifest["scene_xml_sha256"],
                descriptor["camera_sha256"], scene.world.width, scene.world.height,
                scene.world.observer_width, scene.world.observer_height, renderer_state["generation"])
            return frame_cache.read(rid, sim_time_s=now, absolute_time_s=clock.absolute(),
                episode=(frame_episode, *state[:3]), camera_identity=identity,
                observation_id=f"{rid}-{port._observation_sequence[rid]+1:06d}")

        def save_observation(observation):
            rid, oid = observation["robot_id"], observation["observation_id"]
            for label, item in observation["images"].items():
                data = base64.b64decode(item["jpeg_base64"], validate=True)
                (scene.out / "rgb" / f"{oid}-{label}.jpg").write_bytes(data)
            with (scene.out / "skill-inputs.jsonl").open("a") as stream:
                stream.write(json.dumps({**observation, "images": {k: {
                    "path": f"rgb/{oid}-{k}.jpg", "sha256": v["sha256"]} for k, v in observation["images"].items()}})+"\n")

        worker_log_path = scene.out / "worker-timing.jsonl"
        def save_supervisor(row):
            with worker_log_path.open("a") as stream:
                stream.write(json.dumps(row, allow_nan=False)+"\n")

        def factory(rid, role, name):
            spec = SKILLS[name]
            if spec["team_size"] == 1:
                return SoloActorSkill(rid, static_map, spec)
            return PairActorSkill(rid, role, static_map, spec, (skill, grasp, stages, reference))

        next_sample = [origin+.1]
        def advance(delta):
            # DispatchScene.step ticks every real CameraRobotPort before each
            # physics substep; this is the sole post-setup world step caller.
            frame_cache.invalidate()
            started_step = time.monotonic()
            scene.step(delta)
            stepped = time.monotonic()
            now = clock.target(port._now_s+delta)
            if now+1e-8 >= next_sample[0]:
                referee.sample()
                next_sample[0] = now+.1
            sampled = time.monotonic()
            video.capture()
            port._supervisor_event({"event": "RGB_OWNER_ADVANCED", "timestamp_s": now-origin,
                "physics_step_wall_s": stepped-started_step,
                "referee_sample_wall_s": sampled-stepped,
                "video_capture_wall_s": time.monotonic()-sampled})

        port = RGBSkillExecutionPort({rid: _RelativeEndpoint(p, origin, clock=clock,
                invalidate_frames=frame_cache.invalidate) for rid, p in scene.ports.items()},
            frame_source=frames, static_context=static_context, skill_factory=factory,
            advance_physics=advance, command_states={rid: dict(INITIAL_COMMANDS) for rid in scene.ports},
            max_sim_s=config["max_sim_s"], max_commands=config["max_commands"], record_observation=save_observation,
            simulation_clock=clock, record_supervisor=save_supervisor)
        provenance = {**descriptor, "scene_xml_sha256": scene.manifest["scene_xml_sha256"],
            "execution_bundle_id": execution_bundle["id"],
            "execution_bundle_sha256": bundle_sha,
            "execution_bundle_status": execution_bundle["status"],
            "effective_execution": applied_contract,
            "diff_from_historical_f1": baseline_diff(applied_contract),
            "execution_source": source_identity(),
            "execution_environment": environment_fingerprint(),
            "map_to_scene": {**descriptor["map_to_scene"], "scene_xml_sha256": scene.manifest["scene_xml_sha256"]},
            "sim_origin_s": origin, "initial_invariants": scene.initial_invariants,
            "video": str(scene.out / "execution.mp4"), "referee": str(scene.out / "referee-only.jsonl")}
        (scene.out / "backend-provenance.json").write_text(json.dumps(provenance, indent=2)+"\n")
        (scene.out / "episode-setup-only.json").write_text(json.dumps(scene.config, indent=2)+"\n")
        cached = []
        closed = [False]
        def snapshot():
            if cached:
                return copy.deepcopy(cached[0])
            # Referee.finish is post-run and closes its stream. Refuse to turn
            # a live measurement into an actor-visible completion signal.
            if not port._closed:
                raise ValueError("close actor_port before post-run evaluation")
            referee.sample()
            video.capture(force=True)
            records = list(port._task_records.values())
            raw = {}
            for dock in ("dock_a", "dock_b"):
                tasks = [t for t in records if t["dock"] == dock]
                task_ids = {t["task_id"] for t in tasks}
                for rid, commands in port._own_commands.items():
                    scene.command_history[rid] = [{**c, "issued_at_s": c["issued_at_s"]+origin,
                        "stage": "TRANSIT" if c["stage"] == "carry" else c["stage"]}
                        for c in commands if c["task_id"] in task_ids]
                raw[dock] = referee.finish({"dock": dock, "tasks": tasks})
            objects = {}
            for obj in ("box", "beam"):
                selected = [r for r in records if r["object"] == obj]
                goal = selected[-1]["dock"] if selected else None
                report = raw[goal]["cargo"][obj] if goal else None
                objects[obj] = {"attempted": bool(selected), "selected_dock": goal,
                    "physical_success": bool(report and report["physical_success"]), "raw": report}
            complete = all(v["physical_success"] for v in objects.values()) and len({v["selected_dock"] for v in objects.values()}) == 1
            external = {"mission_complete": complete, "objects": objects, "raw_by_dock": raw,
                "recovery": {"required": False, "reached": False, "succeeded": False},
                "meaning": "post-run sampled physical evaluator; solo/pair diagnostic targets are derived separately"}
            value = {"schema": "ugrp.rgb_evaluation_snapshot.v1", "clock_domain": "sim",
                "timestamp_s": clock.relative_or_none(), "external_evaluation": external,
                "coordination_audit": copy.deepcopy(port._audit), "active_task_ids": [], "pending_task_ids": [],
                "resource_owners": {}, "meaning": "evaluation-only; forbidden as actor input or completion feedback"}
            cached.append(value)
            (scene.out / "backend-evaluation.json").write_text(json.dumps(value, indent=2)+"\n")
            return copy.deepcopy(value)

        def close():
            if closed[0]:
                return
            closed[0] = True
            try:
                port.close(None)
                snapshot()
            finally:
                try:
                    video.close()
                finally:
                    referee.file.close()
                    clock.freeze()
                    scene.close()
        return RGBBackendBundle(_ActorFacade(port), snapshot, descriptor, provenance, close, port.clock_snapshot)
    except Exception:
        if port is not None:
            port.close(None)
        try:
            if video is not None:
                video.close()
        finally:
            if referee is not None:
                referee.file.close()
            scene.close()
        raise
