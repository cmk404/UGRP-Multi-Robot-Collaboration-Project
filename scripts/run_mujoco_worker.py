#!/usr/bin/env python3
from __future__ import annotations
import argparse, base64, json, os, time, threading
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import URLError

from sim.grasp_physics import PhysicsGraspWorld
from sim.rl_skill import GraspPolicySkill
from sim.mujoco_world import MasterPiWorld
from sim.masterpi_physics import MasterPiPhysicsWorld


SPEED_FILE = Path(os.environ.get("UGRP_SIM_SPEED_FILE", "/tmp/ugrp_sim_speed"))

def read_speed() -> float:
    try:
        value = float(SPEED_FILE.read_text().strip())
    except Exception:
        value = 1.0
    return value if value in (1.0, 2.0, 3.0) else 1.0

def apply_speed(world) -> float:
    speed = read_speed()
    if hasattr(world, "set_speed_multiplier"):
        world.set_speed_multiplier(speed)
    return speed

def request_json(url, token, payload=None, timeout=30):
    headers={"X-UGRP-Sim-Token":token}
    data=None; method="GET"
    if payload is not None:
        headers["Content-Type"]="application/json"; data=json.dumps(payload).encode(); method="POST"
    req=Request(url, data=data, headers=headers, method=method)
    with urlopen(req, timeout=timeout) as r: return json.loads(r.read().decode())


class PublicationSequencer:
    """Monotonic capture-order sequence for state/frame/result publications."""
    def __init__(self):
        self._lock = threading.Lock()
        self._value = time.time_ns()
    def next(self) -> int:
        with self._lock:
            # Wall-clock base survives worker process restarts; max(+1) preserves
            # strict monotonicity even if two captures occur in one clock tick.
            self._value = max(self._value + 1, time.time_ns())
            return self._value

class AsyncFrameSender:
    """Keep only the newest observational frame and send it off the control path."""
    def __init__(self, bridge, token):
        self.bridge, self.token = bridge, token
        self._cv = threading.Condition()
        self._pending = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, payload):
        with self._cv:
            # Drop stale camera payloads if network is slower than rendering.
            self._pending = payload
            self._cv.notify()

    def _run(self):
        while True:
            with self._cv:
                while self._pending is None:
                    self._cv.wait()
                payload, self._pending = self._pending, None
            try:
                request_json(self.bridge+"/worker/frame", self.token, payload, timeout=15)
            except Exception as e:
                print("frame publish retry", e, flush=True)


def push_frame(world, bridge, token, *, observer_names=None, sender=None, sequencer=None):
    # Sequence is assigned when this snapshot begins, before slow rendering.
    # A result captured later can therefore fence this frame if delivery is late.
    seq = sequencer.next() if sequencer is not None else None
    payload = {
        "jpeg_b64": base64.b64encode(world.render_jpeg()).decode(),
        "state": world.state(),
    }
    if seq is not None:
        payload["state_seq"] = seq
    if observer_names:
        observer_jpegs = {}
        if hasattr(world, "render_jpeg"):
            for name in observer_names:
                try:
                    observer_jpegs[name] = world.render_jpeg(camera=name, quality=78)
                except TypeError:
                    observer_jpegs = {}
                    break
        if not observer_jpegs and hasattr(world, "render_observer_jpegs"):
            all_jpegs = world.render_observer_jpegs(quality=78)
            observer_jpegs = {name: all_jpegs[name] for name in observer_names if name in all_jpegs}
        if observer_jpegs:
            payload["observer_jpegs_b64"] = {
                name: base64.b64encode(jpeg).decode() for name, jpeg in observer_jpegs.items()
            }
            payload["observer_jpeg_b64"] = base64.b64encode(next(iter(observer_jpegs.values()))).decode()
    if sender is None:
        request_json(bridge+"/worker/frame", token, payload, timeout=15)
    else:
        sender.submit(payload)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--bridge", default=os.environ.get("UGRP_SIM_BRIDGE_URL"))
    ap.add_argument("--token", default=os.environ.get("UGRP_SIM_TOKEN"))
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--legacy-attach", action="store_true", help="use old nonphysical attach simulator")
    ap.add_argument("--legacy-ppo", action="store_true", help="use the old 6-axis PPO/contact world")
    ap.add_argument("--policy-url", default=os.environ.get("UGRP_GRASP_POLICY_URL", "http://127.0.0.1:8093"))
    a=ap.parse_args()
    if not a.bridge or not a.token: raise SystemExit("set UGRP_SIM_BRIDGE_URL and UGRP_SIM_TOKEN")
    bridge=a.bridge.rstrip("/")
    frame_sender = AsyncFrameSender(bridge, a.token)
    publication_seq = PublicationSequencer()
    if a.legacy_attach:
        world = MasterPiWorld(seed=a.seed)
    elif a.legacy_ppo:
        world = PhysicsGraspWorld(seed=a.seed)
    else:
        world = MasterPiPhysicsWorld(seed=a.seed)
    grasp_skill = GraspPolicySkill(a.policy_url) if a.legacy_ppo else None
    speed = apply_speed(world)
    # UI rendering is observational and must never dominate control latency.
    # Push robot-camera frames at a bounded wall-clock rate. CCTV cameras update
    # one-at-a-time round-robin; all four are synchronized at action boundaries.
    cctv_names = ("cctv_front_left", "cctv_front_right", "cctv_rear_left", "cctv_top")
    frame_clock = {"robot": 0.0, "observer": 0.0, "index": 0}
    def next_cctv():
        i = frame_clock["index"] % len(cctv_names)
        names = (cctv_names[i], cctv_names[(i + 1) % len(cctv_names)])
        frame_clock["index"] += 2
        frame_clock["observer"] = time.monotonic()
        return names
    def push_intermediate():
        now = time.monotonic()
        speed_now = float(getattr(world, "speed_multiplier", 1.0))
        # Faster simulation intentionally emits fewer wall-clock frames.
        if now - frame_clock["robot"] < 0.14 * speed_now:
            return
        # Keep motion feedback responsive: render ONLY the robot camera here.
        # Four software-rendered CCTV views would otherwise block each motion frame
        # for ~2 seconds on this Virtio/kms_swrast host. CCTV is synchronized at
        # action boundaries below instead.
        push_frame(world, bridge, a.token, observer_names=None, sender=frame_sender, sequencer=publication_seq)
        frame_clock["robot"] = time.monotonic()
    if hasattr(world, "frame_callback"):
        world.frame_callback = push_intermediate
    push_frame(world, bridge, a.token, observer_names=cctv_names, sender=frame_sender, sequencer=publication_seq)
    frame_clock["robot"] = frame_clock["observer"] = time.monotonic()
    print("UGRP MuJoCo worker connected", {"speed": speed, **world.state()}, flush=True)
    last_id = None
    last_payload = None
    while True:
        try:
            # Heartbeat only refreshes the agent camera. The idle world is static,
            # so re-rendering four identical CCTV images wastes most CPU time.
            push_frame(world, bridge, a.token, observer_names=None, sender=frame_sender, sequencer=publication_seq)
            frame_clock["robot"] = time.monotonic()
            # The world is static while idle, so long-poll without re-rendering identical frames.
            # During motion, frame_callback pushes fresh robot + CCTV views at bounded cadence.
            obj=request_json(bridge+"/worker/next?timeout=20", a.token, timeout=30)
            cmd=obj.get("command")
            if not cmd:
                push_frame(world, bridge, a.token, observer_names=None, sender=frame_sender, sequencer=publication_seq); continue
            cid = str(cmd["id"])
            action = cmd["action"]
            if cid == last_id and last_payload is not None:
                # Bridge may redeliver a claimed command if the previous HTTP
                # response was lost. Re-send the cached result; never move twice.
                push_frame(world, bridge, a.token, observer_names=next_cctv(), sender=frame_sender, sequencer=publication_seq)
                frame_clock["robot"] = time.monotonic()
                request_json(bridge+"/worker/result", a.token, {"id": cid, "result": last_payload}, timeout=15)
                print(action, "duplicate-redelivery: resent cached result", flush=True)
                continue
            speed = apply_speed(world)
            if action == "reset" and cmd.get("seed") is not None and isinstance(world, MasterPiPhysicsWorld):
                result = world.act(action, seed=int(cmd["seed"]))
            else:
                result = grasp_skill.run(world) if action == "grasp_rl" and grasp_skill is not None else world.act(action)
            vision = world.camera_red_detection() if hasattr(world, "camera_red_detection") else None
            yellow_vision = world.camera_yellow_detection() if hasattr(world, "camera_yellow_detection") else None
            blue_vision = world.camera_blue_detection() if hasattr(world, "camera_blue_detection") else None
            result_seq = publication_seq.next()
            # Use current world state at the action boundary; it includes semantic
            # relation updates such as RED=ON_BLUE/ON_YELLOW.
            payload = {"ok":result.ok,"action":result.action,"reason":result.reason,"state":world.state(),"state_seq":result_seq,"sim_speed":speed}
            if vision is not None:
                payload["vision"] = vision
            if yellow_vision is not None:
                payload["yellow_vision"] = yellow_vision
            if blue_vision is not None:
                payload["blue_vision"] = blue_vision
            last_id, last_payload = cid, payload
            # Return the command result first; camera refresh is observational and
            # must never extend action latency.
            request_json(bridge+"/worker/result", a.token, {"id":cid,"result":payload}, timeout=15)
            push_frame(world, bridge, a.token, observer_names=cctv_names, sender=frame_sender, sequencer=publication_seq)
            frame_clock["robot"] = frame_clock["observer"] = time.monotonic()
            print(action, result.ok, result.reason, result.state, flush=True)
        except (URLError, TimeoutError, OSError) as e:
            print("bridge retry", e, flush=True); time.sleep(2)

if __name__ == "__main__": main()
