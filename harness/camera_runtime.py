"""Independent image/action loops; this module has no simulator state interface.

The caller owns physics and offline evaluation. Actor ports expose only a camera
and bounded actuators. Futures are consumed independently, never in team rounds.
"""
from concurrent.futures import ThreadPoolExecutor
import base64
import copy
import json
from pathlib import Path
import time

from harness.coela_modules import Communication, ROBOTS
from harness.warehouse_runtime import Journal


def run_camera_episode(ports, planners, *, clock, step, output, destination,
                       mode="natural", timeout_s=90., max_calls=24,
                       frame_max_age_s=5., capture_video=None, realtime=True):
    if set(ports) != set(ROBOTS) or set(planners) != set(ROBOTS):
        raise ValueError("THREE_INDEPENDENT_ACTORS_REQUIRED")
    if len({id(p) for p in planners.values()}) != 3:
        raise ValueError("SHARED_PLANNER_FORBIDDEN")
    if timeout_s <= 0 or max_calls < 1 or frame_max_age_s <= 0:
        raise ValueError("INVALID_CAMERA_BUDGET")
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    frames = out / "inputs"
    frames.mkdir(exist_ok=True)
    journal = Journal(out / "episode.jsonl")
    bus = Communication(mode, max_messages=max_calls, max_bytes=max_calls*3000)
    memory = {r: [] for r in ROBOTS}
    calls = {r: 0 for r in ROBOTS}
    applied = {r: 0 for r in ROBOTS}
    errors = []
    pending, ready_at = {}, {r: float(clock()) for r in ROBOTS}
    start, wall_start = float(clock()), time.monotonic()
    journal.write("study_begin", architecture="camera_pixels_only_v1", mode=mode,
                  actor_inputs="own RGB JPEG, own command history, explicit inbox, static task legend",
                  execution="bounded raw actuators; no automatic pickup or oracle navigation",
                  timeout_s=timeout_s, max_calls=max_calls, start_sim_time=start)
    pool = ThreadPoolExecutor(max_workers=3, thread_name_prefix="camera-actor")
    with (out / "dialogue.jsonl").open("w") as dialogue:
        try:
            while float(clock())-start < timeout_s:
                now = float(clock())
                for rid in ROBOTS:
                    ports[rid].tick(now)
                    if rid in pending and pending[rid][0].done():
                        future, obs = pending.pop(rid)
                        try:
                            decision = future.result()
                            journal.write("decision", robot_id=rid, sim_time=now,
                                          frame_id=obs["frame_id"], decision=decision)
                            if now-float(obs["sim_time"]) > frame_max_age_s:
                                raise ValueError("STALE_CAMERA_FRAME_ACTION_REJECTED")
                            message = decision.get("message")
                            if message:
                                event = bus.send(rid, message, now)
                                journal.write("message_sent", **event)
                                dialogue.write(json.dumps({"robot_id":rid,
                                    "recipients":event["recipients"],
                                    "message":event["content"] if isinstance(event["content"],str) else json.dumps(event["content"],ensure_ascii=False),
                                    "sim_time":now, "video_time":now-start,
                                    "source":"actual_camera_policy_message"},ensure_ascii=False)+"\n")
                                dialogue.flush()
                            feedback = ports[rid].apply(decision["action"], now)
                            applied[rid] += 1
                            journal.write("actuator_command", robot_id=rid, sim_time=now,
                                          frame_id=obs["frame_id"], action=decision["action"], feedback=feedback)
                            memory[rid].append({"frame_id":obs["frame_id"], "sim_time":now,
                                "own_action":decision["action"], "private_reason":decision.get("private_reason",""),
                                "own_message":message, "actuator_feedback":feedback})
                            duration = decision["action"].get("duration_s", 0.8 if decision["action"]["kind"] in {"look","arm"} else .15)
                            ready_at[rid] = max(now+float(duration), float(feedback.get("busy_until",now)))+.2
                        except Exception as exc:
                            ports[rid].stop()
                            reason = (exc.strerror if isinstance(exc,OSError) and exc.strerror else str(exc))[:1000]
                            error = {"robot_id":rid, "sim_time":now, "reason":reason}
                            errors.append(error)
                            journal.write("actor_error", **error, raw=getattr(exc,"raw",None), usage=getattr(exc,"usage",None))
                            memory[rid].append({"actuator_feedback":{"ok":False,"reason":reason}})
                            ready_at[rid] = now+.2
                    if rid not in pending and calls[rid] < max_calls and now >= ready_at[rid]:
                        obs = ports[rid].capture()
                        path = frames / f"{rid}-{calls[rid]+1:03d}.jpg"
                        path.write_bytes(base64.b64decode(obs["image"], validate=True))
                        reports = bus.receive(rid, float(clock()))
                        memory[rid].extend({"received_message":m} for m in reports)
                        memory[rid] = memory[rid][-32:]
                        audit_obs = {k:v for k,v in obs.items() if k != "image"}
                        journal.write("planner_input", robot_id=rid, observation=audit_obs,
                                      image_path=str(path.relative_to(out)), memory=memory[rid], messages=reports)
                        pending[rid] = (pool.submit(planners[rid].decide, copy.deepcopy(obs),
                            copy.deepcopy(memory[rid]), copy.deepcopy(reports), mode=mode,
                            destination=destination), obs)
                        calls[rid] += 1
                if all(calls[r] >= max_calls and r not in pending and float(clock()) >= ready_at[r] for r in ROBOTS):
                    break
                step()
                if capture_video:
                    capture_video()
                if realtime:
                    delay = (float(clock())-start)-(time.monotonic()-wall_start)
                    if delay > 0:
                        time.sleep(min(.02,delay))
        finally:
            for port in ports.values():
                port.stop()
            for rid, (future, obs) in pending.items():
                journal.write("pending_call_discarded", robot_id=rid, frame_id=obs["frame_id"],
                              reason="EPISODE_ENDED_NO_LATE_ACTUATION")
                future.cancel()
            pool.shutdown(wait=True, cancel_futures=True)
    result = {"architecture":"camera_pixels_only_v1", "mode":mode,
              "calls":calls, "applied_commands":applied, "errors":errors,
              "elapsed_sim_s":float(clock())-start, "elapsed_wall_s":time.monotonic()-wall_start,
              "messages":len(bus.sent), "pending_discarded":len(pending),
              "reason":"CALL_BUDGET" if all(calls[r]>=max_calls for r in ROBOTS) else "TIME_BUDGET"}
    journal.write("study_end", **result)
    return result
