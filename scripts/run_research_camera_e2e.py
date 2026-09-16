#!/usr/bin/env python3
"""Live independent LLMs: negotiate -> prepare -> five-stage RGB transport attempt.

No synthetic visual verdicts, demonstrations, ideal-position control or welds.
Fixed budgets retain failures. This two-robot feasibility pilot is not the full
three-robot mixed-workload research comparison or evidence of communication gain.
"""
from __future__ import annotations

import argparse
import base64
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback
from unittest.mock import patch
from urllib.request import urlopen

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))

from harness.research_camera_actor import ROBOTS, TASK, build_request, validate_reply, preparation_ready, agreed_roles
from harness.task_stage_execution import TaskStageExecution
from harness.task_stage_sync import TaskPlan


def write(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+"\n")


def evaluate_trial(samples):
    """Output-only assessment; model DONE is never physical success."""
    from scripts.run_camera_pair_transport import evaluate_grasp_samples
    tail=samples[-11:]
    sustained=bool(len(tail)==11 and tail[-1]["sim_time_s"]-tail[0]["sim_time_s"]>=.99)
    result={**evaluate_grasp_samples(samples),"final_goal":samples[-1]["goal"] if samples else None,
        "any_weld":any(any(s["constraints_active"].values()) for s in samples),
        "goal_stable_at_end":sustained and all(s["goal"]["success"] for s in tail),
        "released_at_end":sustained and not any(s["contacts"][r][finger] for s in tail for r in ROBOTS for finger in ("left","right"))}
    result["physical_success"]=bool(result["grasp_success"] and result["goal_stable_at_end"]
                                    and result["released_at_end"] and not result["any_weld"])
    return result


def run(output, *, communication="natural", seed=11, rounds=40, model="gemini-3.8-flash", timeout=30.,
        max_input_tokens=180000, max_wall_s=600.):
    import mujoco
    import sim.multi_masterpi_production as production
    from sim.camera_robot_port import CameraRobotPort
    from sim.cooperative_payload import beam_pose, evaluate_beam_mission
    from harness.gemini_proxy import GeminiProxyCompleter
    from scripts.probe_dual_grasp_sync import _plain_beam_xml, _plain_beam_contact, _pose_metrics, Video

    if subprocess.check_output(["git","status","--porcelain"],cwd=ROOT,text=True).strip():
        raise RuntimeError("commit and freeze the complete source before a trial")
    output.mkdir(parents=True,exist_ok=False)
    for r in ROBOTS: (output/r).mkdir()
    source=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip()
    report={"source_sha":source,"scope":"Live two-robot L2 feasibility pilot; no synthetic stage verdicts; single fixed scene",
        "task":TASK,"config":{"communication":communication,"seed":seed,"rounds":rounds,"model":model,
            "timeout_s":timeout,"max_input_tokens":max_input_tokens,"max_wall_s":max_wall_s,"slice_s":.3,"lease_s":.2},
        "environment":{"python":sys.version,"platform":platform.platform(),"mujoco":mujoco.__version__},
        "calls":[],"turns":[],"messages":[],"phase_events":[],"error":None,"stop_reason":None,
        "protocol_finish":False,"roles":None,"cost_usd":None,"cost_note":"proxy supplies tokens, not a billing amount",
        "shared_clock":"SIM pauses during model inference; no asynchronous physical latency claim"}
    world=execution=video=referee=None
    ports={}
    history={r:[] for r in ROBOTS};inbox={r:[] for r in ROBOTS};previous={r:None for r in ROBOTS}
    wire_counts={r:0 for r in ROBOTS};samples=[];phase="NEGOTIATE";roles=None
    started=time.monotonic();last_sample=0.;all_waits=0
    def now(): return float(world.data.time)
    def transition(new_phase):
        nonlocal phase
        report["phase_events"].append({"from":phase,"to":new_phase,"sim_time_s":now(),"wall_s":time.monotonic()-started})
        phase=new_phase
    def evaluate_sample():
        # The only reader is the final evaluator. No branch selecting an action reads this.
        sample={"phase":phase,**_pose_metrics(world),"contacts":{r:_plain_beam_contact(world,r) for r in ROBOTS}}
        sample["goal"]=evaluate_beam_mission(beam_pose(world.data,world.model))
        samples.append(sample);referee.write(json.dumps(sample)+"\n");referee.flush()
    def step(seconds):
        nonlocal last_sample
        end=now()+seconds
        while now()+1e-9<end:
            if execution: execution.tick(now())
            else:
                for p in ports.values():p.tick(now())
            world._physics_step_for(world.controllers["r1"])
            video.stage=phase;video.capture()
            if now()+1e-9>=last_sample+.1:
                evaluate_sample();last_sample=now()
        if execution:execution.tick(now())
        else:
            for p in ports.values():p.tick(now())
    def camera(rid,index,top):
        own=bytes(world.render_jpeg(robot_id=rid,camera="robot_cam"))
        if execution:
            packet=execution.capture(rid,own_rgb=own,top_rgb=top,now_s=now())
            # References in the stage adapter are relative to its own evidence root.
            for image in packet["images"].values():image["ref"]="stage-execution/"+image["ref"]
            return packet
        images={}
        for label,data in (("own_rgb",own),("top_rgb",top)):
            relative=f"{rid}/{index:03d}-{label}.jpg";(output/relative).write_bytes(data)
            images[label]={"ref":relative,"sha256":hashlib.sha256(data).hexdigest(),"jpeg_base64":base64.b64encode(data).decode()}
        return {"images":images,"observed_at_s":now()}
    def invoke(rid, request, index):
        write(output/rid/f"{index:03d}-request.json",request)
        entry={"robot_id":rid,"turn":index,"phase":phase,"request":f"{rid}/{index:03d}-request.json"}
        try:
            raw=completers[rid].complete(request["messages"],images=request["images"])
            entry["raw_response"]=raw
            entry["reply"]=validate_reply(raw,phase)
        except Exception:
            entry["error"]=traceback.format_exc()
        finally:
            c=completers[rid]
            entry.update(usage=c.last_usage,model=c.last_model,latency_ms=c.last_latency_ms)
            write(output/rid/f"{index:03d}-decision.json",entry)
        return entry
    try:
        # Same approved plain-beam fixture, startup poses and actor camera calibration
        # as run_camera_pair_transport.py; these setup values never reach an actor.
        with patch.object(production,"build_multi_robot_xml",_plain_beam_xml(production.build_multi_robot_xml)):
            world=production.MultiMasterPiProductionV2(seed=seed,render=True,width=640,height=480)
        for rid,y in (("r1",-2.325),("r3",-1.675)):
            world.controllers[rid].set_base_pose_for_test((-.02,y,.0324),0.)
        for name in ("cctv_top","cctv_warehouse"):
            cid=mujoco.mj_name2id(world.model,mujoco.mjtObj.mjOBJ_CAMERA,name)
            world.model.cam_pos[cid]=(.55,-2.,2.5);world.model.cam_quat[cid]=(1,0,0,0);world.model.cam_fovy[cid]=55
        mujoco.mj_forward(world.model,world.data)
        ports={r:CameraRobotPort(world,r) for r in ROBOTS}
        def invariant():
            return {"camera_pos":world.model.cam_pos.tolist(),"camera_quat":world.model.cam_quat.tolist(),
                    "camera_fov":world.model.cam_fovy.tolist(),"geom_size_hash":hashlib.sha256(world.model.geom_size.tobytes()).hexdigest(),
                    "timestep":float(world.model.opt.timestep)}
        report["invariants_initial"]=invariant()
        mujoco.mj_saveLastXML(str(output/"scene.xml"),world.model)
        descriptor={"id":"camera-pair-plain-beam-v1","version":"1","task":TASK,
                    "static_geometry_source":source,"compiled_xml_sha256":hashlib.sha256((output/"scene.xml").read_bytes()).hexdigest()}
        write(output/"scene-descriptor.json",descriptor)
        completers={}
        for rid in ROBOTS:
            def audited_open(req, *, timeout, _rid=rid):
                wire_counts[_rid]+=1;count=wire_counts[_rid]
                (output/_rid/f"wire-{count:03d}.json").write_bytes(req.data)
                with urlopen(req,timeout=timeout) as response: data=response.read()
                (output/_rid/f"wire-{count:03d}-response.json").write_bytes(data)
                return io.BytesIO(data)
            completers[rid]=GeminiProxyCompleter(model=model,max_tokens=900,timeout=timeout,reasoning_effort="none",http_open=audited_open)
        video=Video(world,output/"motion.mp4",6);video.stage=phase;video.capture(force=True)
        referee=(output/"evaluation-only.jsonl").open("w");last_sample=now();evaluate_sample()
        negotiation_turns=action_turns=0
        with ThreadPoolExecutor(max_workers=2) as pool:
            for index in range(rounds+4):
                if time.monotonic()-started>=max_wall_s:report["stop_reason"]="wall_budget";break
                used=sum((c.get("usage") or {}).get("prompt_tokens",0) for c in report["calls"])
                if used>=max_input_tokens:report["stop_reason"]="input_token_budget";break
                if execution:
                    execution.tick(now())
                    if execution.sync.stage!=phase: transition(execution.sync.stage)
                top=bytes(world.render_team_jpeg(camera="cctv_top"))
                packets={r:camera(r,index,top) for r in ROBOTS}
                requests={r:build_request(r,phase,packets[r],roles=roles,own_history=history[r],inbox=inbox[r],
                    previous=previous[r],communication=communication) for r in ROBOTS}
                # Preserve the exact allowed derivation inputs, independently of referee data.
                for r in ROBOTS:
                    write(output/r/f"{index:03d}-inputs.json",{"phase":phase,"camera":packets[r],"roles":roles,
                        "own_history":history[r][-16:],"inbox":inbox[r][-4:] if communication=="natural" else [],
                        "previous":previous[r],"communication":communication})
                futures={r:pool.submit(invoke,r,requests[r],index) for r in ROBOTS}
                calls={r:futures[r].result() for r in ROBOTS}
                report["calls"].extend(calls.values())
                if any("error" in c for c in calls.values()):report["stop_reason"]="model_or_reply_error";break
                replies={r:calls[r]["reply"] for r in ROBOTS}
                row={"turn":index,"phase":phase,"observed_at_s":now(),"replies":replies,"issued":{}}
                report["turns"].append(row)
                # Deliver only model-produced text, next turn; never central state/GT.
                for sender in ROBOTS:
                    receiver=next(r for r in ROBOTS if r!=sender)
                    if communication=="natural" and replies[sender]["message"].strip():
                        message={"sender":sender,"recipient":receiver,"turn":index,"text":replies[sender]["message"],"source":"peer_claim"}
                        inbox[receiver].append(message);report["messages"].append(message)
                previous=packets
                if phase=="NEGOTIATE":
                    negotiation_turns+=1;roles=agreed_roles(replies)
                    if roles:report["roles"]=roles;transition("PREPARE")
                    elif negotiation_turns>=4:report["stop_reason"]="role_agreement_budget";break
                elif phase=="PREPARE":
                    action_turns+=1
                    if all(preparation_ready(replies[r]) for r in ROBOTS):
                        for p in ports.values():p.hold(now())
                        plan=TaskPlan("live-camera-beam",1,"plain_orange_beam",tuple((r,roles[r]) for r in ROBOTS),
                            "green_square",descriptor["id"],descriptor["version"],hashlib.sha256((output/"scene-descriptor.json").read_bytes()).hexdigest())
                        execution=TaskStageExecution(plan,ports,map_path=output/"scene-descriptor.json",output=output/"stage-execution",now_s=now())
                        transition("GRASP")
                    else:
                        # Independent pre-grasp motion; the five-stage contract starts
                        # only after both model-produced at_grasp_pose/stopped reports.
                        for r in ROBOTS:ports[r].validate_bounded(replies[r]["action"],.2)
                        for r in ROBOTS:
                            action={"kind":"wait"} if preparation_ready(replies[r]) else replies[r]["action"]
                            ports[r].apply_bounded(action,now(),.2)
                            command={"command_id":f"{r}-{index}","action":action,"duration_s":.2,"issued_at_s":now(),"phase":phase}
                            history[r].append(command);row["issued"][r]=command
                else:
                    action_turns+=1
                    for r in ROBOTS:
                        result=replies[r]
                        row.setdefault("reports_accepted",{})[r]=execution.receive(r,{
                            "request_id":packets[r]["request_id"],"status":result["status"],"confidence":result["confidence"],
                            "checks":result["checks"],"command_id":result["command_id"],"reason":result["reason"],"decided_at_s":now()},now_s=now())
                    decision=execution.authorize(now_s=now());row["decision"]=decision
                    if execution.advance(now_s=now()):
                        if execution.authorize(now_s=now())["phase"]=="FINISH":
                            report["protocol_finish"]=True;report["stop_reason"]="model_reported_finish";break
                        transition(execution.sync.stage)
                    elif decision["permission"] and all(x["status"]=="READY" for x in replies.values()):
                        commands={r:{"command_id":f"{r}-{index}","action":replies[r]["action"],"duration_s":.2} for r in ROBOTS}
                        if execution.dispatch_pair(decision["permission"],commands,now_s=now()):
                            for r in ROBOTS:
                                cmd={**commands[r],"issued_at_s":now(),"phase":phase};history[r].append(cmd);row["issued"][r]=cmd
                step(.3)
                report["current_phase"]=phase;report["sim_time_s"]=now()
                write(output/"progress.json",report)
                print(f"{communication} {index:02d} {row['phase']} -> {phase}: "+" / ".join(f"{r} {replies[r].get('status','plan')} {replies[r].get('action',{})}" for r in ROBOTS),flush=True)
                if action_turns>=rounds:report["stop_reason"]="action_round_budget";break
                if phase!="NEGOTIATE":
                    all_waits=all_waits+1 if not row["issued"] or all(c["action"]["kind"]=="wait" for c in row["issued"].values()) else 0
                    if all_waits>=12:report["stop_reason"]="twelve_turns_without_active_command";break
            else: report["stop_reason"]="total_round_budget"
        # Fixed final settling, independent of truth and model success reports.
        for p in ports.values():p.hold(now())
        step(2.2)
        report["current_phase"]=phase;report["sim_time_s"]=now()
        report["evaluation"]=evaluate_trial(samples)
        report["invariants_final"]=invariant()
        e=report["evaluation"]
        report["physical_success"]=bool(e["physical_success"] and report["invariants_initial"]==report["invariants_final"])
    except Exception:
        report["error"]=traceback.format_exc();report["stop_reason"]="runtime_error"
    finally:
        report["wall_seconds"]=time.monotonic()-started
        report["wire_requests"]=wire_counts
        report["token_usage"]={key:sum((c.get("usage") or {}).get(key,0) for c in report["calls"])
            for key in ("prompt_tokens","completion_tokens","total_tokens")}
        report["physical_success"]=bool(report.get("physical_success",False) and not report["error"])
        cleanup=[]
        try:
            if execution:execution.close(now_s=now())
            elif world:
                for p in ports.values():p.hold(now())
        except Exception:cleanup.append(traceback.format_exc())
        for resource in (referee,video,world):
            if resource:
                try:resource.close()
                except Exception:cleanup.append(traceback.format_exc())
        report["cleanup_errors"]=cleanup
        report["files"]={str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in output.rglob("*") if p.is_file()}
        write(output/"result.json",report)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--communication",choices=("natural","none"),default="natural")
    p.add_argument("--rounds",type=int,default=40);p.add_argument("--seed",type=int,default=11)
    p.add_argument("--model",default="gemini-3.8-flash")
    a=p.parse_args()
    if not 1<=a.rounds<=64:p.error("rounds must be 1..64")
    r=run(a.output.resolve(),communication=a.communication,rounds=a.rounds,seed=a.seed,model=a.model)
    print(json.dumps({k:r.get(k) for k in ("current_phase","stop_reason","physical_success","error","wall_seconds","token_usage")},ensure_ascii=False))
    return int(bool(r["error"] or r["cleanup_errors"]))


if __name__=="__main__":raise SystemExit(main())
