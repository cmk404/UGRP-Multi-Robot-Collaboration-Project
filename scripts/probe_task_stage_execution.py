#!/usr/bin/env python3
"""Real camera/actuator diagnostic with SYNTHETIC stage verdicts, not a task demo.

The absent-producer case uses the production fail-closed reply. All other
cases inject contract verdicts to exercise actual MuJoCo ports. No visual
grasp/lift/support/release detector, LLM, grasp fixture, or loaded carry here.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))

from harness.task_stage_execution import TaskStageExecution, uncertain_reply
from harness.task_stage_sync import TaskPlan, READY_CHECKS, DONE_CHECKS, STAGES

CASES = ("absent_producer", "five_stage_fixture", "arm_hold_resume",
         "carry_report_loss", "second_port_failure", "revoked_permission")


def write(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def fixture_reply(request, status="READY", command_id=None):
    checks = DONE_CHECKS if status == "DONE" else READY_CHECKS
    return {"request_id": request["request_id"], "status": status, "confidence": 1.,
            "checks": sorted(checks[request["stage"]]), "command_id": command_id,
            "reason": "SYNTHETIC diagnostic verdict; NOT inferred from RGB",
            "decided_at_s": request["observed_at_s"]}


def run_case(case, output):
    import mujoco
    from PIL import Image, ImageDraw
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2

    output.mkdir(parents=True, exist_ok=False)
    map_path = ROOT / "examples/task_stage_execution/scene.json"
    descriptor = json.loads(map_path.read_text())
    robots = descriptor["participants"]
    plan = TaskPlan(case, 1, "unloaded-actuator-diagnostic", (("r1", "left"), ("r3", "right")),
        descriptor["goal_region"], descriptor["id"], descriptor["version"],
        hashlib.sha256(map_path.read_bytes()).hexdigest())
    world = execution = None
    frames, samples = [], []
    assertions = 0
    started = time.monotonic()
    result = {"case": case, "passed": False, "error": None,
              "scope": "Real RGB and MuJoCo raw ports; synthetic stage verdicts; no autonomous or loaded transport",
              "physical_task_success_evaluated": False, "model_calls": 0, "cost_usd": 0}
    def check(condition, message):
        nonlocal assertions
        assertions += 1
        if not condition: raise AssertionError(message)
    def now(): return float(world.data.time)
    def sample(label):
        # Referee output only: never supplied to receive/dispatch or a producer.
        samples.append({"sim_time_s": now(), "label": label,
            "bases": {r:list(map(float,world.robot(r).base_xyz())) for r in robots},
            "arm_qpos": {r:{name:float(world.data.qpos[world.model.jnt_qposadr[jid]])
                for name,jid in world.robot(r).arm_joint.items()} for r in robots},
            "issued_commands": {r:execution.ports[r]._actuator_state() for r in robots}})
        canvas = Image.new("RGB", (960, 290), "#101720")
        draw = ImageDraw.Draw(canvas)
        draw.text((10, 6), f"{case} | {execution.sync.stage} | {label} | t={now():.2f}s", fill="white")
        draw.text((10, 22), "ACTUATOR TEST / synthetic stage verdicts / no grasp or transport success claim", fill="#ffcc66")
        views = [world.render_team_jpeg(camera="cctv_warehouse")]
        views += [world.render_jpeg(robot_id=r, camera="robot_cam") for r in robots]
        for i, (name, jpeg) in enumerate(zip(("Observer", "R1 own RGB", "R3 own RGB"), views)):
            img = Image.open(io.BytesIO(jpeg)).convert("RGB")
            img.thumbnail((320,240))
            check(len(img.getcolors(img.width*img.height) or range(9)) >= 8, "render must have detail")
            canvas.paste(img, (i*320,50))
            draw.text((i*320+8,36), name, fill="white")
        frames.append(canvas)
    def step(duration, label):
        end = now()+duration
        next_sample = now()+.1
        while now()+1e-9 < end:
            execution.tick(now())
            world._physics_step_for(world.robot("r1"))
            if now()+1e-9 >= next_sample:
                sample(label); next_sample += .1
        execution.tick(now())
    def request(rid):
        return execution.capture(rid, own_rgb=world.render_jpeg(robot_id=rid,camera="robot_cam"),
                                 top_rgb=world.render_team_jpeg(camera="cctv_top"), now_s=now())
    def ready():
        for r in robots: check(execution.receive(r,fixture_reply(request(r)),now_s=now()), "fixture READY accepted")
        return execution.authorize(now_s=now())["permission"]
    counter = 0
    def commands():
        nonlocal counter
        counter += 1
        action = {"kind":"drive","forward":.12,"turn":0.,"duration_s":.2} if execution.sync.stage == "CARRY" else {
            "kind":"arm","servo_id":5,"pulse":1100 if execution.sync.stage in ("GRASP","LIFT") else 1900}
        return {r:{"command_id":f"{case}-{counter}-{r}","action":action,"duration_s":.2} for r in robots}
    def issue(permission=None):
        cmd = commands()
        check(execution.dispatch_pair(permission or ready(),cmd,now_s=now()), "batch issued")
        return cmd
    def done(cmd):
        for r in robots:
            check(execution.receive(r,fixture_reply(request(r),"DONE",cmd[r]["command_id"]),now_s=now()), "fixture DONE accepted")
        check(execution.advance(now_s=now()), "fixture stage transition")
    try:
        world = MultiMasterPiProductionV2(warehouse_layout=descriptor["warehouse_layout"], seed=descriptor["seed"],
            width=640,height=480,render=True,warehouse_cargo_ids=tuple(descriptor["warehouse_cargo_ids"]))
        # No camera/pose/geometry/solver modification; no grasp constraint calls.
        ports = {r:CameraRobotPort(world,r) for r in robots}
        execution = TaskStageExecution(plan,ports,map_path=map_path,output=output/"execution",now_s=now())
        mujoco.mj_saveLastXML(str(output/"scene.xml"),world.model)
        result["scene_xml_sha256"] = hashlib.sha256((output/"scene.xml").read_bytes()).hexdigest()
        actor_cameras = [i for i in range(world.model.ncam) if
            (mujoco.mj_id2name(world.model,mujoco.mjtObj.mjOBJ_CAMERA,i) or "") in ("cctv_top","r1__robot_cam","r3__robot_cam")]
        def camera_record(): return {mujoco.mj_id2name(world.model,mujoco.mjtObj.mjOBJ_CAMERA,i):
            {"pos":world.model.cam_pos[i].tolist(),"quat":world.model.cam_quat[i].tolist(),"fov":float(world.model.cam_fovy[i])} for i in actor_cameras}
        result["cameras_initial"] = camera_record()
        sample("start")
        if case == "absent_producer":
            for r in robots: execution.receive(r,uncertain_reply(request(r)),now_s=now())
            check(not execution.dispatch_pair(None,commands(),now_s=now()), "no producer cannot issue")
            step(.4,"no visual producer: HOLD")
            check(not execution.advance(now_s=now()), "elapsed time is not DONE")
        elif case == "five_stage_fixture":
            for stage in STAGES:
                check(execution.sync.stage == stage,"stage order")
                cmd=issue();step(.22,"bounded command")
                check(not execution.advance(now_s=now()),"command is not DONE")
                done(cmd)
            check(execution.authorize(now_s=now())["phase"] == "FINISH","fixture FINISH")
        elif case == "arm_hold_resume":
            issue();step(.06,"arm moving")
            execution.hold("injected_camera_uncertainty",now_s=now())
            frozen={r:ports[r]._actuator_state()["servo_pulses"] for r in robots}
            step(.3,"HOLD: targets frozen")
            check(all(ports[r]._actuator_state()["servo_pulses"]==frozen[r] for r in robots),"no queued arm motion after HOLD")
            execution.receive("r1",fixture_reply(request("r1")),now_s=now())
            check(execution.authorize(now_s=now())["permission"] is None,"one READY cannot resume")
            execution.receive("r3",fixture_reply(request("r3")),now_s=now())
            issue(execution.authorize(now_s=now())["permission"]);step(.22,"two fresh READY: resume")
            check(any(ports[r]._actuator_state()["servo_pulses"]!=frozen[r] for r in robots),"new command resumes interpolation")
        elif case == "carry_report_loss":
            for _ in range(2): cmd=issue();step(.22,"fixture prefix");done(cmd)
            issue();step(.22,"drive lease expires")
            check(all(ports[r]._actuator_state()["motor_commands"]==[0.]*4 for r in robots),"bounded wheels stop")
            step(.5,"reports missing")
            check(execution.authorize(now_s=now())["phase"] == "HOLD","stale reports HOLD")
            execution.receive("r1",fixture_reply(request("r1")),now_s=now())
            check(execution.authorize(now_s=now())["permission"] is None,"one fresh camera report insufficient")
            execution.receive("r3",fixture_reply(request("r3")),now_s=now())
            issue(execution.authorize(now_s=now())["permission"]);step(.22,"drive resumes")
        elif case == "second_port_failure":
            permission=ready()
            def fail(*args): raise OSError("injected second port failure")
            original=ports["r3"].apply_bounded;ports["r3"].apply_bounded=fail
            try:
                try: execution.dispatch_pair(permission,commands(),now_s=now())
                except OSError: pass
                else: raise AssertionError("port error not propagated")
            finally: ports["r3"].apply_bounded=original
            frozen={r:ports[r]._actuator_state()["servo_pulses"] for r in robots}
            step(.3,"partial submission: both HOLD")
            check(all(ports[r]._actuator_state()["servo_pulses"]==frozen[r] for r in robots),"first queued arm command cancelled")
        else:
            old=ready();execution.hold("injected_hold",now_s=now());ready()
            check(not execution.dispatch_pair(old,commands(),now_s=now()),"old permission refused")
            step(.3,"revoked permission: HOLD")
        sample("final")
        result["final_status"] = execution.authorize(now_s=now())
        result["cameras_final"] = camera_record()
        check(len(actor_cameras)==3 and result["cameras_final"]==result["cameras_initial"],"actor cameras unchanged")
        weld_ids=[i for i in range(world.model.neq) if int(world.model.eq_type[i])==int(mujoco.mjtEq.mjEQ_WELD)]
        result["active_welds_at_end"] = [i for i in weld_ids if world.data.eq_active[i]]
        check(not result["active_welds_at_end"],"weld OFF")
        result["physical_motion_evaluation"]={r:{
            "max_base_displacement_m":max(math.dist(s["bases"][r],samples[0]["bases"][r]) for s in samples),
            "max_arm_joint_change_rad":max(abs(s["arm_qpos"][r][j]-samples[0]["arm_qpos"][r][j])
                for s in samples for j in samples[0]["arm_qpos"][r])} for r in robots}
        if case in ("five_stage_fixture","carry_report_loss"):
            check(all(v["max_base_displacement_m"]>.001 for v in result["physical_motion_evaluation"].values()),
                  "both physical bases moved during diagnostic")
        if case in ("five_stage_fixture","arm_hold_resume"):
            check(all(v["max_arm_joint_change_rad"]>.001 for v in result["physical_motion_evaluation"].values()),
                  "both physical arms moved during diagnostic")
        result["passed"]=True
    except Exception:
        result["error"]=traceback.format_exc()
    finally:
        try:
            if execution:
                result["command_count"]=sum(e["event"]=="LOCAL_COMMAND" for e in execution.events)
                result["sim_time_s"]=now()
                execution.close(now_s=now())
        except Exception:
            result["passed"]=False
            result["cleanup_error"]=traceback.format_exc()
        finally:
            if world: world.close()
        result.update(assertions=assertions,wall_seconds=time.monotonic()-started)
        write(output/"evaluation-only.json",{"samples":samples})
        if frames:
            frames[0].save(output/"motion.gif",save_all=True,append_images=frames[1:],duration=150,loop=0)
            selected=[frames[round(i*(len(frames)-1)/3)] for i in range(4)]
            sheet=Image.new("RGB",(960,1160))
            for i,frame in enumerate(selected):sheet.paste(frame,(0,i*290))
            sheet.save(output/"contact-sheet.png")
        write(output/"result.json",result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--case",choices=CASES,action="append")
    args=parser.parse_args()
    if subprocess.check_output(["git","status","--porcelain"],cwd=ROOT,text=True).strip():
        parser.error("commit execution source before recording experiments")
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    source=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip()
    results=[]
    for case in args.case or CASES:
        result=run_case(case,output/case);results.append(result)
        print(f"{case}: {'PASS' if result['passed'] else 'FAIL'}",flush=True)
    manifest=[{"path":str(p.relative_to(output)),"sha256":hashlib.sha256(p.read_bytes()).hexdigest()}
              for p in sorted(output.rglob("*")) if p.is_file()]
    write(output/"summary.json",{"source_sha":source,"environment":{"python":sys.version,"platform":platform.platform()},
        "results":results,"manifest":manifest})
    return int(not all(r["passed"] for r in results))


if __name__=="__main__": raise SystemExit(main())
