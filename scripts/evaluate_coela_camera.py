"""Evaluate actual camera-only policies; simulator truth is offline scoring only."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
from unittest.mock import patch

from harness.camera_policy import CameraPlanner
from harness.camera_runtime import run_camera_episode
from harness.coela_modules import ROBOTS
from harness.gemini_proxy import GeminiProxyCompleter
from sim.camera_robot_port import CameraRobotPort
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from scripts.evaluate_coela_arena import signature, layout_record, static_geometry_snapshot


def evaluate(*, output, seeds=(41,58,73), modes=("natural",), repeats=1,
             timeout=90., max_calls=24, record=False):
    out=Path(output).resolve()
    out.mkdir(parents=True,exist_ok=False)
    source_hash=signature()
    design={"architecture":"camera_pixels_only_v1", "source_sha256":source_hash,
        "seeds":list(seeds), "modes":list(modes), "repeats":repeats,
        "max_calls_per_robot":max_calls, "timeout_s":timeout,
        "actor_sensor":"own raw RGB JPEG only; no depth/detections/global pose",
        "execution":"bounded raw wheel and physical servo targets",
        "task_legend":"zone colors and cargo descriptions, no coordinate map",
        "evaluation":"simulator truth recorded only outside policy/runtime",
        "legacy_autopilot":"disabled and trapped during runtime",
        "limitations":"raw visual control has no validated transport skill; this is not an optimality study",
        "warehouse_layout":"arena", "zone_side_m":.82, "static_obstacles":True}
    (out/"design.json").write_text(json.dumps(design,ensure_ascii=False,indent=2))
    results=[]
    for seed in seeds:
        for repeat in range(repeats):
            for mode in modes:
                run=out/f"{seed}-camera-{repeat}-{mode}"
                run.mkdir()
                world=video=None
                try:
                    if signature()!=source_hash:
                        raise RuntimeError("SOURCE_CHANGED_DURING_COMPARISON")
                    world=MultiMasterPiProductionV2(warehouse_layout="arena",seed=seed,render=True)
                    # Physical zone paint must be visible to robot cameras.
                    # Group 4 also contains viewer decorations; expose only paint.
                    import mujoco
                    for zone in ("a","b","c"):
                        gid=mujoco.mj_name2id(world.model,mujoco.mjtObj.mjOBJ_GEOM,"warehouse_zone_"+zone)
                        if gid<0:
                            raise RuntimeError("VISIBLE_DESTINATION_MARKING_MISSING")
                        world.model.geom_group[gid]=0
                    before=static_geometry_snapshot(world)
                    (run/"layout-evaluator-only.json").write_text(json.dumps(layout_record(world),indent=2))
                    if getattr(world,"_mixed_engine",None) or getattr(world,"_warehouse_crew",None):
                        raise RuntimeError("AUTOPILOT_ACTIVE")
                    if record:
                        from scripts.record_parallel_warehouse import CrewVideo
                        video=CrewVideo(world,run/"motion-1x.mp4")
                        video.capture(force=True)
                    ports={r:CameraRobotPort(world,r) for r in ROBOTS}
                    planners={r:CameraPlanner(r,GeminiProxyCompleter(max_tokens=768,timeout=30)) for r in ROBOTS}
                    def step():
                        # Plant dynamics can read its own physical state. No derived
                        # state is returned to robot ports or policy code.
                        target=float(world.data.time)+.02
                        while float(world.data.time)<target-1e-9:
                            for port in ports.values():
                                port.tick(float(world.data.time))
                            world._physics_step_for(world.robot("r1"))
                    with ExitStack() as guards:
                        forbidden=("warehouse_state", "act", "_warehouse_navigation_observations",
                                   "_warehouse_terrain_observations", "_sync_warehouse_goal_specs")
                        for method in forbidden:
                            if hasattr(world,method):
                                guards.enter_context(patch.object(world,method,side_effect=RuntimeError("ORACLE_CONTROL_FORBIDDEN:"+method)))
                        result=run_camera_episode(ports,planners,clock=lambda:float(world.data.time),
                            step=step,output=run,destination=world.warehouse_arena.destination_zone,
                            mode=mode,timeout_s=timeout,max_calls=max_calls,
                            capture_video=video.capture if video else None)
                    # Evaluation is performed after all planners stop. It never
                    # becomes actor feedback, a completion hint, or a route input.
                    state=world.warehouse_state()
                    after=static_geometry_snapshot(world)
                    result.update(success=bool(state["success"]) and before==after,
                        delivered_ids=state["delivered_ids"], total_count=state["total_count"],
                        static_geometry_unchanged=before==after,
                        oracle_guards=list(forbidden), source_sha256=source_hash)
                    (run/"evaluation-only.json").write_text(json.dumps(state,ensure_ascii=False,indent=2))
                except Exception as exc:
                    result={"success":False,"reason":str(exc),"source_sha256":source_hash}
                finally:
                    if video:
                        video.close()
                    if world:
                        world.close()
                result.update(seed=seed,mode=mode,repeat=repeat,run_dir=str(run))
                (run/"result.json").write_text(json.dumps(result,ensure_ascii=False,indent=2))
                results.append(result)
                (out/"results.json").write_text(json.dumps(results,ensure_ascii=False,indent=2))
                print(json.dumps(result,ensure_ascii=False),flush=True)
    return results


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output",required=True)
    p.add_argument("--seeds",default="41,58,73")
    p.add_argument("--modes",default="natural")
    p.add_argument("--repeats",type=int,default=1)
    p.add_argument("--timeout",type=float,default=90.)
    p.add_argument("--max-calls",type=int,default=24)
    p.add_argument("--record",action="store_true")
    a=p.parse_args()
    from harness.coela_modules import MODES
    if set(a.modes.split(","))-set(MODES) or a.repeats<1 or a.max_calls<1 or a.timeout<=0:
        p.error("invalid evaluation options")
    evaluate(output=a.output,seeds=[int(s) for s in a.seeds.split(",")],modes=a.modes.split(","),
             repeats=a.repeats,timeout=a.timeout,max_calls=a.max_calls,record=a.record)


if __name__=="__main__":
    main()
