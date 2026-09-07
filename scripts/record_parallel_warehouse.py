"""Record actual independent crew motion, headings, and robot camera views."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
import subprocess
from unittest.mock import patch

import cv2
import mujoco
import numpy as np

from harness.gemini_proxy import GeminiProxyCompleter
from harness.warehouse_runtime import EpisodeBudget, LLMPolicy, LocalEnvironment, RulePolicy, run_episode
from sim.crew_motion_metrics import CrewMotionRecorder
from sim.multi_masterpi_production import MultiMasterPiProductionV2
from sim.warehouse_mission import WAREHOUSE_ZONES


def _scene_extents(world):
    zones = getattr(world, "warehouse_zones", None) or WAREHOUSE_ZONES
    points = []
    for zone in zones.values():
        center, half = np.asarray(zone.center_xy), np.asarray(zone.half_extents_xy)
        points.extend((center-half, center+half))
    points.extend(np.asarray(world.robot(rid).base_xyz()[:2]) for rid in world.robot_ids)
    points = np.asarray(points, dtype=float)
    return points.min(axis=0), points.max(axis=0)


def _presentation_camera_pose(world):
    low, high = _scene_extents(world)
    center = (low+high)/2
    span = max(float(np.ptp((low[0], high[0]))), float(np.ptp((low[1], high[1]))), 2.0)
    position = np.array((center[0], center[1]-.60*span, 1.18*span))
    look_at = np.array((center[0], center[1], 0.0))
    return position, look_at


class CrewVideo:
    def __init__(self, world, path):
        self.world, self.path = world, Path(path)
        self.start = float(world.data.time)
        self.next_frame = self.start
        self.last_cameras = -100.
        self.cameras = {}
        self.frames = 0
        self.process = subprocess.Popen([
            "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
            "-s", "1280x720", "-r", "10", "-i", "-", "-an", "-c:v", "libx264",
            "-preset", "veryfast", "-crf", "21", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path),
        ], stdin=subprocess.PIPE)
        self.camera_id = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_CAMERA, "cctv_warehouse")
        # Presentation-only view. Robot sensor cameras and physics poses stay unchanged.
        pos, look_at = _presentation_camera_pose(world)
        forward = look_at-pos
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, (0., 0., 1.)); right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, np.column_stack((right, up, -forward)).ravel())
        world.model.cam_pos[self.camera_id] = pos
        world.model.cam_quat[self.camera_id] = quat
        world.model.cam_fovy[self.camera_id] = 72.
        mujoco.mj_forward(world.model, world.data)

    def project(self, point):
        world = self.world
        rotation = world.data.cam_xmat[self.camera_id].reshape(3, 3)
        xyz = np.asarray(point)-world.data.cam_xpos[self.camera_id]
        x, y, z = xyz @ rotation
        if z >= -.05:
            return None
        focal = 720/(2*math.tan(math.radians(72)/2))
        return int(480+x/-z*focal), int(360-y/-z*focal)

    def capture(self, force=False):
        now = float(self.world.data.time)
        if not force and now < self.next_frame:
            return
        jpeg = self.world.render_team_jpeg(camera="cctv_warehouse", quality=90)
        base = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        canvas = np.zeros((720, 1280, 3), np.uint8)
        canvas[:, :960] = cv2.resize(base, (960, 720))
        canvas[:, 960:] = (29, 33, 39)
        crew = getattr(self.world, "_warehouse_crew", None)
        zones = getattr(self.world, "warehouse_zones", None) or WAREHOUSE_ZONES
        arena = getattr(self.world, "warehouse_arena", None)
        if isinstance(arena, dict):
            source = str(arena.get("source_zone", "A"))
            destination = str(arena.get("destination_zone", "B"))
        else:
            source = str(getattr(arena, "source_zone", "A"))
            destination = str(getattr(arena, "destination_zone", "B"))
        for zone_id, zone in zones.items():
            label_at = self.project((*zone.center_xy, .035))
            if label_at:
                cv2.putText(canvas, zone_id, (label_at[0]-8, label_at[1]-12),
                            cv2.FONT_HERSHEY_SIMPLEX, .48, (245, 245, 245), 2)
        if source in zones and destination in zones:
            start = self.project((*zones[source].center_xy, .06))
            end = self.project((*zones[destination].center_xy, .06))
            if start and end:
                cv2.arrowedLine(canvas, start, end, (245, 245, 245), 2, tipLength=.08)
        if now-self.last_cameras >= .50 or not self.cameras:
            for rid in self.world.robot_ids:
                self.cameras[rid] = cv2.cvtColor(self.world.render_rgb(robot_id=rid), cv2.COLOR_RGB2BGR)
            self.last_cameras = now
        colors = ((40, 220, 255), (255, 165, 60), (120, 100, 255))
        for index, rid in enumerate(self.world.robot_ids):
            robot = self.world.robot(rid)
            point = np.asarray(robot.base_xyz(), dtype=float)+np.array((0, 0, .16))
            yaw = float(robot.base_rpy()[2])
            end = point+np.array((.28*math.cos(yaw), .28*math.sin(yaw), 0))
            a, b = self.project(point), self.project(end)
            if a and b:
                cv2.arrowedLine(canvas, a, b, colors[index], 3, tipLength=.30)
                cv2.putText(canvas, rid.upper(), (a[0]-12,a[1]-10), cv2.FONT_HERSHEY_SIMPLEX, .48, colors[index], 2)
            y = index*240
            activity = crew.activities.get(rid, "ready") if crew else "observe / decide"
            mixed = getattr(self.world, "_mixed_engine", None)
            if mixed is not None and rid in mixed.solo_tasks:
                activity = "solo:"+mixed.solo_tasks[rid].state
            cv2.putText(canvas, f"{rid.upper()}  {activity}", (972,y+24), cv2.FONT_HERSHEY_SIMPLEX,.48,colors[index],1)
            camera = cv2.resize(self.cameras[rid], (240,180))
            canvas[y+36:y+216,1000:1240] = camera
            cv2.putText(canvas,f"HEADING {math.degrees(yaw):+.0f} deg | WRIST CAMERA",(972,y+232),cv2.FONT_HERSHEY_SIMPLEX,.37,(210,215,220),1)
        cv2.rectangle(canvas,(0,0),(960,52),(22,26,31),-1)
        cv2.putText(canvas,f"THREE ROBOTS | {source} -> {destination} | INDEPENDENT MOTION | SIM 1x",(14,22),cv2.FONT_HERSHEY_SIMPLEX,.56,(240,245,250),1)
        phase = self.world.warehouse_trace[-1].get("phase", "ready") if self.world.warehouse_trace else "ready"
        cv2.putText(canvas,f"t={now-self.start:.1f}s | {phase}",(14,43),cv2.FONT_HERSHEY_SIMPLEX,.43,(160,215,240),1)
        while self.next_frame <= now+1e-9:
            self.process.stdin.write(canvas.tobytes())
            self.frames += 1
            self.next_frame += .10
        if force:
            cv2.imwrite(str(self.path.with_suffix(".png")), canvas)

    def close(self):
        self.capture(force=True)
        self.process.stdin.close()
        if self.process.wait(timeout=30):
            raise RuntimeError("video encoder failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed",type=int,default=11)
    parser.add_argument("--condition",choices=("rule","llm_peer_comm"),default="llm_peer_comm")
    parser.add_argument("--selector",default="all",choices=("all","pipe","plank","crate"))
    parser.add_argument("--output",required=True)
    args = parser.parse_args()
    out = Path(args.output).resolve(); out.mkdir(parents=True,exist_ok=False)
    world = MultiMasterPiProductionV2(seed=args.seed,width=640,height=480,render=True)
    video = CrewVideo(world,out/"parallel-forward-1x.mp4")
    poses = (out/"motion.jsonl").open("w")
    class RecordedEnvironment(LocalEnvironment):
        def request(self, rid, request):
            result = super().request(rid, request)
            if request["operation"] == "begin":
                world._crew_motion_recorder = CrewMotionRecorder(poses)
            return result
    try:
        video.capture(force=True);world.frame_callback=video.capture
        policies = {rid: RulePolicy(rid) if args.condition=="rule" else
                    LLMPolicy(rid,GeminiProxyCompleter(max_tokens=512,timeout=45)) for rid in world.robot_ids}
        with ExitStack() as guards:
            for robot in world.controllers.values():
                guards.enter_context(patch.object(robot,"set_base_pose_for_test",side_effect=AssertionError("POSE_RESET_FORBIDDEN")))
                guards.enter_context(patch.object(robot,"set_free_body_pose_for_reset",side_effect=AssertionError("BODY_RESET_FORBIDDEN")))
            result = run_episode(RecordedEnvironment(world),policies,condition=args.condition,seed=args.seed,
                                 selector=args.selector,budget=EpisodeBudget(max_rounds=12),journal_path=out/"episode.jsonl")
        result["motion"] = world._warehouse_crew_metrics
        result["pose_reset_calls"] = 0
        result["video_speed"] = "1x simulation time"
        (out/"result.json").write_text(json.dumps(result,ensure_ascii=False,indent=2))
        (out/"physics.json").write_text(json.dumps(world.warehouse_state(),indent=2))
        print(json.dumps(result,ensure_ascii=False),flush=True)
        return 0 if result["success"] else 1
    finally:
        world.frame_callback=None
        video.close();poses.close();world.close()


if __name__ == "__main__":
    raise SystemExit(main())
