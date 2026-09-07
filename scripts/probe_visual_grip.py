"""Record a camera-only attachment probe with a physically detached close box."""
from pathlib import Path
import argparse, base64, json, math

from harness.visual_attachment import compare_box_comotion
from harness.visual_box_skill import VisualBoxSkill
from sim.camera_robot_port import CameraRobotPort
from sim.multi_masterpi_production import MultiMasterPiProductionV2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True)
    ap.add_argument("--seed", type=int, default=41)
    args = ap.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
    world = MultiMasterPiProductionV2(warehouse_layout="arena", seed=args.seed,
                                      render=True, warehouse_cargo_ids=("small_box_01",))
    port = CameraRobotPort(world, "r1")
    skill = VisualBoxSkill()

    def step(seconds):
        until = float(world.data.time) + seconds
        while world.data.time < until:
            port.tick(float(world.data.time)); world._physics_step_for(world.robot("r1"))

    def pose(targets, obs):
        start = obs["actuator_state"]["servo_pulses"]
        duration = max(.25, max(abs(v-start[str(k)]) for k, v in targets.items()) / 600.)
        for sample in range(1, max(5, math.ceil(duration/.05))+1):
            u = sample/max(5, math.ceil(duration/.05)); u = u*u*(3-2*u)
            for servo, end in targets.items():
                pulse = round(start[str(servo)] + u*(end-start[str(servo)]))
                raw = {"kind":"look", "pan_pulse":pulse} if servo == 6 else {"kind":"arm", "servo_id":servo, "pulse":pulse}
                port.apply(raw, float(world.data.time))
            step(duration/max(5, math.ceil(duration/.05)))
        step(.12)

    try:
        for _ in range(400):
            obs = port.capture(); phase = skill.phase; action = skill.decide(obs)
            # Stop before close: the target is physically at the gripper but
            # servo 1 remains open, so no attachment can have formed.
            if phase == "lower" and skill.phase == "close":
                break
            if action["kind"] == "drive":
                port.apply({"kind":"drive", "forward":action["fwd"], "turn":action["turn"], "duration_s":action["duration"]}, float(world.data.time)); step(action["duration"]+.2); port.stop()
            elif action["kind"] == "pose": pose(action["pulses"], obs)
            elif action["kind"] == "wait": step(max(.05, action["duration"]))
            else: raise RuntimeError(action)
        else: raise RuntimeError("failed to reach open-gripper close view")

        frames = {}
        for label, pan in (("anchor", 1500), ("left", 1560), ("right", 1440), ("home", 1500)):
            obs = port.capture(); pose({6: pan, 1: 2000}, obs); obs = port.capture()
            raw = base64.b64decode(obs["image"]); (out/f"{label}.jpg").write_bytes(raw)
            frames[label] = obs["image"]
        results = {label: compare_box_comotion(frames["anchor"], frames[label], camera_pan_delta_pwm=pan)
                   for label, pan in (("left",60), ("right",-60), ("home",0))}
        truth = world.warehouse_state()["cargo"]["small_box_01"]
        report = {"seed":args.seed, "controller_inputs":"own RGB + own PWM only",
                  "offline_truth":{"carriers":truth["carriers"], "height_m":truth["position"][2]},
                  "comparisons":results}
        (out/"report.json").write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps(report))
    finally:
        world.close()


if __name__ == "__main__": main()
