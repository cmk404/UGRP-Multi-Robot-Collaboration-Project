"""Run from the repository root: python -m examples.simulation_session.

For a native window use mjpython -m examples.simulation_session --viewer on Mac.
Replace the fixed raw command with your own RGB controller at the same boundary.
"""
import argparse
import time

from sim.session import Simulation
from sim.session_config import load_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    config = load_config("configs/simulation/local.json")
    with Simulation(config, render=True) as sim:
        viewer = sim.launch_viewer() if args.viewer else None
        for episode in range(2):
            sim.reset()
            # Use only observation for a controller; no evaluation_state().
            observation = sim.observe("r1")
            print(f"episode={episode}, RGB={observation['sha256']}")
            sim.apply("r1", {"kind": "drive", "forward": .12, "turn": 0, "duration_s": .5})
            for _ in range(round(1.0 / sim.timestep)):
                if viewer is not None and not viewer.is_running():
                    return
                sim.step()
                if viewer is not None:
                    sim.sync_viewer()
                    time.sleep(sim.timestep)


if __name__ == "__main__":
    main()
