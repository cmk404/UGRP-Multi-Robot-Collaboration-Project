"""Single source of truth for the production remote MuJoCo worker bundle.

All GPU providers must execute the same Dynamics V2 + migrated REAL controller
stack. Provider-specific launchers may differ, but they must not carry their own
independent file lists because that silently creates different simulators.
"""
from __future__ import annotations

BUNDLE_FILES = (
    "sim/masterpi_dynamics_v2.py",
    "sim/masterpi_geometry.py",
    "sim/masterpi_camera_profile.py",
    "sim/calibration_schema.py",
    "sim/worker_contract.py",
    "sim/masterpi_dynamics_calibration.json",
    "sim/masterpi_production_v2.py",
    "sim/multi_masterpi_production.py",
    "sim/cooperative_payload.py",
    "sim/adaptive_warehouse.py",
    "sim/warehouse_mission.py",
    "sim/warehouse_research.py",
    "sim/warehouse_crew.py",
    "sim/mixed_warehouse.py",
    "sim/solo_cargo_task.py",
    "sim/wrist_load_sensor.py",
    "harness/mixed_warehouse_protocol.py",
    "harness/task_recovery.py",
    "sim/assets/warehouse_tags/small_box_01.png",
    "sim/assets/warehouse_tags/small_box_02.png",
    "sim/crew_navigation.py",
    "sim/crew_motion_metrics.py",
    "sim/warehouse_observation.py",
    "sim/assets/warehouse_tags/oak_plank_01.png",
    "sim/assets/warehouse_tags/steel_pipe_01.png",
    "sim/assets/warehouse_tags/wood_crate_01.png",
    "harness/warehouse_protocol.py",
    "sim/team_layout.py",
    "sim/self_observer.py",
    "sim/masterpi_training_env_v2.py",
    "sim/real_stack_adapter.py",
    "sim/masterpi_scene.xml",
    "sim/__init__.py",
    "harness/real_geometry.py",
    "scripts/run_mujoco_ws_worker.py",
    "scripts/verify_training_grasp.py",
    "scripts/masterpi_control.py",
    # Required at runtime by MasterPiProductionV2 primitive dispatch. Omitting
    # this file makes move/strafe/turn fail remotely with ModuleNotFoundError,
    # leaving both physics state and browser camera frames visually frozen.
    "scripts/robot_actions.py",
    "scripts/__init__.py",
    "scripts/red_block/__init__.py",
    "scripts/red_block/approach.py",
    "scripts/red_block/approach_precision_fallback.py",
    "scripts/red_block/camera.py",
    "scripts/red_block/carry.py",
    "scripts/red_block/carry_handoff.py",
    "scripts/red_block/deploy.py",
    "scripts/red_block/fetch.py",
    "scripts/red_block/geometry.py",
    "scripts/red_block/gaze_hold.py",
    "scripts/red_block/near_look_handoff.py",
    "scripts/red_block/physical_state_machine_reference.py",
    "scripts/red_block/pick.py",
    "scripts/red_block/pick_precision_fallback.py",
    "scripts/red_block/pickup_strategy.py",
    "scripts/red_block/place.py",
    "scripts/red_block/put_down.py",
    "scripts/red_block/plan.py",
    "scripts/red_block/poses.py",
    "scripts/red_block/precision_handoff.py",
    "scripts/red_block/primitive.py",
    "scripts/red_block/recorder.py",
    "scripts/red_block/remote_watchdog.py",
    "scripts/red_block/robot.py",
    "scripts/red_block/search.py",
    "scripts/red_block/task_runner.py",
    "scripts/red_block/track.py",
)

# Pin the production worker to the exact package set verified on the active
# Azure A10 worker on 2026-08-31. Recovery must reproduce a known runtime, not
# silently upgrade core simulation/transport packages on every redeploy.
REQUIREMENTS = (
    "mujoco==3.12.0",
    "numpy==2.5.2",
    "pillow==12.3.0",
    "websockets==17.1",
    "opencv-python-headless==5.0.0.93",
    "gymnasium==1.3.0",
)
