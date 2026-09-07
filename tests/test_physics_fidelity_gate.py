import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class PhysicsFidelityGateTests(unittest.TestCase):
    def test_production_is_not_claimed_training_ready(self):
        from scripts.benchmarks.physics_fidelity import audit
        result = audit()
        self.assertFalse(result['training_ready'])
        failures = set(result['production']['critical_failures'])
        self.assertIn('base_6dof_dynamics', failures)
        self.assertIn('mecanum_wheel_dynamics', failures)
        self.assertIn('hardware_command_parity', failures)
        self.assertIn('camera_intrinsics_parity', failures)
        self.assertIn('robot_mass_parity', failures)
        self.assertIn('arm_geometry_parity', failures)

    def test_candidate_v2_passes_structure_but_stays_blocked_on_real_calibration(self):
        from scripts.benchmarks.physics_fidelity import audit
        result = audit()
        candidate = result['candidate_v2']
        self.assertFalse(candidate['training_ready'])
        failures = set(candidate['critical_failures'])
        self.assertEqual(failures, {'dynamics_calibrated_against_real'})
        passed = {c['id'] for c in candidate['checks'] if c['ok']}
        self.assertIn('base_6dof_dynamics', passed)
        self.assertIn('mecanum_wheel_structure', passed)
        self.assertIn('robot_mass_parity', passed)
        self.assertIn('arm_geometry_parity', passed)
        self.assertIn('camera_intrinsics_parity', passed)
        self.assertIn('hardware_command_parity', passed)

    def test_legacy_ppo_world_is_explicitly_non_transfer(self):
        from scripts.benchmarks.physics_fidelity import audit
        result = audit()
        self.assertFalse(result['legacy_ppo']['training_ready'])
        failures = set(result['legacy_ppo']['critical_failures'])
        self.assertIn('legacy_base_not_mocap', failures)
        self.assertIn('legacy_block_scale', failures)
        self.assertIn('legacy_arm_topology', failures)
        self.assertIn('legacy_observation_transferability', failures)

    def test_training_cli_blocks_before_legacy_import(self):
        proc = subprocess.run(
            [sys.executable, 'scripts/train_grasp_ppo.py', '--steps', '1'],
            cwd=ROOT,
            text=True,
            capture_output=True,
            timeout=10,
        )
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn('BLOCKED:', proc.stderr + proc.stdout)
        self.assertIn('--allow-legacy-physics', proc.stderr + proc.stdout)

    def test_evaluation_cli_blocks_by_default(self):
        proc = subprocess.run(
            [sys.executable, 'scripts/eval_grasp_ppo.py', '--episodes', '1'],
            cwd=ROOT,
            text=True,
            capture_output=True,
            timeout=10,
        )
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn('BLOCKED:', proc.stderr + proc.stdout)


if __name__ == '__main__':
    unittest.main()
