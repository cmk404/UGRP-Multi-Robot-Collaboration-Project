from __future__ import annotations
import json, math, tempfile, unittest
from pathlib import Path

import mujoco

from sim.calibration_schema import REQUIRED_VALIDATED_PARAMETERS
from sim.masterpi_dynamics_v2 import MasterPiDynamicsV2
from sim.real_stack_adapter import MigratedRealStack
from scripts.benchmarks.masterpi_servo_calibration_plan import make_plan as make_servo_plan
from scripts.benchmarks.fit_masterpi_servo import predict as servo_predict, run as run_servo_fit
from scripts.benchmarks.masterpi_hand_eye_plan import make_plan as make_handeye_plan
from scripts.benchmarks.fit_masterpi_hand_eye import project as handeye_project, run as run_handeye_fit
from scripts.benchmarks.validate_masterpi_digital_twin import validate, promote


class CompleteTwinPlantTests(unittest.TestCase):
    def test_calibrated_hardware_changes_actual_mujoco_plant(self):
        hw={
            'wheel_radius_m':.041,'wheelbase_m':.12,'track_m':.15,
            'block_mass_kg':.08,'block_floor_friction':1.7,
            'gripper_position_kp':700.,'gripper_finger_friction':4.2,
            'camera_pitch_offset_deg':4.,
        }
        w=MasterPiDynamicsV2(render=False,hardware=hw,use_calibration_manifest=False)
        try:
            flb=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_BODY,'wheel_fl_body')
            flg=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'wheel_fl')
            block=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'red_block_geom')
            finger=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'left_finger')
            self.assertAlmostEqual(float(w.model.body_pos[flb][0]),.06,places=6)
            self.assertAlmostEqual(float(w.model.body_pos[flb][1]),.075,places=6)
            self.assertAlmostEqual(float(w.model.geom_size[flg][0]),.041,places=6)
            block_body=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_BODY,'red_block')
            self.assertAlmostEqual(float(w.model.body_mass[block_body]),.08,places=6)
            self.assertAlmostEqual(float(w.model.geom_friction[block][0]),1.7,places=6)
            self.assertAlmostEqual(float(w.model.geom_friction[finger][0]),4.2,places=6)
            self.assertAlmostEqual(float(w.physical_params['camera_pitch_offset_deg']),4.)
        finally: w.close()

    def test_servo_deadband_and_rate_are_physical_not_metadata_only(self):
        w=MasterPiDynamicsV2(render=False,hardware={'servo_deadband_pwm':20,'servo_rate_pwm_per_s':100},use_calibration_manifest=False)
        try:
            stack=MigratedRealStack(w)
            stack.robot._queue_servo_motion(6,1510,.1)
            motion=stack.robot._servo_motions[6]
            self.assertEqual(motion[1],1500.0)
            stack.robot._servo_motions.clear()
            stack.robot._queue_servo_motion(6,1700,.1)
            motion=stack.robot._servo_motions[6]
            self.assertEqual(motion[1],1700.0)
            self.assertAlmostEqual(motion[3],2.0,places=6)
        finally: w.close()


class CompleteTwinFittersTests(unittest.TestCase):
    def _manifest(self, path:Path):
        obj={'validated':False,'validated_at':None,'parameters':{},'sources':{},'results':{
            'servo_fit_trials':0,'servo_held_out_trials':0,'servo_endpoint_mae_deg':None,'servo_settle_time_relative_error':None,
            'hand_eye_fit_trials':0,'hand_eye_held_out_trials':0,'hand_eye_position_mae_m':None,
        },'acceptance':{},'validation_provenance':None}
        path.write_text(json.dumps(obj))

    def test_servo_fitter_recovers_external_measurement_model(self):
        with tempfile.TemporaryDirectory() as td:
            td=Path(td); trials=td/'servo.jsonl'; manifest=td/'m.json'; self._manifest(manifest)
            rows=make_servo_plan(); deadband=18.; rate=820.
            for r in rows:
                endpoint,settle=servo_predict(r,deadband,rate)
                r['measured_delta_deg']=endpoint; r['settle_time_s']=settle; r['measurement_source']='external_encoder_video'
            trials.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
            out=run_servo_fit(trials,manifest,False)
            self.assertLessEqual(abs(out['parameters']['servo_deadband_pwm']-deadband),3)
            self.assertLess(abs(out['parameters']['servo_rate_pwm_per_s']-rate)/rate,.08)
            self.assertLess(out['holdout']['servo_endpoint_mae_deg'],.2)

    def test_hand_eye_fitter_recovers_physical_projection(self):
        with tempfile.TemporaryDirectory() as td:
            td=Path(td); trials=td/'he.jsonl'; manifest=td/'m.json'; self._manifest(manifest)
            truth={'camera_link_cm':7.55,'camera_z_offset_cm':2.9,'camera_pitch_offset_deg':1.1,'servo6_center_pwm':1516.0}
            rows=make_handeye_plan()
            # Build plausible observations first; ground truth coordinates are generated
            # by the known physical projection only for fitter unit testing.
            for i,r in enumerate(rows):
                r['nx']=.42 + .16*((i%5)/4); r['bottom_ny']=.72 + .08*((i%3)/2)
                pos=handeye_project(r,truth)
                if pos is None:
                    r['bottom_ny']=.82; pos=handeye_project(r,truth)
                self.assertIsNotNone(pos)
                r['true_x_m']=float(pos[0]); r['true_y_m']=float(pos[1]); r['measurement_source']='external_floor_grid_camera'
            trials.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
            out=run_handeye_fit(trials,manifest,False)
            self.assertLess(out['holdout']['hand_eye_position_mae_m'],.006)


class CompleteTwinPromotionGateTests(unittest.TestCase):
    def _make_complete_evidence(self,root:Path,source='external_overhead_camera'):
        static=root/'static.json'; chassis=root/'ch.jsonl'; servo=root/'sv.jsonl'; he=root/'he.jsonl'; task=root/'task.jsonl'; manifest=root/'manifest.json'
        static_obj={'hardware_unit':'ugrp1','measurement_source':source,'wheel_radius_m':.034,'wheelbase_m':.104,'track_m':.134,'block_mass_kg':.05,'block_floor_friction':1.0}
        static.write_text(json.dumps(static_obj))
        def dump(path,rows): path.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
        dump(chassis,[{'trial_id':f'c{i}','split':'fit' if i<72 else 'holdout','dx_m':.1,'dy_m':0.,'dyaw_deg':0.,'peak_speed_mps':.2,'stop_distance_m':.01,'measurement_source':source} for i in range(108)])
        dump(servo,[{'trial_id':f's{i}','split':'fit' if i<40 else 'holdout','servo':3+(i%4),'measured_delta_deg':9.,'settle_time_s':.2,'measurement_source':source} for i in range(60)])
        dump(he,[{'trial_id':f'h{i}','split':'fit' if i<12 else 'holdout','nx':.5,'bottom_ny':.8,'true_x_m':.2,'true_y_m':0.,'measurement_source':source} for i in range(24)])
        dump(task,[{'trial_id':f't{i}','split':'fit' if i<12 else 'holdout','red_x_m':.45,'red_y_m':0.,'red_yaw_deg':0.,'physical_success':True,'measurement_source':source} for i in range(32)])
        params={k:1.0 for k in REQUIRED_VALIDATED_PARAMETERS}; params.update(static_obj)
        params['servo6_center_pwm']=1500.; params['camera_link_cm']=7.; params['camera_z_offset_cm']=2.5; params['camera_pitch_offset_deg']=0.; params['servo_rate_pwm_per_s']=800.; params['servo_deadband_pwm']=10.; params['gripper_position_kp']=450.; params['gripper_finger_friction']=3.4
        acc={'held_out_translation_endpoint_mae_m_max':.025,'held_out_yaw_endpoint_mae_deg_max':5.,'held_out_peak_speed_relative_error_max':.15,'held_out_stop_distance_mae_m_max':.02,'held_out_servo_endpoint_mae_deg_max':3.,'held_out_servo_settle_time_relative_error_max':.2,'held_out_hand_eye_position_mae_m_max':.02,'grasp_success_rate_gap_max':.15,'task_outcome_disagreement_rate_max':.2}
        results={'fit_trials':72,'held_out_trials':36,'servo_fit_trials':40,'servo_held_out_trials':20,'hand_eye_fit_trials':12,'hand_eye_held_out_trials':12,'task_fit_trials':12,'task_held_out_trials':20,'translation_endpoint_mae_m':.01,'yaw_endpoint_mae_deg':2.,'peak_speed_relative_error':.1,'stop_distance_mae_m':.01,'servo_endpoint_mae_deg':1.,'servo_settle_time_relative_error':.1,'hand_eye_position_mae_m':.01,'grasp_success_rate_gap':.05,'task_outcome_disagreement_rate':.1}
        manifest.write_text(json.dumps({'validated':False,'validated_at':None,'parameters':params,'results':results,'acceptance':acc,'validation_provenance':None}))
        return manifest,static,chassis,servo,he,task

    def test_only_complete_physical_evidence_can_promote(self):
        with tempfile.TemporaryDirectory() as td:
            paths=self._make_complete_evidence(Path(td)); report=validate(*paths)
            self.assertTrue(report['pass'],report['reasons'])
            promote(report,paths[0]); obj=json.loads(paths[0].read_text())
            self.assertTrue(obj['validated']); self.assertTrue(obj['validation_provenance']['dataset_sha256'])

    def test_sim_source_is_rejected_even_if_numbers_pass(self):
        with tempfile.TemporaryDirectory() as td:
            paths=self._make_complete_evidence(Path(td),source='mujoco_sim_generated')
            report=validate(*paths); self.assertFalse(report['pass']); self.assertFalse(report['checks']['static_physical_evidence'])


if __name__=='__main__': unittest.main()
