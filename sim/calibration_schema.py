"""Dependency-free schema for parameters required by the calibrated MasterPi twin."""
CALIBRATABLE_DYNAMICS = (
    'motor_time_constant_s',
    'max_forward_force_n',
    'max_lateral_force_n',
    'max_yaw_torque_nm',
    'linear_damping_n_per_mps',
    'yaw_damping_nm_per_radps',
    'stop_linear_damping_n_per_mps',
    'stop_yaw_damping_nm_per_radps',
)
CALIBRATABLE_HARDWARE = (
    'wheel_radius_m',
    'wheelbase_m',
    'track_m',
    'servo_deadband_pwm',
    'servo_rate_pwm_per_s',
    'camera_link_cm',
    'camera_z_offset_cm',
    'camera_pitch_offset_deg',
    'servo6_center_pwm',
    'block_mass_kg',
    'block_floor_friction',
    'gripper_position_kp',
    'gripper_finger_friction',
)
REQUIRED_VALIDATED_PARAMETERS = CALIBRATABLE_DYNAMICS + CALIBRATABLE_HARDWARE
