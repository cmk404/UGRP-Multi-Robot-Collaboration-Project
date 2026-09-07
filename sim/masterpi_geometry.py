"""Nominal physical geometry for the Hiwonder MasterPi platform.

Keep this separate from controller/FK calibration.  The simulator models the
hardware we can see/measure; the migrated REAL controller keeps its own legacy
kinematic assumptions in :mod:`harness.real_geometry`.  A mismatch between the
two is therefore visible in simulation instead of being hidden by construction.

Source classes:
- ``official``: manufacturer product specification/dimension drawing.
- ``controller``: dimensions already used by the physical pickup controller.
- ``photo_nominal``: conservative visual dimensions estimated from Hiwonder's
  assembly/product photos.  These are appearance/collision priors, not claimed
  metrology and remain replaceable by direct measurements.
"""
from __future__ import annotations

# Manufacturer specification / dimension drawing.
OFFICIAL_TOTAL_LENGTH_M = 0.185
OFFICIAL_TOTAL_WIDTH_M = 0.162
OFFICIAL_TOTAL_HEIGHT_M = 0.343
OFFICIAL_TOTAL_MASS_KG = 1.10
OFFICIAL_WHEEL_DIAMETER_M = 0.065

# MasterPi's packing list uses Hiwonder's orange mecanum wheels; the matching
# manufacturer wheel is 65 mm diameter x 31 mm wide.  The product dimension
# drawing gives the outside robot envelope, so derive the wheel-centre geometry
# from those measured/official dimensions rather than a photo-width estimate.
NOMINAL_WHEEL_RADIUS_M = OFFICIAL_WHEEL_DIAMETER_M / 2.0
NOMINAL_WHEEL_WIDTH_M = 0.031
NOMINAL_WHEELBASE_M = OFFICIAL_TOTAL_LENGTH_M - OFFICIAL_WHEEL_DIAMETER_M
NOMINAL_TRACK_M = OFFICIAL_TOTAL_WIDTH_M - NOMINAL_WHEEL_WIDTH_M

# Chassis/electronics silhouette derived from Hiwonder's dimension drawing and
# assembly sequence.  The lower chassis is an inverted-U sheet-metal tray: one
# horizontal top sheet, down-turned skirts, and an open underside around the TT
# motors.  It is not a closed rectangular box.
#
# The 185 mm outside length equals the 120 mm axle spacing plus one 65 mm wheel
# diameter, so the tray ends near the axle planes. Across the robot the official
# 162 mm envelope minus two 31 mm wheels leaves 100 mm between inner wheel faces;
# use a 96 mm tray to preserve the visible side clearance in the assembly photos.
NOMINAL_DECK_LENGTH_M = NOMINAL_WHEELBASE_M
NOMINAL_DECK_WIDTH_M = 0.096

# Hiwonder supplies M4x50 double-pass copper columns in step 1 and the top cover
# is screwed directly to those columns in step 6. Combined with the 101 mm body
# silhouette in the manufacturer drawing, this places the chassis top sheet at
# about 51 mm above the floor. The ~30 mm skirt below it is an assembly/drawing-
# constrained nominal value, not CAD-level metrology; physical measurement may
# refine it without changing the overall manufacturer envelope.
NOMINAL_LOWER_BODY_BOTTOM_FROM_FLOOR_M = 0.017
NOMINAL_LOWER_BODY_TOP_FROM_FLOOR_M = 0.047
NOMINAL_LOWER_BODY_HEIGHT_M = (
    NOMINAL_LOWER_BODY_TOP_FROM_FLOOR_M - NOMINAL_LOWER_BODY_BOTTOM_FROM_FLOOR_M
)
NOMINAL_DECK_TOP_FROM_FLOOR_M = 0.051
NOMINAL_BODY_TOP_FROM_FLOOR_M = 0.101
NOMINAL_CAGE_STANDOFF_HEIGHT_M = 0.050
NOMINAL_ULTRASONIC_X_M = 0.078

# Raspberry Pi / expansion-board stack. Full-size Pi 4/5 boards are about
# 85 x 56 mm and Hiwonder mounts the 85 mm dimension laterally. The 92 mm top
# cover therefore remains slightly narrower than the 96 mm lower tray.
NOMINAL_PI_CAGE_CENTER_X_M = -0.028
NOMINAL_PI_CAGE_HALF_LENGTH_M = 0.032
NOMINAL_PI_CAGE_HALF_WIDTH_M = 0.046
NOMINAL_PI_BOARD_HALF_LENGTH_M = 0.028
NOMINAL_PI_BOARD_HALF_WIDTH_M = 0.0425
NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M = NOMINAL_DECK_TOP_FROM_FLOOR_M
NOMINAL_PI_CAGE_TOP_FROM_FLOOR_M = (
    NOMINAL_PI_CAGE_BOTTOM_FROM_FLOOR_M + NOMINAL_CAGE_STANDOFF_HEIGHT_M
)
NOMINAL_PI_BOARD_Z_FROM_FLOOR_M = 0.060
NOMINAL_PI_HEATSINK_Z_FROM_FLOOR_M = 0.067
NOMINAL_EXPANSION_BOARD_Z_FROM_FLOOR_M = 0.074

# Controller-derived arm geometry. Hiwonder's straight-arm reference image,
# scaled by the official 65 mm wheel, shows that the 93 mm quantity is relative
# to the chassis/base reference near the wheel axle, not an absolute floor
# height.  Treating it as floor height was the main V2 arm-placement error.
CONTROLLER_BASE_TO_SHOULDER_M = 0.093
# Backward-compatible alias; the name is historical and should not be read as a
# floor-referenced height.
CONTROLLER_SHOULDER_AXIS_HEIGHT_M = CONTROLLER_BASE_TO_SHOULDER_M
CONTROLLER_UPPER_ARM_M = 0.065
CONTROLLER_FOREARM_M = 0.062
CONTROLLER_TOOL_M = 0.100
CONTROLLER_CAMERA_LINK_M = 0.070
CONTROLLER_CAMERA_WORLD_Z_OFFSET_M = 0.025

# Eye-in-hand presentation/perception mount anchored to a successful REAL
# capture on 2026-08-30.  At CAPTURE_ARM_POSE the physical 30 mm red cube was
# observed at ny=0.440 with a 175x175 px bounding box.  Fitting only the rigid
# wrist mount while retaining the 48 deg render VFOV gives the values below.
# These are SIM sensor-model anchors, not changes to the REAL controller FK.
REAL_CAPTURE_CAMERA_LINK_M = 0.0670
REAL_CAPTURE_CAMERA_LOCAL_Z_M = 0.0136
# Existing simulator convention: negative camera_pitch_offset_deg tips the
# optical axis downward relative to the wrist/tool axis.
REAL_CAPTURE_CAMERA_PITCH_OFFSET_DEG = -7.95

# Nominal yaw bearing location estimated from Hiwonder's side reference image.
# The same image gives visible pitch-axis heights of about 128/190/249 mm from
# the floor, consistent with wheel-axis + 93 mm and the 65/62 mm link spacings.
NOMINAL_YAW_AXIS_FROM_BASE_M = 0.0625
NOMINAL_SHOULDER_AXIS_FROM_BASE_M = CONTROLLER_BASE_TO_SHOULDER_M
NOMINAL_YAW_TO_SHOULDER_M = NOMINAL_SHOULDER_AXIS_FROM_BASE_M - NOMINAL_YAW_AXIS_FROM_BASE_M
NOMINAL_SHOULDER_FLOOR_M = NOMINAL_WHEEL_RADIUS_M + NOMINAL_SHOULDER_AXIS_FROM_BASE_M
NOMINAL_ELBOW_FLOOR_M = NOMINAL_SHOULDER_FLOOR_M + CONTROLLER_UPPER_ARM_M
NOMINAL_WRIST_FLOOR_M = NOMINAL_ELBOW_FLOOR_M + CONTROLLER_FOREARM_M

# Provenance for the photo-scale estimate (official straight-arm side image).
OFFICIAL_REFERENCE_WHEEL_PX = 149.0
OFFICIAL_REFERENCE_PX_PER_MM = OFFICIAL_REFERENCE_WHEEL_PX / 65.0
OFFICIAL_REFERENCE_JOINT_Y_PX = {"shoulder": 520.0, "elbow": 378.0, "wrist": 242.0}
OFFICIAL_REFERENCE_WHEEL_BOTTOM_Y_PX = 813.0

# Photo-derived visual envelope for the Hiwonder metal brackets / bus servos.
# Hiwonder LD-1501MG is approximately 40 x 20 x 40.5 mm.  The former
# photo-box prior was nearly 46-50 mm wide and made the arm/body look much
# bulkier than the physical MasterPi.  These are visual envelopes only; joint
# axes and collision capsules remain unchanged.
SERVO_BODY_HALF = (0.0200, 0.0100, 0.02025)
# The gripper uses the much smaller LFD-01M (22.3 x 12 x 23.2 mm).
MICRO_SERVO_BODY_HALF = (0.01115, 0.0060, 0.01160)
ARM_PLATE_HALF_WIDTH_M = 0.018
# Hiwonder's metal servo brackets are 2 mm aluminium: box size fields are
# half-extents, hence 1 mm here rather than the previous 3 mm (6 mm total).
ARM_PLATE_THICKNESS_M = 0.001
# Side-plate centreline: 20 mm servo body + small horn clearance + 2 mm plates.
ARM_PLATE_SIDE_OFFSET_M = 0.0125
ARM_HORN_SIDE_OFFSET_M = 0.0145
ARM_HORN_HALF_THICKNESS_M = 0.0015
ARM_PLATE_HALF_HEIGHT_M = 0.018
# The arm links are *open frame brackets*, not solid orange planks. Official
# assembly/product imagery shows two narrow rails around a large cut-out. Keep
# the material rail itself thin while placing the two rails apart vertically.
# This gives an approximately 21 mm outer link envelope without filling the
# cut-out with fake metal.
ARM_LINK_HALF_HEIGHT_M = 0.0032
WRIST_LINK_HALF_HEIGHT_M = 0.0030
ARM_LINK_RAIL_Z_OFFSET_M = 0.0095
WRIST_LINK_RAIL_Z_OFFSET_M = 0.0090
CAMERA_BODY_HALF = (0.017, 0.022, 0.014)
# MasterPi's actual contact surface is the slim orange rubber sleeve fitted to
# the end of a thin metal finger. Keep the physical contact proxy separate from
# the visible pad so simulator contact tuning never dictates the rendered shape.
# Dimensions remain photo-derived nominal values pending direct caliper/STP data.
FINGER_HALF_LENGTH_M = 0.014
FINGER_HALF_WIDTH_M = 0.0045
FINGER_HALF_HEIGHT_M = 0.0035
GRIPPER_PAD_HALF_LENGTH_M = 0.015
GRIPPER_PAD_HALF_WIDTH_M = 0.0040
GRIPPER_PAD_HALF_HEIGHT_M = 0.0022


def nominal_wheel_envelope() -> tuple[float, float]:
    """Return nominal outside wheel envelope (length, width) in metres."""
    return (
        NOMINAL_WHEELBASE_M + 2.0 * NOMINAL_WHEEL_RADIUS_M,
        NOMINAL_TRACK_M + NOMINAL_WHEEL_WIDTH_M,
    )
