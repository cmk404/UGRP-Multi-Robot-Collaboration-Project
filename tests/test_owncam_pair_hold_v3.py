"""Pair hold check v3 (whole-view beam/band mask IoU) on recorded own frames."""
import base64
from pathlib import Path

from harness import owncam_pair_beam as ob
from harness import owncam_pair_hold_v3 as hv3

FIX = Path(__file__).resolve().parent / 'fixtures' / 'owncam_pair_v3'


def _b64(name):
    return base64.b64encode((FIX / name).read_bytes()).decode()


def test_626_false_alarm_frame_is_held_for_v3_but_not_for_v1():
    lift, carry = _b64('lift_626_r1_00078.jpg'), _b64('carry_626_r1_00081.jpg')
    anchor = hv3.hold_view_mask(lift)
    assert hv3.hold_iou(anchor, carry) >= .95                       # GT: both grips held, tilt 0.81 deg
    v1_ratio = ob.signature_fraction(ob.held_signature(carry)) / ob.signature_fraction(ob.held_signature(lift))
    assert v1_ratio < .5                                            # the v1 lime ratio that raised the alarm


def test_deliberate_drop_frame_is_not_held():
    lift, dropped = _b64('lift_611drop_r1_00059.jpg'), _b64('dropped_611drop_r1_00070.jpg')
    anchor = hv3.hold_view_mask(lift)
    assert hv3.hold_iou(anchor, lift) == 1.
    assert hv3.hold_iou(anchor, dropped) < hv3.HOLD_MIN_IOU - .2      # gripper opened by the experimenter


def test_threshold_constants():
    assert hv3.HOLD_MIN_IOU == .70 and hv3.LOST_FRAMES == 2
