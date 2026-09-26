"""M2 door v3: low-light lift co-motion check (recorded own frames), runner flags, jaw intervention port."""
import base64
import json
import subprocess
import sys
from pathlib import Path

from harness import owncam_pair_beam as ob
from harness import owncam_pair_beam_v2 as ob2
from harness import owncam_pair_lift_v3 as lv3

ROOT = Path(__file__).resolve().parents[1]
FIX = Path(__file__).resolve().parent / 'fixtures' / 'm2_pair_door_v3'


def _b64(name):
    return base64.b64encode((FIX / name).read_bytes()).decode()


def test_824_dark_floor_lift_is_held_for_v3_not_v2():
    # stage 2b ed15489 seed 824, r2 third grasp at base x 2.84 (east of the door); GT: beam 0.061 m, 5.48 N
    g, li = _b64('grasp_824_r2_00757.jpg'), _b64('lift_824_r2_00759.jpg')
    assert ob.signature_iou(ob2.co_motion_signature(g), ob2.co_motion_signature(li)) < .05
    assert lv3.lift_iou(g, li) >= .80


def test_bright_floor_lift_still_held():
    g, li = _b64('grasp_824_r2_00619.jpg'), _b64('lift_824_r2_00621.jpg')      # first grasp, west floor
    assert lv3.lift_iou(g, li) >= .80


def test_not_the_beam_is_rejected():
    # a grasp anchor against a floor view (approach frame) is not co-motion
    assert lv3.lift_iou(_b64('grasp_824_r2_00757.jpg'), _b64('approach_824_r2_00010.jpg')) < lv3.HOLD_MIN_IOU


def test_band_not_admitted_by_low_brightness_floor():
    frame = ob.decode(_b64('grasp_824_r2_00757.jpg'))
    import cv2
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    band = (hsv[..., 2] <= ob2.BAND_V_MAX) & (hsv[..., 1] < ob2.BAND_S_MAX)
    assert not (lv3.beam_colour_mask_low(frame) & band).any()
    assert lv3.HOLD_MIN_IOU == .45 and lv3.BEAM_V_MIN_LOW == 60


def test_runner_status_channel_default_on_and_help_text():
    out = subprocess.run([sys.executable, str(ROOT / 'scripts' / 'run_m2_pair.py'), '--help'],
                         capture_output=True, text=True, check=True).stdout
    assert 'pending' not in out.lower() and 'candidate' not in out.lower()
    assert '--status-channel {on,off}' in out and 'v3' in out and '--inject-open-at-lift' in out
    sys.path.insert(0, str(ROOT))
    import scripts.run_m2_pair as rm
    import inspect
    src = inspect.getsource(rm.main)
    assert "'--status-channel', choices=('on', 'off'), default='on'" in src


def test_stage2c_scenarios_match_generator_output():
    import scripts.run_m2_pair as rm
    reg = json.loads((ROOT / 'experiments' / '2026-09-26-zone-m2-pair' / 'stage2c_test_scenarios.json').read_text())
    assert sorted(int(k) for k in reg) == list(rm.STAGE2C_TEST_SEEDS) == list(range(831, 839))
    for k, v in reg.items():
        sc = rm.DOOR_SCENARIOS[int(k)]
        assert list(sc['beam']) == v['beam'] and {r: list(x) for r, x in sc['start'].items()} == v['start']


def test_jaw_override_port_replaces_only_servo1_after_activation():
    import scripts.run_m2_pair as rm

    class Fake:
        def __init__(self):
            self.got = []

        def apply(self, action, t):
            self.got.append(dict(action))

        def hold(self, t):
            self.got.append({'kind': 'hold'})

    fake = Fake()
    port = rm.JawOverridePort(fake, 2000)
    port.apply({'kind': 'arm', 'servo_id': 1, 'pulse': 1500}, 0.)
    assert fake.got[-1]['pulse'] == 1500 and port.replaced == 0
    port.activate(1.)
    assert fake.got[-1] == {'kind': 'arm', 'servo_id': 1, 'pulse': 2000} and port.active_since == 1.
    port.apply({'kind': 'arm', 'servo_id': 1, 'pulse': 1500}, 1.1)
    port.apply({'kind': 'arm', 'servo_id': 3, 'pulse': 1400}, 1.1)
    assert fake.got[-2]['pulse'] == 2000 and fake.got[-1]['pulse'] == 1400 and port.replaced == 1
    port.hold(2.)
    assert fake.got[-1] == {'kind': 'hold'}


def test_door_v3_takes_every_door_v2_branch():
    # dev13-813 at 9b0ca56: the depot-slot rule was gated on door_version == 'v2' only, so v3 ran without
    # it (crossing approach paths, 105 robot-robot contact samples). v3 must be v2 + the lift check.
    src = (ROOT / 'scripts' / 'run_m2_pair.py').read_text()
    assert "door_version == 'v2'" not in src and "version == 'v2'" not in src
    assert "a.door_version in ('v2', 'v3')" in src
