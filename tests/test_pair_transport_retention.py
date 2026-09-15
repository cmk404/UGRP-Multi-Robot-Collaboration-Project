from pathlib import Path
import cv2
import pytest
from harness.pair_transport_vision import OwnCarryMonitor, GripUncertain
from scripts.pair_grasp_recovery import slip_requested

FIX=Path(__file__).parent/'fixtures/pair_grasp_retention'
def frame(name): return cv2.imread(str(FIX/name))

def test_still_image_and_one_frame_disturbance_do_not_trigger_slip():
    monitor=OwnCarryMonitor(slip_guard=True)
    start,drift=frame('baseline-start.jpg'),frame('baseline-drift-4s.jpg')
    for _ in range(30): monitor.observe(start)
    monitor.observe(drift)
    for _ in range(30): monitor.observe(start)
    assert not monitor.last['slip_suspected']

def test_sustained_real_beam_edge_change_triggers_without_width_loss():
    monitor=OwnCarryMonitor(slip_guard=True)
    for _ in range(21): monitor.observe(frame('baseline-start.jpg'))
    with pytest.raises(GripUncertain,match='beam-edge drift'):
        for _ in range(21): monitor.observe(frame('baseline-drift-4s.jpg'))
    assert monitor.last['slip_suspected']
    assert monitor.last['wide_beam_fraction']>.85

def test_invalid_vision_is_not_permission_to_regrasp():
    assert not slip_requested({'r1':{'ready':False,'error':'bad image'},'r3':{'ready':True}})
    assert slip_requested({'r1':{'own_carry_observation':{'slip_suspected':True}},'r3':{'ready':True}})
