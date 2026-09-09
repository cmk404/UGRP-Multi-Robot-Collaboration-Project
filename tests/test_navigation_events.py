import base64
import hashlib
import cv2
import numpy as np

from harness.navigation_events import NavigationEvents, InferenceInputBudget


def nav():
    pixels = np.full((480,640,3), 70, np.uint8)
    pixels[::16, :, :] = 180  # Textured fixture: flat imagery is unobservable.
    pixels[:, ::16, :] = 180
    ok, jpg = cv2.imencode('.jpg', pixels)
    assert ok
    raw = jpg.tobytes()
    return {'camera':'nav_cam','image':base64.b64encode(raw).decode(),
            'sha256':hashlib.sha256(raw).hexdigest()}


def test_identical_pixels_stop_after_window_not_at_every_frame():
    event = NavigationEvents()
    action = {'kind':'drive','fwd':.1,'turn':0,'duration':4}
    assert event.inspect(nav(),action,0)['allowed']
    assert event.inspect(nav(),action,.25)['allowed']
    assert event.inspect(nav(),action,2)['reason'] == 'OWN_RGB_STAGNATION'


def test_new_message_interrupts_without_treating_content_as_truth():
    event = NavigationEvents()
    event.reset(['old'])
    action = {'kind':'drive','fwd':.1,'turn':0,'duration':4}
    assert event.inspect(nav(),action,0,[{'message_id':'old'}])['allowed']
    assert event.inspect(nav(),action,.25,[{'message_id':'new','content':'anything'}])['reason'] == 'NEW_PEER_MESSAGE'


def test_budget_counts_rejected_responses_once_and_records_missing_usage():
    budget = InferenceInputBudget(100)
    budget.record('a',{'prompt_tokens':70})
    budget.record('a',{'prompt_tokens':70})
    assert not budget.exhausted
    budget.record('b',None)
    assert budget.calls_without_usage == 1
    budget.record('c',{'prompt_tokens':35})
    assert budget.exhausted and budget.tokens == 105
