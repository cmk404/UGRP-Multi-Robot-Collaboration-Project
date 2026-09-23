import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from harness.camera_goal_transport import wheel_heading
from harness.carry_arrival import arrival_evidence
from harness.dispatch_beam_tracker import CarriedBeamTracker
from sim.research_dispatch_arena import authored_map

FIXTURES = Path(__file__).parent/'fixtures/research_entry_stop'


def image(name):
    record=json.loads((FIXTURES/'provenance.json').read_text())[name]
    raw=(FIXTURES/name).read_bytes()
    assert hashlib.sha256(raw).hexdigest()==record['sha256']
    return raw, record


@pytest.mark.parametrize('case', ['test-minus','test-yaw'])
def test_pixel_cell_rounding_accepts_recorded_four_corner_wheels(case):
    raw,record=image('coarse-'+case+'.jpg')
    frame=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    yellow=cv2.inRange(hsv,np.array((20,70,50),np.uint8),np.array((40,255,255),np.uint8))
    n,labels,stats,_=cv2.connectedComponentsWithStats(yellow)
    clean=np.isin(labels,[i for i in range(1,n) if stats[i,4]<=300]).astype(np.uint8)*255
    cx,cy=record['center_hint'];yy,xx=np.indices(clean.shape)
    clean[(abs(xx-cx)>55)|(abs(yy-cy)>48)]=0
    result=wheel_heading(clean,pixel_tolerance=2.)
    assert result is not None and abs(result['angle_deg'])<2
    assert min(result['corner_pixels'])>=5
    # A missing wheel corner must still fail; no one-sided silhouette fallback.
    clean[(xx>cx)&(yy>cy)]=0
    assert wheel_heading(clean,pixel_tolerance=2.) is None


def test_heading_still_rejects_large_or_ambiguous_painted_shapes():
    mask=np.zeros((720,960),np.uint8)
    for x in (100,170):
        for y in (100,160):mask[y:y+5,x:x+5]=255
    assert wheel_heading(mask,pixel_tolerance=2.) is None
    with pytest.raises(ValueError):wheel_heading(mask,pixel_tolerance=3.)


@pytest.mark.parametrize('condition,expected',[('RGB',True),('ACT-before',False),('ACT-expanded',False)])
def test_rgb_stop_admission_distinguishes_actual_successful_arrival(condition,expected):
    raw,_=image('stop-'+condition+'.jpg')
    feature=CarriedBeamTracker().observe(raw)
    result=arrival_evidence(feature,authored_map('open'),'dock_a')
    assert result['arrived'] is expected
