"""Renew a PREPARE drive intent only while fresh RGB supports continued motion.

This is a conservative local execution adapter, not a grasp or path planner.
It reuses the existing image-motion identity and orange-beam extractor. Peers
must be held during its isolated observation window. It never reports READY.
"""
from __future__ import annotations

import math

from harness.camera_beam_features import extract_beams, select_beam
from harness.camera_motion_identity import ImageMotionIdentity, _decode_jpeg
import cv2
import numpy as np


class VisualStillness:
    """Require two quiet image intervals; issuing HOLD is not proof of rest."""
    def __init__(self, first):
        self.previous=_decode_jpeg(first);self.quiet=0

    def update(self, jpeg):
        current=_decode_jpeg(jpeg)
        if current.shape!=self.previous.shape:
            self.previous=current;self.quiet=0
            return {'ready':False,'changed_pixels':None,'quiet_intervals':0}
        changed=int(np.count_nonzero(cv2.absdiff(current,self.previous).max(axis=2)>20))
        self.previous=current;self.quiet=self.quiet+1 if changed<20 else 0
        return {'ready':self.quiet>=2,'changed_pixels':changed,'quiet_intervals':self.quiet}


def segment_distance(point, ends):
    a,b=ends;d=[b[i]-a[i] for i in (0,1)]
    denom=sum(v*v for v in d)
    t=max(0.,min(1.,sum((point[i]-a[i])*d[i] for i in (0,1))/max(denom,1e-12)))
    return math.dist(point,[a[i]+t*d[i] for i in (0,1)])


class VisualDriveLease:
    def __init__(self, before_top, action, *, max_steps=10):
        if action.get("kind")!="drive" or action.get("duration_s")!=.2:
            raise ValueError("bounded drive intent required")
        if not 1<=max_steps<=10:raise ValueError("max_steps must be 1..10")
        self.action=dict(action);self.max_steps=max_steps;self.steps=0
        self.identity=ImageMotionIdentity();self.identity.update(before_top,None)
        self.target_center=None;self.last_gap=None;self.no_progress=0
        self.terminal=False

    def after_step(self, fresh_top):
        if self.terminal:return {"renew":False,"reason":"lease_already_stopped"}
        self.steps+=1
        motion=self.identity.update(fresh_top,self.action)
        result={"renew":False,"step":self.steps,"identity":motion}
        reason=None
        if self.steps>=self.max_steps:reason="local_budget"
        elif not motion["valid"] or not motion["fresh"]:reason="fresh_own_motion_unconfirmed"
        elif self.action["forward"]<=0 or self.action['turn']!=0:
            reason="turn_requires_new_high_level_observation"
        if reason is None:
            target=select_beam(extract_beams(fresh_top,robust_shaft=True),motion["center"],self.target_center)
            if target is None:reason="beam_lost"
            else:
                self.target_center=target["center"]
                gap=segment_distance(motion["center"],target["endpoints"])
                result.update(beam=target,gap_image_units=gap)
                # Deliberately conservative: body/motion-anchor-to-beam spacing
                # is not a gripper alignment verdict. Return control near it.
                if gap<.085:reason="near_beam_reobserve_fine_alignment"
                if self.last_gap is not None:
                    if gap>self.last_gap+.008:reason="visual_gap_increased"
                    self.no_progress=self.no_progress+1 if gap>=self.last_gap-.0005 else 0
                    if self.no_progress>=3:reason="visual_progress_stalled"
                self.last_gap=gap
        if reason is None:
            result.update(renew=True,reason="fresh_motion_and_beam_clearance")
        else:
            result["reason"]=reason;self.terminal=True
        return result
