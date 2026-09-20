"""Recent own-camera appearance continuity during a TOP-validated loaded turn."""
import math
from harness.camera_goal_transport import own_payload


class OwnHoldContinuity:
    def __init__(self,anchor_rgb):
        self.anchor=own_payload(anchor_rgb,hue_upper=35)
        self.previous=self.anchor

    def observe(self,own_rgb):
        current=own_payload(own_rgb,hue_upper=35)
        motion=math.dist(current[1:],self.previous[1:]) if current and self.previous else None
        step_ratio=current[0]/self.previous[0] if current and self.previous else None
        anchor_ratio=current[0]/self.anchor[0] if current and self.anchor else None
        # At this unchanged near-field camera, a small geometric change can
        # expose another face abruptly. Preserve the original absolute area
        # bound; use centroid continuity rather than an area-change assumption.
        held=bool(current and self.anchor and self.previous and .25<=anchor_ratio<=4 and motion<=.035)
        result={'held_estimate':held,'current':current,'anchor':self.anchor,'previous':self.previous,
            'temporal_motion_norm':motion,'step_area_ratio':step_ratio,'anchor_area_ratio':anchor_ratio,
            'method':'own RGB presence and recent appearance continuity; TOP shaft/body coupling independently required',
            'not_a_contact_measurement':True}
        if held:self.previous=current
        return result
