"""Execution primitives selected by an external image-grounded LLM.

This class does not select routes or decide when to grasp/release. It executes
only the manipulation primitive explicitly selected by its robot's planner.
"""
from harness.visual_box_skill import VisualBoxSkill


class LLMTransportSkill:
    def __init__(self, robot_id, cargo_id, destination_zone):
        self.robot_id=robot_id;self.cargo_id=cargo_id;self.destination_zone=destination_zone
        self.box=VisualBoxSkill(task='external_navigation',robot_id=robot_id,cargo_id=cargo_id,destination_zone=destination_zone)
        self.state='idle';self.operation=None;self.reason='RUNNING';self.pending_hover=None
        self.last_llm_reason=''
        self.last_guard_reason=None
        self.placement_evidence=None
        self.recovery_attempts=0
        self.max_recovery_attempts=3

    @property
    def phase(self):
        return self.box.phase if self.operation else {'carrying':'navigate','released':'verify_release'}.get(self.state,self.state)

    @property
    def held(self):return self.box.held

    def observe_placement(self, evidence):
        if not isinstance(evidence, dict):raise ValueError('INVALID_PLACEMENT_EVIDENCE')
        if evidence.get('status') not in ('inside','outside','uncertain'):raise ValueError('INVALID_PLACEMENT_STATUS')
        if evidence.get('stage') not in ('before_release','released'):raise ValueError('INVALID_PLACEMENT_STAGE')
        forbidden={'pose','position','positions','coordinates','coords','x','y','z','world_state','map','seed','layout'}
        def check(value):
            if isinstance(value,dict):
                for key,item in value.items():
                    if not isinstance(key,str) or key.lower() in forbidden:raise ValueError('PLACEMENT_TRUTH_FORBIDDEN')
                    check(item)
            elif isinstance(value,(list,tuple)):
                for item in value:check(item)
            elif not isinstance(value,(str,int,float,bool,type(None))):raise ValueError('INVALID_PLACEMENT_EVIDENCE')
        check(evidence)
        self.placement_evidence=dict(evidence)
        return self.placement_evidence

    def request(self, action, placement_evidence=None):
        if placement_evidence is not None:self.observe_placement(placement_evidence)
        kind=action['kind']
        if kind=='check_grip':
            if self.state!='grip_uncertain':raise ValueError('GRIP_CHECK_NOT_REQUIRED')
            self.box.phase='verify_lift';self.box.reason='RUNNING'
            self.operation='pick';self.state='picking';return None
        if kind=='approach':
            if self.state=='released':
                if self.recovery_attempts>=self.max_recovery_attempts:raise ValueError('PLACEMENT_RECOVERY_EXHAUSTED')
                self.recovery_attempts+=1
                # A released object is a new visual acquisition.  Never reuse
                # stale attachment anchors, tracker phase, or pending hover.
                self.box=VisualBoxSkill(task='external_navigation',robot_id=self.robot_id,
                                        cargo_id=self.cargo_id,destination_zone=self.destination_zone,
                                        near_field_reacquisition=True)
                self.pending_hover=None;self.placement_evidence=None
            elif self.state not in ('idle','approaching'):raise ValueError('APPROACH_NOT_AVAILABLE')
            self.state='approaching';self.operation='approach';return None
        if kind=='pick':
            if self.state!='ready_to_pick':raise ValueError('PICK_REQUIRES_VISUAL_APPROACH')
            self.state='picking';self.operation='pick'
            result=self.pending_hover;self.pending_hover=None;return result
        if kind=='release':
            if self.state!='carrying':raise ValueError('RELEASE_REQUIRES_HELD_BOX')
            if not self.placement_evidence or self.placement_evidence.get('status')!='inside' or self.placement_evidence.get('stage')!='before_release':
                raise ValueError('SAFE_INSIDE_EVIDENCE_REQUIRED_BEFORE_RELEASE')
            self.state='releasing';self.operation='release';self.box.phase='release';return None
        if kind=='finish':
            if self.state!='released':raise ValueError('FINISH_REQUIRES_RELEASE')
            if not self.placement_evidence or self.placement_evidence.get('status')!='inside' or self.placement_evidence.get('stage')!='released':
                raise ValueError('CAMERA_INSIDE_EVIDENCE_REQUIRED_BEFORE_FINISH')
            self.state='finished';self.reason='VISUAL_RELEASE_CONFIRMED'
            return {'kind':'finish','reason':self.reason}
        if kind in ('drive','wait'):
            if self.operation:raise ValueError('PRIMITIVE_BUSY')
            if kind=='drive' and self.state=='grip_uncertain':raise ValueError('CHECK_GRIP_BEFORE_DRIVING')
            return dict(action)
        raise ValueError('UNKNOWN_LLM_SKILL')

    def advance(self,wrist):
        """Advance only an already authorized local manipulation primitive."""
        if self.operation is None:
            if self.state=='carrying':
                guard=self.box.decide(wrist)
                if guard['kind']=='finish':
                    self.last_guard_reason=guard['reason']
                    if guard['reason'] in ('VISUAL_GRASP_DRIFT','VISUAL_LOAD_DROPPED_OR_OCCLUDED',
                                           'TOP_GEOMETRY_AMBIGUOUS_FOR_DROP'):
                        self.state='grip_uncertain';return None
                    self.state='failure';self.reason=guard['reason'];return guard
            return None
        action=self.box.decide(wrist)
        if action['kind']=='finish':
            self.operation=None
            if action['reason']=='VISUAL_RELEASE_CONFIRMED':
                self.state='released';self.placement_evidence=None;return None
            self.state='failure';self.reason=action['reason'];return action
        if self.operation=='approach' and self.box.phase=='lower':
            self.pending_hover=action;self.operation=None;self.state='ready_to_pick';return None
        if self.operation=='pick' and self.box.phase=='carry':
            self.operation=None;self.state='carrying';return None
        return action
