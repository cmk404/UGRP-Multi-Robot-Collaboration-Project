"""Robot-local composition of wrist manipulation and chassis RGB navigation."""
from harness.visual_box_skill import VisualBoxSkill
from harness.visual_navigation import VisualNavigator


class VisualTransport:
    def __init__(self, robot_id, cargo_id, destination_zone):
        self.robot_id = robot_id
        self.cargo_id = cargo_id
        self.destination_zone = destination_zone
        self.box = VisualBoxSkill(task='external_navigation', robot_id=robot_id,
                                  cargo_id=cargo_id, destination_zone=destination_zone)
        self.navigator = VisualNavigator(destination_zone=destination_zone, robot_id=robot_id)
        self.reason = 'RUNNING'
        self.navigation_complete = False

    @property
    def phase(self):
        if self.reason != 'RUNNING':
            return 'finished'
        return 'navigate' if self.box.phase == 'carry' else self.box.phase

    @property
    def held(self):
        return self.box.held

    def decide(self, wrist_observation, navigation_observation):
        if self.reason != 'RUNNING':
            return {'kind': 'finish', 'reason': self.reason}
        before = self.box.phase
        action = self.box.decide(wrist_observation)
        if action['kind'] == 'finish':
            self.reason = action['reason']
            return action
        if before == 'carry':
            action = self.navigator.decide(navigation_observation)
            if action['kind'] == 'finish':
                if action['reason'] == 'NAVIGATION_ARRIVED':
                    self.navigation_complete = True
                    self.box.phase = 'release'
                    return {'kind': 'wait', 'duration': .05}
                self.reason = action['reason']
        return action
