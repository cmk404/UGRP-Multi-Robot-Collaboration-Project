"""Replace act() with your policy; it receives copied RGB and own commands.

This example is an open-loop motion demo, not an RGB navigation policy.
For JPEG decoding: base64.b64decode(observation['image']).
"""


class Controller:
    def __init__(self, params):
        self.power = params.get("power", .08)

    def act(self, observation):
        if observation["episode_time_s"] < .6:
            return {"kind": "nudge", "params": {"power": self.power, "seconds": .2}}
        return {"kind": "wait"}


def create_controller(*, robot_id, seed, params):
    # A fresh instance is created for this robot on every reset.
    return Controller(params)


def create_idle_controller(*, robot_id, seed, params):
    # Select controller.py:create_idle_controller to compare issued commands.
    return Controller({"power": 0})
