"""Independent planner instances for the three robot team.

Each robot keeps its own LLM conversation state and provider instance. The
team bus remains the only coordination channel; this object does not merge
plans between robots.
"""

from __future__ import annotations

from typing import Any


class MultiRobotCompleter:
    def __init__(self, factories: dict[str, Any], default: str = "r1"):
        self._completers = {str(k): v for k, v in factories.items()}
        self._default = default

    def for_robot(self, robot_id: str):
        return self._completers.get(str(robot_id), self._completers[self._default])

    def complete(self, messages: list[dict[str, Any]], image: str | None = None) -> str:
        return self.for_robot(self._default).complete(messages, image=image)
