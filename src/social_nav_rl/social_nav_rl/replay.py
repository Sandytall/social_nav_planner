"""Failure replay: capture a full episode so a failure can be answered with "why here?".

The env appends a per-step frame (robot state, humans, predictions summary, RL action, the
safety-filtered command + intervention, reward components, events); on a failed episode the
whole thing is saved as JSON and can be reloaded and stepped through offline. Pure (json).
"""
import json
from dataclasses import asdict, dataclass, field
from typing import List


@dataclass
class FailureReplay:
    environment: str
    scenario: str
    difficulty: str
    seed: int
    goal: list
    outcome: str = "unknown"          # reached / collision / human_collision / timeout
    steps: List[dict] = field(default_factory=list)

    def add(self, **frame):
        """Append one step frame; keys are free-form (t, robot, humans, action, safe_action,
        safety_kind, reward, reward_components, events, ...)."""
        self.steps.append(frame)

    def finalize(self, outcome: str):
        self.outcome = outcome

    def save(self, path: str) -> str:
        with open(path, "w") as f:
            json.dump(asdict(self), f)
        return path


def load(path: str) -> dict:
    with open(path) as f:
        return json.load(f)
