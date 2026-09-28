from dataclasses import dataclass, field
from typing import Literal
import numpy as np


AgentKind = Literal["car", "rickshaw", "pedestrian", "cattle", "cart", "truck"]


@dataclass
class RoadAgent:
    agent_id: str
    kind: AgentKind
    position: np.ndarray
    velocity: np.ndarray
    radius: float
    wander: float = 0.0

    def advance(self, dt: float, width: float, height: float, rng: np.random.Generator) -> None:
        """Advance a mixed-traffic actor with bounded stochastic lateral motion."""
        if self.wander:
            self.velocity += rng.normal(0.0, self.wander, 2) * dt
        speed = np.linalg.norm(self.velocity)
        if speed > 5.5:
            self.velocity *= 5.5 / speed
        self.position += self.velocity * dt
        # Keep actors inside the simulated road and reverse only the escaping component.
        for axis, limit in enumerate((width, height)):
            if self.position[axis] < self.radius:
                self.position[axis] = self.radius
                self.velocity[axis] = abs(self.velocity[axis])
            elif self.position[axis] > limit - self.radius:
                self.position[axis] = limit - self.radius
                self.velocity[axis] = -abs(self.velocity[axis])


@dataclass
class VehicleState:
    position: np.ndarray
    heading: float = 0.0
    speed: float = 0.0


@dataclass
class EpisodeMetrics:
    scenario: str
    steps: int = 0
    replans: int = 0
    planner_ms: list[float] = field(default_factory=list)
    min_clearance: float = float("inf")
    collision: bool = False
    reached_goal: bool = False

    def as_dict(self) -> dict[str, object]:
        latency = float(np.mean(self.planner_ms)) if self.planner_ms else 0.0
        return {
            "scenario": self.scenario,
            "steps": self.steps,
            "replans": self.replans,
            "mean_replan_ms": round(latency, 2),
            "max_replan_ms": round(max(self.planner_ms, default=0.0), 2),
            "min_clearance_m": round(self.min_clearance, 2),
            "collision": self.collision,
            "reached_goal": self.reached_goal,
        }
