"""Local, deterministic substitutes for RoadRunner scenarios.

All positions use a right-handed 2-D world in metres.  The absence of lanes is
intentional: the planner receives drivable space and dynamic actors only.
"""
from dataclasses import dataclass
import numpy as np

from .models import RoadAgent


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    start: tuple[float, float]
    goal: tuple[float, float]
    agents: tuple[RoadAgent, ...]
    static_obstacles: tuple[tuple[float, float, float], ...] = ()
    speed_limit: float = 5.0


def _a(identity: str, kind: str, x: float, y: float, vx: float, vy: float, radius: float, wander: float = 0.0) -> RoadAgent:
    return RoadAgent(identity, kind, np.array([x, y], dtype=float), np.array([vx, vy], dtype=float), radius, wander)


def build_scenarios() -> dict[str, Scenario]:
    return {
        "village": Scenario(
            "village", "Unmarked village road with cattle, pedestrians, and opposing traffic.", (3, 18), (56, 18),
            (_a("car-1", "car", 30, 18, -2.2, 0, 1.2, .10), _a("cow-1", "cattle", 24, 11, .15, .65, 1.0, .25), _a("walker-1", "pedestrian", 38, 25, -.25, -.55, .45, .20)),
            ((17, 8, 2.2), (43, 28, 2.0)), 4.5),
        "market": Scenario(
            "market", "Dense mixed traffic: carts, rickshaws, pedestrians, and parked stalls.", (3, 9), (56, 27),
            (_a("rickshaw-1", "rickshaw", 18, 14, .8, .25, 1.0, .45), _a("cart-1", "cart", 29, 17, .35, -.05, .9, .15), _a("walker-1", "pedestrian", 35, 18, .05, -.75, .45, .35), _a("rickshaw-2", "rickshaw", 43, 20, -.7, .0, 1.0, .3)),
            ((12, 16, 1.4), (23, 23, 1.6), (39, 11, 1.6), (49, 25, 1.4)), 3.5),
        "intersection": Scenario(
            "intersection", "Unsignalised intersection with crossing rickshaws and cars.", (3, 18), (56, 18),
            (_a("cross-car", "car", 30, 3, 0, 2.4, 1.2, .12), _a("auto", "rickshaw", 18, 30, .85, -1.0, 1.0, .40), _a("pedestrian", "pedestrian", 27, 25, .45, -.6, .45, .30)),
            ((25, 10, 1.3), (35, 27, 1.3)), 4.0),
        "merge": Scenario(
            "merge", "Highway merging zone with faster cars and a large truck.", (3, 10), (56, 24),
            (_a("truck", "truck", 27, 20, 2.8, 0, 1.65, .04), _a("car-fast", "car", 19, 25, 3.4, -.25, 1.2, .10), _a("car-merge", "car", 38, 11, 2.0, .8, 1.2, .20)),
            ((14, 4, 2.0), (47, 31, 2.0)), 6.0),
        "school": Scenario(
            "school", "School/hospital zone with crossings, parked vehicles, and speed humps.", (3, 18), (56, 18),
            (_a("child-1", "pedestrian", 26, 9, .05, .85, .42, .25), _a("child-2", "pedestrian", 34, 27, .0, -.75, .42, .25), _a("parked", "car", 42, 13, 0, 0, 1.25)),
            ((17, 25, 1.5), (25, 25, 1.5), (46, 24, 1.6)), 3.0),
    }
