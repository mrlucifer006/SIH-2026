from __future__ import annotations
import time
import numpy as np

from .config import SimulationConfig
from .controller import BicycleController
from .models import EpisodeMetrics, VehicleState
from .perception import CudaPerception
from .planner import AStarPlanner
from .prediction import MotionPredictor
from .scenarios import Scenario


class SimulationEngine:
    def __init__(self, config: SimulationConfig | None = None, seed: int = 7) -> None:
        self.config = config or SimulationConfig()
        self.rng = np.random.default_rng(seed)
        self.perception = CudaPerception(self.config)
        self.predictor = MotionPredictor(self.perception.device)
        self.planner = AStarPlanner()
        self.controller = BicycleController(self.config)

    @property
    def device_name(self) -> str:
        return str(self.perception.device)

    def run(self, scenario: Scenario, capture_frames: bool = False) -> tuple[EpisodeMetrics, list[dict]]:
        # Clone agents so repeated simulations of one scenario are independent.
        agents = [type(a)(a.agent_id, a.kind, a.position.copy(), a.velocity.copy(), a.radius, a.wander) for a in scenario.agents]
        state = VehicleState(np.array(scenario.start, dtype=float))
        goal = np.array(scenario.goal, dtype=float)
        metrics, path, frames = EpisodeMetrics(scenario.name), np.empty((0, 2)), []
        for step in range(self.config.max_steps):
            for actor in agents:
                actor.advance(self.config.dt, self.config.world_width, self.config.world_height, self.rng)
            if step % self.config.replan_every == 0:
                started = time.perf_counter()
                predicted = self.predictor.predict(agents)
                occupancy = self.perception.occupancy(predicted, scenario.static_obstacles).cpu().numpy()
                start = self.planner.world_to_cell(state.position, self.config.resolution, occupancy.shape)
                end = self.planner.world_to_cell(goal, self.config.resolution, occupancy.shape)
                path = self.planner.cells_to_path(self.planner.plan(occupancy, start, end), self.config.resolution)
                metrics.planner_ms.append((time.perf_counter() - started) * 1000)
                metrics.replans += 1
            state = self.controller.step(state, path, scenario.speed_limit)
            clearances = [np.linalg.norm(state.position - a.position) - self.config.ego_radius - a.radius for a in agents]
            clearances.extend(np.linalg.norm(state.position - np.array([x, y])) - self.config.ego_radius - r for x, y, r in scenario.static_obstacles)
            metrics.min_clearance = min(metrics.min_clearance, min(clearances, default=float("inf")))
            metrics.steps = step + 1
            if capture_frames:
                frames.append({"ego": state.position.copy(), "agents": [a.position.copy() for a in agents], "path": path.copy(), "occupancy": occupancy.copy() if 'occupancy' in locals() else None})
            if metrics.min_clearance < 0:
                metrics.collision = True
                break
            if np.linalg.norm(state.position - goal) < 1.2:
                metrics.reached_goal = True
                break
        return metrics, frames
