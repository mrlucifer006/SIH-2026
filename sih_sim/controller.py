import math
import numpy as np

from .config import SimulationConfig
from .models import VehicleState


class BicycleController:
    """Pure-pursuit steering and bounded longitudinal control."""
    def __init__(self, config: SimulationConfig) -> None:
        self.config = config

    def step(self, state: VehicleState, path: np.ndarray, speed_limit: float) -> VehicleState:
        if len(path) == 0:
            target_speed, steering = 0.0, 0.0
        else:
            distances = np.linalg.norm(path - state.position, axis=1)
            lookahead = 2.5 + state.speed * .35
            target = path[np.searchsorted(distances, lookahead, side="left").clip(0, len(path) - 1)]
            local_angle = math.atan2(target[1] - state.position[1], target[0] - state.position[0]) - state.heading
            local_angle = math.atan2(math.sin(local_angle), math.cos(local_angle))
            steering = float(np.clip(math.atan2(2 * self.config.wheelbase * math.sin(local_angle), lookahead), -.55, .55))
            target_speed = min(speed_limit, self.config.max_speed)
        acceleration = float(np.clip((target_speed - state.speed) * 1.8, -5.0, 2.5))
        speed = max(0.0, state.speed + acceleration * self.config.dt)
        heading = state.heading + speed / self.config.wheelbase * math.tan(steering) * self.config.dt
        position = state.position + speed * np.array([math.cos(heading), math.sin(heading)]) * self.config.dt
        return VehicleState(position, heading, speed)
