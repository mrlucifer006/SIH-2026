from dataclasses import dataclass


@dataclass(frozen=True)
class SimulationConfig:
    """Central, deliberately conservative simulation settings (metres, seconds)."""

    world_width: float = 60.0
    world_height: float = 36.0
    resolution: float = 0.5
    dt: float = 0.1
    max_steps: int = 500
    replan_every: int = 2
    ego_radius: float = 1.15
    safety_margin: float = 1.0
    max_speed: float = 6.0
    wheelbase: float = 2.7
    device: str = "auto"  # auto, cuda, or cpu

    @property
    def grid_shape(self) -> tuple[int, int]:
        return (round(self.world_height / self.resolution), round(self.world_width / self.resolution))
