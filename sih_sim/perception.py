"""GPU occupancy-grid construction from idealised sensor detections."""
import torch
import torch.nn.functional as F

from .config import SimulationConfig
from .models import RoadAgent


class CudaPerception:
    def __init__(self, config: SimulationConfig) -> None:
        requested = config.device.lower()
        if requested == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but no CUDA-enabled PyTorch device is available.")
        self.device = torch.device("cuda" if requested == "cuda" or (requested == "auto" and torch.cuda.is_available()) else "cpu")
        self.config = config
        h, w = config.grid_shape
        ys = (torch.arange(h, device=self.device) + .5) * config.resolution
        xs = (torch.arange(w, device=self.device) + .5) * config.resolution
        self.y_grid, self.x_grid = torch.meshgrid(ys, xs, indexing="ij")

    def occupancy(self, agents: list[RoadAgent], static_obstacles: tuple[tuple[float, float, float], ...]) -> torch.Tensor:
        """Rasterise all discs in parallel, then apply a GPU safety dilation."""
        circles = [(float(a.position[0]), float(a.position[1]), a.radius + self.config.ego_radius + self.config.safety_margin) for a in agents]
        circles.extend((x, y, r + self.config.ego_radius + self.config.safety_margin) for x, y, r in static_obstacles)
        if not circles:
            return torch.zeros(self.config.grid_shape, dtype=torch.bool, device=self.device)
        data = torch.tensor(circles, dtype=torch.float32, device=self.device)
        dx = self.x_grid.unsqueeze(0) - data[:, 0, None, None]
        dy = self.y_grid.unsqueeze(0) - data[:, 1, None, None]
        blocked = ((dx.square() + dy.square()) <= data[:, 2, None, None].square()).any(dim=0).float()
        # A small max-pool accounts for sensor quantisation and is CUDA-accelerated.
        return F.max_pool2d(blocked[None, None], kernel_size=3, stride=1, padding=1)[0, 0].bool()
