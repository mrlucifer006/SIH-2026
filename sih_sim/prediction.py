"""Short-horizon, batched probabilistic motion prediction using PyTorch/CUDA."""
import numpy as np
import torch

from .models import RoadAgent


class MotionPredictor:
    def __init__(self, device: torch.device, horizon_s: float = 1.5, samples: int = 16) -> None:
        self.device, self.horizon_s, self.samples = device, horizon_s, samples

    def predict(self, agents: list[RoadAgent]) -> list[RoadAgent]:
        """Return conservative predicted agents; uncertainty grows with erraticness."""
        if not agents:
            return []
        positions = torch.tensor(np.array([a.position for a in agents]), dtype=torch.float32, device=self.device)
        velocities = torch.tensor(np.array([a.velocity for a in agents]), dtype=torch.float32, device=self.device)
        noise = torch.randn((self.samples, len(agents), 2), device=self.device)
        uncertainty = torch.tensor([.15 + a.wander * 1.2 for a in agents], device=self.device)
        future = positions[None] + velocities[None] * self.horizon_s + noise * uncertainty[None, :, None]
        centre = future.mean(dim=0).cpu().numpy()
        spread = future.std(dim=0).norm(dim=1).cpu().numpy()
        predicted: list[RoadAgent] = []
        for index, actor in enumerate(agents):
            clone = RoadAgent(actor.agent_id, actor.kind, centre[index], actor.velocity.copy(), actor.radius + float(spread[index]), actor.wander)
            predicted.append(clone)
        return predicted
