"""MATLAB/Simulink bridge for the existing Python autonomy pipeline.

The public entry point is ``step_json`` so MATLAB only needs its built-in
JSON encoder/decoder; no direct NumPy conversion is required on the MATLAB
side.
"""
from __future__ import annotations

import json
import time
import numpy as np

from .config import SimulationConfig
from .controller import BicycleController
from .models import RoadAgent, VehicleState
from .perception import CudaPerception
from .planner import AStarPlanner
from .prediction import MotionPredictor


_RUNTIMES = {}


def _runtime(device: str):
    if device not in _RUNTIMES:
        config = SimulationConfig(device=device)
        perception = CudaPerception(config)
        _RUNTIMES[device] = (config, perception, MotionPredictor(perception.device), AStarPlanner(), BicycleController(config))
    return _RUNTIMES[device]


def _rows(value, width: int) -> np.ndarray:
    array = np.asarray(value if value is not None else [], dtype=float)
    if array.size == 0:
        return np.empty((0, width), dtype=float)
    return array.reshape((-1, width))


def _make_agents(payload: dict) -> list[RoadAgent]:
    agents: list[RoadAgent] = []
    camera = _rows(payload.get("cameraDetections"), 3)
    for i, row in enumerate(camera):
        agents.append(RoadAgent(f"camera-{i}", "car", row[:2], np.zeros(2), 0.8, 0.1))
    lidar = _rows(payload.get("lidarDetections"), 4)
    for i, row in enumerate(lidar):
        agents.append(RoadAgent(f"lidar-{i}", "car", row[:2], np.zeros(2), max(row[2], 0.5), 0.1))
    radar = _rows(payload.get("radarDetections"), 5)
    for i, row in enumerate(radar):
        agents.append(RoadAgent(f"radar-{i}", "car", row[:2], row[2:4], 0.8, 0.1))
    # Basic proximity merge prevents a camera/lidar/radar observation of the
    # same actor from being counted as three separate obstacles.
    merged: list[RoadAgent] = []
    for actor in agents:
        match = next((other for other in merged if np.linalg.norm(actor.position - other.position) < 1.5), None)
        if match is None:
            merged.append(actor)
        else:
            match.position = (match.position + actor.position) / 2.0
            match.velocity = (match.velocity + actor.velocity) / 2.0
            match.radius = max(match.radius, actor.radius)
    return merged


def step_json(request: str) -> str:
    """Run one closed-loop step and return a JSON response."""
    payload = json.loads(str(request))
    device = str(payload.get("device", "auto"))
    config, perception, predictor, planner, controller = _runtime(device)
    ego = np.asarray(payload["egoState"], dtype=float).reshape(-1)
    goal = np.asarray(payload["goal"], dtype=float).reshape(-1)
    agents = _make_agents(payload)
    static = tuple(tuple(map(float, row)) for row in _rows(payload.get("staticObstacles"), 3))
    started = time.perf_counter()
    predicted = predictor.predict(agents)
    occupancy = perception.occupancy(predicted, static).cpu().numpy()
    start = planner.world_to_cell(ego[:2], config.resolution, occupancy.shape)
    end = planner.world_to_cell(goal[:2], config.resolution, occupancy.shape)
    cells = planner.plan(occupancy, start, end)
    path = planner.cells_to_path(cells, config.resolution) if cells else np.empty((0, 2))
    state = VehicleState(ego[:2], float(ego[2]), float(ego[3]))
    command_state = controller.step(state, path, config.max_speed)
    acceleration = np.clip((command_state.speed - state.speed) / config.dt, -5.0, 2.5)
    command = [0.0, float(max(0.0, acceleration)), float(max(0.0, -acceleration))]
    tracks = [[float(a.position[0]), float(a.position[1]), float(a.velocity[0]), float(a.velocity[1]), float(a.radius)] for a in predicted]
    return json.dumps({
        "command": command,
        "tracks": tracks,
        "path": path.tolist(),
        "debug": {"device": str(perception.device), "replanMs": (time.perf_counter() - started) * 1000, "numTracks": len(tracks)},
    })
