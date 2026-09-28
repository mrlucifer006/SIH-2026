import heapq
import math
from typing import Iterable
import numpy as np


class AStarPlanner:
    """Eight-connected grid planner; obstacle rasterisation happens on the GPU."""

    _NEIGHBOURS = tuple((dy, dx, math.hypot(dx, dy)) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy)

    @staticmethod
    def world_to_cell(point: np.ndarray | tuple[float, float], resolution: float, shape: tuple[int, int]) -> tuple[int, int]:
        x, y = point
        return (int(np.clip(round(y / resolution - .5), 0, shape[0] - 1)), int(np.clip(round(x / resolution - .5), 0, shape[1] - 1)))

    @staticmethod
    def cell_to_world(cell: tuple[int, int], resolution: float) -> np.ndarray:
        return np.array([(cell[1] + .5) * resolution, (cell[0] + .5) * resolution])

    def plan(self, occupancy: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        occupancy = occupancy.copy()
        occupancy[start] = False
        occupancy[goal] = False
        queue = [(0.0, start)]
        parent: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        cost = {start: 0.0}
        height, width = occupancy.shape
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                return self._reconstruct(parent, current)
            for dy, dx, move_cost in self._NEIGHBOURS:
                nxt = current[0] + dy, current[1] + dx
                if not (0 <= nxt[0] < height and 0 <= nxt[1] < width) or occupancy[nxt]:
                    continue
                candidate = cost[current] + move_cost
                if candidate < cost.get(nxt, float("inf")):
                    cost[nxt] = candidate
                    parent[nxt] = current
                    heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heapq.heappush(queue, (candidate + heuristic, nxt))
        return []

    @staticmethod
    def _reconstruct(parent: dict, current: tuple[int, int]) -> list[tuple[int, int]]:
        path = [current]
        while parent[current] is not None:
            current = parent[current]
            path.append(current)
        return list(reversed(path))

    def cells_to_path(self, cells: Iterable[tuple[int, int]], resolution: float) -> np.ndarray:
        return np.array([self.cell_to_world(c, resolution) for c in cells])
