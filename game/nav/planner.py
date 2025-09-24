# game/nav/planner.py
import collections
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .astar import astar_bool_grid
from .clearance import clearance_mask_for_ship, distance_field_cells
from .grid import rasterize_water
from .smooth import decimate_by_spacing, string_pull_on_mask
from .types import Grid, GridSpec, Vec2


@dataclass
class PlanResult:
    waypoints: List[Vec2]  # world meters, start excluded

class NavPlanner:
    def __init__(self, world_origin: Vec2, world_size: Vec2, cell_m: float, land_polys: List[List[Vec2]]):
        w, h = int(world_size[0] // cell_m), int(world_size[1] // cell_m)
        self.spec = GridSpec(cell_m=cell_m, size_x=w, size_y=h, origin_m=world_origin)
        self.grid = rasterize_water(self.spec, land_polys)

        land = ~self.grid.water
        self._dist_cells = distance_field_cells(land)
        self._cell_m = self.spec.cell_m

    def distance_to_land_m(self, p):
        i = int((p[0] - self.spec.origin_m[0]) / self._cell_m)
        j = int((p[1] - self.spec.origin_m[1]) / self._cell_m)
        i = max(0, min(self.spec.size_x - 1, i))
        j = max(0, min(self.spec.size_y - 1, j))
        return float(self._dist_cells[j, i]) * self._cell_m

    def land_normal(self, p):
        # central-diff gradient of distance (points away from land)
        i = int((p[0] - self.spec.origin_m[0]) / self._cell_m)
        j = int((p[1] - self.spec.origin_m[1]) / self._cell_m)
        W, H = self.spec.size_x, self.spec.size_y
        i0 = max(0, min(W - 1, i - 1))
        i1 = max(0, min(W - 1, i + 1))
        j0 = max(0, min(H - 1, j - 1))
        j1 = max(0, min(H - 1, j + 1))
        dx = (self._dist_cells[j, i1] - self._dist_cells[j, i0]) * 0.5
        dy = (self._dist_cells[j1, i] - self._dist_cells[j0, i]) * 0.5
        m = math.hypot(dx, dy)
        if m > 1e-6:
            return (dx / m, dy / m)
        return (0.0, 0.0)

    def _world_to_cell(self, p: Vec2) -> Tuple[int, int]:
        i = int((p[0] - self.spec.origin_m[0]) / self.spec.cell_m)
        j = int((p[1] - self.spec.origin_m[1]) / self.spec.cell_m)
        i = max(0, min(self.spec.size_x - 1, i))
        j = max(0, min(self.spec.size_y - 1, j))
        return (i, j)

    def _snap_to_traversable(self, traversable: np.ndarray, cell: Tuple[int,int], max_radius_cells: int = 8) -> Optional[Tuple[int,int]]:
        """BFS ring search to find nearest traversable cell if the input is blocked."""
        i0, j0 = cell
        if traversable[j0, i0]:
            return (i0, j0)
        H, W = traversable.shape
        q = collections.deque([(i0, j0)])
        seen = set([(i0, j0)])
        steps = 0
        # 4-neighbor expansion (faster?)
        while q and steps < max_radius_cells * max_radius_cells:
            for _ in range(len(q)):
                i, j = q.popleft()
                for di, dj in ((1,0), (-1,0), (0,1), (0,-1)):
                    ni, nj = i+di, j+dj
                    if 0 <= ni < W and 0 <= nj < H and (ni, nj) not in seen:
                        if traversable[nj, ni]:
                            return (ni, nj)
                        seen.add((ni, nj))
                        q.append((ni, nj))
            steps += 1
        return None
    

    def plan_ship_path(
        self,
        start: Vec2,
        goal: Vec2,
        hull_radius_m: float,
        safety_m: float = 20.0,
        min_waypoint_spacing_m: float = 10.0,   #spacing control
    ) -> Optional[PlanResult]:

        traversable = clearance_mask_for_ship(self.grid, hull_radius_m, safety_m=safety_m)

        s = self._world_to_cell(start)
        g = self._world_to_cell(goal)

        # Snap start/goal onto nearest traversable cells if needed
        s_snapped = self._snap_to_traversable(traversable, s) or s
        g_snapped = self._snap_to_traversable(traversable, g) or g

        path_cells = astar_bool_grid(traversable, s_snapped, g_snapped)
        if not path_cells:
            return None

        # Build a temp grid whose "water" is the traversable mask (so smoothing respects clearance)
        trav_grid = Grid(spec=self.spec, water=traversable)

        smoothed = string_pull_on_mask(trav_grid, path_cells, traversable)
        if not smoothed:
            return None

        # Drop the first point (start), then enforce minimum spacing, keeping the end
        if len(smoothed) > 1:
            smoothed = smoothed[1:]
        smoothed = decimate_by_spacing(smoothed, min_waypoint_spacing_m)

        return PlanResult(waypoints=smoothed)

    def los_clear_for_ship(self, a, b, hull_radius_m: float, safety_m: float) -> bool:
        """
        True if every sample along segment a->b is traversable for a ship of
        (hull_radius_m + safety_m) clearance, and over WATER.
        """
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        length = math.hypot(bx - ax, by - ay)

        # clearance threshold in cells
        thresh_cells = (hull_radius_m + safety_m) / max(self._cell_m, 1e-6)

        # sample every ~half cell for safety
        steps = max(1, int(length / max(self._cell_m * 0.5, 1e-6)))
        H, W = self._dist_cells.shape

        for s in range(steps + 1):
            t = s / steps
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            i = int((x - self.spec.origin_m[0]) / self._cell_m)
            j = int((y - self.spec.origin_m[1]) / self._cell_m)
            if j < 0 or j >= H or i < 0 or i >= W:
                return False
            # must be water AND have enough clearance
            if not self.grid.water[j, i]:
                return False
            if self._dist_cells[j, i] < thresh_cells:
                return False
        return True
