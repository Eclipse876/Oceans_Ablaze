import math
from typing import List

from .types import Grid, IVec2, Vec2


def cell_center(grid: Grid, cell: IVec2) -> Vec2:
    i, j = cell
    return (grid.spec.origin_m[0] + (i + 0.5) * grid.spec.cell_m,
            grid.spec.origin_m[1] + (j + 0.5) * grid.spec.cell_m)

def sampled_log_on_mask(grid: Grid, mask, a: Vec2, b: Vec2) -> bool:
    """Sample the segment every ~half cell and require ALL samples to return water.
    Modified to (Hopefully) avoid tunneling."""
    ax, ay = a
    bx, by = b
    length = math.hypot(bx - ax, by - ay)
    # sample every ~cell/3 for safety
    steps = max(1, int(length / max(grid.spec.cell_m / 5.0, 1e-6)))
    for s in range(steps + 1):
        t = s / steps
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        i = int((x - grid.spec.origin_m[0]) / grid.spec.cell_m)
        j = int((y - grid.spec.origin_m[1]) / grid.spec.cell_m)
        if j < 0 or j >= grid.spec.size_y or i < 0 or i >= grid.spec.size_x:
            return False
        if not mask[j, i]:
            return False
    return True

def string_pull_on_mask(grid: Grid, path_cells: List[IVec2], mask) -> List[Vec2]:
    """Shortcut, keep skipping ahead as long as the LOS stays clear"""
    if not path_cells:
        return []

    pts = [cell_center(grid, c) for c in path_cells]
    out = [pts[0]]

    k = 2
    while k <= len(pts):
        if k == len(pts) or not sampled_log_on_mask(grid, mask, out[-1], pts[k]):
            out.append(pts[k-1])
            last_keep = k - 1
        k += 1
    return out

def decimate_by_spacing(points: List[Vec2], min_dist: float) -> List[Vec2]:
    """
    Keep the first and last; between them, keep points at least 'min_dist' apart.
    """
    if len(points) <= 2 or min_dist <= 0:
        return points[:]
    out = [points[0]]
    lastx, lasty = out[0]
    md2 = min_dist * min_dist
    for p in points[1:-1]:
        dx = p[0] - lastx
        dy = p[1] - lasty
        if dx * dx + dy * dy >= md2:
            out.append(p)
            lastx, lasty = p
    out.append(points[-1])
    return out