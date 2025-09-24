import numpy as np

from .types import Grid


def distance_field_cells(obstacle: np.ndarray) -> np.ndarray:
    """Return distance in cells to the nearest obstacle. True = Obstacle.
    Chamfer distance in pure numpy; good enough to start"""
    H,W = obstacle.shape
    INF = 10**9
    dist = np.where(obstacle, 0, INF).astype(np.int32)

    #forward pass
    for y in range(H):
        for x in range(W):
            d = dist[y,x]
            if y > 0:
                d = min(d, dist[y-1, x] +1)

                if x > 0:
                    d = min(d, dist[y-1, x-1]+2)

                if x < W-1:
                    d = min(d, dist[y-1, x+1]+2)

            if x > 0:
                d = min(d, dist[y, x-1]+1)
            dist[y,x] = d

    return dist.astype(np.float32)

def clearance_mask_for_ship(grid: Grid, ship_radius_m: float, safety_m: float = 20) -> np.ndarray:
    """Given water occupancy, return a boolean mask of cells traversable by a ship with hull
    radius + safety clearance. Use a distance field so larger ships are, naturally kept
    further away from land."""

    #Make obstacles land NOT water
    land = ~grid.water
    dist_cells = distance_field_cells(land)

    #convert threshold from meters to cells using average of 4-neighbor cost
    threshold_m = ship_radius_m + safety_m
    threshold_cells = threshold_m / grid.spec.cell_m
    return (dist_cells >= threshold_cells) & grid.water

