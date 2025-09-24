import heapq
import math
from typing import List, Optional

import numpy as np

from .types import IVec2

NEIGH = [(-1,-1), (0, -1),  (1, -1),
         (-1,  0),          ( 1, 0),
         (-1, 1), (0,  1),  (1, 1)]

def astar_bool_grid(trav: np.ndarray, start:IVec2, goal:IVec2) -> Optional[List[IVec2]]:
    H, W = trav.shape
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < W and 0 <= sy < H and 0 <= gx < W and 0 <= gy<H):
        return None
    if not (trav[sy,sx] and trav[gy,gx]):
        return None

    h = lambda x, y: math.hypot(x - gx, y - gy)
    openq = [(h(sx,sy), 0.0, (sx,sy))]
    best_g = { (sx,sy): 0.0}
    came = {}

    while openq:
        f, g, (x,y) = heapq.heappop(openq)
        if (x,y) == (gx,gy):
            path = [(x,y)]
            while (x,y) in came:
                x, y = came[(x,y)]
                path.append((x,y))
            path.reverse()
            return path

        for dx,dy in NEIGH:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            if not trav[ny,nx]:
                continue

            #this should stop the diagonal traversal of blocked cells
            if dx and dy:
                if not (trav[y,nx] and trav[ny,x]): #both adjacents must be free
                    continue

            step = 1.4142 if dx and dy else 1.0
            ng = g + step
            if ng < best_g.get((nx,ny), 1e30):
                best_g[nx,ny] = ng
                came[nx,ny] = (x,y)
                heapq.heappush(openq, (ng + h(nx, ny), ng, (nx, ny)))
    return None

