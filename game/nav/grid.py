from typing import List

import numpy as np

from .types import Grid, GridSpec, Vec2


def point_in_polygon(pt: Vec2, poly: List[Vec2]) -> bool:
    # Even-odd rule (ray cast)
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        # check if edge crosses scanline
        if (y1 > y) != (y2 > y):
            x_at_y = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1
            if x < x_at_y:
                inside = not inside
    return inside


def rasterize_water(spec: GridSpec, land_polys: List[List[Vec2]]) -> Grid:
    H, W = spec.size_y, spec.size_x
    water = np.ones((H, W), dtype=bool)
    ox, oy = spec.origin_m
    cs = spec.cell_m
    for j in range(H):
        cy = oy + (j + 0.5) * cs
        for i in range(W):
            cx = ox + (i + 0.5) * cs
            #if point is inside a land polygone it is NOT water.
            for poly in land_polys:
                if point_in_polygon((cx,cy), poly):
                    water[j,i] = False
                    break
    return Grid(spec=spec, water=water)


# --- Coarse Grid Helpers ---

def make_coarse_spec_from_fine(fine: GridSpec, coarse_cell_m: float) -> GridSpec:
    sx = max(1, int((fine.size_x * fine.cell_m) // coarse_cell_m))
    sy = max(1, int((fine.size_y * fine.cell_m) // coarse_cell_m))
    return GridSpec(cell_m=coarse_cell_m, size_x=sx, size_y=sy, origin_m=fine.origin_m)

def downsample_water_to_coarse(fine: GridSpec, coarse_spec: GridSpec) -> Grid:
    """Mark a coarse cell as WATER if all fine cells that overlap are WATER
    This should prevent 'threading the needle' at a macro scale to improve pathfinding"""
    import numpy as np
    Hc,Wc = coarse_spec.size_y, coarse_spec.size_x
    water_c = np.ones((Hc,Wc), dtype=bool)

    fxm = fine.spec.cell_m
    cxm = coarse_spec.cell_m

    for jc in range(Hc):
        for ic in range(Wc):
            #Creating the coarse cell bounds in world meters
            ox, oy = coarse_spec.origin_m
            x0 = ox + ic * cxm
            y0 = oy +jc * cxm
            x1 = x0 + cxm
            y1 = y0 + cxm
            #now overlap fine index's range
            i0 = max(0, int((x0 - fine.spec.origin_m[0]) // fxm))
            j0 = max(0, int((y0 - fine.spec.origin_m[1]) // cxm))
            i1 = min(fine.spec.size_x, int((x1 - fine.spec.origin_m[0] + fxm - 1e-6) // fxm))
            j1 = min(fine.spec.size_y, int((y1 - fine.spec.origin_m[1] + fxm - 1e-6) // fxm))
            #if ANY overlapping fine cells are LAND make the entire coarse cell LAND (water_c = False)
            if not fine.water[j0:j1, i0:i1].all():
                water_c[jc, ic] = False
    return Grid(spec=coarse_spec, water=water_c)