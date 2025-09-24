from dataclasses import dataclass
from enum import Enum, IntFlag
from typing import Tuple

Vec2 = Tuple[float, float]
IVec2 = Tuple[int, int]


class NavLayer(Enum):
    WATER = 1
    AIR = 2


class TraversalMask(IntFlag):
    NONE = 0
    WATER = 1 << 0
    AIR = 1 << 1


@dataclass(frozen=True)
class GridSpec:
    cell_m: float  # Meters per cell, using pixels but 1 px = 1M
    size_x: int  # Width in cells
    size_y: int  # Height in cells
    origin_m: Vec2  # World origin (meters) of the grid (lower-left)


@dataclass
class Grid:
    spec: GridSpec
    # True = WATER cell. False = LAND cell.
    water: "object"
