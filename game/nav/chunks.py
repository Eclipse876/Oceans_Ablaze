from typing import Tuple

from .types import Vec2

CHUNK_SIZE_M = 2048.0

def world_to_chunk_ix(pos_m: Vec2) -> Tuple[int, int]:
    x, y = pos_m
    return int(x // CHUNK_SIZE_M), int(y // CHUNK_SIZE_M)

def chunk_origin_m(ix: Tuple[int, int]) -> Vec2:
    cx, cy = ix
    return cx * CHUNK_SIZE_M, cy * CHUNK_SIZE_M