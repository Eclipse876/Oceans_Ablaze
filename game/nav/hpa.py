import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .types import Grid, GridSpec, IVec2, Vec2


@dataclass(frozen=True)
class ClusterId:
    cx:int
    cy:int


@dataclass
class Portal:
    a_cluster: ClusterId
    b_cluster: ClusterId
    a_cell: IVec2 #Coarse cells with coords on A side (i,j)
    b_cell: IVec2 #Coarse cells with coords on B side (i,j)


@dataclass
class MacroGraph:
    spec: GridSpec
    cluster_size: int
    clusters: List[ClusterId]
    portals: List[Portal]
    # adjacency: Cluster -> list[(neighbor_cluster, portal_index, cost)]
    adj: Dict[ClusterId, List[Tuple[ClusterId, int, float]]]


def build_clusters(coarse: Grid, cluster_size: int) -> List[ClusterId]:
    Cx = math.ceil(coarse.spec.size_x / cluster_size)
    Cy = math.ceil(coarse.spec.size_y / cluster_size)
    return [ClusterId(x, y) for y in range(Cy) for x in range(Cx)]


def cluster_bounds(cluster: ClusterId, coarse: Grid, K: int) -> Tuple[int,int,int,int]:
    i0 = cluster.cx * K
    j0 = cluster.cy * K
    i1 = min(coarse.spec.size_x, i0 + K)
    j1 = min(coarse.spec.size_y, j0 + K)
    return i0, j0, i1, j1


def find_portals_between(coarse: Grid, K: int, A: ClusterId, B: ClusterId) -> List[Portal]:
    ports: List[Portal] = []
    # neighbors only if they share an edge
    dx = B.cx - A.cx; dy = B.cy - A.cy
    if abs(dx) + abs(dy) != 1:  # only 4-neighbors for simplicity
        return ports
    ai0, aj0, ai1, aj1 = cluster_bounds(A, coarse, K)
    bi0, bj0, bi1, bj1 = cluster_bounds(B, coarse, K)

    if dx == 1:  # B to the right of A: shared vertical edge at i = ai1-1 / bi0
        iA = ai1 - 1
        iB = bi0
        for j in range(max(aj0, bj0), min(aj1, bj1)):
            # water must be on both sides to make a portal
            if coarse.water[j, iA] and coarse.water[j, iB]:
                ports.append(Portal(A, B, (iA, j), (iB, j)))
    elif dx == -1:  # B to the left
        iA = ai0
        iB = bi1 - 1
        for j in range(max(aj0, bj0), min(aj1, bj1)):
            if coarse.water[j, iA] and coarse.water[j, iB]:
                ports.append(Portal(A, B, (iA, j), (iB, j)))
    elif dy == 1:  # B above A: shared horizontal edge at j = aj1-1 / bj0
        jA = aj1 - 1
        jB = bj0
        for i in range(max(ai0, bi0), min(ai1, bi1)):
            if coarse.water[jA, i] and coarse.water[jB, i]:
                ports.append(Portal(A, B, (i, jA), (i, jB)))
    elif dy == -1:  # B below
        jA = aj0
        jB = bj1 - 1
        for i in range(max(ai0, bi0), min(ai1, bi1)):
            if coarse.water[jA, i] and coarse.water[jB, i]:
                ports.append(Portal(A, B, (i, jA), (i, jB)))
    return ports


def build_macro_graph(coarse: Grid, cluster_size: int = 8) -> MacroGraph:
    clusters = build_clusters(coarse, cluster_size)
    adj: Dict[ClusterId, List[Tuple[ClusterId, int, float]]] = {c: [] for c in clusters}
    all_ports: List[Portal] = []

    # map for quick index lookup
    port_index_by_pair: Dict[Tuple[ClusterId, ClusterId, Tuple[int,int], Tuple[int,int]], int] = {}

    # neighbor directions (4-neighbor clusters)
    dirs = [(1,0), (-1,0), (0,1), (0,-1)]
    for c in clusters:
        for dx,dy in dirs:
            n = ClusterId(c.cx+dx, c.cy+dy)
            # bounds check
            i0,j0,i1,j1 = cluster_bounds(c, coarse, cluster_size)
            if n not in adj:  # might be outside edge
                continue
            # build portals between c and n
            ports = find_portals_between(coarse, cluster_size, c, n)
            for p in ports:
                key = (p.a_cluster, p.b_cluster, p.a_cell, p.b_cell)
                if key in port_index_by_pair:
                    idx = port_index_by_pair[key]
                else:
                    idx = len(all_ports)
                    all_ports.append(p)
                    port_index_by_pair[key] = idx
                # cost = straight-line distance between portal cells (could be 1)
                cost = math.hypot(p.b_cell[0] - p.a_cell[0], p.b_cell[1] - p.a_cell[1])
                adj[c].append((n, idx, cost))

    return MacroGraph(spec=coarse.spec, cluster_size=cluster_size,
                      clusters=clusters, portals=all_ports, adj=adj)

def coarse_cell_of_world(spec: GridSpec, p: Vec2) -> IVec2:
    i = int((p[0] - spec.origin_m[0]) // spec.cell_m)
    j = int((p[1] - spec.origin_m[1]) // spec.cell_m)
    return (min(max(i, 0), spec.size_x - 1),
            min(max(j, 0), spec.size_y - 1))

def cluster_of_cell(g: MacroGraph, cell: IVec2) -> ClusterId:
    return ClusterId(cell[0] // g.cluster_size, cell[1] // g.cluster_size)

def macro_astar(g: MacroGraph, start_world: Vec2, goal_world: Vec2) -> Optional[List[Portal]]:
    s_cell = coarse_cell_of_world(g.spec, start_world)
    t_cell = coarse_cell_of_world(g.spec, goal_world)
    s_c = cluster_of_cell(g, s_cell)
    t_c = cluster_of_cell(g, t_cell)

    # trivial case: same cluster (no portals needed)
    if s_c == t_c:
        return []

    # cluster-graph A*
    def h(c: ClusterId) -> float:
        return math.hypot(c.cx - t_c.cx, c.cy - t_c.cy)

    openq = [(h(s_c), 0.0, s_c, None)]  # (f, g, cluster, via_port_idx)
    best_g = {s_c: 0.0}
    came: Dict[ClusterId, Tuple[ClusterId, int]] = {}

    while openq:
        f, gc, c, via_port = heapq.heappop(openq)
        if c == t_c:
            # reconstruct portals
            seq: List[Portal] = []
            cur = c
            while cur != s_c:
                prev, port_idx = came[cur]
                seq.append(g.portals[port_idx])
                cur = prev
            seq.reverse()
            return seq

        for (n, port_idx, cost) in g.adj.get(c, []):
            ng = gc + cost
            if ng < best_g.get(n, 1e30):
                best_g[n] = ng
                came[n] = (c, port_idx)
                heapq.heappush(openq, (ng + h(n), ng, n, port_idx))

    return None