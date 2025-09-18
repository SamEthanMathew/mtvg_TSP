# mtvg/visibility_graph.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Tuple
import math
from .geometry import Coord, Obstacle, make_obstacles_union, visible, extract_convex_vertices

NodeId = int

@dataclass
class Graph:
    nodes: List[Coord]
    adj: Dict[NodeId, List[Tuple[NodeId, float]]]

    @classmethod
    def empty(cls) -> "Graph":
        return cls(nodes=[], adj={})

    def add_node(self, coord: Coord) -> NodeId:
        nid = len(self.nodes)
        self.nodes.append(coord)
        self.adj[nid] = []
        return nid

    def add_undirected_edge(self, u: NodeId, v: NodeId, w: float) -> None:
        self.adj[u].append((v, w))
        self.adj[v].append((u, w))

def euclid(a: Coord, b: Coord) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _unique_points(pts: List[Coord], tol: float = 1e-9) -> List[Coord]:
    """Deduplicate points with a small tolerance."""
    out: List[Coord] = []
    for x, y in pts:
        is_dup = False
        for ux, uy in out:
            if abs(x - ux) <= tol and abs(y - uy) <= tol:
                is_dup = True
                break
        if not is_dup:
            out.append((float(x), float(y)))
    return out

def build_visibility_graph(obstacles: List[Obstacle], points: List[Coord]) -> Graph:
    """
    Nodes = user-supplied points ∪ convex obstacle vertices.
    Add an edge iff the segment is collision-free; weight = Euclidean.
    """
    convex = extract_convex_vertices(obstacles)
    all_points = _unique_points(points + convex)

    G = Graph.empty()
    for p in all_points:
        G.add_node(p)

    obs_union = make_obstacles_union(obstacles)

    n = len(all_points)
    for i in range(n):
        pi = all_points[i]
        for j in range(i + 1, n):
            pj = all_points[j]
            if visible(pi, pj, obs_union):
                G.add_undirected_edge(i, j, euclid(pi, pj))
    return G

def dijkstra(G: Graph, src: NodeId, dst: NodeId) -> Tuple[float, List[NodeId]]:
    import heapq
    n = len(G.nodes)
    dist = [math.inf] * n
    prev = [-1] * n
    dist[src] = 0.0
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]: continue
        if u == dst: break
        for v, w in G.adj[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if not math.isfinite(dist[dst]): return math.inf, []
    path = []
    cur = dst
    while cur != -1:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return dist[dst], path


