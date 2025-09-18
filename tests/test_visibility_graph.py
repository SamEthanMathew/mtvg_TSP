# tests/test_visibility_graph.py
import math
from mtvg.geometry import Obstacle
from mtvg.visibility_graph import build_visibility_graph, dijkstra

def test_no_obstacles_complete_graph():
    pts = [(0,0), (1,0), (1,1), (0,1)]
    G = build_visibility_graph([], pts)
    # Each node should connect to 3 others in a square (complete)
    assert all(len(G.adj[i]) == 3 for i in range(4))
    dist, path = dijkstra(G, 0, 2)
    assert math.isclose(dist, math.sqrt(2), rel_tol=1e-6)
    assert path in ([0,2], [0,1,2], [0,3,2])  # allow direct or alt if identical costs

def test_center_box_blocks_diagonal():
    obs = [Obstacle(vertices=((0.4,0.4),(0.6,0.4),(0.6,0.6),(0.4,0.6)))]
    pts = [(0,0), (1,1), (0,1), (1,0)]
    G = build_visibility_graph(obs, pts)
    d_direct = math.sqrt(2)
    dist, _ = dijkstra(G, 0, 1)  # (0,0) -> (1,1)
    assert dist > d_direct  # must go around the box
    assert math.isfinite(dist)
