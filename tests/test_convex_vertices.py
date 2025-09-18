import math
from mtvg.geometry import Obstacle
from mtvg.visibility_graph import build_visibility_graph, dijkstra

# Old behavior: a local builder WITHOUT convex-vertex augmentation
def build_visibility_graph_no_aug(obstacles, points):
    # — minimal copy of your pre-1B logic —
    from mtvg.geometry import make_obstacles_union, visible
    from mtvg.visibility_graph import Graph, euclid
    G = Graph.empty()
    for p in points: G.add_node(p)
    union = make_obstacles_union(obstacles)
    n = len(points)
    for i in range(n):
        for j in range(i + 1, n):
            if visible(points[i], points[j], union):
                G.add_undirected_edge(i, j, euclid(points[i], points[j]))
    return G

def test_connectivity_improves_with_convex_vertices():
    # A square obstacle in the middle; start/end on opposite corners.
    obs = [Obstacle(vertices=((0.4,0.4),(0.6,0.4),(0.6,0.6),(0.4,0.6)))]
    pts = [(0.0, 0.0), (1.0, 1.0)]  # just the endpoints; no intermediate nodes

    G_no = build_visibility_graph_no_aug(obs, pts)
    d_no, path_no = dijkstra(G_no, 0, 1)
    # With no augmentation, there is no way to go around (graph has only 2 nodes)
    assert not math.isfinite(d_no), "Unexpected: path exists without convex vertices."

    G_yes = build_visibility_graph(obs, pts)  # WITH convex vertex augmentation
    d_yes, path_yes = dijkstra(G_yes, 0, 1)
    assert math.isfinite(d_yes), "Path should exist once convex corners are added."
    # The length must be > sqrt(2) since we must route around the box
    assert d_yes > math.sqrt(2)

def test_no_regression_in_free_space():
    obs = []  # no obstacles
    pts = [(0,0), (1,0), (1,1), (0,1)]
    G_no = build_visibility_graph_no_aug(obs, pts)
    G_yes = build_visibility_graph(obs, pts)
    # Distances should match between any pair (no extra corners to help/hurt)
    import itertools
    for i, j in itertools.combinations(range(len(pts)), 2):
        d_no, _ = dijkstra(G_no, i, j)
        d_yes, _ = dijkstra(G_yes, i, j)
        assert math.isfinite(d_no) and math.isfinite(d_yes)
        assert abs(d_no - d_yes) <= 1e-9
