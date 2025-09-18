#!/usr/bin/env python3

from mtvg.geometry import Obstacle, extract_convex_vertices, make_obstacles_union, visible
from mtvg.visibility_graph import build_visibility_graph, dijkstra

# Test case from the failing test
obs = [Obstacle(vertices=((0.4,0.4),(0.6,0.4),(0.6,0.6),(0.4,0.6)))]
pts = [(0.0, 0.0), (1.0, 1.0)]

print("=== Debug Analysis ===")
print(f"Obstacle vertices: {obs[0].vertices}")
print(f"Test points: {pts}")

# Check convex vertices
convex = extract_convex_vertices(obs)
print(f"Convex vertices: {convex}")

# Check obstacle union
obs_union = make_obstacles_union(obs)
print(f"Obstacle union: {obs_union}")

# Build visibility graph
G = build_visibility_graph(obs, pts)
print(f"Graph nodes: {G.nodes}")
print(f"Graph adjacency: {G.adj}")

# Test visibility between key points
print("\n=== Visibility Tests ===")
for i, p1 in enumerate(G.nodes):
    for j, p2 in enumerate(G.nodes):
        if i < j:
            is_visible = visible(p1, p2, obs_union)
            print(f"Visibility {p1} -> {p2}: {is_visible}")

# Test pathfinding
print("\n=== Pathfinding ===")
dist, path = dijkstra(G, 0, 1)
print(f"Distance from (0,0) to (1,1): {dist}")
print(f"Path: {path}")
if path:
    print("Path coordinates:")
    for node_id in path:
        print(f"  Node {node_id}: {G.nodes[node_id]}")
