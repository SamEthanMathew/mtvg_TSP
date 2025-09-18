#!/usr/bin/env python3

from mtvg.geometry import Obstacle

# Test case from the failing test
obs = [Obstacle(vertices=((0.4,0.4),(0.6,0.4),(0.6,0.6),(0.4,0.6)))]

print("=== Convex Vertex Analysis ===")
print(f"Obstacle vertices: {obs[0].vertices}")

poly = obs[0].polygon()
coords = list(poly.exterior.coords)[:-1]
print(f"Polygon coordinates: {coords}")

n = len(coords)
print(f"Number of vertices: {n}")

for i in range(n):
    p_prev = coords[(i - 1) % n]
    p_cur = coords[i]
    p_next = coords[(i + 1) % n]
    
    v1 = (p_cur[0] - p_prev[0], p_cur[1] - p_prev[1])
    v2 = (p_next[0] - p_cur[0], p_next[1] - p_cur[1])
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    
    print(f"Vertex {i}: {p_cur}")
    print(f"  Previous: {p_prev}, Next: {p_next}")
    print(f"  Vector 1: {v1}, Vector 2: {v2}")
    print(f"  Cross product: {cross}")
    print(f"  Is convex (cross > 0): {cross > 0}")
    print()
