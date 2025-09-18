from mtvg.geometry import Obstacle
from mtvg.visibility_graph import build_visibility_graph
from mtvg.viz import PolyObstacle, draw_scene
import matplotlib.pyplot as plt

# Scene
obs = [Obstacle(vertices=((0.4,0.4),(0.6,0.4),(0.6,0.6),(0.4,0.6)))]
pts = [(0,0), (1,1), (0,1), (1,0)]

# Build VG
G = build_visibility_graph(obs, pts)

# Convert for viz
poly_obs = [PolyObstacle(vertices=o.vertices) for o in obs]
edges = []
for i, nbrs in G.adj.items():
    for j, _w in nbrs:
        if j > i:  # avoid double-plotting
            edges.append((i, j, G.nodes[i], G.nodes[j]))

ax = draw_scene(poly_obs, depot=None, static_points=G.nodes, visibility_edges=edges)
ax.set_title("Visibility graph: edges that avoid the box obstacle")

# Highlight a specific shortest path (0 -> 1) if you want:
# (For now we don't recompute it; just draw a candidate route around the box.)
# import math
# from mtvg.visibility_graph import dijkstra
# dist, path = dijkstra(G, 0, 1)
# xs = [G.nodes[k][0] for k in path]
# ys = [G.nodes[k][1] for k in path]
# ax.plot(xs, ys, linewidth=2)

plt.show()
