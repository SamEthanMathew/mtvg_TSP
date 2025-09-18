from mtvg.geometry import Obstacle
from mtvg.visibility_graph import build_visibility_graph, dijkstra

obs = [Obstacle(vertices=((0.4,0.4),(0.6,0.4),(0.6,0.6),(0.4,0.6)))]
pts = [(0,0), (1,1), (0,1), (1,0)]

G = build_visibility_graph(obs, pts)
dist, path = dijkstra(G, 0, 1)

print("distance:", dist, "path:", path)
