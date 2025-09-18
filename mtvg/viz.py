"""
Basic visualization helpers for MTVG-TSP.
"""

from dataclasses import dataclass
from typing import Sequence, Tuple, Optional
import matplotlib.pyplot as plt

Coord = Tuple[float, float]

@dataclass
class PolyObstacle:
    vertices: Sequence[Coord]

def draw_scene(
    obstacles: Sequence[PolyObstacle],
    depot: Optional[Coord] = None,
    static_points: Optional[Sequence[Coord]] = None,
    visibility_edges: Optional[Sequence[Tuple[int, int, Coord, Coord]]] = None,
    ax: Optional[plt.Axes] = None,
) -> plt.Axes:
    """Draw obstacles, points, and edges."""
    if ax is None:
        fig, ax = plt.subplots()

    # Obstacles
    for obs in obstacles:
        xs = [p[0] for p in obs.vertices] + [obs.vertices[0][0]]
        ys = [p[1] for p in obs.vertices] + [obs.vertices[0][1]]
        ax.fill(xs, ys, alpha=0.3, color="gray", edgecolor="black")

    # Depot
    if depot:
        ax.scatter([depot[0]], [depot[1]], marker="*", s=150, label="depot")

    # Points
    if static_points:
        xs = [p[0] for p in static_points]
        ys = [p[1] for p in static_points]
        ax.scatter(xs, ys, c="blue", s=30, label="nodes")

    # Edges
    if visibility_edges:
        for _, _, pi, pj in visibility_edges:
            ax.plot([pi[0], pj[0]], [pi[1], pj[1]], c="blue", alpha=0.5)

    ax.set_aspect("equal", "box")
    ax.grid(True, alpha=0.2)
    return ax
