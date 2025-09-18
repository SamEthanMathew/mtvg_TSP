# mtvg/geometry.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable, Tuple
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
from typing import List

Coord = Tuple[float, float]

@dataclass(frozen=True)
class Obstacle:
    """Simple polygon obstacle. Vertices should be CCW and non-self-intersecting."""
    vertices: Tuple[Coord, ...]
    def polygon(self) -> Polygon:
        return Polygon(self.vertices)

def make_obstacles_union(obstacles: Iterable[Obstacle]) -> Polygon:
    """Union of all obstacles (used for fast intersection tests)."""
    polys = [o.polygon() for o in obstacles]
    return unary_union(polys) if polys else Polygon()

def segment_intersects_obstacles(p0: Coord, p1: Coord, obstacles_union: Polygon, *, eps: float = 1e-9) -> bool:
    """Conservative: treat boundary touches as blocked using a tiny buffer."""
    if obstacles_union.is_empty:
        return False
    seg = LineString([p0, p1])
    return seg.intersects(obstacles_union.buffer(eps))

def visible(p0: Coord, p1: Coord, obstacles_union: Polygon) -> bool:
    """True iff the straight segment p0->p1 stays in free space (no intersection)."""
    return not segment_intersects_obstacles(p0, p1, obstacles_union)

def extract_convex_vertices(obstacles: list[Obstacle]) -> list[Coord]:
    convex_vertices = []
    for obs in obstacles:
        poly = obs.polygon()
        coords = list(poly.exterior.coords)[:-1]
        n = len(coords)
        for i in range(n):
            p_prev = coords[(i - 1) % n]
            p_cur = coords[i]
            p_next = coords[(i + 1) % n]

            v1 = (p_cur[0] - p_prev[0], p_cur[1] - p_prev[1])
            v2 = (p_next[0] - p_cur[0], p_next[1] - p_cur[1])
            cross = v1[0] * v2[1] - v1[1] * v2[0]

            # For CCW polygons (as specified in Obstacle docstring), convex vertices have cross > 0
            if cross > 0:
                convex_vertices.append(p_cur)
    return convex_vertices

def _near(a, b, tol: float = 1e-9) -> bool:
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol

def segment_intersects_obstacles(p0: Coord, p1: Coord, obstacles_union: Polygon, *, eps: float = 1e-9) -> bool:
    """
    Conservative intersection test, but allow a segment to *touch* an obstacle exactly
    at an endpoint (so we can connect to convex vertices). Still block any true crossing
    or overlap along edges.
    """
    if obstacles_union.is_empty:
        return False

    seg = LineString([p0, p1])

    if not seg.intersects(obstacles_union):
        return False

    inter = seg.intersection(obstacles_union)

    if inter.is_empty:
        return False

    if inter.geom_type == "Point":
        pt = (inter.x, inter.y)
        if _near(pt, p0, eps) or _near(pt, p1, eps):
            return False  # touching at endpoint is OK

    if inter.geom_type == "MultiPoint":
        pts = [(g.x, g.y) for g in inter.geoms]
        if all(_near(pt, p0, eps) or _near(pt, p1, eps) for pt in pts):
            return False

    return True