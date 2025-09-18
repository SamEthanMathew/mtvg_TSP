# mtvg/models.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Iterable, List, Tuple, Optional, Dict
from .geometry import Obstacle, Coord

# A target’s trajectory: t -> (x, y)
XYFunc = Callable[[float], Coord]
Window = Tuple[float, float]  # (t0, tf)

@dataclass(frozen=True)
class Depot:
    xy: Coord

@dataclass
class Target:
    """
    Moving target with (possibly multiple) time windows.
    - xy(t): R -> R^2 gives position at time t (assume piecewise-constant velocity within each window).
    - windows: list of (t0, tf) with t0 < tf.
    - name: optional label.
    """
    id: int
    xy: XYFunc
    windows: List[Window]
    name: str = "target"

    def domain(self) -> Window:
        """Overall time span covered by windows."""
        if not self.windows:
            return (0.0, 0.0)
        return (min(t0 for t0, _ in self.windows), max(tf for _, tf in self.windows))

@dataclass
class Scene:
    """
    Planning scene.
    - obstacles: polygon obstacles
    - v_max: agent's max speed (must be >= any target speed in windows, per paper assumptions)
    - depot: starting/ending location
    """
    obstacles: List[Obstacle]
    v_max: float
    depot: Depot

    def as_points(self) -> List[Coord]:
        """Convenience: points that must be in the visibility graph (at least the depot)."""
        return [self.depot.xy]

# Convenience generator for straight-line trajectories (constant velocity)
def make_linear_xy(p0: Coord, p1: Coord, t0: float, tf: float) -> XYFunc:
    """
    Returns xy(t) that moves from p0 at t0 to p1 at tf (clamped outside [t0, tf]).
    """
    x0, y0 = p0
    dx = p1[0] - x0
    dy = p1[1] - y0
    T = max(tf - t0, 1e-12)

    def xy(t: float) -> Coord:
        if t <= t0: return (x0, y0)
        if t >= tf: return (p1[0], p1[1])
        alpha = (t - t0) / T
        return (x0 + alpha * dx, y0 + alpha * dy)

    return xy
