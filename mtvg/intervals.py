# mtvg/intervals.py
from __future__ import annotations
from typing import Callable, List, Tuple, Optional
import math

from .geometry import Coord, make_obstacles_union, visible
from .models import Target, Window, Scene

# Type aliases
TimeInterval = Tuple[float, float]

def _sample_times(t0: float, tf: float, n: int) -> List[float]:
    if n <= 1:
        return [t0, tf]
    return [t0 + (tf - t0) * i / (n - 1) for i in range(n)]

def _binary_refine_visibility(
    q: Coord,
    tau: Callable[[float], Coord],
    t_lo: float,
    t_hi: float,
    obs_union,
    want_visible: bool,
    tol: float = 1e-5,
    max_iter: int = 40,
) -> float:
    """
    Find boundary time between visible and non-visible inside [t_lo, t_hi].
    Assumes at t_lo visibility == want_visible and at t_hi visibility != want_visible.
    Returns the boundary t where visibility changes (approx).
    """
    a, b = t_lo, t_hi
    fa = visible(q, tau(a), obs_union)
    # We expect fa == want_visible
    for _ in range(max_iter):
        m = 0.5 * (a + b)
        fm = visible(q, tau(m), obs_union)
        if fm == want_visible:
            a = m
            fa = fm
        else:
            b = m
        if abs(b - a) < tol:
            break
    # return the endpoint that is visible -> so if want_visible True, return a (last visible)
    return a if want_visible else b

def visible_intervals(
    q: Coord,
    target: Target,
    window: Window,
    scene: Scene,
    *,
    n_samples: int = 300,
    refine_tol: float = 1e-5,
    earliest_departure: Optional[float] = None,
    require_kinematic: bool = False,
) -> List[TimeInterval]:
    """
    Compute time intervals within `window` where the straight segment q -> tau(t)
    is collision-free. Optionally filter those times by kinematic reachability:
      dist(q, tau(t)) / v_max <= t - earliest_departure
    (i.e., agent can depart at earliest_departure or later and still reach by t,
    waiting at q allowed).

    Args:
      q: static point (Coord)
      target: Target object with xy(t)
      window: (t0, tf)
      scene: Scene (contains obstacles and v_max)
      n_samples: number of time samples used to find candidate intervals
      refine_tol: binary search tolerance to refine interval endpoints
      earliest_departure: earliest time agent can leave q (if None, skip kinematic check)
      require_kinematic: if True, apply the kinematic filter (requires earliest_departure not None)

    Returns:
      list of disjoint closed intervals [(a,b), ...] with a < b (or empty).
    """
    t0, tf = window
    if tf <= t0:
        return []

    obs_union = make_obstacles_union(scene.obstacles)
    tau = target.xy

    # 1) sample times
    times = _sample_times(t0, tf, n_samples)
    vis_flags = [visible(q, tau(t), obs_union) for t in times]

    # 2) find contiguous true segments in vis_flags -> candidate intervals
    intervals: List[TimeInterval] = []
    i = 0
    while i < len(times):
        if vis_flags[i]:
            j = i
            while j + 1 < len(times) and vis_flags[j + 1]:
                j += 1
            # times[i]..times[j] is a candidate visible block; refine endpoints
            left = times[i]
            right = times[j]
            # refine left boundary: find last non-visible point before left (if exists)
            if i > 0 and not vis_flags[i - 1]:
                left = _binary_refine_visibility(q, tau, times[i - 1], times[i], obs_union, True, tol=refine_tol)
            # refine right boundary: find first non-visible after right (if exists)
            if j + 1 < len(times) and not vis_flags[j + 1]:
                right = _binary_refine_visibility(q, tau, times[j], times[j + 1], obs_union, True, tol=refine_tol)
            intervals.append((left, right))
            i = j + 1
        else:
            i += 1

    # 3) Optionally apply kinematic reachability filter
    if require_kinematic:
        if earliest_departure is None:
            raise ValueError("earliest_departure must be provided when require_kinematic=True")
        v_max = scene.v_max
        kin_intervals: List[TimeInterval] = []
        for (a, b) in intervals:
            # For kinematic feasibility we need times t where dist(q, tau(t)) / v_max <= t - earliest_departure
            # Define f(t) = t - (dist(q,t)/v_max) - earliest_departure. We need f(t) >= 0.
            def f(t: float) -> float:
                p = tau(t)
                d = math.hypot(q[0] - p[0], q[1] - p[1])
                return t - (d / v_max) - earliest_departure

            # If f(b) < 0, no feasible t in [a,b]
            if f(b) < 0:
                continue
            # If f(a) >= 0, whole interval feasible
            if f(a) >= 0:
                kin_intervals.append((a, b))
                continue
            # Otherwise find root in (a,b] where f(t)=0 via binary search
            lo, hi = a, b
            for _ in range(50):
                mid = 0.5 * (lo + hi)
                if f(mid) >= 0:
                    hi = mid
                else:
                    lo = mid
                if hi - lo < refine_tol:
                    break
            kin_intervals.append((hi, b))
        intervals = kin_intervals

    return intervals
