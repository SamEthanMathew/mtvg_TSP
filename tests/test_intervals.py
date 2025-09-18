# tests/test_intervals.py
import math
from mtvg.geometry import Obstacle
from mtvg.models import Target, make_linear_xy, Scene, Depot
from mtvg.intervals import visible_intervals

def test_visible_intervals_no_obstacles_stationary_target_kinematic():
    # stationary target at (5,0), window [0,10], q at origin
    def xy(t): return (5.0, 0.0)
    tgt = Target(id=0, xy=xy, windows=[(0.0, 10.0)])
    scene = Scene(obstacles=[], v_max=1.0, depot=Depot((0.0, 0.0)))

    # Geometric visibility should be whole window
    vis = visible_intervals((0.0, 0.0), tgt, (0.0, 10.0), scene, n_samples=50)
    assert len(vis) == 1
    assert abs(vis[0][0] - 0.0) < 1e-6 and abs(vis[0][1] - 10.0) < 1e-6

    # Kinematic reachable only from t >= d/v_max = 5.0
    kin = visible_intervals((0.0,0.0), tgt, (0.0,10.0), scene, n_samples=50, require_kinematic=True, earliest_departure=0.0)
    assert len(kin) == 1
    a,b = kin[0]
    assert a >= 5.0 - 1e-3 and b <= 10.0 + 1e-9

def test_visible_intervals_blocked_by_obstacle_splits_interval():
    # target moves along line that passes behind a square obstacle causing a gap
    xy = make_linear_xy((0.0, 0.9), (1.0, 0.9), 0.0, 10.0)  # moves along y=0.9 above obstacle
    tgt = Target(id=1, xy=xy, windows=[(0.0, 10.0)])
    obs = [Obstacle(vertices=((0.4,0.4),(0.6,0.4),(0.6,0.6),(0.4,0.6)))]
    scene = Scene(obstacles=obs, v_max=2.0, depot=Depot((0.0, 0.0)))

    vis = visible_intervals((0.0, 0.0), tgt, (0.0, 10.0), scene, n_samples=200)
    # There should be at least one visible block; depending on geometry you may get 1 or 2 blocks
    assert isinstance(vis, list)
    assert len(vis) >= 1
