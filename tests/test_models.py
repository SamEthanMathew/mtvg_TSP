# tests/test_models.py
from mtvg.models import Scene, Depot, Target, make_linear_xy
from mtvg.geometry import Obstacle

def test_models_linear_target_single_window():
    scene = Scene(obstacles=[], v_max=1.0, depot=Depot((0.0, 0.0)))

    t0, tf = 0.0, 10.0
    xy = make_linear_xy((0.0, 0.0), (10.0, 0.0), t0, tf)
    tgt = Target(id=0, xy=xy, windows=[(t0, tf)], name="T0")

    # basic sampling
    assert tgt.xy(0.0) == (0.0, 0.0)
    mid = tgt.xy(5.0)
    assert abs(mid[0] - 5.0) < 1e-9 and abs(mid[1] - 0.0) < 1e-9
    assert tgt.xy(12.0) == (10.0, 0.0)

    # domain spans the window
    assert tgt.domain() == (t0, tf)

    # scene convenience
    assert scene.as_points() == [scene.depot.xy]
