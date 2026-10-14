"""Frontier detection/clustering and wander goal sampling on synthetic grids (no ROS)."""

import math

from cpsl_nav import grid as g
import numpy as np
import pytest

U, F, OCC = -1, 0, 100
RES = 0.1


def room(h=40, w=60, known_cols=None):
    """Return a walled room; columns >= `known_cols` unknown (None = all known)."""
    d = np.full((h, w), F, dtype=np.int16)
    d[0, :] = d[-1, :] = d[:, 0] = d[:, -1] = OCC
    if known_cols is not None:
        d[:, known_cols:] = U
    return d


def test_geometry_round_trip():
    gr = g.Grid(room(), RES, -3.0, -2.0)
    assert gr.to_cell(-3.0, -2.0) == (0, 0)
    assert gr.to_cell(-2.95, -1.75) == (2, 0)
    x, y = gr.to_world(2, 5)
    assert (round(x, 3), round(y, 3)) == (-2.45, -1.75)
    assert gr.to_cell(x, y) == (2, 5)
    assert gr.value_at(-3.0, -2.0) == OCC and gr.value_at(-10, -10) == U
    assert gr.known_area() == pytest.approx(40 * 60 * RES * RES)


def test_from_msg_shape():
    class Info:
        width, height, resolution = 3, 2, 0.5

        class origin:
            class position:
                x, y = 1.0, 2.0

    class Msg:
        info = Info
        data = [0, 0, -1, 100, 0, 0]

    gr = g.Grid.from_msg(Msg)
    assert gr.shape == (2, 3) and gr.data[0, 2] == U and gr.data[1, 0] == OCC
    assert gr.origin == (1.0, 2.0)


def test_closed_known_room_has_no_frontier():
    gr = g.Grid(room(), RES)
    assert not g.frontier_mask(gr.data).any()
    assert g.frontiers(gr) == []


def test_frontier_is_the_free_edge_next_to_unknown():
    d = room(known_cols=30)
    m = g.frontier_mask(d)
    # free cells in column 29 (rows 1..38) touch unknown column 30
    assert m[:, 29].sum() == 38 and not m[:, :29].any() and not m[:, 30:].any()
    # walls next to unknown are not frontiers
    assert not m[0, 29] and not m[39, 29]


def test_diagonal_only_unknown_is_not_a_frontier():
    d = np.zeros((5, 5), dtype=np.int16)
    d[0, 0] = U
    m = g.frontier_mask(d)
    assert not m[1, 1] and m[0, 1] and m[1, 0]


def test_clusters_are_8_connected_and_separate():
    m = np.zeros((10, 10), dtype=bool)
    m[1, 1] = m[2, 2] = m[3, 3] = True          # diagonal chain: one cluster
    m[7, 7] = m[7, 8] = True                    # another
    cs = g.clusters(m)
    assert sorted(len(c) for c in cs) == [2, 3]
    assert [tuple(c[0]) for c in cs] == [(1, 1), (7, 7)]


def test_two_openings_two_frontiers_min_size_filters():
    d = room()
    d[5:15, -1] = U          # a 10-cell doorway on the right wall, unknown beyond
    d[-1, 10:13] = U         # a 3-cell gap in the bottom wall
    gr = g.Grid(d, RES)
    found = g.frontiers(gr, min_size=1)
    assert [f.size for f in found] == [10, 3]           # largest first
    assert [f.size for f in g.frontiers(gr, min_size=5)] == [10]
    big = found[0]
    assert set(map(tuple, big.cells)) == {(r, 58) for r in range(5, 15)}
    gx, gy = big.goal
    assert gr.to_cell(gx, gy)[1] == 58 and 5 <= gr.to_cell(gx, gy)[0] < 15
    assert big.centroid == pytest.approx(((58 + 0.5) * RES, (9.5 + 0.5) * RES))


def test_goal_prefers_clearance():
    d = room(known_cols=30)
    gr = g.Grid(d, RES)
    (f,) = g.frontiers(gr, min_size=5, clearance=0.5)
    r, c = gr.to_cell(*f.goal)
    assert 5 <= r <= 34          # not hugging the top or bottom wall


def test_choose_nearest_large_skips_blacklist_and_blocked():
    d = room()
    d[5:15, -1] = U                     # right, 10 cells
    d[20:28, 0] = U                     # left, 8 cells
    gr = g.Grid(d, RES)
    found = g.frontiers(gr, min_size=5)
    left = next(f for f in found if f.size == 8)
    right = next(f for f in found if f.size == 10)
    near_left = (1.0, 2.4)
    assert g.choose(found, near_left, RES) is left
    assert g.choose(found, near_left, RES, blacklist=[left.goal]) is right
    assert g.choose(found, near_left, RES, blocked=lambda x, y: x < 1.0) is right
    assert g.choose(found, near_left, RES, blacklist=[left.goal, right.goal]) is None
    # min_distance skips a frontier the robot is standing on
    assert g.choose(found, left.goal, RES, min_distance=0.3) is right


def test_frontier_near():
    gr = g.Grid(room(known_cols=30), RES)
    assert g.frontier_near(gr, gr.to_world(20, 29), 0.1)
    assert g.frontier_near(gr, gr.to_world(20, 26), 0.35)
    assert not g.frontier_near(gr, gr.to_world(20, 20), 0.35)
    assert not g.frontier_near(gr, (-50.0, -50.0), 1.0)


def test_dilate_disc():
    m = np.zeros((9, 9), dtype=bool)
    m[4, 4] = True
    d = g.dilate(m, 2)
    assert d[4, 2] and d[2, 4] and d[3, 3] and not d[2, 2] and d.sum() == 13
    assert (g.dilate(m, 0) == m).all()


def test_blocked_by_mask():
    mask = np.zeros((20, 20), dtype=np.int16)
    mask[5:10, 5:10] = 100
    mg = g.Grid(mask, RES, -1.0, -1.0)
    assert g.blocked_by_mask(mg, -1.0 + 0.75, -1.0 + 0.75)          # cell (7, 7)
    assert not g.blocked_by_mask(mg, -1.0 + 0.15, -1.0 + 0.15)
    assert g.blocked_by_mask(mg, -1.0 + 0.45, -1.0 + 0.75, margin=0.1)
    assert not g.blocked_by_mask(None, 0.0, 0.0)
    assert not g.blocked_by_mask(mg, 50.0, 50.0, margin=1.0)


def test_free_goal_mask_keeps_clearance_from_walls_and_unknown():
    gr = g.Grid(room(known_cols=30), RES)
    m = g.free_goal_mask(gr, clearance=0.3)
    rows, cols = np.nonzero(m)
    assert rows.min() >= 4 and rows.max() <= 35       # 3 cells off the walls
    assert cols.min() >= 4 and cols.max() <= 26       # and off the unknown edge at 30


def test_sample_goals_distance_blocked_deterministic():
    gr = g.Grid(room(), RES)
    robot = (1.0, 1.0)

    def blocked(x, y):
        return x > 4.0
    a = g.sample_goals(gr, robot, np.random.default_rng(7), count=20, min_distance=1.0,
                       clearance=0.3, blocked=blocked)
    b = g.sample_goals(gr, robot, np.random.default_rng(7), count=20, min_distance=1.0,
                       clearance=0.3, blocked=blocked)
    assert a == b and len(a) == 20 and len(set(a)) == 20
    for x, y in a:
        assert math.hypot(x - robot[0], y - robot[1]) >= 1.0
        assert x <= 4.0
        assert gr.value_at(x, y) == F
    near = g.sample_goals(gr, robot, np.random.default_rng(1), count=50, min_distance=0.5,
                          max_distance=1.0)
    assert near and all(0.5 <= math.hypot(x - 1, y - 1) <= 1.0 for x, y in near)


def test_sample_goals_none_possible():
    gr = g.Grid(room(10, 10), RES)          # 1 m room: nothing 1 m away with clearance
    assert g.sample_goals(gr, (0.5, 0.5), np.random.default_rng(0), min_distance=1.0) == []
    gr = g.Grid(np.full((10, 10), U, dtype=np.int16), RES)
    assert g.sample_goals(gr, (0.5, 0.5), np.random.default_rng(0)) == []
