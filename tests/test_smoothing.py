"""Line-of-sight reduction: the four hard invariants.

The reduction removes waypoints, so the risk is that it removes a waypoint
that was carrying the path around an obstacle. Each invariant is checked in
isolation, and the validator is also checked on a deliberately broken input to
confirm it actually detects a violation instead of always returning "OK".
"""

from __future__ import annotations

import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt

from uav_autonomy.grid import FREE, OCCUPIED, inflated_clear_grid
from uav_autonomy.planning import a_star
from uav_autonomy.smoothing import (
    bresenham_cells, bresenham_los_clear, polyline_clearance_along_cells,
    polyline_length_metres, string_pull, validate_invariants,
)


def _dist_field(grid, res=1.0):
    return distance_transform_edt(~(grid == OCCUPIED)) * res


# ------------------------------------------------------------- Bresenham

def test_bresenham_includes_both_endpoints():
    cells = list(bresenham_cells(0, 0, 3, 5))
    assert cells[0] == (0, 0)
    assert cells[-1] == (3, 5)


def test_bresenham_is_connected():
    """Consecutive cells never jump more than one step in either axis."""
    for a, b in [((0, 0), (7, 3)), ((5, 9), (0, 0)), ((2, 2), (2, 9))]:
        cells = list(bresenham_cells(*a, *b))
        for p, q in zip(cells, cells[1:]):
            assert abs(q[0] - p[0]) <= 1 and abs(q[1] - p[1]) <= 1


def test_bresenham_single_cell():
    assert list(bresenham_cells(4, 4, 4, 4)) == [(4, 4)]


def test_los_detects_a_blocking_cell():
    clear = np.ones((10, 10), dtype=bool)
    assert bresenham_los_clear((0, 0), (9, 9), clear)
    clear[5, 5] = False
    assert not bresenham_los_clear((0, 0), (9, 9), clear)


# ------------------------------------------------------------ string pull

def test_string_pull_collapses_a_straight_staircase():
    clear = np.ones((10, 10), dtype=bool)
    raw = [(0, i) for i in range(10)]
    assert string_pull(raw, clear) == [(0, 0), (0, 9)]


def test_string_pull_preserves_endpoints_and_shortens():
    grid = np.full((30, 30), FREE, dtype=np.uint8)
    grid[10:20, 15] = OCCUPIED
    clear = inflated_clear_grid(grid, 0, unknown_is_blocked=False)
    res = a_star(clear, (0, 0), frozenset([(29, 29)]), (29, 29), 0)
    assert res.success
    reduced = string_pull(res.path_cells, clear)

    assert reduced[0] == res.path_cells[0]
    assert reduced[-1] == res.path_cells[-1]
    assert len(reduced) <= len(res.path_cells)
    assert (polyline_length_metres(reduced, 1.0)
            <= polyline_length_metres(res.path_cells, 1.0) + 1e-9)


def test_string_pull_keeps_a_waypoint_that_is_load_bearing():
    """A wall with a single gap forces the reduction to keep a turn point."""
    grid = np.full((21, 21), FREE, dtype=np.uint8)
    grid[10, :] = OCCUPIED
    grid[10, 3] = FREE  # the only doorway
    clear = inflated_clear_grid(grid, 0, unknown_is_blocked=False)
    res = a_star(clear, (0, 15), frozenset([(20, 15)]), (20, 15), 0)
    assert res.success
    reduced = string_pull(res.path_cells, clear)
    assert len(reduced) > 2, "reduction must not shortcut through the wall"
    for a, b in zip(reduced, reduced[1:]):
        assert bresenham_los_clear(a, b, clear)


def test_string_pull_handles_degenerate_inputs():
    clear = np.ones((5, 5), dtype=bool)
    assert string_pull([], clear) == []
    assert string_pull([(1, 1)], clear) == [(1, 1)]
    assert string_pull([(1, 1), (2, 2)], clear) == [(1, 1), (2, 2)]


# -------------------------------------------------------------- clearance

def test_clearance_samples_the_segment_interior_not_only_waypoints():
    """The whole point of rasterised sampling: a mid-segment cell can be the
    closest approach to an obstacle."""
    grid = np.full((11, 11), FREE, dtype=np.uint8)
    grid[5, 5] = OCCUPIED
    dist = _dist_field(grid)
    waypoints = [(5, 0), (5, 10)]  # passes straight through the obstacle cell
    mn, _ = polyline_clearance_along_cells(waypoints, dist)
    endpoints_only = min(dist[w] for w in waypoints)
    assert mn == 0.0
    assert endpoints_only > mn, "endpoint-only sampling would have missed it"


def test_clearance_on_empty_and_single_waypoint():
    dist = np.ones((5, 5))
    assert polyline_clearance_along_cells([], dist) == (float("inf"), float("inf"))
    assert polyline_clearance_along_cells([(2, 2)], dist) == (1.0, 1.0)


# ------------------------------------------------------------- invariants

def test_validate_invariants_accepts_a_correct_reduction():
    grid = np.full((30, 30), FREE, dtype=np.uint8)
    grid[10:20, 15] = OCCUPIED
    clear = inflated_clear_grid(grid, 1, unknown_is_blocked=False)
    dist = _dist_field(grid)
    # Inflation radius 1 blocks the border ring, so start inside it.
    start, goal = (2, 2), (27, 27)
    assert clear[start] and clear[goal]
    res = a_star(clear, start, frozenset([goal]), goal, 0)
    assert res.success
    reduced = string_pull(res.path_cells, clear)
    assert validate_invariants(res.path_cells, reduced, clear, dist, 1.0, 1.0) == []


def test_validate_invariants_catches_a_moved_endpoint():
    clear = np.ones((10, 10), dtype=bool)
    dist = np.full((10, 10), 5.0)
    raw = [(0, 0), (0, 5), (0, 9)]
    broken = [(0, 1), (0, 9)]
    errors = validate_invariants(raw, broken, clear, dist, 1.0, 1.0)
    assert any("endpoint preservation" in e for e in errors)


def test_validate_invariants_catches_a_line_of_sight_violation():
    clear = np.ones((10, 10), dtype=bool)
    clear[0, 5] = False
    dist = np.full((10, 10), 5.0)
    raw = [(0, 0), (0, 9)]
    errors = validate_invariants(raw, [(0, 0), (0, 9)], clear, dist, 1.0, 1.0)
    assert any("line-of-sight" in e for e in errors)


def test_validate_invariants_catches_a_clearance_floor_violation():
    clear = np.ones((10, 10), dtype=bool)
    dist = np.full((10, 10), 0.2)
    raw = [(0, 0), (0, 9)]
    errors = validate_invariants(raw, [(0, 0), (0, 9)], clear, dist, 1.0,
                                 robot_radius_m=1.0)
    assert any("clearance floor" in e for e in errors)


def test_validate_invariants_catches_a_longer_output():
    clear = np.ones((20, 20), dtype=bool)
    dist = np.full((20, 20), 5.0)
    raw = [(0, 0), (0, 5)]
    longer = [(0, 0), (10, 10), (0, 5)]
    errors = validate_invariants(raw, longer, clear, dist, 1.0, 1.0)
    assert any("length monotonicity" in e for e in errors)


def test_validate_invariants_reports_empty_input():
    clear = np.ones((5, 5), dtype=bool)
    dist = np.ones((5, 5))
    assert validate_invariants([], [(0, 0)], clear, dist, 1.0, 1.0) == ["raw path is empty"]
    assert validate_invariants([(0, 0)], [], clear, dist, 1.0, 1.0) == ["smoothed path is empty"]
