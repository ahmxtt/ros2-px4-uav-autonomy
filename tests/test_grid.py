"""World/cell mapping and inflation.

Off-by-one errors in the grid transform are silent: the planner still returns
a path, but every clearance number computed from it is wrong. These tests pin
the transform at the places where such an error would first appear -- the
corners, the origin, and the round trip.
"""

from __future__ import annotations

import numpy as np
import pytest

from uav_autonomy.grid import (
    FREE, OCCUPIED, UNKNOWN,
    cell_to_world, classify_map, collect_goal_cells, in_bounds,
    inflated_clear_grid, world_to_cell,
)

# Geometry of the published execution prior map.
RES = 0.05
ORIGIN = [-80.30, -100.25, 0.0]
HEIGHT = 3911
WIDTH = 2913


def test_cell_to_world_returns_cell_centre():
    """Bottom-left cell centre sits half a cell inside the origin corner."""
    x, y = cell_to_world((HEIGHT - 1, 0), ORIGIN, RES, HEIGHT)
    assert x == pytest.approx(ORIGIN[0] + 0.5 * RES)
    assert y == pytest.approx(ORIGIN[1] + 0.5 * RES)


def test_row_zero_is_top_of_map():
    """Row 0 is the highest y. Getting this inverted flips the whole map."""
    _, y_top = cell_to_world((0, 0), ORIGIN, RES, HEIGHT)
    _, y_bottom = cell_to_world((HEIGHT - 1, 0), ORIGIN, RES, HEIGHT)
    assert y_top > y_bottom
    assert y_top == pytest.approx(ORIGIN[1] + (HEIGHT - 0.5) * RES)


def test_world_to_cell_roundtrip_over_the_whole_grid():
    """cell -> world -> cell is the identity for every sampled cell."""
    rows = np.linspace(0, HEIGHT - 1, 97, dtype=int)
    cols = np.linspace(0, WIDTH - 1, 89, dtype=int)
    for r in rows:
        for c in cols:
            xy = cell_to_world((int(r), int(c)), ORIGIN, RES, HEIGHT)
            back = world_to_cell(xy, ORIGIN, RES, HEIGHT)
            assert back == (int(r), int(c)), f"roundtrip failed at {(r, c)} via {xy}"


def test_world_to_cell_at_exact_corners():
    assert world_to_cell((ORIGIN[0], ORIGIN[1]), ORIGIN, RES, HEIGHT) == (HEIGHT - 1, 0)
    x_max = ORIGIN[0] + WIDTH * RES
    y_max = ORIGIN[1] + HEIGHT * RES
    # A point just inside the far corner maps to the last cell.
    assert world_to_cell((x_max - 1e-9, y_max - 1e-9), ORIGIN, RES, HEIGHT) == (0, WIDTH - 1)


def test_in_bounds():
    shape = (HEIGHT, WIDTH)
    assert in_bounds((0, 0), shape)
    assert in_bounds((HEIGHT - 1, WIDTH - 1), shape)
    assert not in_bounds((-1, 0), shape)
    assert not in_bounds((HEIGHT, 0), shape)
    assert not in_bounds((0, WIDTH), shape)


def test_classify_map_thresholds():
    """ROS occupancy convention: dark pixels are occupied when negate == 0."""
    img = np.array([[0, 128, 254, 255]], dtype=np.uint8)
    grid = classify_map(img, negate=0, occupied_thresh=0.65, free_thresh=0.196)
    assert grid[0, 0] == OCCUPIED   # occupancy 1.00
    assert grid[0, 1] == UNKNOWN    # occupancy 0.50
    assert grid[0, 2] == FREE       # occupancy 0.004
    assert grid[0, 3] == FREE


def test_classify_map_negate_inverts():
    img = np.array([[0, 255]], dtype=np.uint8)
    grid = classify_map(img, negate=1, occupied_thresh=0.65, free_thresh=0.196)
    assert grid[0, 0] == FREE
    assert grid[0, 1] == OCCUPIED


def test_inflation_blocks_a_disk_of_the_requested_radius():
    grid = np.full((41, 41), FREE, dtype=np.uint8)
    grid[20, 20] = OCCUPIED
    clear = inflated_clear_grid(grid, radius_cells=5, unknown_is_blocked=False)
    assert not clear[20, 20]
    assert not clear[20, 25]  # exactly on the radius
    assert clear[20, 27]      # outside
    # Diagonal at r=5 is outside a Euclidean disk of radius 5 (5*sqrt2 > 5).
    assert clear[25, 25]


def test_inflation_treats_the_border_as_blocked():
    """Padding with 'blocked' stops a path from leaving the grid."""
    grid = np.full((21, 21), FREE, dtype=np.uint8)
    clear = inflated_clear_grid(grid, radius_cells=3, unknown_is_blocked=False)
    assert not clear[0, 10]
    assert not clear[10, 0]
    assert clear[10, 10]


def test_unknown_policy_switches_blocking():
    grid = np.full((11, 11), UNKNOWN, dtype=np.uint8)
    relaxed = inflated_clear_grid(grid, 0, unknown_is_blocked=False)
    strict = inflated_clear_grid(grid, 0, unknown_is_blocked=True)
    assert relaxed.all()
    assert not strict.any()


def test_collect_goal_cells_only_returns_inflation_safe_cells():
    grid = np.full((101, 101), FREE, dtype=np.uint8)
    grid[50, 55] = OCCUPIED
    clear = inflated_clear_grid(grid, radius_cells=4, unknown_is_blocked=False)
    origin = [0.0, 0.0, 0.0]
    centre_xy = cell_to_world((50, 50), origin, 0.1, 101)
    cells, centre_cell, radius_cells = collect_goal_cells(
        centre_xy, 1.0, origin, 0.1, grid.shape, clear,
    )
    assert centre_cell == (50, 50)
    assert radius_cells == 10
    assert cells, "goal region should contain free cells"
    assert all(clear[c] for c in cells)
    assert (50, 55) not in cells


def test_collect_goal_cells_returns_empty_when_fully_blocked():
    grid = np.full((51, 51), OCCUPIED, dtype=np.uint8)
    clear = inflated_clear_grid(grid, 0, unknown_is_blocked=False)
    cells, _, _ = collect_goal_cells(
        (2.5, 2.5), 1.0, [0.0, 0.0, 0.0], 0.1, grid.shape, clear,
    )
    assert cells == []
