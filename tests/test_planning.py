"""A* invariants: heuristic admissibility, no corner cutting, determinism.

These are the properties the planner's correctness argument rests on. A
regression in any of them still produces a plausible-looking path, so each is
pinned explicitly rather than inferred from an end-to-end result.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from uav_autonomy.grid import FREE, OCCUPIED, inflated_clear_grid
from uav_autonomy.planning import (
    NEIGHBORS_8, OCTILE_DIAG, SQRT2,
    a_star, a_star_clearance, a_star_comfort, cell_penalty, comfort_penalty,
    octile_heuristic_cells, path_length_metres,
)


def open_grid(n=40):
    return np.ones((n, n), dtype=bool)


def plan(clear, start, goal, goal_radius_cells=0):
    return a_star(clear, start, frozenset([goal]), goal, goal_radius_cells)


# --------------------------------------------------------------- heuristic

def test_neighbour_costs_are_geometric():
    for dr, dc, cost in NEIGHBORS_8:
        assert cost == pytest.approx(math.hypot(dr, dc))
    assert len({(dr, dc) for dr, dc, _ in NEIGHBORS_8}) == 8


def test_octile_heuristic_is_zero_at_the_goal():
    assert octile_heuristic_cells((7, 7), (7, 7), 0) == 0.0


def test_octile_heuristic_matches_the_closed_form():
    h = octile_heuristic_cells((0, 0), (3, 7), 0)
    assert h == pytest.approx(OCTILE_DIAG * 3 + 7)


def test_octile_heuristic_never_exceeds_true_cost():
    """Admissibility on an obstacle-free grid, where true cost is exactly
    the octile distance. h > true cost anywhere would break optimality."""
    clear = open_grid(30)
    goal = (25, 20)
    rng = np.random.default_rng(0)
    for _ in range(60):
        start = (int(rng.integers(0, 30)), int(rng.integers(0, 30)))
        res = plan(clear, start, goal)
        assert res.success
        true_cost = sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(res.path_cells[:-1], res.path_cells[1:])
        )
        h = octile_heuristic_cells(start, goal, 0)
        assert h <= true_cost + 1e-9, f"inadmissible at {start}: h={h} > {true_cost}"


def test_octile_heuristic_subtracts_the_goal_radius():
    """The goal is a disc, so the heuristic must aim at its boundary."""
    assert octile_heuristic_cells((0, 0), (0, 10), 4) == pytest.approx(6.0)
    # Never negative inside the disc.
    assert octile_heuristic_cells((0, 8), (0, 10), 4) == 0.0


def _disc_cells(radius):
    return [(dr, dc) for dr in range(-radius, radius + 1)
            for dc in range(-radius, radius + 1) if dr * dr + dc * dc <= radius * radius]


@pytest.mark.parametrize("radius", [6, 13])
def test_goal_region_heuristic_overestimate_is_bounded(radius):
    """With a goal disc the heuristic is octile-to-centre minus the radius.
    That is *not* exactly admissible: a goal cell inside the Euclidean disc can
    sit up to sqrt(1 + OCTILE_DIAG**2) * radius from the centre in octile
    distance. This pins the documented bound rather than the stronger claim.
    6 cells is the goal radius used throughout (0.30 m at 0.05 m/cell)."""
    bound = (math.sqrt(1.0 + OCTILE_DIAG ** 2) - 1.0) * radius
    goal = _disc_cells(radius)
    worst = 0.0
    for r in range(-30, 31):
        for c in range(-30, 31):
            true_cost = min(OCTILE_DIAG * min(abs(r - g[0]), abs(c - g[1]))
                            + max(abs(r - g[0]), abs(c - g[1])) for g in goal)
            worst = max(worst, octile_heuristic_cells((r, c), (0, 0), radius) - true_cost)
    assert worst <= bound + 1e-9
    assert worst > 0.0, "if this ever fails the heuristic became exactly admissible"


def test_goal_region_suboptimality_stays_within_the_bound():
    """Consequence of the bounded overestimate: on an open grid the returned
    path to a 6-cell goal disc is within the bound of the optimum. Uses the
    unpenalised a_star, where planning cost equals geometric octile length."""
    radius = 6
    bound = (math.sqrt(1.0 + OCTILE_DIAG ** 2) - 1.0) * radius
    clear = open_grid(60)
    centre = (30, 30)
    goal = frozenset((centre[0] + dr, centre[1] + dc) for dr, dc in _disc_cells(radius))
    rng = np.random.default_rng(1)
    for _ in range(40):
        start = (int(rng.integers(0, 60)), int(rng.integers(0, 60)))
        if start in goal:
            continue
        res = a_star(clear, start, goal, centre, radius)
        assert res.success and res.path_cells[-1] in goal
        optimum = min(OCTILE_DIAG * min(abs(start[0] - g[0]), abs(start[1] - g[1]))
                      + max(abs(start[0] - g[0]), abs(start[1] - g[1])) for g in goal)
        assert path_length_metres(res.path_cells, 1.0) <= optimum + bound + 1e-9


# ---------------------------------------------------------- corner cutting

def test_no_corner_cutting_through_a_diagonal_gap():
    """Two obstacles touching at a corner must not be passable diagonally.

    Layout (X = obstacle). Going (0,0) -> (2,2) must not use the (1,1) pinch.
        . X .
        X . .        <- the only diagonal route is blocked at both orthogonals
        . . .
    """
    grid = np.full((3, 3), FREE, dtype=np.uint8)
    grid[0, 1] = OCCUPIED
    grid[1, 0] = OCCUPIED
    clear = inflated_clear_grid(grid, 0, unknown_is_blocked=False)
    assert clear[0, 0] and clear[1, 1]

    res = plan(clear, (0, 0), (1, 1))
    # (0,0) -> (1,1) is a diagonal whose orthogonal neighbours (0,1) and (1,0)
    # are both blocked, so the step must be rejected and no path exists.
    assert not res.success


def test_diagonal_allowed_when_both_orthogonals_are_clear():
    clear = open_grid(3)
    res = plan(clear, (0, 0), (1, 1))
    assert res.success
    assert res.path_cells == [(0, 0), (1, 1)]


def test_single_blocked_orthogonal_still_blocks_the_diagonal():
    grid = np.full((3, 3), FREE, dtype=np.uint8)
    grid[0, 1] = OCCUPIED
    clear = inflated_clear_grid(grid, 0, unknown_is_blocked=False)
    res = plan(clear, (0, 0), (1, 1))
    assert res.success
    # Must detour through (1, 0), never step (0,0) -> (1,1) directly.
    assert res.path_cells == [(0, 0), (1, 0), (1, 1)]


def test_path_never_traverses_a_blocked_cell():
    rng = np.random.default_rng(7)
    grid = np.full((40, 40), FREE, dtype=np.uint8)
    grid[rng.random((40, 40)) < 0.18] = OCCUPIED
    grid[0, 0] = FREE
    grid[39, 39] = FREE
    clear = inflated_clear_grid(grid, 0, unknown_is_blocked=False)
    res = plan(clear, (0, 0), (39, 39))
    if res.success:
        assert all(clear[c] for c in res.path_cells)


# ------------------------------------------------------------ determinism

def test_astar_is_bit_identical_across_repeats():
    rng = np.random.default_rng(11)
    grid = np.full((60, 60), FREE, dtype=np.uint8)
    grid[rng.random((60, 60)) < 0.15] = OCCUPIED
    grid[0, 0] = FREE
    grid[59, 59] = FREE
    clear = inflated_clear_grid(grid, 1, unknown_is_blocked=False)

    first = plan(clear, (0, 0), (59, 59), 2)
    for _ in range(15):
        again = plan(clear, (0, 0), (59, 59), 2)
        assert again.path_cells == first.path_cells
        assert again.nodes_expanded == first.nodes_expanded


def test_all_three_planners_are_deterministic():
    rng = np.random.default_rng(3)
    grid = np.full((50, 50), FREE, dtype=np.uint8)
    grid[rng.random((50, 50)) < 0.12] = OCCUPIED
    grid[0, 0] = FREE
    grid[49, 49] = FREE
    clear = inflated_clear_grid(grid, 1, unknown_is_blocked=False)
    field = np.full(grid.shape, 5.0)

    kw = dict(start=(0, 0), goal_cells_set=frozenset([(49, 49)]),
              goal_centre=(49, 49), goal_radius_cells=1)
    runs = {
        "clearance": lambda: a_star_clearance(
            clear_grid=clear, clearance_m_field=field,
            r_eff=0.65, r_pref=0.90, w_clearance=5.0, **kw),
        "comfort": lambda: a_star_comfort(
            clear_grid=clear, clearance_m_field=field,
            r_plan_min=1.10, r_comfort=1.20, w_comfort=3.0, **kw),
    }
    for name, fn in runs.items():
        first = fn()
        for _ in range(5):
            assert fn().path_cells == first.path_cells, f"{name} not deterministic"


# ----------------------------------------------------------- soft penalties

def test_cell_penalty_bounds_and_monotonicity():
    assert cell_penalty(1.00, 0.65, 0.90) == 0.0
    assert cell_penalty(0.90, 0.65, 0.90) == 0.0
    assert cell_penalty(0.65, 0.65, 0.90) == 1.0
    assert cell_penalty(0.50, 0.65, 0.90) == 1.0
    mid = cell_penalty(0.775, 0.65, 0.90)
    assert 0.0 < mid < 1.0
    # Non-increasing in clearance: closer to an obstacle never costs less.
    xs = np.linspace(0.6, 1.0, 40)
    ps = [cell_penalty(x, 0.65, 0.90) for x in xs]
    assert all(a >= b - 1e-12 for a, b in zip(ps, ps[1:]))


def test_comfort_penalty_handles_a_zero_width_band():
    """A collapsed band must disable the cost, not divide by zero."""
    assert comfort_penalty(1.0, r_plan_min=1.2, r_comfort=1.2) == 0.0
    assert comfort_penalty(1.0, r_plan_min=1.3, r_comfort=1.2) == 0.0


def test_clearance_penalty_steers_away_from_a_tight_corridor():
    """With room available, the penalised planner should not hug the wall."""
    n = 41
    clear = np.ones((n, n), dtype=bool)
    field = np.full((n, n), 2.0)
    # A band of low clearance along row 20; rows 18-19 are roomy.
    field[20, :] = 0.66
    kw = dict(start=(20, 0), goal_cells_set=frozenset([(20, n - 1)]),
              goal_centre=(20, n - 1), goal_radius_cells=0)

    plain = a_star(clear, (20, 0), frozenset([(20, n - 1)]), (20, n - 1), 0)
    penalised = a_star_clearance(
        clear_grid=clear, clearance_m_field=field,
        r_eff=0.65, r_pref=0.90, w_clearance=5.0, **kw)

    assert plain.success and penalised.success
    rows_plain = {c[0] for c in plain.path_cells}
    rows_pen = {c[0] for c in penalised.path_cells}
    assert rows_plain == {20}, "unpenalised planner should go straight"
    assert rows_pen != {20}, "penalised planner should leave the low-clearance row"


# ---------------------------------------------------------------- metrics

def test_path_length_scales_with_resolution():
    path = [(0, 0), (0, 1), (1, 2)]
    assert path_length_metres(path, 1.0) == pytest.approx(1.0 + SQRT2)
    assert path_length_metres(path, 0.05) == pytest.approx((1.0 + SQRT2) * 0.05)


def test_straight_line_is_optimal_on_an_open_grid():
    clear = open_grid(20)
    res = plan(clear, (0, 0), (0, 10))
    assert res.success
    assert path_length_metres(res.path_cells, 1.0) == pytest.approx(10.0)
