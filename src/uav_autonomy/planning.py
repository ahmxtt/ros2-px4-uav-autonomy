"""Deterministic 2D grid A* — geometric, clearance-aware and comfort-aware.

Three planners share one search core and differ only in edge cost. All three
enforce safety *hard*, by searching on an inflated grid; the soft terms only
shape the preference among already-safe cells.

``a_star``
    Pure geometric cost. Step cost is 1 (orthogonal) or sqrt(2) (diagonal) in
    cell units.

``a_star_clearance``
    Adds a penalty on cells whose true clearance falls inside
    ``[r_eff, r_pref)``. This is the *boundary* candidate of the mission
    policy, and the planner used by the local ROI replanner where the detour
    is short and the corridor tight.

``a_star_comfort``
    Hard inflation at ``r_plan_min = r_pref + spline_shrinkage_budget``, plus a
    penalty inside ``[r_plan_min, r_comfort)``. This is the *robust* candidate.
    Planning above ``r_pref`` reserves the clearance that the downstream
    B-spline is allowed to consume, so the refined trajectory can still meet
    ``r_pref`` after the spline cuts corners.

The mission policy describes computing both the robust and the boundary
candidate and selecting between them; that wrapper is not part of this release
(see ``sim/config/mission_policy.yaml``). Each tool here runs one candidate.

Invariants shared by all three:

* **8-connected** neighbourhood, fixed iteration order (orthogonal first, then
  diagonals), so expansion is deterministic.
* **No corner cutting.** A diagonal step is rejected unless both adjacent
  orthogonal cells are clear.
* **Octile heuristic**, measured to the *boundary* of the goal disc.
* **Deterministic tie-break.** Heap entries are ``(f, g, row, col)``; the
  trailing ``(row, col)`` makes the pop order total, so repeated runs on
  identical input produce byte-identical paths.

Admissibility: the octile distance to a single cell never exceeds the true
remaining cost, and in both penalised variants the penalty is non-negative, so
every penalised edge costs at least its geometric length. The goal-*region*
heuristic (octile to the disc centre minus the disc radius, see
:func:`octile_heuristic_cells`) is consistent but only approximately
admissible: it can overestimate the cost to the nearest goal cell by at most
``(sqrt(1 + OCTILE_DIAG**2) - 1) * radius`` cells, about 0.082 x radius. For
the 6-cell goal discs used here that bound is 0.49 cells and the largest
observed overestimate is 0.24 cells, so the returned path can be longer than
the optimum by at most that amount (under 0.025 m at 0.05 m/cell).
``tests/test_planning.py`` pins the bound. The heuristic is left as is because
every published node count depends on it.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from typing import FrozenSet, List, Optional, Tuple

import numpy as np

from .grid import OCCUPIED, Cell

__all__ = [
    "SQRT2", "OCTILE_DIAG", "NEIGHBORS_8",
    "AStarResult", "AStarClearanceResult", "AStarComfortResult", "CostBreakdown",
    "octile_heuristic_cells", "a_star", "a_star_clearance", "a_star_comfort",
    "cell_penalty", "comfort_penalty",
    "path_length_metres", "path_clearances", "path_cells_to_world",
]

SQRT2 = math.sqrt(2.0)
OCTILE_DIAG = SQRT2 - 1.0

# Fixed order: orthogonal first, then diagonals. Changing this order changes
# which of several equal-cost paths is returned.
NEIGHBORS_8: Tuple[Tuple[int, int, float], ...] = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, SQRT2),
    (-1, 1, SQRT2),
    (1, -1, SQRT2),
    (1, 1, SQRT2),
)


@dataclass
class AStarResult:
    success: bool
    path_cells: List[Cell]
    nodes_expanded: int
    planning_time_ms: float
    goal_cell_reached: Optional[Cell]


@dataclass
class AStarClearanceResult:
    success: bool
    path_cells: List[Cell]
    nodes_expanded: int
    planning_time_ms: float
    goal_cell_reached: Optional[Cell]
    g_at_goal: float  # cell-unit cost with the clearance penalty included


@dataclass
class CostBreakdown:
    """Decomposition of the comfort planner's path cost, in cell units."""

    length_cost: float
    comfort_penalty_sum: float
    clearance_penalty_sum: float
    total_cost: float


@dataclass
class AStarComfortResult:
    success: bool
    path_cells: List[Cell]
    nodes_expanded: int
    planning_time_ms: float
    goal_cell_reached: Optional[Cell]
    g_at_goal: float
    cost: CostBreakdown


# ---------------------------------------------------------------------------
# Heuristic
# ---------------------------------------------------------------------------

def octile_heuristic_cells(
    cell: Cell,
    goal_centre: Cell,
    goal_radius_cells: int,
) -> float:
    """Octile distance from ``cell`` to the *boundary* of the goal disc.

    Subtracting the goal radius approximates an admissible bound for the
    multi-target goal-region search. It is not exact: a goal cell at Euclidean
    distance ``<= radius`` from the centre can lie at octile distance up to
    ``sqrt(1 + OCTILE_DIAG**2) * radius`` from it, so the heuristic can
    overestimate by up to about ``0.082 * radius`` cells. See the module
    docstring for the consequence, which is bounded and small.
    """
    dr = abs(cell[0] - goal_centre[0])
    dc = abs(cell[1] - goal_centre[1])
    octile_to_centre = OCTILE_DIAG * min(dr, dc) + max(dr, dc)
    return max(0.0, octile_to_centre - goal_radius_cells)


def cell_penalty(clearance_m: float, r_eff: float, r_pref: float) -> float:
    """Soft clearance penalty in [0, 1] over the band ``[r_eff, r_pref)``.

    0 at or above ``r_pref``, 1 at or below ``r_eff``, quadratic in between.
    Non-negative by construction, which preserves heuristic admissibility.
    """
    if clearance_m >= r_pref:
        return 0.0
    if clearance_m <= r_eff:
        return 1.0
    t = (r_pref - clearance_m) / (r_pref - r_eff)
    return t * t


def comfort_penalty(clearance_m: float, r_plan_min: float, r_comfort: float) -> float:
    """Soft comfort penalty in [0, 1] over the band ``[r_plan_min, r_comfort)``.

    A zero-width or inverted band disables the penalty entirely rather than
    dividing by zero.
    """
    if r_comfort <= r_plan_min:
        return 0.0
    if clearance_m >= r_comfort:
        return 0.0
    if clearance_m <= r_plan_min:
        return 1.0
    t = (r_comfort - clearance_m) / (r_comfort - r_plan_min)
    return t * t


# ---------------------------------------------------------------------------
# Search core
# ---------------------------------------------------------------------------

def _reconstruct(parent_r, parent_c, goal_reached: Cell) -> List[Cell]:
    path: List[Cell] = []
    r, c = goal_reached
    while True:
        path.append((r, c))
        pr = int(parent_r[r, c])
        pc = int(parent_c[r, c])
        if pr == -1 and pc == -1:
            break
        r, c = pr, pc
    path.reverse()
    return path


def a_star(
    clear_grid: np.ndarray,
    start: Cell,
    goal_cells_set: FrozenSet[Cell],
    goal_centre: Cell,
    goal_radius_cells: int,
) -> AStarResult:
    """Deterministic 8-connected A* on an inflated boolean grid."""
    t0 = time.perf_counter()
    height, width = clear_grid.shape

    g = np.full(clear_grid.shape, np.inf, dtype=np.float64)
    g[start] = 0.0
    parent_r = np.full(clear_grid.shape, -1, dtype=np.int32)
    parent_c = np.full(clear_grid.shape, -1, dtype=np.int32)
    closed = np.zeros(clear_grid.shape, dtype=bool)

    h0 = octile_heuristic_cells(start, goal_centre, goal_radius_cells)
    open_heap: List[Tuple[float, float, int, int]] = [(h0, 0.0, start[0], start[1])]
    nodes_expanded = 0
    goal_reached: Optional[Cell] = None

    while open_heap:
        _, gn, r, c = heapq.heappop(open_heap)
        if closed[r, c]:
            continue
        if gn > g[r, c]:
            continue
        closed[r, c] = True
        nodes_expanded += 1

        if (r, c) in goal_cells_set:
            goal_reached = (r, c)
            break

        for dr, dc, step_cost in NEIGHBORS_8:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= height or nc < 0 or nc >= width:
                continue
            if not clear_grid[nr, nc]:
                continue
            if closed[nr, nc]:
                continue
            if dr != 0 and dc != 0:
                # No corner cutting: both orthogonal neighbours must be clear.
                if not clear_grid[r + dr, c] or not clear_grid[r, c + dc]:
                    continue
            new_g = gn + step_cost
            if new_g < g[nr, nc]:
                g[nr, nc] = new_g
                parent_r[nr, nc] = r
                parent_c[nr, nc] = c
                new_h = octile_heuristic_cells((nr, nc), goal_centre, goal_radius_cells)
                heapq.heappush(open_heap, (new_g + new_h, new_g, nr, nc))

    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    if goal_reached is None:
        return AStarResult(False, [], nodes_expanded, elapsed_ms, None)
    return AStarResult(
        True, _reconstruct(parent_r, parent_c, goal_reached),
        nodes_expanded, elapsed_ms, goal_reached,
    )


def a_star_clearance(
    *,
    clear_grid: np.ndarray,
    clearance_m_field: np.ndarray,
    start: Cell,
    goal_cells_set: FrozenSet[Cell],
    goal_centre: Cell,
    goal_radius_cells: int,
    r_eff: float,
    r_pref: float,
    w_clearance: float,
) -> AStarClearanceResult:
    """A* with a soft clearance penalty inside the comfort band.

    ``edge_cost = step_cost * (1 + w_clearance * penalty(clearance))``

    Hard safety is still the inflated ``clear_grid``; the penalty only biases
    the path away from the inflation boundary where room allows.
    """
    t0 = time.perf_counter()
    height, width = clear_grid.shape

    g = np.full(clear_grid.shape, np.inf, dtype=np.float64)
    g[start] = 0.0
    parent_r = np.full(clear_grid.shape, -1, dtype=np.int32)
    parent_c = np.full(clear_grid.shape, -1, dtype=np.int32)
    closed = np.zeros(clear_grid.shape, dtype=bool)

    h0 = octile_heuristic_cells(start, goal_centre, goal_radius_cells)
    open_heap: List[Tuple[float, float, int, int]] = [(h0, 0.0, start[0], start[1])]
    nodes_expanded = 0
    goal_reached: Optional[Cell] = None
    g_at_goal = float("inf")

    while open_heap:
        _, gn, r, c = heapq.heappop(open_heap)
        if closed[r, c]:
            continue
        if gn > g[r, c]:
            continue
        closed[r, c] = True
        nodes_expanded += 1

        if (r, c) in goal_cells_set:
            goal_reached = (r, c)
            g_at_goal = float(gn)
            break

        for dr, dc, step_cost in NEIGHBORS_8:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= height or nc < 0 or nc >= width:
                continue
            if not clear_grid[nr, nc]:
                continue
            if closed[nr, nc]:
                continue
            if dr != 0 and dc != 0:
                if not clear_grid[r + dr, c] or not clear_grid[r, c + dc]:
                    continue
            penalty = cell_penalty(
                float(clearance_m_field[nr, nc]), r_eff=r_eff, r_pref=r_pref,
            )
            edge_cost = step_cost * (1.0 + w_clearance * penalty)
            new_g = gn + edge_cost
            if new_g < g[nr, nc]:
                g[nr, nc] = new_g
                parent_r[nr, nc] = r
                parent_c[nr, nc] = c
                new_h = octile_heuristic_cells((nr, nc), goal_centre, goal_radius_cells)
                heapq.heappush(open_heap, (new_g + new_h, new_g, nr, nc))

    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    if goal_reached is None:
        return AStarClearanceResult(
            False, [], nodes_expanded, elapsed_ms, None, float("inf"),
        )
    return AStarClearanceResult(
        True, _reconstruct(parent_r, parent_c, goal_reached),
        nodes_expanded, elapsed_ms, goal_reached, g_at_goal,
    )


def a_star_comfort(
    *,
    clear_grid: np.ndarray,
    clearance_m_field: np.ndarray,
    start: Cell,
    goal_cells_set: FrozenSet[Cell],
    goal_centre: Cell,
    goal_radius_cells: int,
    r_plan_min: float,
    r_comfort: float,
    w_comfort: float,
) -> AStarComfortResult:
    """A* with a soft comfort penalty above the planning minimum.

    ``edge_cost = step_cost * (1 + w_comfort * comfort_penalty(clearance))``

    ``clear_grid`` is expected to be inflated at ``r_plan_min``, which is
    already above ``r_pref``. The penalty then biases the path toward roomier
    corridors *inside* the planning-safe region, reserving clearance for the
    downstream spline.
    """
    t0 = time.perf_counter()
    height, width = clear_grid.shape

    g = np.full(clear_grid.shape, np.inf, dtype=np.float64)
    g[start] = 0.0
    parent_r = np.full(clear_grid.shape, -1, dtype=np.int32)
    parent_c = np.full(clear_grid.shape, -1, dtype=np.int32)
    closed = np.zeros(clear_grid.shape, dtype=bool)

    h0 = octile_heuristic_cells(start, goal_centre, goal_radius_cells)
    open_heap: List[Tuple[float, float, int, int]] = [(h0, 0.0, start[0], start[1])]
    nodes_expanded = 0
    goal_reached: Optional[Cell] = None
    g_at_goal = float("inf")

    while open_heap:
        _, gn, r, c = heapq.heappop(open_heap)
        if closed[r, c]:
            continue
        if gn > g[r, c]:
            continue
        closed[r, c] = True
        nodes_expanded += 1

        if (r, c) in goal_cells_set:
            goal_reached = (r, c)
            g_at_goal = float(gn)
            break

        for dr, dc, step_cost in NEIGHBORS_8:
            nr, nc = r + dr, c + dc
            if nr < 0 or nr >= height or nc < 0 or nc >= width:
                continue
            if not clear_grid[nr, nc]:
                continue
            if closed[nr, nc]:
                continue
            if dr != 0 and dc != 0:
                if not clear_grid[r + dr, c] or not clear_grid[r, c + dc]:
                    continue
            pen = comfort_penalty(
                float(clearance_m_field[nr, nc]),
                r_plan_min=r_plan_min, r_comfort=r_comfort,
            )
            edge = step_cost * (1.0 + w_comfort * pen)
            new_g = gn + edge
            if new_g < g[nr, nc]:
                g[nr, nc] = new_g
                parent_r[nr, nc] = r
                parent_c[nr, nc] = c
                new_h = octile_heuristic_cells((nr, nc), goal_centre, goal_radius_cells)
                heapq.heappush(open_heap, (new_g + new_h, new_g, nr, nc))

    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    if goal_reached is None:
        return AStarComfortResult(
            False, [], nodes_expanded, elapsed_ms, None, float("inf"),
            CostBreakdown(0.0, 0.0, 0.0, 0.0),
        )

    path = _reconstruct(parent_r, parent_c, goal_reached)

    # Re-walk the path to decompose the cost into length and comfort terms.
    length_cost = 0.0
    comfort_sum = 0.0
    for (r1, c1), (r2, c2) in zip(path[:-1], path[1:]):
        dr = r2 - r1
        dc = c2 - c1
        step_cells = math.sqrt(dr * dr + dc * dc)
        length_cost += step_cells
        pen = comfort_penalty(
            float(clearance_m_field[r2, c2]),
            r_plan_min=r_plan_min, r_comfort=r_comfort,
        )
        comfort_sum += step_cells * w_comfort * pen

    return AStarComfortResult(
        True, path, nodes_expanded, elapsed_ms, goal_reached, g_at_goal,
        CostBreakdown(
            length_cost=length_cost,
            comfort_penalty_sum=comfort_sum,
            # Kept at 0 for schema symmetry with the boundary candidate, which
            # reports a clearance-band penalty instead.
            clearance_penalty_sum=0.0,
            total_cost=length_cost + comfort_sum,
        ),
    )


# ---------------------------------------------------------------------------
# Path metrics
# ---------------------------------------------------------------------------

def path_length_metres(path: List[Cell], resolution_m: float) -> float:
    total = 0.0
    for (r1, c1), (r2, c2) in zip(path[:-1], path[1:]):
        dr, dc = r2 - r1, c2 - c1
        total += math.sqrt(dr * dr + dc * dc)
    return total * resolution_m


def path_clearances(
    path: List[Cell],
    grid: np.ndarray,
    resolution_m: float,
) -> Tuple[float, float]:
    """``(min, mean)`` clearance in metres from each path cell to the nearest
    **raw** occupied cell.

    Inflation is deliberately excluded: this reports the real safety margin
    against actual obstacles, not the inflated proxy the search ran on.
    """
    from scipy.ndimage import distance_transform_edt

    raw_occupied = grid == OCCUPIED
    if not raw_occupied.any():
        return float("inf"), float("inf")
    distance_to_obstacle_cells = distance_transform_edt(~raw_occupied)
    rows = np.array([c[0] for c in path], dtype=np.int32)
    cols = np.array([c[1] for c in path], dtype=np.int32)
    distances_m = distance_to_obstacle_cells[rows, cols] * resolution_m
    return float(distances_m.min()), float(distances_m.mean())


def path_cells_to_world(
    path: List[Cell],
    origin: List[float],
    resolution: float,
    height: int,
) -> np.ndarray:
    """Convert a cell path to an ``(N, 2)`` array of world-frame centres."""
    from .grid import cell_to_world

    return np.array(
        [cell_to_world(c, origin, resolution, height) for c in path],
        dtype=float,
    )
