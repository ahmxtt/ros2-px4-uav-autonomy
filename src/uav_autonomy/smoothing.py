"""Line-of-sight path reduction (string pulling) on the inflated grid.

This is *geometric path simplification*, not trajectory smoothing: the output
is still a piecewise-linear polyline of grid cells. Curvature-continuous
refinement lives in :mod:`uav_autonomy.trajectory`.

The reduction reuses the exact inflated grid the planner searched on, so any
waypoint it accepts is by construction acceptable to the planner.

Four hard invariants are checked by :func:`validate_invariants`:

1. **Endpoint preservation** — first and last cell are unchanged.
2. **Line of sight** — every reduced segment is fully inflation-safe under a
   Bresenham rasterisation.
3. **Length monotonicity** — the reduced path is never longer than the input
   (a chord cannot exceed the polyline it replaces).
4. **Clearance floor** — minimum clearance along the reduced path stays at or
   above the robot radius.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Tuple

import numpy as np

from .grid import Cell

__all__ = [
    "bresenham_cells", "bresenham_los_clear", "string_pull",
    "polyline_length_metres", "polyline_clearance_along_cells",
    "validate_invariants",
]


def bresenham_cells(r1: int, c1: int, r2: int, c2: int) -> Iterable[Cell]:
    """Yield ``(row, col)`` cells along the integer line, endpoints included.

    Order is start-to-end and deterministic.
    """
    dr = abs(r2 - r1)
    dc = abs(c2 - c1)
    sr = 1 if r1 < r2 else -1
    sc = 1 if c1 < c2 else -1
    err = dc - dr
    r, c = r1, c1
    while True:
        yield r, c
        if r == r2 and c == c2:
            break
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr


def bresenham_los_clear(start: Cell, end: Cell, clear_grid: np.ndarray) -> bool:
    """True if every rasterised cell between ``start`` and ``end`` is clear."""
    for cell in bresenham_cells(start[0], start[1], end[0], end[1]):
        if not clear_grid[cell]:
            return False
    return True


def string_pull(path: List[Cell], clear_grid: np.ndarray) -> List[Cell]:
    """Greedy look-ahead string pull.

    From index ``i``, find the furthest ``j`` whose direct Bresenham line from
    ``path[i]`` is fully clear, accept it as the next waypoint, and repeat.
    Endpoints are preserved by construction: ``path[-1]`` is always a candidate
    for the final ``j``, and ``path[i + 1]`` is always reachable because it is
    grid-adjacent.
    """
    n = len(path)
    if n <= 2:
        return list(path)
    smoothed: List[Cell] = [path[0]]
    i = 0
    while i < n - 1:
        # Search downward from the end so the first hit is the furthest.
        j_best = i + 1
        for j in range(n - 1, i + 1, -1):
            if bresenham_los_clear(path[i], path[j], clear_grid):
                j_best = j
                break
        smoothed.append(path[j_best])
        i = j_best
    return smoothed


def polyline_length_metres(path: List[Cell], resolution_m: float) -> float:
    total = 0.0
    for (r1, c1), (r2, c2) in zip(path[:-1], path[1:]):
        dr, dc = r2 - r1, c2 - c1
        total += math.sqrt(dr * dr + dc * dc)
    return total * resolution_m


def polyline_clearance_along_cells(
    waypoints: List[Cell],
    distance_to_obstacle_m: np.ndarray,
) -> Tuple[float, float]:
    """``(min, mean)`` clearance over the rasterised cells of the polyline.

    Sampling the rasterised interior — not only the waypoints — matters: after
    reduction a segment can span many cells, and an interior cell may sit
    closer to an obstacle than either endpoint.

    The same method is applied to the raw and the reduced path so the two are
    directly comparable. For a raw A* path every segment is one cell long, so
    the value coincides with an endpoint-only evaluation, but the *method* is
    identical.
    """
    if not waypoints:
        return float("inf"), float("inf")
    if len(waypoints) == 1:
        v = float(distance_to_obstacle_m[waypoints[0]])
        return v, v
    seen = set()
    values: List[float] = []
    for (r1, c1), (r2, c2) in zip(waypoints[:-1], waypoints[1:]):
        for cell in bresenham_cells(r1, c1, r2, c2):
            if cell in seen:
                continue
            seen.add(cell)
            values.append(float(distance_to_obstacle_m[cell]))
    return min(values), sum(values) / len(values)


def validate_invariants(
    raw: List[Cell],
    smoothed: List[Cell],
    clear_grid: np.ndarray,
    distance_to_obstacle_m: np.ndarray,
    resolution_m: float,
    robot_radius_m: float,
) -> List[str]:
    """Return a list of invariant-violation messages. Empty list means OK."""
    errors: List[str] = []

    if not raw:
        errors.append("raw path is empty")
        return errors
    if not smoothed:
        errors.append("smoothed path is empty")
        return errors

    if smoothed[0] != raw[0]:
        errors.append(f"endpoint preservation: smoothed[0]={smoothed[0]} != raw[0]={raw[0]}")
    if smoothed[-1] != raw[-1]:
        errors.append(f"endpoint preservation: smoothed[-1]={smoothed[-1]} != raw[-1]={raw[-1]}")

    for a, b in zip(smoothed[:-1], smoothed[1:]):
        if not bresenham_los_clear(a, b, clear_grid):
            errors.append(f"line-of-sight: segment {a} -> {b} crosses an inflation-blocked cell")
            break

    raw_len = polyline_length_metres(raw, resolution_m)
    sm_len = polyline_length_metres(smoothed, resolution_m)
    if sm_len > raw_len + 1e-6:
        errors.append(
            f"length monotonicity: smoothed_length {sm_len:.6f} m > raw_length {raw_len:.6f} m"
        )

    sm_min, _ = polyline_clearance_along_cells(smoothed, distance_to_obstacle_m)
    if sm_min < robot_radius_m - 1e-6:
        errors.append(
            f"clearance floor: smoothed_min {sm_min:.6f} m < robot_radius {robot_radius_m:.6f} m"
        )

    return errors
