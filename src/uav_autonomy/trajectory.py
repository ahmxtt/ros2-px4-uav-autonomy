"""Cubic B-spline trajectory refinement with clearance-validated selection,
plus the curvature-aware speed profile.

Refinement runs a fixed three-stage cascade. Each stage produces a candidate,
the candidate is validated on dense samples against the *raw* clearance field,
and the first candidate that clears ``r_eff`` is selected:

===========  ==============================================================
Attempt 1    Interpolating cubic B-spline (``splprep``, ``k=3``, ``s=0``)
             through control points resampled at 2.0 m arc length.
Attempt 2    Same, resampled at 1.0 m — denser control points track the input
             path more closely and therefore cut corners less.
Fallback     Arc-length-resampled polyline at 0.5 m. No smoothing at all;
             linear interpolation between control points.
===========  ==============================================================

Classification of the selected candidate:

``robust_smoothed``
    ``clearance_min >= r_pref``
``boundary_smoothed``
    ``r_eff <= clearance_min < r_pref``
``conservative_fallback``
    Produced by the polyline fallback. The marker is kept even when its
    clearance happens to exceed ``r_pref``, so a degraded run is never
    reported as a nominal one.
``fail``
    ``clearance_min < r_eff``.

Validation is on dense samples (default 0.02 m), not on the control points:
a spline can bulge between control points, and that bulge is what has to
clear the obstacle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.spatial import cKDTree

from .grid import MapData, sample_clearances

__all__ = [
    "cumulative_arc_length", "deduplicate_consecutive", "arc_length_resample",
    "curvature_series", "tangent_yaw", "deviation_to_astar",
    "SpeedParams", "speed_profile", "estimated_travel_time",
    "AttemptResult", "SelectedReference", "refine_trajectory",
    "check_motion_limits",
]


# ---------------------------------------------------------------------------
# Arc-length geometry
# ---------------------------------------------------------------------------

def cumulative_arc_length(xy: np.ndarray) -> np.ndarray:
    diffs = np.diff(xy, axis=0)
    seg = np.linalg.norm(diffs, axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def deduplicate_consecutive(xy: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """Drop consecutive duplicate points; ``splprep`` rejects zero-length spans."""
    if len(xy) < 2:
        return xy
    keep = [0]
    for i in range(1, len(xy)):
        if np.linalg.norm(xy[i] - xy[keep[-1]]) > eps:
            keep.append(i)
    return xy[np.asarray(keep, dtype=int)]


def arc_length_resample(xy: np.ndarray, step_m: float) -> np.ndarray:
    """Resample a polyline at uniform arc length, preserving both endpoints."""
    s = cumulative_arc_length(xy)
    total = float(s[-1])
    if total <= step_m:
        return xy.copy()
    n = int(math.ceil(total / step_m))
    targets = np.linspace(0.0, total, n + 1)
    x = np.interp(targets, s, xy[:, 0])
    y = np.interp(targets, s, xy[:, 1])
    out = np.column_stack([x, y])
    out[0] = xy[0]
    out[-1] = xy[-1]
    return out


def curvature_series(d1: np.ndarray, d2: np.ndarray) -> np.ndarray:
    """Analytic curvature from parametric first and second derivatives."""
    num = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
    denom = np.power(d1[:, 0] ** 2 + d1[:, 1] ** 2, 1.5)
    out = np.zeros_like(num)
    mask = denom > 1e-12
    out[mask] = num[mask] / denom[mask]
    return out


def tangent_yaw(d1: np.ndarray) -> np.ndarray:
    return np.arctan2(d1[:, 1], d1[:, 0])


def deviation_to_astar(
    samples_xy: np.ndarray,
    astar_xy: np.ndarray,
) -> Tuple[float, float]:
    """``(max, mean)`` nearest-neighbour distance from samples to the input path."""
    tree = cKDTree(astar_xy)
    d, _ = tree.query(samples_xy, k=1)
    return float(d.max()), float(d.mean())


# ---------------------------------------------------------------------------
# Speed profile
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SpeedParams:
    v_nominal: float
    v_min: float
    a_lat_max: float
    a_long_max: float
    epsilon: float = 1.0e-6


def speed_profile(
    curvatures: np.ndarray,
    segment_ds: np.ndarray,
    sp: SpeedParams,
) -> dict:
    """Curvature-bounded speed with forward/backward longitudinal limiting.

    ``v_curve = sqrt(a_lat_max / kappa)`` clamped to ``[v_min, v_nominal]``,
    then a forward pass enforcing ``v[i+1]^2 <= v[i]^2 + 2*a*ds`` and a
    backward pass enforcing the symmetric deceleration bound.

    ``segment_ds`` has length ``N-1`` (distance from sample ``i`` to ``i+1``).
    """
    eps = sp.epsilon
    v_curve = np.sqrt(sp.a_lat_max / np.maximum(curvatures, eps))
    v_clamp = np.clip(v_curve, sp.v_min, sp.v_nominal)

    v_fwd = v_clamp.copy()
    for i in range(len(segment_ds)):
        v_max_next = math.sqrt(max(v_fwd[i] ** 2 + 2.0 * sp.a_long_max * segment_ds[i], 0.0))
        if v_fwd[i + 1] > v_max_next:
            v_fwd[i + 1] = v_max_next

    v_back = v_fwd.copy()
    for i in range(len(segment_ds) - 1, -1, -1):
        v_max_curr = math.sqrt(max(v_back[i + 1] ** 2 + 2.0 * sp.a_long_max * segment_ds[i], 0.0))
        if v_back[i] > v_max_curr:
            v_back[i] = v_max_curr

    v_final = np.maximum(v_back, sp.v_min)
    lat_acc = (v_final ** 2) * curvatures
    return {
        "v_curve": v_curve,
        "v_after_clamp": v_clamp,
        "v_after_fwd": v_fwd,
        "v_final": v_final,
        "lateral_accel": lat_acc,
    }


def estimated_travel_time(v: np.ndarray, segment_ds: np.ndarray) -> float:
    v_avg = 0.5 * (v[:-1] + v[1:])
    v_avg = np.maximum(v_avg, 1e-6)
    return float(np.sum(segment_ds / v_avg))


def check_motion_limits(
    *,
    v_ref: np.ndarray,
    lateral_accel: np.ndarray,
    segment_ds: np.ndarray,
    sp: SpeedParams,
    tol: float = 1e-6,
) -> dict:
    """Post-hoc feasibility check of a generated speed profile.

    Reports the realised extrema and whether each configured bound holds. The
    longitudinal check is derived from consecutive speeds and spacing, so it
    validates the profile as sampled rather than as intended.
    """
    v = np.asarray(v_ref, dtype=float)
    lat = np.abs(np.asarray(lateral_accel, dtype=float))
    ds = np.asarray(segment_ds, dtype=float)

    if len(v) >= 2 and len(ds) >= 1:
        dv2 = np.abs(v[1:] ** 2 - v[:-1] ** 2)
        a_long = dv2 / (2.0 * np.maximum(ds, 1e-9))
        a_long_max = float(a_long.max())
    else:
        a_long_max = 0.0

    v_max = float(v.max()) if len(v) else 0.0
    v_min = float(v.min()) if len(v) else 0.0
    lat_max = float(lat.max()) if len(lat) else 0.0

    return {
        "v_max_mps": v_max,
        "v_min_mps": v_min,
        "a_lat_max_mps2": lat_max,
        "a_long_max_mps2": a_long_max,
        "v_within_nominal": bool(v_max <= sp.v_nominal + tol),
        "v_above_min": bool(v_min >= sp.v_min - tol),
        "a_lat_within_limit": bool(lat_max <= sp.a_lat_max + tol),
        "a_long_within_limit": bool(a_long_max <= sp.a_long_max + tol),
    }


# ---------------------------------------------------------------------------
# Refinement cascade
# ---------------------------------------------------------------------------

@dataclass
class AttemptResult:
    name: str
    success: bool
    samples_xy: np.ndarray
    control_pts: np.ndarray
    s: np.ndarray
    yaw: np.ndarray
    curvature: np.ndarray
    clearance: np.ndarray
    classification: str
    clearance_min_m: float
    clearance_mean_m: float
    max_deviation_from_astar_m: float
    mean_deviation_from_astar_m: float
    curvature_max_1_per_m: float
    collision_free: bool
    in_bounds_all: bool
    trajectory_length_m: float
    n_samples: int
    n_control_points: int
    method: str
    control_step_m: float
    notes: List[str] = field(default_factory=list)


@dataclass
class SelectedReference:
    selected: AttemptResult
    attempts: List[AttemptResult]
    v_ref: np.ndarray
    lateral_accel: np.ndarray
    estimated_travel_time_s: float
    speed_params: SpeedParams


def _bspline_evaluate(
    control_pts: np.ndarray, n_samples: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Interpolating cubic B-spline (s=0). Returns ``(xy, d1, d2)``."""
    s_arc = cumulative_arc_length(control_pts)
    u = s_arc / s_arc[-1]
    tck, _ = splprep([control_pts[:, 0], control_pts[:, 1]], u=u, k=3, s=0.0)
    u_eval = np.linspace(0.0, 1.0, n_samples)
    x, y = splev(u_eval, tck)
    dx, dy = splev(u_eval, tck, der=1)
    ddx, ddy = splev(u_eval, tck, der=2)
    return (
        np.column_stack([np.asarray(x), np.asarray(y)]),
        np.column_stack([np.asarray(dx), np.asarray(dy)]),
        np.column_stack([np.asarray(ddx), np.asarray(ddy)]),
    )


def _polyline_evaluate(
    control_pts: np.ndarray, eval_step_m: float,
) -> Tuple[np.ndarray, np.ndarray]:
    s_cp = cumulative_arc_length(control_pts)
    total = float(s_cp[-1])
    n_eval = max(int(math.ceil(total / eval_step_m)) + 1, 8)
    s_eval = np.linspace(0.0, total, n_eval)
    x = np.interp(s_eval, s_cp, control_pts[:, 0])
    y = np.interp(s_eval, s_cp, control_pts[:, 1])
    return np.column_stack([x, y]), s_eval


def _curvature_finite_diff(samples_xy: np.ndarray, s: np.ndarray) -> np.ndarray:
    """Three-point finite-difference curvature.

    Used for the polyline fallback, which has no analytic derivative. At sharp
    polyline corners this yields high but bounded values (~1/eval_step).
    """
    n = len(samples_xy)
    curv = np.zeros(n)
    if n < 3:
        return curv
    dx = np.gradient(samples_xy[:, 0], s)
    dy = np.gradient(samples_xy[:, 1], s)
    ddx = np.gradient(dx, s)
    ddy = np.gradient(dy, s)
    num = np.abs(dx * ddy - dy * ddx)
    denom = np.power(dx * dx + dy * dy, 1.5)
    mask = denom > 1e-9
    curv[mask] = num[mask] / denom[mask]
    return curv


def _yaw_from_path(samples_xy: np.ndarray, s: np.ndarray) -> np.ndarray:
    dx = np.gradient(samples_xy[:, 0], s)
    dy = np.gradient(samples_xy[:, 1], s)
    return np.arctan2(dy, dx)


def _classify(clr_min: float, r_eff: float, r_pref: float, method: str) -> str:
    if clr_min < r_eff - 1e-9:
        return "fail"
    if method == "polyline":
        # The fallback keeps its marker even above r_pref: a degraded run must
        # never be reported as nominal.
        return "conservative_fallback"
    return "robust_smoothed" if clr_min >= r_pref - 1e-9 else "boundary_smoothed"


def _run_attempt(
    *,
    astar_xy: np.ndarray,
    map_data: MapData,
    clearance_m_field: np.ndarray,
    r_eff: float,
    r_pref: float,
    method: str,
    control_step_m: float,
    eval_step_m: float,
) -> AttemptResult:
    cp = arc_length_resample(astar_xy, control_step_m)
    notes: List[str] = []
    if len(cp) < 4 and method == "bspline_s0":
        notes.append(
            f"only {len(cp)} control points after resample at {control_step_m} m; "
            "B-spline (k=3) requires >= 4. Skipping."
        )
        return AttemptResult(
            name=f"{method}_{control_step_m:.2f}m",
            success=False, samples_xy=np.zeros((0, 2)),
            control_pts=cp, s=np.zeros(0), yaw=np.zeros(0),
            curvature=np.zeros(0), clearance=np.zeros(0),
            classification="fail",
            clearance_min_m=0.0, clearance_mean_m=0.0,
            max_deviation_from_astar_m=float("inf"),
            mean_deviation_from_astar_m=float("inf"),
            curvature_max_1_per_m=float("inf"),
            collision_free=False, in_bounds_all=False,
            trajectory_length_m=0.0, n_samples=0,
            n_control_points=int(len(cp)),
            method=method, control_step_m=control_step_m, notes=notes,
        )

    astar_len = float(cumulative_arc_length(astar_xy)[-1])
    n_eval = max(int(math.ceil(astar_len / eval_step_m)) + 1, 1024)

    if method == "bspline_s0":
        samples_xy, d1, d2 = _bspline_evaluate(cp, n_eval)
        s = cumulative_arc_length(samples_xy)
        curv = curvature_series(d1, d2)
        yaw = tangent_yaw(d1)
    else:
        samples_xy, _ = _polyline_evaluate(cp, eval_step_m)
        s = cumulative_arc_length(samples_xy)
        curv = _curvature_finite_diff(samples_xy, s)
        yaw = _yaw_from_path(samples_xy, s)

    clearances, ibmask = sample_clearances(samples_xy, clearance_m_field, map_data)
    all_in = bool(ibmask.all())
    if all_in:
        clr_min = float(clearances.min())
        clr_mean = float(clearances.mean())
    else:
        clr_min = 0.0
        clr_mean = float(clearances[ibmask].mean()) if ibmask.any() else 0.0
    coll_free = bool(all_in and (clearances > 0.0).all())
    max_dev, mean_dev = deviation_to_astar(samples_xy, astar_xy)

    classification = _classify(clr_min, r_eff=r_eff, r_pref=r_pref, method=method)

    return AttemptResult(
        name=f"{method}_{control_step_m:.2f}m",
        success=(classification != "fail"),
        samples_xy=samples_xy, control_pts=cp, s=s, yaw=yaw,
        curvature=curv, clearance=clearances,
        classification=classification,
        clearance_min_m=clr_min, clearance_mean_m=clr_mean,
        max_deviation_from_astar_m=float(max_dev),
        mean_deviation_from_astar_m=float(mean_dev),
        curvature_max_1_per_m=float(curv.max() if len(curv) else 0.0),
        collision_free=coll_free, in_bounds_all=all_in,
        trajectory_length_m=float(s[-1] if len(s) else 0.0),
        n_samples=int(len(samples_xy)),
        n_control_points=int(len(cp)),
        method=method, control_step_m=control_step_m, notes=notes,
    )


def refine_trajectory(
    *,
    astar_xy: np.ndarray,
    map_data: MapData,
    clearance_m_field: np.ndarray,
    r_eff: float,
    r_pref: float,
    sp: SpeedParams,
    eval_step_m: float = 0.02,
    control_step_attempt1_m: float = 2.0,
    control_step_attempt2_m: float = 1.0,
    fallback_step_m: float = 0.5,
) -> SelectedReference:
    """Run the Attempt 1 / Attempt 2 / Fallback cascade and profile the winner.

    The fallback is only evaluated if both spline attempts fail, so a healthy
    input costs two attempts, not three.
    """
    attempts: List[AttemptResult] = []

    a1 = _run_attempt(
        astar_xy=astar_xy, map_data=map_data,
        clearance_m_field=clearance_m_field,
        r_eff=r_eff, r_pref=r_pref,
        method="bspline_s0", control_step_m=control_step_attempt1_m,
        eval_step_m=eval_step_m,
    )
    attempts.append(a1)

    a2 = _run_attempt(
        astar_xy=astar_xy, map_data=map_data,
        clearance_m_field=clearance_m_field,
        r_eff=r_eff, r_pref=r_pref,
        method="bspline_s0", control_step_m=control_step_attempt2_m,
        eval_step_m=eval_step_m,
    )
    attempts.append(a2)

    selected: Optional[AttemptResult] = None
    if a1.classification in ("robust_smoothed", "boundary_smoothed"):
        selected = a1
    elif a2.classification in ("robust_smoothed", "boundary_smoothed"):
        selected = a2
    else:
        fb = _run_attempt(
            astar_xy=astar_xy, map_data=map_data,
            clearance_m_field=clearance_m_field,
            r_eff=r_eff, r_pref=r_pref,
            method="polyline", control_step_m=fallback_step_m,
            eval_step_m=eval_step_m,
        )
        attempts.append(fb)
        if fb.classification == "conservative_fallback":
            selected = fb

    if selected is None:
        return SelectedReference(
            selected=attempts[-1], attempts=attempts,
            v_ref=np.zeros(0), lateral_accel=np.zeros(0),
            estimated_travel_time_s=float("inf"), speed_params=sp,
        )

    segment_ds = np.diff(selected.s)
    speeds = speed_profile(selected.curvature, segment_ds, sp)
    v_final = speeds["v_final"]
    lat_acc = speeds["lateral_accel"]
    t_total = estimated_travel_time(v_final, segment_ds)
    return SelectedReference(
        selected=selected, attempts=attempts,
        v_ref=v_final, lateral_accel=lat_acc,
        estimated_travel_time_s=float(t_total), speed_params=sp,
    )
