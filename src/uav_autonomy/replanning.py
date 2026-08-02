"""Local region-of-interest replanning around a newly sensed obstacle.

When an obstacle that is absent from the prior map is detected on the active
mission segment, the vehicle stops, a *local* detour is planned, and the
mission is resumed at a downstream anchor on the original reference. Only a
cropped, patched neighbourhood of the map is searched — not the whole map.

Pipeline (:func:`run_local_roi_replan`):

1. Derive a path-aligned rectangular ROI around the obstacle.
2. Crop the prior map to that rectangle.
3. Stamp the sensed obstacle into the crop as a filled occupied disc.
4. Build the clearance field and the inflated grid on the crop.
5. Clearance-aware A* from the hover pose to the resume anchor.
6. Splice original-reference lead-in / tail-in samples onto the A* path.
7. B-spline refinement with clearance validation.
8. Slice the refined path back to the hover..resume span and re-validate.

**Lead-in / tail-in.** A few original segment samples are prepended before the
hover pose and appended after the resume anchor before smoothing. This anchors
the spline tangent at both ends to the original mission heading, collapsing
the yaw discontinuity at the join. Those samples are themselves points of the
original validated reference, so this neither weakens the clearance argument
nor introduces synthetic geometry.

**ROI sizing** (:func:`derive_roi_spec`) is derived from the motion envelope,
not tuned: minimum turn radius ``v^2/a_lat``, stopping distance
``v^2/(2*a_stop)``, the replanning time budget, and the obstacle radius plus
the preferred clearance.

Acceptance is strict. The polyline fallback is *not* accepted as a pass here:
in flight, a controlled abort is preferable to continuing on a degraded,
unsmoothed reference.
"""

from __future__ import annotations

import csv
import math
import time as _time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

from .grid import (
    OCCUPIED, PGM_FREE, PGM_OCCUPIED, MapData,
    classify_map, collect_goal_cells, cell_to_world, in_bounds,
    inflated_clear_grid, sample_clearances, world_to_cell,
)
from .planning import a_star_clearance, path_length_metres
from .trajectory import SpeedParams, refine_trajectory

__all__ = [
    "R_EFF_M", "TRACKING_MARGIN_M", "R_PREF_M", "W_CLEARANCE",
    "V_REF", "A_LAT_MAX", "A_STOP", "LOOKAHEAD_M", "R_OBS",
    "T_REPLAN_TARGET", "GOAL_RADIUS_M", "LEAD_IN_M", "TAIL_IN_M",
    "MAX_LATERAL_ACCEL", "SPEED_PARAMS",
    "RoiSpec", "derive_roi_spec", "CroppedMap",
    "crop_world_bounds", "stamp_obstacle_into_crop", "path_aligned_world_bounds",
    "ReplanCandidate", "run_local_roi_replan",
    "load_segment_rows", "seg_xy_arrays", "idx_at_s", "nearest_idx_xy",
    "head_jump_deg", "rejoin_metrics",
]

# Planning policy. Mirrors sim/config/mission_policy.yaml.
R_EFF_M = 0.65
TRACKING_MARGIN_M = 0.25
R_PREF_M = R_EFF_M + TRACKING_MARGIN_M  # 0.90
W_CLEARANCE = 5.0

# Motion envelope. Must match the nominal mission build.
V_REF = 1.5
A_LAT_MAX = 1.0
A_STOP = 1.0
LOOKAHEAD_M = 7.5
R_OBS = 1.5
T_REPLAN_TARGET = 3.0

GOAL_RADIUS_M = 0.30

# Lead-in / tail-in length in metres (see module docstring).
LEAD_IN_M = 3.0
TAIL_IN_M = 3.0

MAX_LATERAL_ACCEL = 1.0

SPEED_PARAMS = SpeedParams(v_nominal=1.5, v_min=0.3, a_lat_max=1.0, a_long_max=1.0)


# ---------------------------------------------------------------------------
# Mission-segment helpers
# ---------------------------------------------------------------------------

def load_segment_rows(mission_csv: Path, segment_name: str) -> List[dict]:
    with Path(mission_csv).open() as f:
        return [r for r in csv.DictReader(f) if r["segment_name"] == segment_name]


def seg_xy_arrays(rows: List[dict]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    xs = np.fromiter((float(r["x_m"]) for r in rows), dtype=float)
    ys = np.fromiter((float(r["y_m"]) for r in rows), dtype=float)
    ss = np.fromiter((float(r["s_m"]) for r in rows), dtype=float)
    return xs, ys, ss


def idx_at_s(
    ss: np.ndarray,
    s_target: float,
    monotonic_after: int = 0,
    clamp: bool = True,
) -> int:
    """Index whose arc length is closest to ``s_target``, searching forward only.

    Restricting to ``index >= monotonic_after`` prevents the resume anchor from
    landing behind the vehicle on a segment that revisits arc lengths.
    """
    if clamp:
        if s_target <= ss[monotonic_after]:
            return monotonic_after
        if s_target >= ss[-1]:
            return len(ss) - 1
    sub = ss[monotonic_after:]
    return monotonic_after + int(np.argmin(np.abs(sub - s_target)))


def nearest_idx_xy(rows: List[dict], xy: Tuple[float, float]) -> int:
    return int(min(
        range(len(rows)),
        key=lambda i: math.hypot(float(rows[i]["x_m"]) - xy[0],
                                 float(rows[i]["y_m"]) - xy[1]),
    ))


def head_jump_deg(a_yaw: float, b_yaw: float) -> float:
    """Absolute heading difference in degrees, wrapped to [0, 180]."""
    dy = (b_yaw - a_yaw + math.pi) % (2 * math.pi) - math.pi
    return abs(math.degrees(dy))


def rejoin_metrics(
    *,
    replan_end_xy: Tuple[float, float],
    replan_end_yaw: float,
    resume_xy: Tuple[float, float],
    resume_yaw: float,
) -> Dict[str, float]:
    """Continuity of the join between the local detour and the resumed mission.

    ``pos_jump`` and ``yaw_jump`` measure the discontinuity at the handover
    point. They are distinct from the *rejoin distance*, which is how far the
    vehicle actually is from the original reference once it has rejoined.
    """
    return {
        "pos_jump_replan_to_resume_m": float(
            math.hypot(resume_xy[0] - replan_end_xy[0], resume_xy[1] - replan_end_xy[1])
        ),
        "yaw_jump_replan_to_resume_deg": float(head_jump_deg(replan_end_yaw, resume_yaw)),
    }


# ---------------------------------------------------------------------------
# ROI derivation
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class RoiSpec:
    grid_resolution_m: float
    R_min_m: float
    d_stop_m: float
    smoothing_margin_m: float
    ROI_min_m: float
    upstream_margin_m: float
    resume_distance_m: float
    lateral_half_width_m: float


def derive_roi_spec(grid_resolution_m: float) -> RoiSpec:
    """Derive ROI extents from the motion envelope.

    ``R_min``   minimum turn radius at reference speed
    ``d_stop``  stopping distance at the configured deceleration
    ``ROI_min`` longest of: obstacle + two preferred clearances, the distance
                covered during the replanning budget, and the detection
                lookahead plus stopping distance
    """
    R_min = (V_REF * V_REF) / A_LAT_MAX
    d_stop = (V_REF * V_REF) / (2.0 * A_STOP)
    sm = max(2.0 * grid_resolution_m, 0.25)
    roi_min = max(
        R_OBS + 2.0 * R_PREF_M + sm,
        V_REF * T_REPLAN_TARGET + R_PREF_M,
        LOOKAHEAD_M + d_stop,
    )
    upstream = max(2.0 * R_min, d_stop + R_PREF_M + sm)
    resume = max(2.0 * LOOKAHEAD_M, 6.0 * R_min, 15.0)
    lateral = R_OBS + R_PREF_M + 2.0 * R_min + sm
    return RoiSpec(
        grid_resolution_m=grid_resolution_m,
        R_min_m=R_min, d_stop_m=d_stop, smoothing_margin_m=sm,
        ROI_min_m=roi_min, upstream_margin_m=upstream,
        resume_distance_m=resume, lateral_half_width_m=lateral,
    )


# ---------------------------------------------------------------------------
# Map cropping and obstacle stamping
# ---------------------------------------------------------------------------

@dataclass
class CroppedMap:
    image: np.ndarray            # uint8, PGM convention
    resolution: float
    origin_x: float
    origin_y: float
    height: int
    width: int
    world_bounds: tuple          # (xmin, ymin, xmax, ymax) actually covered

    def map_data(self) -> MapData:
        return MapData(
            image=self.image, resolution=self.resolution,
            origin_x=self.origin_x, origin_y=self.origin_y,
            height=self.height, width=self.width,
        )

    @property
    def y_max(self) -> float:
        return self.origin_y + self.height * self.resolution


def crop_world_bounds(
    full_image: np.ndarray,
    meta: dict,
    world_bounds: tuple,
) -> CroppedMap:
    """Crop a full map to a world-frame rectangle and re-georeference it."""
    res = float(meta["resolution"])
    ox = float(meta["origin"][0])
    oy = float(meta["origin"][1])
    H, W = full_image.shape
    y_max_full = oy + H * res

    xmin, ymin, xmax, ymax = world_bounds
    col_lo = int(math.floor((xmin - ox) / res))
    col_hi = int(math.ceil((xmax - ox) / res))
    row_lo = int(math.floor((y_max_full - ymax) / res))
    row_hi = int(math.ceil((y_max_full - ymin) / res))
    col_lo = max(0, col_lo)
    col_hi = min(W, col_hi)
    row_lo = max(0, row_lo)
    row_hi = min(H, row_hi)
    if col_hi <= col_lo or row_hi <= row_lo:
        raise RuntimeError(f"empty crop for world_bounds={world_bounds}")

    crop = full_image[row_lo:row_hi, col_lo:col_hi].copy()
    Hc, Wc = crop.shape
    new_ox = ox + col_lo * res
    new_oy = y_max_full - row_hi * res
    bounds_actual = (new_ox, new_oy, new_ox + Wc * res, new_oy + Hc * res)
    return CroppedMap(
        image=crop, resolution=res, origin_x=new_ox, origin_y=new_oy,
        height=Hc, width=Wc, world_bounds=bounds_actual,
    )


def stamp_obstacle_into_crop(
    crop: CroppedMap,
    cx: float,
    cy: float,
    radius_m: float,
) -> int:
    """Stamp a filled occupied disc into the crop. Returns cells flipped."""
    res = crop.resolution
    H, W = crop.height, crop.width
    y_max = crop.y_max
    r_pix = radius_m / res
    col_center = (cx - crop.origin_x) / res - 0.5
    row_center = (y_max - cy) / res - 0.5
    rmin = max(0, int(np.floor(row_center - r_pix)))
    rmax = min(H - 1, int(np.ceil(row_center + r_pix)))
    cmin = max(0, int(np.floor(col_center - r_pix)))
    cmax = min(W - 1, int(np.ceil(col_center + r_pix)))
    if rmin > rmax or cmin > cmax:
        return 0
    rs, cs = np.ogrid[rmin:rmax + 1, cmin:cmax + 1]
    dr = rs - row_center
    dc = cs - col_center
    mask = (dr * dr + dc * dc) <= r_pix * r_pix
    before = (crop.image[rmin:rmax + 1, cmin:cmax + 1] == PGM_FREE) & mask
    crop.image[rmin:rmax + 1, cmin:cmax + 1][mask] = PGM_OCCUPIED
    return int(before.sum())


def path_aligned_world_bounds(
    seg_xs: np.ndarray,
    seg_ys: np.ndarray,
    seg_ss: np.ndarray,
    s_min: float,
    s_max: float,
    lateral_half_width: float,
    extra_pad_m: float = 1.0,
) -> tuple:
    """Axis-aligned bounding box of the lateral strip along the segment.

    The strip follows the path between ``s_min`` and ``s_max``, offset by
    ``lateral_half_width`` along the local normal. ``extra_pad_m`` leaves room
    for the inflation kernel at the crop border.
    """
    mask = (seg_ss >= s_min) & (seg_ss <= s_max)
    if not mask.any():
        i_lo = int(np.argmin(np.abs(seg_ss - s_min)))
        i_hi = int(np.argmin(np.abs(seg_ss - s_max)))
        lo, hi = min(i_lo, i_hi), max(i_lo, i_hi)
        mask = np.zeros_like(seg_ss, dtype=bool)
        mask[lo:hi + 1] = True

    xs_in = seg_xs[mask]
    ys_in = seg_ys[mask]

    n = int(mask.sum())
    px = np.empty(2 * n)
    py = np.empty(2 * n)
    sub_idx = np.where(mask)[0]
    for k, idx in enumerate(sub_idx):
        i0 = max(0, idx - 1)
        i1 = min(len(seg_xs) - 1, idx + 1)
        tx = seg_xs[i1] - seg_xs[i0]
        ty = seg_ys[i1] - seg_ys[i0]
        norm = math.hypot(tx, ty) or 1.0
        tx /= norm
        ty /= norm
        nx, ny = -ty, tx
        px[k] = xs_in[k] + lateral_half_width * nx
        py[k] = ys_in[k] + lateral_half_width * ny
        px[n + k] = xs_in[k] - lateral_half_width * nx
        py[n + k] = ys_in[k] - lateral_half_width * ny

    xmin = float(min(px.min(), xs_in.min()) - extra_pad_m)
    xmax = float(max(px.max(), xs_in.max()) + extra_pad_m)
    ymin = float(min(py.min(), ys_in.min()) - extra_pad_m)
    ymax = float(max(py.max(), ys_in.max()) + extra_pad_m)
    return (xmin, ymin, xmax, ymax)


# ---------------------------------------------------------------------------
# Replan
# ---------------------------------------------------------------------------

@dataclass
class ReplanCandidate:
    success: bool
    fail_reason: Optional[str]
    astar_xy: np.ndarray
    smoothed_xy: np.ndarray
    smoothed_yaw: np.ndarray
    smoothed_curv: np.ndarray
    smoothed_v: np.ndarray
    smoothed_s: np.ndarray
    smoothed_lat_accel: np.ndarray
    classification: str
    selected_attempt: str
    clearance_min_m: float
    clearance_mean_m: float
    clearance_min_ge_r_pref: bool
    clearance_min_ge_r_eff: bool
    collision_free: bool
    in_bounds_all: bool
    max_lateral_accel_mps2: float
    max_curvature_1_per_m: float
    astar_path_length_m: float
    smoothed_length_m: float
    astar_nodes_expanded: int
    t_crop: float
    t_patch: float
    t_clearance_field: float
    t_inflate: float
    t_astar: float
    t_smoother: float
    t_validate: float
    t_total: float


def _empty_candidate(reason: str, **timings) -> ReplanCandidate:
    base = dict(
        t_crop=0.0, t_patch=0.0, t_clearance_field=0.0, t_inflate=0.0,
        t_astar=0.0, t_smoother=0.0, t_validate=0.0, t_total=0.0,
    )
    base.update(timings)
    return ReplanCandidate(
        success=False, fail_reason=reason,
        astar_xy=np.zeros((0, 2)), smoothed_xy=np.zeros((0, 2)),
        smoothed_yaw=np.zeros(0), smoothed_curv=np.zeros(0),
        smoothed_v=np.zeros(0), smoothed_s=np.zeros(0),
        smoothed_lat_accel=np.zeros(0),
        classification="fail", selected_attempt="n/a",
        clearance_min_m=0.0, clearance_mean_m=0.0,
        clearance_min_ge_r_pref=False, clearance_min_ge_r_eff=False,
        collision_free=False, in_bounds_all=False,
        max_lateral_accel_mps2=0.0, max_curvature_1_per_m=0.0,
        astar_path_length_m=0.0, smoothed_length_m=0.0,
        astar_nodes_expanded=int(base.pop("astar_nodes_expanded", 0)),
        **base,
    )


def _segment_leadin_xy(
    seg_xs: np.ndarray, seg_ys: np.ndarray, seg_ss: np.ndarray,
    hover_idx: int, lead_in_m: float,
) -> np.ndarray:
    """Original samples from ``s_hover - lead_in`` up to (excluding) hover."""
    if hover_idx <= 0 or lead_in_m <= 0.0:
        return np.zeros((0, 2), dtype=float)
    s_hover = float(seg_ss[hover_idx])
    s_lo = s_hover - lead_in_m
    lo = int(np.searchsorted(seg_ss, s_lo, side="left"))
    lo = max(0, min(lo, hover_idx))
    if lo >= hover_idx:
        return np.zeros((0, 2), dtype=float)
    return np.column_stack([seg_xs[lo:hover_idx], seg_ys[lo:hover_idx]])


def _segment_tailin_xy(
    seg_xs: np.ndarray, seg_ys: np.ndarray, seg_ss: np.ndarray,
    resume_idx: int, tail_in_m: float,
) -> np.ndarray:
    """Original samples strictly after resume, up to ``s_resume + tail_in``."""
    if resume_idx >= len(seg_ss) - 1 or tail_in_m <= 0.0:
        return np.zeros((0, 2), dtype=float)
    s_resume = float(seg_ss[resume_idx])
    s_hi = s_resume + tail_in_m
    hi = int(np.searchsorted(seg_ss, s_hi, side="right"))
    hi = max(resume_idx + 1, min(hi, len(seg_ss)))
    return np.column_stack([seg_xs[resume_idx + 1:hi], seg_ys[resume_idx + 1:hi]])


def run_local_roi_replan(
    *,
    full_image: np.ndarray,
    full_meta: dict,
    seg_xs: np.ndarray,
    seg_ys: np.ndarray,
    seg_ss: np.ndarray,
    obstacle_xy: Tuple[float, float],
    obs_r: float,
    s_min: float,
    s_max: float,
    lateral_half_width: float,
    hover_xy: Tuple[float, float],
    resume_xy: Tuple[float, float],
    hover_idx: int = -1,
    resume_idx: int = -1,
) -> ReplanCandidate:
    """Crop, patch, plan, refine and validate a local detour.

    Every stage is timed separately; ``t_total`` is what the flight-time
    replanning budget is measured against.
    """
    t0 = _time.perf_counter()
    wb = path_aligned_world_bounds(
        seg_xs, seg_ys, seg_ss, s_min, s_max, lateral_half_width, extra_pad_m=1.0,
    )
    crop = crop_world_bounds(full_image, full_meta, wb)
    t_crop = _time.perf_counter() - t0

    t1 = _time.perf_counter()
    stamp_obstacle_into_crop(crop, obstacle_xy[0], obstacle_xy[1], obs_r)
    t_patch = _time.perf_counter() - t1

    t2 = _time.perf_counter()
    grid = classify_map(crop.image, negate=0, occupied_thresh=0.65, free_thresh=0.196)
    occupied = grid == OCCUPIED
    if not occupied.any():
        return _empty_candidate(
            "no_occupied_cells_in_crop",
            t_crop=t_crop, t_patch=t_patch, t_total=_time.perf_counter() - t0,
        )
    clearance_m_field = distance_transform_edt(~occupied).astype(np.float64) * crop.resolution
    t_clearance_field = _time.perf_counter() - t2

    t3 = _time.perf_counter()
    inflation_cells = int(math.ceil(R_EFF_M / crop.resolution))
    clear_grid = inflated_clear_grid(grid, inflation_cells, unknown_is_blocked=False)
    t_inflate = _time.perf_counter() - t3

    origin = [crop.origin_x, crop.origin_y, 0.0]
    common = dict(
        t_crop=t_crop, t_patch=t_patch,
        t_clearance_field=t_clearance_field, t_inflate=t_inflate,
    )

    start_cell = world_to_cell(hover_xy, origin, crop.resolution, crop.height)
    if not in_bounds(start_cell, clear_grid.shape):
        return _empty_candidate(
            "start_out_of_crop", **common, t_total=_time.perf_counter() - t0,
        )
    if not clear_grid[start_cell]:
        return _empty_candidate(
            "start_not_inflation_safe", **common, t_total=_time.perf_counter() - t0,
        )

    goal_cells, goal_centre, goal_radius_cells = collect_goal_cells(
        resume_xy, GOAL_RADIUS_M, origin, crop.resolution, clear_grid.shape, clear_grid,
    )
    if not goal_cells:
        return _empty_candidate(
            "goal_no_inflation_safe_cells", **common, t_total=_time.perf_counter() - t0,
        )

    t4 = _time.perf_counter()
    astar = a_star_clearance(
        clear_grid=clear_grid,
        clearance_m_field=clearance_m_field,
        start=start_cell,
        goal_cells_set=frozenset(goal_cells),
        goal_centre=goal_centre,
        goal_radius_cells=goal_radius_cells,
        r_eff=R_EFF_M, r_pref=R_PREF_M, w_clearance=W_CLEARANCE,
    )
    t_astar = _time.perf_counter() - t4
    if not astar.success:
        return _empty_candidate(
            "astar_failed", **common, t_astar=t_astar,
            astar_nodes_expanded=int(astar.nodes_expanded),
            t_total=_time.perf_counter() - t0,
        )

    astar_xy = np.array(
        [cell_to_world(c, origin, crop.resolution, crop.height) for c in astar.path_cells],
        dtype=float,
    )
    astar_len = path_length_metres(astar.path_cells, crop.resolution)

    lead_xy = (
        _segment_leadin_xy(seg_xs, seg_ys, seg_ss, hover_idx, LEAD_IN_M)
        if hover_idx >= 0 else np.zeros((0, 2), dtype=float)
    )
    tail_xy = (
        _segment_tailin_xy(seg_xs, seg_ys, seg_ss, resume_idx, TAIL_IN_M)
        if resume_idx >= 0 else np.zeros((0, 2), dtype=float)
    )

    # astar_xy already contains the hover and resume endpoints; drop duplicates.
    eps = 1e-6
    if len(lead_xy) and len(astar_xy) and np.linalg.norm(lead_xy[-1] - astar_xy[0]) < eps:
        lead_xy = lead_xy[:-1]
    if len(tail_xy) and len(astar_xy) and np.linalg.norm(tail_xy[0] - astar_xy[-1]) < eps:
        tail_xy = tail_xy[1:]
    extended_xy = np.concatenate(
        [arr for arr in (lead_xy, astar_xy, tail_xy) if len(arr)], axis=0,
    )

    t5 = _time.perf_counter()
    sel = refine_trajectory(
        astar_xy=extended_xy,
        map_data=crop.map_data(),
        clearance_m_field=clearance_m_field,
        r_eff=R_EFF_M, r_pref=R_PREF_M, sp=SPEED_PARAMS,
    )
    t_smoother = _time.perf_counter() - t5

    t6 = _time.perf_counter()
    a = sel.selected
    smoothed_xy_full = a.samples_xy
    classification = a.classification

    # Locate hover and resume inside the extended smoothed path; the replan
    # slice between them is what gets stitched into the mission.
    if len(smoothed_xy_full):
        i_hover_s = int(np.argmin(np.linalg.norm(smoothed_xy_full - np.asarray(hover_xy), axis=1)))
        i_resume_s = int(np.argmin(np.linalg.norm(smoothed_xy_full - np.asarray(resume_xy), axis=1)))
        if i_resume_s <= i_hover_s:
            i_resume_s = min(len(smoothed_xy_full) - 1, i_hover_s + 1)
    else:
        i_hover_s = 0
        i_resume_s = 0

    smoothed_xy = smoothed_xy_full[i_hover_s:i_resume_s + 1]
    sliced_yaw = a.yaw[i_hover_s:i_resume_s + 1] if len(a.yaw) else np.zeros(0)
    sliced_curv = a.curvature[i_hover_s:i_resume_s + 1] if len(a.curvature) else np.zeros(0)
    sliced_s = (a.s[i_hover_s:i_resume_s + 1] - a.s[i_hover_s]) if len(a.s) else np.zeros(0)
    sliced_v = sel.v_ref[i_hover_s:i_resume_s + 1] if len(sel.v_ref) else np.zeros(0)
    sliced_lat = sel.lateral_accel[i_hover_s:i_resume_s + 1] if len(sel.lateral_accel) else np.zeros(0)

    smoothed_clr, ibmask = sample_clearances(smoothed_xy, clearance_m_field, crop.map_data())
    if ibmask.all():
        clr_min = float(smoothed_clr.min())
        clr_mean = float(smoothed_clr.mean())
    else:
        clr_min = 0.0
        clr_mean = float(smoothed_clr[ibmask].mean()) if ibmask.any() else 0.0
    collision_free = bool(ibmask.all() and (smoothed_clr > 0.0).all())
    max_lat = float(np.max(np.abs(sliced_lat))) if len(sliced_lat) else 0.0
    t_validate = _time.perf_counter() - t6
    t_total = _time.perf_counter() - t0

    # Strict acceptance: in flight, abort beats continuing on a degraded path.
    fail_reason = None
    success = True
    if classification == "fail":
        fail_reason = "smoother_failed"
        success = False
    elif classification == "conservative_fallback":
        fail_reason = "conservative_fallback_not_allowed_as_pass"
        success = False
    elif clr_min < R_EFF_M - 1e-9:
        fail_reason = f"clearance_min_lt_r_eff({clr_min:.3f}<{R_EFF_M})"
        success = False
    elif not collision_free:
        fail_reason = "not_collision_free"
        success = False
    elif max_lat > MAX_LATERAL_ACCEL + 1e-6:
        fail_reason = f"max_lateral_accel({max_lat:.3f}>{MAX_LATERAL_ACCEL})"
        success = False

    return ReplanCandidate(
        success=success, fail_reason=fail_reason,
        astar_xy=astar_xy, smoothed_xy=smoothed_xy,
        smoothed_yaw=sliced_yaw, smoothed_curv=sliced_curv,
        smoothed_v=sliced_v, smoothed_s=sliced_s,
        smoothed_lat_accel=sliced_lat,
        classification=classification, selected_attempt=a.name,
        clearance_min_m=clr_min, clearance_mean_m=clr_mean,
        clearance_min_ge_r_pref=bool(clr_min >= R_PREF_M - 1e-9),
        clearance_min_ge_r_eff=bool(clr_min >= R_EFF_M - 1e-9),
        collision_free=collision_free, in_bounds_all=bool(ibmask.all()),
        max_lateral_accel_mps2=max_lat,
        max_curvature_1_per_m=float(np.max(np.abs(sliced_curv))) if len(sliced_curv) else 0.0,
        astar_path_length_m=float(astar_len),
        smoothed_length_m=float(sliced_s[-1]) if len(sliced_s) else 0.0,
        astar_nodes_expanded=int(astar.nodes_expanded),
        t_crop=t_crop, t_patch=t_patch,
        t_clearance_field=t_clearance_field, t_inflate=t_inflate,
        t_astar=t_astar, t_smoother=t_smoother, t_validate=t_validate,
        t_total=t_total,
    )
