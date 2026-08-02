"""Stop-Replan-Resume (SRR) local replanning geometry.

*Stop-Replan-Resume* is the obstacle-response sequence: stop, plan a local
detour, resume the original mission. The replanner is the component that runs
under time pressure while the vehicle hovers, so its failure modes matter:

* an ROI that does not actually contain the obstacle,
* a detour that clips the obstacle because the spline bulged between control
  points,
* a resume anchor placed *behind* the vehicle,
* a degraded fallback silently accepted as a nominal result,
* non-deterministic output.

Each is tested here. The final test reproduces the published run exactly.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from uav_autonomy import grid as G
from uav_autonomy import mapping as M
from uav_autonomy import replanning as R
from uav_autonomy.frames import ned_to_world

REPO = Path(__file__).resolve().parents[1]
SRR = REPO / "results" / "stop_replan_resume"
CANOPY_M = {"Pine Tree": 1.5, "Oak tree": 4.0}


# ---------------------------------------------------------------- ROI spec

def test_roi_spec_is_derived_from_the_motion_envelope():
    """The ROI is derived, not tuned. Pin the derivation."""
    roi = R.derive_roi_spec(0.05)
    assert roi.R_min_m == pytest.approx(R.V_REF ** 2 / R.A_LAT_MAX)      # 2.25
    assert roi.d_stop_m == pytest.approx(R.V_REF ** 2 / (2 * R.A_STOP))  # 1.125
    assert roi.smoothing_margin_m == pytest.approx(max(2 * 0.05, 0.25))
    assert roi.ROI_min_m == pytest.approx(max(
        R.R_OBS + 2 * R.R_PREF_M + roi.smoothing_margin_m,
        R.V_REF * R.T_REPLAN_TARGET + R.R_PREF_M,
        R.LOOKAHEAD_M + roi.d_stop_m))
    # The ROI must at least cover the detection lookahead plus stopping.
    assert roi.ROI_min_m >= R.LOOKAHEAD_M + roi.d_stop_m


def test_roi_scales_with_grid_resolution():
    coarse = R.derive_roi_spec(0.20)
    fine = R.derive_roi_spec(0.05)
    assert coarse.smoothing_margin_m > fine.smoothing_margin_m
    assert coarse.lateral_half_width_m > fine.lateral_half_width_m


def test_roi_lateral_width_admits_a_detour_around_the_obstacle():
    """Half-width must exceed obstacle radius plus clearance, or no detour fits."""
    roi = R.derive_roi_spec(0.05)
    assert roi.lateral_half_width_m > R.R_OBS + R.R_PREF_M


# -------------------------------------------------------------- ROI bounds

def _straight_segment(n=400, x0=-40.0, y0=-60.0, x1=0.0, y1=-20.0):
    xs = np.linspace(x0, x1, n)
    ys = np.linspace(y0, y1, n)
    ss = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))])
    return xs, ys, ss


def test_path_aligned_bounds_contain_the_strip():
    xs, ys, ss = _straight_segment()
    half = 7.0
    bounds = R.path_aligned_world_bounds(xs, ys, ss, 10.0, 30.0, half, extra_pad_m=1.0)
    xmin, ymin, xmax, ymax = bounds
    mask = (ss >= 10.0) & (ss <= 30.0)
    assert xmin <= xs[mask].min() and xmax >= xs[mask].max()
    assert ymin <= ys[mask].min() and ymax >= ys[mask].max()
    # Wide enough for the lateral offset in both directions.
    assert (xmax - xmin) >= 2 * half * math.cos(math.pi / 4)


def test_path_aligned_bounds_fall_back_when_the_window_is_empty():
    """An s-window outside the segment must clamp, not crash."""
    xs, ys, ss = _straight_segment()
    b = R.path_aligned_world_bounds(xs, ys, ss, 1e6, 2e6, 7.0)
    assert all(math.isfinite(v) for v in b)


# ------------------------------------------------------- crop and stamping

def _synthetic_map(free_value=G.PGM_FREE, w=400, h=400, res=0.05):
    image = np.full((h, w), free_value, dtype=np.uint8)
    meta = {"resolution": res, "origin": [0.0, 0.0, 0.0], "image": "map.pgm"}
    return image, meta


def test_crop_preserves_world_georeference():
    image, meta = _synthetic_map()
    crop = R.crop_world_bounds(image, meta, (4.0, 4.0, 8.0, 8.0))
    xmin, ymin, xmax, ymax = crop.world_bounds
    assert xmin <= 4.0 and ymin <= 4.0
    assert xmax >= 8.0 and ymax >= 8.0
    # A point inside the crop maps to the same world position via either frame.
    md = crop.map_data()
    assert md.origin_x == pytest.approx(crop.origin_x)
    assert md.y_max == pytest.approx(crop.origin_y + crop.height * crop.resolution)


def test_crop_rejects_an_empty_window():
    image, meta = _synthetic_map()
    with pytest.raises(RuntimeError, match="empty crop"):
        R.crop_world_bounds(image, meta, (-100.0, -100.0, -99.0, -99.0))


def test_obstacle_stamping_marks_a_disk_of_the_right_size():
    image, meta = _synthetic_map()
    crop = R.crop_world_bounds(image, meta, (0.0, 0.0, 20.0, 20.0))
    radius = 1.5
    flipped = R.stamp_obstacle_into_crop(crop, 10.0, 10.0, radius)
    expected_cells = math.pi * (radius / crop.resolution) ** 2
    assert flipped == pytest.approx(expected_cells, rel=0.02)

    # Clearance is measured to the disk EDGE, not to its centre: a point one
    # metre beyond the rim has ~1 m of clearance, not 1 m + radius.
    md = crop.map_data()
    field = G.clearance_field(md)
    inside, mask_in = G.sample_clearances(np.array([[10.0, 10.0]]), field, md)
    gap = 1.0
    outside, mask_out = G.sample_clearances(
        np.array([[10.0 + radius + gap, 10.0]]), field, md)
    assert mask_in[0] and inside[0] == pytest.approx(0.0)
    assert mask_out[0]
    assert outside[0] == pytest.approx(gap, abs=2 * crop.resolution)


def test_stamping_is_idempotent():
    image, meta = _synthetic_map()
    crop = R.crop_world_bounds(image, meta, (0.0, 0.0, 20.0, 20.0))
    first = R.stamp_obstacle_into_crop(crop, 10.0, 10.0, 1.5)
    second = R.stamp_obstacle_into_crop(crop, 10.0, 10.0, 1.5)
    assert first > 0
    assert second == 0, "re-stamping the same disk must flip no further cells"


def test_stamping_outside_the_crop_is_a_no_op():
    image, meta = _synthetic_map()
    crop = R.crop_world_bounds(image, meta, (0.0, 0.0, 5.0, 5.0))
    assert R.stamp_obstacle_into_crop(crop, 500.0, 500.0, 1.5) == 0


# ------------------------------------------------ resume anchor and rejoin

def test_resume_anchor_is_downstream_of_the_vehicle():
    """A resume anchor behind the vehicle would command a reversal."""
    _, _, ss = _straight_segment()
    hover = 100
    idx = R.idx_at_s(ss, float(ss[hover]) + 15.0, monotonic_after=hover)
    assert idx > hover
    assert ss[idx] >= ss[hover]


def test_idx_at_s_clamps_at_both_ends():
    _, _, ss = _straight_segment()
    assert R.idx_at_s(ss, -1e6) == 0
    assert R.idx_at_s(ss, 1e6) == len(ss) - 1


def test_idx_at_s_never_returns_an_index_before_the_vehicle():
    _, _, ss = _straight_segment()
    for after in (0, 50, 200, 399):
        assert R.idx_at_s(ss, 0.0, monotonic_after=after) >= after


def test_head_jump_is_a_wrapped_absolute_difference():
    assert R.head_jump_deg(0.0, 0.0) == pytest.approx(0.0)
    assert R.head_jump_deg(0.0, math.pi / 2) == pytest.approx(90.0)
    # Across the wrap: -179 deg to +179 deg is 2 deg apart, not 358.
    assert R.head_jump_deg(math.radians(-179), math.radians(179)) == pytest.approx(2.0, abs=1e-6)
    assert 0.0 <= R.head_jump_deg(3.0, -3.0) <= 180.0


def test_rejoin_metrics_separate_position_and_yaw_jump():
    m = R.rejoin_metrics(
        replan_end_xy=(0.0, 0.0), replan_end_yaw=0.0,
        resume_xy=(0.3, 0.4), resume_yaw=math.radians(10.0),
    )
    assert m["pos_jump_replan_to_resume_m"] == pytest.approx(0.5)
    assert m["yaw_jump_replan_to_resume_deg"] == pytest.approx(10.0)


# ----------------------------------------------------- replan success/failure

@pytest.fixture(scope="module")
def open_world():
    """A large obstacle-free crop with a straight reference through it."""
    res = 0.05
    w = h = 1200
    image = np.full((h, w), G.PGM_FREE, dtype=np.uint8)
    # A border of obstacles so a clearance field exists at all.
    image[0, :] = G.PGM_OCCUPIED
    image[-1, :] = G.PGM_OCCUPIED
    image[:, 0] = G.PGM_OCCUPIED
    image[:, -1] = G.PGM_OCCUPIED
    meta = {"resolution": res, "origin": [0.0, 0.0, 0.0], "image": "map.pgm"}
    n = 600
    xs = np.linspace(10.0, 50.0, n)
    ys = np.full(n, 30.0)
    ss = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))])
    return image, meta, xs, ys, ss


def test_replan_routes_around_an_obstacle_on_the_reference(open_world):
    image, meta, xs, ys, ss = open_world
    hover = (20.0, 30.0)
    resume = (40.0, 30.0)
    obstacle = (30.0, 30.0)   # squarely on the straight reference
    cand = R.run_local_roi_replan(
        full_image=image, full_meta=meta, seg_xs=xs, seg_ys=ys, seg_ss=ss,
        obstacle_xy=obstacle, obs_r=1.5,
        s_min=float(ss[0]), s_max=float(ss[-1]),
        lateral_half_width=8.0, hover_xy=hover, resume_xy=resume,
        hover_idx=int(np.argmin(np.abs(xs - hover[0]))),
        resume_idx=int(np.argmin(np.abs(xs - resume[0]))),
    )
    assert cand.success, cand.fail_reason
    assert cand.collision_free
    assert cand.clearance_min_m >= R.R_EFF_M

    # The detour must actually leave the straight line near the obstacle.
    near = cand.smoothed_xy[np.abs(cand.smoothed_xy[:, 0] - obstacle[0]) < 2.0]
    assert near.size, "detour has no samples near the obstacle"
    assert np.abs(near[:, 1] - 30.0).max() > 1.0, "detour did not deviate"

    # And it must keep clear of the stamped obstacle.
    d = np.hypot(cand.smoothed_xy[:, 0] - obstacle[0],
                 cand.smoothed_xy[:, 1] - obstacle[1])
    assert d.min() >= 1.5, f"detour approached the obstacle to {d.min():.3f} m"


def test_replan_is_deterministic(open_world):
    image, meta, xs, ys, ss = open_world
    kw = dict(full_image=image, full_meta=meta, seg_xs=xs, seg_ys=ys, seg_ss=ss,
              obstacle_xy=(30.0, 30.0), obs_r=1.5,
              s_min=float(ss[0]), s_max=float(ss[-1]), lateral_half_width=8.0,
              hover_xy=(20.0, 30.0), resume_xy=(40.0, 30.0),
              hover_idx=150, resume_idx=450)
    a = R.run_local_roi_replan(**kw)
    b = R.run_local_roi_replan(**kw)
    assert a.success and b.success
    assert np.array_equal(a.smoothed_xy, b.smoothed_xy)
    assert a.astar_nodes_expanded == b.astar_nodes_expanded
    assert a.clearance_min_m == b.clearance_min_m


def test_replan_fails_when_the_hover_pose_is_inside_the_obstacle(open_world):
    """Starting inside the stamped obstacle must fail loudly, not route out."""
    image, meta, xs, ys, ss = open_world
    cand = R.run_local_roi_replan(
        full_image=image, full_meta=meta, seg_xs=xs, seg_ys=ys, seg_ss=ss,
        obstacle_xy=(20.0, 30.0), obs_r=3.0,   # centred on the hover pose
        s_min=float(ss[0]), s_max=float(ss[-1]), lateral_half_width=8.0,
        hover_xy=(20.0, 30.0), resume_xy=(40.0, 30.0),
        hover_idx=150, resume_idx=450,
    )
    assert not cand.success
    assert cand.fail_reason == "start_not_inflation_safe"


def test_replan_fails_when_the_resume_anchor_is_blocked(open_world):
    image, meta, xs, ys, ss = open_world
    cand = R.run_local_roi_replan(
        full_image=image, full_meta=meta, seg_xs=xs, seg_ys=ys, seg_ss=ss,
        obstacle_xy=(40.0, 30.0), obs_r=3.0,   # centred on the resume anchor
        s_min=float(ss[0]), s_max=float(ss[-1]), lateral_half_width=8.0,
        hover_xy=(20.0, 30.0), resume_xy=(40.0, 30.0),
        hover_idx=150, resume_idx=450,
    )
    assert not cand.success
    assert cand.fail_reason == "goal_no_inflation_safe_cells"


def test_replan_fails_when_the_corridor_is_sealed(open_world):
    """A wall across the whole ROI leaves no route: must fail, not clip through."""
    image, meta, xs, ys, ss = open_world
    sealed = image.copy()
    res = meta["resolution"]
    col = int(30.0 / res)
    sealed[:, col - 4:col + 4] = G.PGM_OCCUPIED
    cand = R.run_local_roi_replan(
        full_image=sealed, full_meta=meta, seg_xs=xs, seg_ys=ys, seg_ss=ss,
        obstacle_xy=(30.0, 30.0), obs_r=1.0,
        s_min=float(ss[0]), s_max=float(ss[-1]), lateral_half_width=8.0,
        hover_xy=(20.0, 30.0), resume_xy=(40.0, 30.0),
        hover_idx=150, resume_idx=450,
    )
    assert not cand.success
    assert cand.fail_reason in ("astar_failed", "start_not_inflation_safe",
                                "goal_no_inflation_safe_cells")


def test_conservative_fallback_is_not_accepted_as_success():
    """In flight a degraded polyline must abort, never pass silently."""
    from uav_autonomy.trajectory import _classify
    assert _classify(1.5, r_eff=0.65, r_pref=0.90, method="polyline") == "conservative_fallback"
    # run_local_roi_replan turns exactly that classification into a failure.
    import inspect
    src = inspect.getsource(R.run_local_roi_replan)
    assert 'conservative_fallback_not_allowed_as_pass' in src


# --------------------------------------------- regression vs published run

pytest_slow = pytest.mark.slow


@pytest.fixture(scope="module")
def published_srr():
    return {
        "runtime": yaml.safe_load((SRR / "replan_runtime_metrics.yaml").read_text()),
        "join": yaml.safe_load((SRR / "resume_join_metrics.yaml").read_text()),
        "sensed": yaml.safe_load((SRR / "sensed_obstacle_metrics.yaml").read_text()),
    }


@pytest.fixture(scope="module")
def reproduced_srr():
    """Re-run the published local replan from public inputs only."""
    obstacles = M.parse_obstacles(REPO / "sim/worlds/forest_map2.sdf")
    geom = M.compute_grid(obstacles, M.RESOLUTION_M, M.PADDING_M)
    canopy = [M.Obstacle(o.name, o.model_type, o.x, o.y, CANOPY_M[o.model_type])
              for o in obstacles]
    image = M.rasterize(canopy, geom)
    meta = {"resolution": geom.resolution,
            "origin": [geom.origin_x, geom.origin_y, 0.0]}

    xs, ys, ss = [], [], []
    with (SRR / "mission_segment_window.csv").open() as f:
        for row in csv.DictReader(f):
            ss.append(float(row["s_m"]))
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
    xs, ys, ss = np.asarray(xs), np.asarray(ys), np.asarray(ss)

    sensed = yaml.safe_load((SRR / "sensed_obstacle_metrics.yaml").read_text())
    obs_xy = tuple(sensed["perception_sensed_obstacle"]["centroid_world_xy"])
    obs_r = float(sensed["perception_sensed_obstacle"]["radius_m"])
    join = yaml.safe_load((SRR / "resume_join_metrics.yaml").read_text())
    resume_xy = tuple(join["resume_xy_world"])

    hover_xy = None
    with (SRR / "event_log.csv").open() as f:
        for row in csv.DictReader(f):
            if row["event"] == "stop_command_sent":
                inner = row["info"].split("hover_ned=(", 1)[1].split(")", 1)[0]
                n, e, _d = (float(v) for v in inner.split(","))
                hover_xy = ned_to_world(n, e)
    assert hover_xy is not None

    roi = R.derive_roi_spec(geom.resolution)
    i_hover = int(np.argmin(np.hypot(xs - hover_xy[0], ys - hover_xy[1])))
    i_resume = int(np.argmin(np.hypot(xs - resume_xy[0], ys - resume_xy[1])))
    return R.run_local_roi_replan(
        full_image=image, full_meta=meta, seg_xs=xs, seg_ys=ys, seg_ss=ss,
        obstacle_xy=obs_xy, obs_r=obs_r,
        s_min=float(ss[i_hover]) - roi.upstream_margin_m,
        s_max=float(ss[i_resume]) + roi.smoothing_margin_m,
        lateral_half_width=roi.lateral_half_width_m,
        hover_xy=hover_xy, resume_xy=resume_xy,
        hover_idx=i_hover, resume_idx=i_resume,
    )


@pytest_slow
def test_published_srr_replan_reproduces_exactly(reproduced_srr, published_srr):
    c, ref = reproduced_srr, published_srr["runtime"]
    assert c.success, c.fail_reason
    assert c.classification == ref["classification"]
    assert c.selected_attempt == ref["selected_attempt"]
    assert c.astar_nodes_expanded == ref["astar_nodes_expanded"]
    assert c.astar_path_length_m == pytest.approx(ref["astar_path_length_m"], abs=1e-9)
    assert c.smoothed_length_m == pytest.approx(ref["smoothed_length_m"], abs=1e-9)
    assert c.clearance_min_m == pytest.approx(ref["clearance_min_m"], abs=1e-12)
    assert c.max_curvature_1_per_m == pytest.approx(ref["max_curvature_1_per_m"], abs=1e-9)
    assert c.max_lateral_accel_mps2 == pytest.approx(ref["max_lateral_accel_mps2"], abs=1e-12)
    assert c.collision_free == ref["collision_free"]


@pytest_slow
def test_published_srr_path_matches_the_recorded_geometry(reproduced_srr):
    """Compare against the recorded CSV, which stores six decimals."""
    ref = []
    with (SRR / "local_replan_smoothed.csv").open() as f:
        for row in csv.DictReader(f):
            ref.append((float(row["x_world_m"]), float(row["y_world_m"])))
    ref = np.asarray(ref)
    assert len(reproduced_srr.smoothed_xy) == len(ref)
    assert np.abs(reproduced_srr.smoothed_xy - ref).max() <= 1e-6


@pytest_slow
def test_published_srr_rejoin_continuity_reproduces(reproduced_srr, published_srr):
    ref = published_srr["join"]
    end = reproduced_srr.smoothed_xy[-1]
    m = R.rejoin_metrics(
        replan_end_xy=(float(end[0]), float(end[1])),
        replan_end_yaw=float(reproduced_srr.smoothed_yaw[-1]),
        resume_xy=tuple(ref["resume_xy_world"]),
        resume_yaw=float(ref["resume_yaw_world_rad"]),
    )
    assert m["pos_jump_replan_to_resume_m"] == pytest.approx(
        ref["pos_jump_replan_to_resume_m"], abs=1e-6)
    assert m["yaw_jump_replan_to_resume_deg"] == pytest.approx(
        ref["yaw_jump_replan_to_resume_deg"], abs=1e-4)


@pytest_slow
def test_published_srr_is_boundary_not_robust(reproduced_srr):
    """The published detour clears r_eff but NOT r_pref. Pin that, so the
    qualification cannot quietly disappear from the documentation."""
    c = reproduced_srr
    assert c.clearance_min_m >= R.R_EFF_M
    assert c.clearance_min_m < R.R_PREF_M
    assert c.classification == "boundary_smoothed"
