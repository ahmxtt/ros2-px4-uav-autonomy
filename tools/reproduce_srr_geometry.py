#!/usr/bin/env python3
"""Offline reproduction of the Stop-Replan-Resume (SRR) planning geometry.

*Stop-Replan-Resume* is this project's name for the obstacle-response
sequence: on detecting an obstacle absent from the prior map, the vehicle
stops, plans a local detour, and resumes the original mission.

This tool re-derives the **geometry** of the published SRR replan from public
inputs only, with no ROS, no PX4 and no Gazebo:

1. rebuild the execution prior map from ``sim/worlds/forest_map2.sdf``;
2. read the *sensed* obstacle from the published SRR result bundle -- the same
   perception output the vehicle acted on, not the ground truth;
3. reconstruct the mission segment around the event from the published
   local-replan reference;
4. run the canonical local-ROI replan (``uav_autonomy.replanning``);
5. compare the resulting geometry and metrics against the published reference.

Scope, stated plainly
---------------------

**Reproduced here (geometry, deterministic):** the ROI extents, the cropped
and patched map, the clearance-aware A* detour, the B-spline refinement, the
resulting clearance, curvature, lateral acceleration, path lengths and the
rejoin continuity.

**NOT reproduced here (live SITL only):** the detection latency, the
stop-to-hover transition, the wall-clock replanning runtime, the tracking
error and the mission outcome. Those are properties of a flown run. The
runtime figure quoted in the documentation (0.25 s) is the value measured
online during the controlled SITL mission; the runtime printed by this tool is
the offline recomputation time on the current machine and is a different
quantity.

Usage::

    python3 tools/reproduce_srr_geometry.py --output-dir /tmp/srr_geom

Exits non-zero on a genuine geometric regression. Run it from the repository
root; repository inputs are resolved relative to this file rather than to the
shell's working directory, so an absolute-path invocation from elsewhere works
too, and CI does exactly that.
"""

from __future__ import annotations

import argparse
import csv
import math
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_autonomy import grid as G  # noqa: E402
from uav_autonomy import mapping as M  # noqa: E402
from uav_autonomy import replanning as R  # noqa: E402
from uav_autonomy.artifacts import (  # noqa: E402
    format_float, write_csv, write_hashes, write_yaml,
)

REFERENCE_DIR = REPO_ROOT / "results" / "stop_replan_resume"

# Geometric tolerances. The map and the A* are exact; the spline metrics carry
# the same 1e-9 floating-point allowance as the nominal regression test.
TOL_EXACT = 0.0
TOL_GEOM_M = 1e-6
TOL_METRIC = 1e-9


def rule(title: str) -> None:
    print(f"\n{'=' * 74}\n{title}\n{'=' * 74}")


def build_execution_prior(out_dir: Path, world_sdf: Path):
    """Rasterise the SDF at canopy radii on the trunk-derived grid geometry."""
    image, geom, _ = M.execution_prior(world_sdf)
    d = out_dir / "01_map"
    d.mkdir(parents=True, exist_ok=True)
    M.write_pgm(image, d / "map.pgm")
    M.write_map_yaml(d / "map.yaml", geom, "map.pgm")
    write_hashes(d / "hashes.txt", [d / "map.pgm", d / "map.yaml"])
    return d / "map.yaml", geom, image


def load_hover_setpoint_world():
    """The commanded hover setpoint, recovered from the published event log.

    The replan starts from the hover *setpoint*, not from the first sample of
    the recorded path, so this is the faithful start pose. The event log stores
    it in the PX4 local NED frame; ``uav_autonomy.frames`` converts it back.
    """
    from uav_autonomy.frames import ned_to_world
    with (REFERENCE_DIR / "event_log.csv").open() as f:
        for row in csv.DictReader(f):
            if row["event"] != "stop_command_sent":
                continue
            inner = row["info"].split("hover_ned=(", 1)[1].split(")", 1)[0]
            n, e, _d = (float(v) for v in inner.split(","))
            return ned_to_world(n, e)
    raise RuntimeError("stop_command_sent event not found in event_log.csv")


def load_reference():
    """Published SRR reference: sensed obstacle, replan metrics, join metrics."""
    sensed = yaml.safe_load((REFERENCE_DIR / "sensed_obstacle_metrics.yaml").read_text())
    runtime = yaml.safe_load((REFERENCE_DIR / "replan_runtime_metrics.yaml").read_text())
    join = yaml.safe_load((REFERENCE_DIR / "resume_join_metrics.yaml").read_text())
    smoothed = []
    with (REFERENCE_DIR / "local_replan_smoothed.csv").open() as f:
        for row in csv.DictReader(f):
            smoothed.append((float(row["x_world_m"]), float(row["y_world_m"])))
    astar = []
    with (REFERENCE_DIR / "local_replan_astar.csv").open() as f:
        rd = csv.DictReader(f)
        xk = "x_world_m" if "x_world_m" in rd.fieldnames else "x_m"
        yk = "y_world_m" if "y_world_m" in rd.fieldnames else "y_m"
        for row in rd:
            astar.append((float(row[xk]), float(row[yk])))
    return sensed, runtime, join, np.asarray(smoothed), np.asarray(astar)


def load_segment_window():
    """Load the published window of the nominal mission segment.

    This is the actual input the local replanner consumed: the replanner reads
    the segment for the ROI bounding box and for the lead-in / tail-in anchors
    that pin the spline tangent at both ends.
    """
    path = REFERENCE_DIR / "mission_segment_window.csv"
    xs, ys, ss = [], [], []
    with path.open() as f:
        for row in csv.DictReader(f):
            ss.append(float(row["s_m"]))
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
    return np.asarray(xs), np.asarray(ys), np.asarray(ss)


def nearest_index(xs, ys, xy):
    return int(np.argmin(np.hypot(xs - xy[0], ys - xy[1])))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--world-sdf", type=Path, default=REPO_ROOT / "sim/worlds/forest_map2.sdf")
    ap.add_argument("--no-figure", action="store_true")
    args = ap.parse_args()

    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.perf_counter()

    print(f"repository : {REPO_ROOT}")
    print(f"output     : {out.resolve()}")
    print(f"python     : {platform.python_version()}   numpy {np.__version__}")

    sensed, ref_runtime, ref_join, smoothed_ref, astar_ref = load_reference()
    obs_xy = tuple(sensed["perception_sensed_obstacle"]["centroid_world_xy"])
    obs_r = float(sensed["perception_sensed_obstacle"]["radius_m"])
    resume_xy = tuple(ref_join["resume_xy_world"])
    hover_xy = load_hover_setpoint_world()

    rule("Step 1/4  Rebuild the execution prior map from the SDF")
    map_yaml, geom, _ = build_execution_prior(out, args.world_sdf)
    full_image, full_meta = G.load_full_pgm(map_yaml)
    print(f"  grid            : {geom.width_cells} x {geom.height_cells} @ {geom.resolution} m")
    print(f"  origin          : ({geom.origin_x:.3f}, {geom.origin_y:.3f})")

    rule("Step 2/4  Sensed obstacle and region of interest")
    print(f"  sensed centroid : ({obs_xy[0]:.4f}, {obs_xy[1]:.4f}) m   (perception output)")
    print(f"  sensed radius   : {obs_r:.4f} m")
    print(f"  hover setpoint  : ({hover_xy[0]:.4f}, {hover_xy[1]:.4f}) m "
          f"(from event_log.csv, NED -> world)")
    print(f"  resume anchor   : ({resume_xy[0]:.4f}, {resume_xy[1]:.4f}) m")

    roi = R.derive_roi_spec(float(full_meta["resolution"]))
    print(f"  R_min           : {roi.R_min_m:.4f} m      (v^2 / a_lat)")
    print(f"  d_stop          : {roi.d_stop_m:.4f} m      (v^2 / 2 a_stop)")
    print(f"  ROI_min         : {roi.ROI_min_m:.4f} m")
    print(f"  lateral half w  : {roi.lateral_half_width_m:.4f} m")
    print(f"  resume distance : {roi.resume_distance_m:.4f} m")

    seg_xs, seg_ys, seg_ss = load_segment_window()
    i_hover = nearest_index(seg_xs, seg_ys, hover_xy)
    i_resume = nearest_index(seg_xs, seg_ys, resume_xy)
    s_min = float(seg_ss[i_hover]) - roi.upstream_margin_m
    s_max = float(seg_ss[i_resume]) + roi.smoothing_margin_m
    print(f"  segment window  : {len(seg_xs)} samples, "
          f"s = [{seg_ss[0]:.2f}, {seg_ss[-1]:.2f}] m")
    print(f"  hover index     : {i_hover} (s = {seg_ss[i_hover]:.3f} m)")
    print(f"  resume index    : {i_resume} (s = {seg_ss[i_resume]:.3f} m)")
    print(f"  ROI arc window  : s = [{s_min:.3f}, {s_max:.3f}] m")

    rule("Step 3/4  Local ROI replan (crop, patch, A*, refine, validate)")
    t_replan = time.perf_counter()
    cand = R.run_local_roi_replan(
        full_image=full_image, full_meta=full_meta,
        seg_xs=seg_xs, seg_ys=seg_ys, seg_ss=seg_ss,
        obstacle_xy=obs_xy, obs_r=obs_r,
        s_min=s_min, s_max=s_max,
        lateral_half_width=roi.lateral_half_width_m,
        hover_xy=hover_xy, resume_xy=resume_xy,
        hover_idx=i_hover, resume_idx=i_resume,
    )
    offline_wall_s = time.perf_counter() - t_replan

    if not cand.success:
        print(f"  FAIL: {cand.fail_reason}", file=sys.stderr)
        return 1

    print(f"  classification  : {cand.classification} ({cand.selected_attempt})")
    print(f"  A* nodes        : {cand.astar_nodes_expanded}")
    print(f"  A* length       : {cand.astar_path_length_m:.4f} m")
    print(f"  refined length  : {cand.smoothed_length_m:.4f} m")
    print(f"  clearance min   : {cand.clearance_min_m:.4f} m "
          f"(r_eff {R.R_EFF_M}, r_pref {R.R_PREF_M})")
    print(f"  max curvature   : {cand.max_curvature_1_per_m:.4f} 1/m")
    print(f"  max lateral acc : {cand.max_lateral_accel_mps2:.4f} m/s^2")
    print(f"  collision-free  : {cand.collision_free}")
    print(f"  offline recompute wall clock: {offline_wall_s:.3f} s "
          f"(NOT the online SITL runtime)")

    # Determinism: the same input must give the same output.
    again = R.run_local_roi_replan(
        full_image=full_image, full_meta=full_meta,
        seg_xs=seg_xs, seg_ys=seg_ys, seg_ss=seg_ss,
        obstacle_xy=obs_xy, obs_r=obs_r,
        s_min=s_min, s_max=s_max,
        lateral_half_width=roi.lateral_half_width_m,
        hover_xy=hover_xy, resume_xy=resume_xy,
        hover_idx=i_hover, resume_idx=i_resume,
    )
    deterministic = bool(np.array_equal(again.smoothed_xy, cand.smoothed_xy)
                         and again.astar_nodes_expanded == cand.astar_nodes_expanded)
    print(f"  re-run identical: {deterministic}")

    rule("Step 4/4  Comparison with the published SRR reference")
    end_xy = (float(cand.smoothed_xy[-1, 0]), float(cand.smoothed_xy[-1, 1]))
    end_yaw = float(cand.smoothed_yaw[-1]) if len(cand.smoothed_yaw) else 0.0
    join = R.rejoin_metrics(
        replan_end_xy=end_xy, replan_end_yaw=end_yaw,
        resume_xy=resume_xy,
        resume_yaw=float(ref_join["resume_yaw_world_rad"]),
    )

    checks = [
        ("classification", cand.classification, ref_runtime["classification"], None),
        ("selected attempt", cand.selected_attempt, ref_runtime["selected_attempt"], None),
        ("A* nodes expanded", cand.astar_nodes_expanded,
         ref_runtime["astar_nodes_expanded"], TOL_EXACT),
        ("A* path length [m]", cand.astar_path_length_m,
         ref_runtime["astar_path_length_m"], TOL_GEOM_M),
        ("refined length [m]", cand.smoothed_length_m,
         ref_runtime["smoothed_length_m"], TOL_GEOM_M),
        ("clearance min [m]", cand.clearance_min_m,
         ref_runtime["clearance_min_m"], TOL_METRIC),
        ("max curvature [1/m]", cand.max_curvature_1_per_m,
         ref_runtime["max_curvature_1_per_m"], TOL_METRIC),
        ("max lateral acc [m/s2]", cand.max_lateral_accel_mps2,
         ref_runtime["max_lateral_accel_mps2"], TOL_METRIC),
        ("clearance >= r_eff", cand.clearance_min_ge_r_eff,
         ref_runtime["clearance_min_ge_r_eff"], None),
        ("clearance >= r_pref", cand.clearance_min_ge_r_pref,
         ref_runtime["clearance_min_ge_r_pref"], None),
        ("collision-free", cand.collision_free, ref_runtime["collision_free"], None),
    ]

    failures = []
    comparison = {}
    for name, mine, ref, tol in checks:
        if tol is None:
            ok = mine == ref
            delta = None
        else:
            delta = abs(float(mine) - float(ref))
            ok = delta <= tol
        comparison[name] = {
            "reproduced": mine if tol is None else float(mine),
            "published": ref if tol is None else float(ref),
            "abs_delta": delta, "match": bool(ok),
        }
        if not ok:
            failures.append(name)
        ds = "" if delta is None else f"   delta {delta:.3e}"
        print(f"  {'OK  ' if ok else 'FAIL'} {name:24s} {str(mine)[:22]:>22} vs "
              f"{str(ref)[:22]:>22}{ds}")

    # Geometry of the refined path against the published CSV.
    n_ref, n_mine = len(smoothed_ref), len(cand.smoothed_xy)
    if n_ref == n_mine:
        dmax = float(np.abs(cand.smoothed_xy - smoothed_ref).max())
        ok = dmax <= TOL_GEOM_M
        print(f"  {'OK  ' if ok else 'FAIL'} {'refined path samples':24s} "
              f"{n_mine} points, max |dxy| = {dmax:.3e} m")
        comparison["refined_path_max_abs_deviation_m"] = {
            "value": dmax, "n_samples": n_mine, "match": bool(ok)}
        if not ok:
            failures.append("refined path geometry")
    else:
        print(f"  NOTE  refined path sample count {n_mine} vs published {n_ref}; "
              f"comparing by nearest-point distance instead")
        from scipy.spatial import cKDTree
        d, _ = cKDTree(smoothed_ref).query(cand.smoothed_xy, k=1)
        dmax = float(d.max())
        ok = dmax <= 1e-3
        print(f"  {'OK  ' if ok else 'FAIL'} {'refined path deviation':24s} "
              f"max nearest-point distance = {dmax:.3e} m")
        comparison["refined_path_max_nearest_point_m"] = {
            "value": dmax, "n_samples_reproduced": n_mine,
            "n_samples_published": n_ref, "match": bool(ok)}
        if not ok:
            failures.append("refined path geometry")

    print(f"\n  rejoin continuity (recomputed): position jump "
          f"{join['pos_jump_replan_to_resume_m']:.4f} m, "
          f"yaw jump {join['yaw_jump_replan_to_resume_deg']:.2f} deg")
    print(f"  published (live run)          : position jump "
          f"{ref_join['pos_jump_replan_to_resume_m']:.4f} m, "
          f"yaw jump {ref_join['yaw_jump_replan_to_resume_deg']:.2f} deg")

    # ---------------------------------------------------------------- outputs
    plan_dir = out / "02_replan"
    plan_dir.mkdir(exist_ok=True)
    astar_csv = plan_dir / "local_replan_astar.csv"
    write_csv(astar_csv, ["idx", "x_world_m", "y_world_m"],
              [(i, format_float(x), format_float(y))
               for i, (x, y) in enumerate(cand.astar_xy)])
    sm_csv = plan_dir / "local_replan_smoothed.csv"
    write_csv(sm_csv,
              ["idx", "s_m", "x_world_m", "y_world_m", "yaw_world_rad",
               "curvature_1_per_m", "v_ref_mps", "a_lat_mps2"],
              [(i, format_float(cand.smoothed_s[i]),
                format_float(cand.smoothed_xy[i, 0]),
                format_float(cand.smoothed_xy[i, 1]),
                format_float(cand.smoothed_yaw[i]),
                format_float(cand.smoothed_curv[i]),
                format_float(cand.smoothed_v[i]),
                format_float(cand.smoothed_lat_accel[i]))
               for i in range(len(cand.smoothed_xy))])
    write_hashes(plan_dir / "hashes.txt", [astar_csv, sm_csv])

    write_yaml(out / "metrics.yaml", {
        "tool": "tools/reproduce_srr_geometry.py",
        "scope": "Offline geometric reproduction of the local Stop-Replan-Resume "
                 "detour. Geometry only.",
        "not_reproduced_here": [
            "detection latency", "stop-to-hover transition",
            "online replanning runtime", "tracking error", "mission outcome",
        ],
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {"python": platform.python_version(),
                        "numpy": np.__version__,
                        "platform": platform.platform()},
        "sensed_obstacle": {
            "source": "results/stop_replan_resume/sensed_obstacle_metrics.yaml "
                      "(perception output of the live run, not ground truth)",
            "centroid_world_xy_m": [float(obs_xy[0]), float(obs_xy[1])],
            "radius_m": obs_r,
        },
        "roi_spec": {
            "grid_resolution_m": roi.grid_resolution_m,
            "R_min_m": roi.R_min_m, "d_stop_m": roi.d_stop_m,
            "smoothing_margin_m": roi.smoothing_margin_m,
            "ROI_min_m": roi.ROI_min_m,
            "upstream_margin_m": roi.upstream_margin_m,
            "resume_distance_m": roi.resume_distance_m,
            "lateral_half_width_m": roi.lateral_half_width_m,
        },
        "reproduced_replan": {
            "classification": cand.classification,
            "selected_attempt": cand.selected_attempt,
            "astar_nodes_expanded": int(cand.astar_nodes_expanded),
            "astar_path_length_m": float(cand.astar_path_length_m),
            "smoothed_length_m": float(cand.smoothed_length_m),
            "clearance_min_m": float(cand.clearance_min_m),
            "clearance_mean_m": float(cand.clearance_mean_m),
            "max_curvature_1_per_m": float(cand.max_curvature_1_per_m),
            "max_lateral_accel_mps2": float(cand.max_lateral_accel_mps2),
            "collision_free": bool(cand.collision_free),
            "deterministic_on_rerun": deterministic,
        },
        "rejoin_continuity_recomputed": join,
        "offline_recompute_wall_clock_s": round(offline_wall_s, 4),
        "offline_recompute_note":
            "Wall-clock time of this offline recomputation on the current "
            "machine. This is NOT the online replanning runtime; that figure "
            "comes from the live SITL run and is recorded in "
            "results/stop_replan_resume/srr_metrics.yaml.",
        "online_runtime_from_live_run_s":
            float(ref_runtime["t_total_s"]),
        "comparison_with_published": comparison,
        "regression": bool(failures),
        "failures": failures,
    })

    # ------------------------------------------------------------- figure
    figure_written = False
    if not args.no_figure:
        figure_written = write_figure(
            out / "srr_geometry.png",
            map_yaml=map_yaml, obs_xy=obs_xy, obs_r=obs_r,
            gt=sensed.get("ground_truth_eval_only"),
            seg_xs=seg_xs, seg_ys=seg_ys,
            astar_xy=cand.astar_xy, smoothed_xy=cand.smoothed_xy,
            hover_xy=hover_xy, resume_xy=resume_xy,
            clearance=cand.clearance_min_m, s=cand.smoothed_s,
        )
        if figure_written:
            print(f"\n  wrote {out / 'srr_geometry.png'}")

    rule("Summary")
    total = round(time.perf_counter() - t0, 3)
    print(f"  wall clock      : {total} s")
    print(f"  outputs         : {out.resolve()}")
    if failures:
        print(f"  RESULT          : REGRESSION in {len(failures)} check(s): {failures}")
        return 1
    print("  RESULT          : PASS (geometry reproduces the published reference)")
    return 0


def write_figure(out_png: Path, *, map_yaml, obs_xy, obs_r, gt, seg_xs, seg_ys,
                 astar_xy, smoothed_xy, hover_xy, resume_xy, clearance, s) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from PIL import Image
    except Exception as exc:  # pragma: no cover - environment dependent
        print(f"  matplotlib unavailable ({exc}); skipping figure")
        return False

    meta = yaml.safe_load(Path(map_yaml).read_text())
    img = np.array(Image.open(Path(map_yaml).parent / meta["image"]))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

    pad = 14.0
    xs = np.concatenate([astar_xy[:, 0], smoothed_xy[:, 0], [obs_xy[0]]])
    ys = np.concatenate([astar_xy[:, 1], smoothed_xy[:, 1], [obs_xy[1]]])
    xlim = (xs.min() - pad, xs.max() + pad)
    ylim = (ys.min() - pad, ys.max() + pad)

    fig, ax = plt.subplots(figsize=(9, 8))
    ax.imshow(img, cmap="gray", origin="upper",
              extent=[ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res],
              interpolation="nearest")
    ax.plot(seg_xs, seg_ys, lw=1.0, ls="--", color="0.45",
            label="original mission reference")
    ax.plot(astar_xy[:, 0], astar_xy[:, 1], lw=1.1, color="tab:blue",
            label="local A* detour")
    ax.plot(smoothed_xy[:, 0], smoothed_xy[:, 1], lw=1.8, color="tab:red",
            label=f"refined detour (clearance {clearance:.3f} m)")
    ax.add_patch(plt.Circle(obs_xy, obs_r, fc="tab:orange", ec="darkorange",
                            alpha=0.55, label="sensed obstacle"))
    if gt:
        ax.add_patch(plt.Circle(tuple(gt["centroid_world_xy"]), float(gt["radius_m"]),
                                fc="none", ec="k", ls=":", lw=1.4,
                                label="ground truth (evaluation only)"))
    ax.plot(*hover_xy, "o", ms=10, color="tab:cyan", mec="k", label="hover / stop")
    ax.plot(*resume_xy, "*", ms=16, color="gold", mec="k", label="resume anchor")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (world ENU)")
    ax.set_ylabel("y [m] (world ENU)")
    ax.set_title("Stop-Replan-Resume: local detour geometry (offline reproduction)")
    ax.legend(loc="best", fontsize=8, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130, metadata={"Software": None})
    plt.close(fig)
    return True


if __name__ == "__main__":
    sys.exit(main())
