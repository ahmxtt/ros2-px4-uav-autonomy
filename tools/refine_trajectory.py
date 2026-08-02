#!/usr/bin/env python3
"""Refine an A* path into a cubic B-spline reference with a speed profile.

Thin CLI over :mod:`uav_autonomy.trajectory`. Runs the clearance-validated
cascade, applies the curvature-bounded speed profile to the selected
candidate, and checks the realised motion limits.

Writes ``trajectory.csv``, ``control_points.csv``, ``metrics.yaml``,
``attempt_comparison.yaml``, ``manifest.yaml`` and ``hashes.txt``.
"""

from __future__ import annotations

import argparse
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_autonomy import grid as G  # noqa: E402
from uav_autonomy import trajectory as T  # noqa: E402
from uav_autonomy.artifacts import (  # noqa: E402
    format_float, sha256_of_file, write_csv, write_hashes, write_yaml,
)


def load_path_csv(path_csv: Path) -> np.ndarray:
    import csv
    xs, ys = [], []
    with Path(path_csv).open() as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x_m"]))
            ys.append(float(row["y_m"]))
    if len(xs) < 4:
        raise RuntimeError(f"{path_csv}: too few points ({len(xs)}) for a cubic spline")
    return np.column_stack([np.asarray(xs), np.asarray(ys)])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--path-csv", type=Path, required=True)
    ap.add_argument("--map-yaml", type=Path, required=True)
    ap.add_argument("--robot", type=Path, default=REPO_ROOT / "sim/config/robot_map2_safe.yaml")
    ap.add_argument("--policy", type=Path, default=REPO_ROOT / "sim/config/mission_policy.yaml")
    ap.add_argument("--v-nominal-mps", type=float, default=1.5)
    ap.add_argument("--v-min-mps", type=float, default=0.3)
    ap.add_argument("--a-lat-max-mps2", type=float, default=1.0)
    ap.add_argument("--a-long-max-mps2", type=float, default=1.0)
    ap.add_argument("--eval-step-m", type=float, default=0.02)
    ap.add_argument("--control-step-attempt1-m", type=float, default=2.0)
    ap.add_argument("--control-step-attempt2-m", type=float, default=1.0)
    ap.add_argument("--fallback-step-m", type=float, default=0.5)
    ap.add_argument("--output-dir", type=Path, required=True)
    args = ap.parse_args()

    robot = yaml.safe_load(args.robot.read_text(encoding="utf-8"))
    policy = yaml.safe_load(args.policy.read_text(encoding="utf-8"))["planning_policy"]
    r_eff = float(robot["robot_radius"])
    r_pref = r_eff + float(policy["tracking_margin_m"])

    astar_xy = load_path_csv(args.path_csv)
    map_data = G.load_map(args.map_yaml)
    clearance = G.clearance_field(map_data)
    sp = T.SpeedParams(
        v_nominal=args.v_nominal_mps, v_min=args.v_min_mps,
        a_lat_max=args.a_lat_max_mps2, a_long_max=args.a_long_max_mps2,
    )

    sel = T.refine_trajectory(
        astar_xy=astar_xy, map_data=map_data, clearance_m_field=clearance,
        r_eff=r_eff, r_pref=r_pref, sp=sp,
        eval_step_m=args.eval_step_m,
        control_step_attempt1_m=args.control_step_attempt1_m,
        control_step_attempt2_m=args.control_step_attempt2_m,
        fallback_step_m=args.fallback_step_m,
    )
    a = sel.selected
    if a.classification == "fail":
        print(f"FAIL: no candidate reached r_eff={r_eff} m "
              f"(best clearance_min={a.clearance_min_m:.4f} m)", file=sys.stderr)
        return 1

    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    seg_ds = np.diff(a.s)
    limits = T.check_motion_limits(
        v_ref=sel.v_ref, lateral_accel=sel.lateral_accel, segment_ds=seg_ds, sp=sp,
    )

    traj_csv = out / "trajectory.csv"
    write_csv(
        traj_csv,
        ["idx", "s_m", "x_m", "y_m", "yaw_rad", "curvature_1_per_m",
         "v_ref_mps", "a_lat_mps2", "clearance_m"],
        [
            (i, format_float(a.s[i]), format_float(a.samples_xy[i, 0]),
             format_float(a.samples_xy[i, 1]), format_float(a.yaw[i]),
             format_float(a.curvature[i]), format_float(sel.v_ref[i]),
             format_float(sel.lateral_accel[i]), format_float(a.clearance[i]))
            for i in range(a.n_samples)
        ],
    )
    cp_csv = out / "control_points.csv"
    write_csv(
        cp_csv, ["idx", "x_m", "y_m"],
        [(i, format_float(x), format_float(y)) for i, (x, y) in enumerate(a.control_pts)],
    )

    write_yaml(out / "metrics.yaml", {
        "success": True,
        "selected_attempt": a.name,
        "selected_method": a.method,
        "selected_control_step_m": float(a.control_step_m),
        "classification": a.classification,
        "n_control_points": int(a.n_control_points),
        "n_samples": int(a.n_samples),
        "trajectory_length_m": float(a.trajectory_length_m),
        "clearance_min_m": float(a.clearance_min_m),
        "clearance_mean_m": float(a.clearance_mean_m),
        "clearance_min_ge_r_eff": bool(a.clearance_min_m >= r_eff - 1e-9),
        "clearance_min_ge_r_pref": bool(a.clearance_min_m >= r_pref - 1e-9),
        "collision_free": bool(a.collision_free),
        "in_bounds_all": bool(a.in_bounds_all),
        "max_deviation_from_astar_m": float(a.max_deviation_from_astar_m),
        "mean_deviation_from_astar_m": float(a.mean_deviation_from_astar_m),
        "curvature_max_1_per_m": float(a.curvature_max_1_per_m),
        "estimated_travel_time_s": float(sel.estimated_travel_time_s),
        "motion_limits": limits,
        "r_eff_m": r_eff,
        "r_pref_m": r_pref,
    })

    write_yaml(out / "attempt_comparison.yaml", {
        "attempts": [
            {
                "name": at.name,
                "method": at.method,
                "control_step_m": float(at.control_step_m),
                "n_control_points": int(at.n_control_points),
                "n_samples": int(at.n_samples),
                "classification": at.classification,
                "trajectory_length_m": float(at.trajectory_length_m),
                "clearance_min_m": float(at.clearance_min_m),
                "clearance_mean_m": float(at.clearance_mean_m),
                "max_deviation_from_astar_m": float(at.max_deviation_from_astar_m),
                "curvature_max_1_per_m": float(at.curvature_max_1_per_m),
                "collision_free": bool(at.collision_free),
                "notes": list(at.notes),
            }
            for at in sel.attempts
        ],
        "selected_name": a.name,
        "selected_classification": a.classification,
    })

    write_yaml(out / "manifest.yaml", {
        "tool": "tools/refine_trajectory.py",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "python_version": platform.python_version(),
        "numpy_version": np.__version__,
        "inputs": {
            "path_csv_sha256": sha256_of_file(args.path_csv),
            "map_yaml_sha256": sha256_of_file(args.map_yaml),
            "robot_yaml_sha256": sha256_of_file(args.robot),
            "policy_yaml_sha256": sha256_of_file(args.policy),
        },
        "parameters": {
            "eval_step_m": args.eval_step_m,
            "control_step_attempt1_m": args.control_step_attempt1_m,
            "control_step_attempt2_m": args.control_step_attempt2_m,
            "fallback_step_m": args.fallback_step_m,
            "v_nominal_mps": args.v_nominal_mps,
            "v_min_mps": args.v_min_mps,
            "a_lat_max_mps2": args.a_lat_max_mps2,
            "a_long_max_mps2": args.a_long_max_mps2,
        },
    })

    write_hashes(out / "hashes.txt", [traj_csv, cp_csv])

    print(f"[refine_trajectory] selected       = {a.name} ({a.classification})")
    print(f"[refine_trajectory] length_m       = {a.trajectory_length_m:.6f}")
    print(f"[refine_trajectory] clearance_min  = {a.clearance_min_m:.4f} m "
          f"(r_eff {r_eff}, r_pref {r_pref})")
    print(f"[refine_trajectory] max deviation  = {a.max_deviation_from_astar_m:.4f} m")
    print(f"[refine_trajectory] motion limits  = {limits}")
    print(f"[refine_trajectory] output         = {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
