#!/usr/bin/env python3
"""Reduce an A* path by line-of-sight string pulling.

Thin CLI over :mod:`uav_autonomy.smoothing`. Collapses redundant interior
waypoints into the fewest waypoints whose piecewise-linear path still respects
every planner invariant, then re-checks all four invariants and fails with
exit code 2 if any is violated.

Output is still a polyline. Curvature-continuous refinement is a separate
stage; see ``tools/refine_trajectory.py``.
"""

from __future__ import annotations

import argparse
import csv
import math
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_autonomy import grid as G  # noqa: E402
from uav_autonomy import smoothing as S  # noqa: E402
from uav_autonomy.artifacts import (  # noqa: E402
    format_float, sha256_of_file, write_csv, write_hashes, write_yaml,
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--path-csv", type=Path, required=True)
    ap.add_argument("--map-yaml", type=Path, required=True)
    ap.add_argument("--robot", type=Path, default=REPO_ROOT / "sim/config/robot_map2_safe.yaml")
    ap.add_argument("--inflation-radius-m", type=float, default=None,
                    help="Override the inflation radius. Default: robot_radius "
                         "from --robot. Must match the radius the path was planned with.")
    ap.add_argument("--unknown-is-blocked", action=argparse.BooleanOptionalAction,
                    default=False)
    ap.add_argument("--output-dir", type=Path, required=True)
    args = ap.parse_args()

    meta = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8"))
    robot = yaml.safe_load(args.robot.read_text(encoding="utf-8"))
    radius = args.inflation_radius_m
    if radius is None:
        radius = float(robot["robot_radius"])

    image_path = args.map_yaml.parent / meta["image"]
    image = np.array(Image.open(image_path).convert("L"))
    res = float(meta["resolution"])
    origin = list(meta["origin"])
    grid = G.classify_map(
        image, int(meta.get("negate", 0)),
        float(meta.get("occupied_thresh", 0.65)),
        float(meta.get("free_thresh", 0.196)),
    )
    height = grid.shape[0]
    clear_grid = G.inflated_clear_grid(
        grid, int(math.ceil(radius / res)), args.unknown_is_blocked,
    )

    raw = []
    with args.path_csv.open() as f:
        for row in csv.DictReader(f):
            raw.append((int(row["row"]), int(row["col"])))
    if not raw:
        print(f"ERROR: {args.path_csv} is empty", file=sys.stderr)
        return 2

    reduced = S.string_pull(raw, clear_grid)
    dist_m = distance_transform_edt(~(grid == G.OCCUPIED)) * res
    errors = S.validate_invariants(raw, reduced, clear_grid, dist_m, res, radius)

    raw_len = S.polyline_length_metres(raw, res)
    red_len = S.polyline_length_metres(reduced, res)
    raw_clr_min, raw_clr_mean = S.polyline_clearance_along_cells(raw, dist_m)
    red_clr_min, red_clr_mean = S.polyline_clearance_along_cells(reduced, dist_m)

    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    reduced_csv = out / "path_reduced.csv"
    write_csv(
        reduced_csv, ["idx", "x_m", "y_m", "row", "col"],
        [
            (i, format_float(x), format_float(y), c[0], c[1])
            for i, (c, (x, y)) in enumerate(
                (c, G.cell_to_world(c, origin, res, height)) for c in reduced
            )
        ],
    )

    write_yaml(out / "metrics.yaml", {
        "success": not errors,
        "waypoints_before": len(raw),
        "waypoints_after": len(reduced),
        "reduction_factor": float(len(raw) / len(reduced)) if reduced else None,
        "length_before_m": float(raw_len),
        "length_after_m": float(red_len),
        "length_reduction_m": float(raw_len - red_len),
        "clearance_min_before_m": float(raw_clr_min),
        "clearance_min_after_m": float(red_clr_min),
        "clearance_mean_before_m": float(raw_clr_mean),
        "clearance_mean_after_m": float(red_clr_mean),
        "inflation_radius_m": float(radius),
        "invariant_errors": errors,
    })

    write_yaml(out / "manifest.yaml", {
        "tool": "tools/reduce_path_los.py",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "python_version": platform.python_version(),
        "inputs": {
            "path_csv_sha256": sha256_of_file(args.path_csv),
            "map_yaml_sha256": sha256_of_file(args.map_yaml),
            "robot_yaml_sha256": sha256_of_file(args.robot),
        },
        "invariants_checked": [
            "endpoint preservation",
            "full-segment line of sight on the inflated grid",
            "length monotonicity",
            "clearance floor at the inflation radius",
        ],
    })
    write_hashes(out / "hashes.txt", [reduced_csv])

    print(f"[reduce_path_los] waypoints    = {len(raw)} -> {len(reduced)}")
    print(f"[reduce_path_los] length_m     = {raw_len:.4f} -> {red_len:.4f}")
    print(f"[reduce_path_los] clearance_min= {raw_clr_min:.4f} -> {red_clr_min:.4f} m")
    if errors:
        print("[reduce_path_los] INVARIANT VIOLATIONS:", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        return 2
    print("[reduce_path_los] all invariants hold")
    return 0


if __name__ == "__main__":
    sys.exit(main())
