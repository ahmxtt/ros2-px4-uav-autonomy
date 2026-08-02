#!/usr/bin/env python3
"""Plan a single leg with the deterministic A* planner.

Thin CLI over :mod:`uav_autonomy.planning`. Three candidate types are exposed,
matching the mission policy:

``--candidate robust`` (default)
    Hard inflation at ``r_plan_min = r_pref + spline_shrinkage_budget``, plus a
    soft comfort cost inside ``[r_plan_min, r_comfort)``. This reserves the
    clearance the downstream B-spline is allowed to consume.

``--candidate boundary``
    Hard inflation at ``r_eff``, plus a soft clearance cost inside
    ``[r_eff, r_pref)``. Used where the robust candidate has no solution.

``--candidate geometric``
    Hard inflation at ``r_eff``, pure geometric cost. Reference behaviour with
    no soft shaping.

Writes ``path.csv``, ``metrics.yaml``, ``manifest.yaml`` and ``hashes.txt``.
"""

from __future__ import annotations

import argparse
import math
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_autonomy import grid as G  # noqa: E402
from uav_autonomy import planning as P  # noqa: E402
from uav_autonomy.artifacts import (  # noqa: E402
    format_float, sha256_of_file, write_csv, write_hashes, write_yaml,
)


def load_policy(path: Path) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))["planning_policy"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--map-yaml", type=Path, required=True)
    ap.add_argument("--robot", type=Path, default=REPO_ROOT / "sim/config/robot_map2_safe.yaml")
    ap.add_argument("--policy", type=Path, default=REPO_ROOT / "sim/config/mission_policy.yaml")
    ap.add_argument("--scenario", type=Path, default=REPO_ROOT / "sim/scenarios/forest_map2.yaml")
    ap.add_argument("--candidate", choices=("robust", "boundary", "geometric"),
                    default="robust")
    ap.add_argument("--start-x", type=float, default=None)
    ap.add_argument("--start-y", type=float, default=None)
    ap.add_argument("--goal-x", type=float, default=None)
    ap.add_argument("--goal-y", type=float, default=None)
    ap.add_argument("--goal-radius", type=float, default=None)
    ap.add_argument("--unknown-is-blocked", action=argparse.BooleanOptionalAction,
                    default=False,
                    help="Treat unknown cells as obstacles (default: relaxed).")
    ap.add_argument("--output-dir", type=Path, required=True)
    args = ap.parse_args()

    meta = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8"))
    robot = yaml.safe_load(args.robot.read_text(encoding="utf-8"))
    scenario = yaml.safe_load(args.scenario.read_text(encoding="utf-8"))
    policy = load_policy(args.policy)

    r_eff = float(robot["robot_radius"])
    r_pref = r_eff + float(policy["tracking_margin_m"])
    r_plan_min = r_pref + float(policy["spline_shrinkage_budget_m"])
    r_comfort = r_pref + float(policy["comfort_margin_m"])
    w_clearance = float(policy["w_clearance"])
    w_comfort = float(policy["w_comfort"])

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

    start_cfg = scenario.get("planner_start_world_frame") or scenario["start_pose_world_frame"]
    goal_cfg = scenario["goal_region_world_frame"]
    start_xy = (
        args.start_x if args.start_x is not None else float(start_cfg["x"]),
        args.start_y if args.start_y is not None else float(start_cfg["y"]),
    )
    goal_xy = (
        args.goal_x if args.goal_x is not None else float(goal_cfg["center"]["x"]),
        args.goal_y if args.goal_y is not None else float(goal_cfg["center"]["y"]),
    )
    goal_radius = args.goal_radius if args.goal_radius is not None else float(goal_cfg["radius"])

    hard_radius = r_plan_min if args.candidate == "robust" else r_eff
    inflation_cells = int(math.ceil(hard_radius / res))
    clear_grid = G.inflated_clear_grid(grid, inflation_cells, args.unknown_is_blocked)

    start_cell = G.world_to_cell(start_xy, origin, res, height)
    if not G.in_bounds(start_cell, grid.shape):
        print(f"ERROR: start {start_xy} is outside the map", file=sys.stderr)
        return 2
    if not clear_grid[start_cell]:
        print(f"ERROR: start cell {start_cell} is not inflation-safe at "
              f"r={hard_radius} m", file=sys.stderr)
        return 2

    goal_cells, goal_centre, goal_radius_cells = G.collect_goal_cells(
        goal_xy, goal_radius, origin, res, grid.shape, clear_grid,
    )
    if not goal_cells:
        print(f"ERROR: goal region at {goal_xy} r={goal_radius} m has no "
              f"inflation-safe cells", file=sys.stderr)
        return 2

    clearance_field = G.compute_clearance_metres(grid, res)
    common = dict(
        start=start_cell, goal_cells_set=frozenset(goal_cells),
        goal_centre=goal_centre, goal_radius_cells=goal_radius_cells,
    )

    if args.candidate == "robust":
        result = P.a_star_comfort(
            clear_grid=clear_grid, clearance_m_field=clearance_field,
            r_plan_min=r_plan_min, r_comfort=r_comfort, w_comfort=w_comfort,
            **common,
        )
    elif args.candidate == "boundary":
        result = P.a_star_clearance(
            clear_grid=clear_grid, clearance_m_field=clearance_field,
            r_eff=r_eff, r_pref=r_pref, w_clearance=w_clearance,
            **common,
        )
    else:
        result = P.a_star(clear_grid, start_cell, frozenset(goal_cells),
                          goal_centre, goal_radius_cells)

    if not result.success:
        print(f"FAIL: no path found. nodes_expanded={result.nodes_expanded}",
              file=sys.stderr)
        return 1

    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    length_m = P.path_length_metres(result.path_cells, res)
    clr_min, clr_mean = P.path_clearances(result.path_cells, grid, res)
    straight = math.dist(start_xy, goal_xy)
    xy = P.path_cells_to_world(result.path_cells, origin, res, height)

    path_csv = out / "path.csv"
    write_csv(
        path_csv, ["idx", "x_m", "y_m", "row", "col"],
        [(i, format_float(x), format_float(y), c[0], c[1])
         for i, ((x, y), c) in enumerate(zip(xy, result.path_cells))],
    )

    metrics = {
        "success": True,
        "candidate": args.candidate,
        "path_length_m": float(length_m),
        "straight_line_distance_m": float(straight),
        "alpha_actual": float(length_m / straight) if straight > 0 else None,
        "n_path_points": int(len(result.path_cells)),
        "nodes_expanded": int(result.nodes_expanded),
        "planning_time_ms": float(result.planning_time_ms),
        "path_clearance_min_m_to_raw_obstacles": float(clr_min),
        "path_clearance_mean_m_to_raw_obstacles": float(clr_mean),
        "hard_inflation_radius_m": float(hard_radius),
        "r_eff_m": r_eff,
        "r_pref_m": r_pref,
        "r_plan_min_m": r_plan_min,
        "r_comfort_m": r_comfort,
        "connectivity": 8,
        "heuristic": "octile",
        "corner_cutting": False,
        "unknown_policy": "strict" if args.unknown_is_blocked else "relaxed",
    }
    if hasattr(result, "cost"):
        metrics["cost_breakdown"] = {
            "length_cost": float(result.cost.length_cost),
            "comfort_penalty_sum": float(result.cost.comfort_penalty_sum),
            "total_cost": float(result.cost.total_cost),
        }
    write_yaml(out / "metrics.yaml", metrics)

    write_yaml(out / "manifest.yaml", {
        "tool": "tools/plan_astar.py",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "python_version": platform.python_version(),
        "inputs": {
            "map_yaml_sha256": sha256_of_file(args.map_yaml),
            "map_pgm_sha256": sha256_of_file(image_path),
            "robot_yaml_sha256": sha256_of_file(args.robot),
            "policy_yaml_sha256": sha256_of_file(args.policy),
            "scenario_yaml_sha256": sha256_of_file(args.scenario),
        },
        "query": {
            "start_xy_world": [float(start_xy[0]), float(start_xy[1])],
            "goal_xy_world": [float(goal_xy[0]), float(goal_xy[1])],
            "goal_radius_m": float(goal_radius),
        },
        "determinism": "deterministic tie-break on (f, g, row, col); no RNG",
    })

    # metrics.yaml holds a wall-clock timing and is excluded from hashes.txt.
    write_hashes(out / "hashes.txt", [path_csv])

    print(f"[plan_astar] candidate      = {args.candidate}")
    print(f"[plan_astar] path_length_m  = {length_m:.6f}")
    print(f"[plan_astar] n_points       = {len(result.path_cells)}")
    print(f"[plan_astar] nodes_expanded = {result.nodes_expanded}")
    print(f"[plan_astar] clearance_min  = {clr_min:.4f} m  (mean {clr_mean:.4f} m)")
    print(f"[plan_astar] output         = {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
