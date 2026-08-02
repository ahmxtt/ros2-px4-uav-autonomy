#!/usr/bin/env python3
"""Rasterise a Gazebo SDF world into a 2D occupancy grid.

Thin CLI over :mod:`uav_autonomy.mapping`. Writes ``map.pgm``, ``map.yaml``,
``metrics.yaml``, ``manifest.yaml`` and ``hashes.txt``.

The rasterisation is deterministic, so re-running on the same SDF reproduces a
byte-identical PGM. Verify with::

    cd <output-dir> && sha256sum -c hashes.txt
"""

from __future__ import annotations

import argparse
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_autonomy import mapping as M  # noqa: E402
from uav_autonomy.artifacts import sha256_of_file, write_hashes, write_yaml  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world-sdf", type=Path,
                    default=REPO_ROOT / "sim/worlds/forest_map2.sdf",
                    help="SDF world to rasterise.")
    ap.add_argument("--resolution", type=float, default=M.RESOLUTION_M,
                    help="Grid resolution in metres per cell.")
    ap.add_argument("--padding", type=float, default=M.PADDING_M,
                    help="Buffer around the obstacle bounding box, in metres.")
    ap.add_argument("--output-dir", type=Path, required=True,
                    help="Directory for map.pgm / map.yaml / metrics / manifest / hashes.")
    args = ap.parse_args()

    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    obstacles = M.parse_obstacles(args.world_sdf)
    grid = M.compute_grid(obstacles, args.resolution, args.padding)
    image = M.rasterize(obstacles, grid)

    pgm = out / "map.pgm"
    map_yaml = out / "map.yaml"
    M.write_pgm(image, pgm)
    M.write_map_yaml(map_yaml, grid, pgm.name)

    n_occ = int((image == M.PGM_OCCUPIED).sum())
    by_type: dict = {}
    for o in obstacles:
        by_type[o.model_type] = by_type.get(o.model_type, 0) + 1

    write_yaml(out / "metrics.yaml", {
        "n_obstacles": len(obstacles),
        "obstacles_by_type": by_type,
        "resolution_m": float(grid.resolution),
        "width_cells": int(grid.width_cells),
        "height_cells": int(grid.height_cells),
        "origin_xy_m": [float(grid.origin_x), float(grid.origin_y)],
        "extent_m": [
            float(grid.width_cells * grid.resolution),
            float(grid.height_cells * grid.resolution),
        ],
        "occupied_cells": n_occ,
        "occupied_ratio": float(n_occ / image.size),
    })

    write_yaml(out / "manifest.yaml", {
        "tool": "tools/build_prior_map.py",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "python_version": platform.python_version(),
        "inputs": {
            "world_sdf": str(args.world_sdf.relative_to(REPO_ROOT))
            if args.world_sdf.is_relative_to(REPO_ROOT) else str(args.world_sdf),
            "world_sdf_sha256": sha256_of_file(args.world_sdf),
        },
        "parameters": {
            "resolution_m": float(args.resolution),
            "padding_m": float(args.padding),
            "tree_footprints_m": dict(M.TREE_FOOTPRINTS_M),
        },
        "determinism": "no RNG; identical SDF input yields a byte-identical PGM",
    })

    # manifest.yaml carries a timestamp and is deliberately excluded.
    write_hashes(out / "hashes.txt", [pgm, map_yaml, out / "metrics.yaml"])

    print(f"[build_prior_map] obstacles      = {len(obstacles)} {by_type}")
    print(f"[build_prior_map] grid           = {grid.width_cells} x {grid.height_cells} "
          f"@ {grid.resolution} m")
    print(f"[build_prior_map] origin         = ({grid.origin_x:.3f}, {grid.origin_y:.3f})")
    print(f"[build_prior_map] occupied cells = {n_occ}")
    print(f"[build_prior_map] output         = {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
