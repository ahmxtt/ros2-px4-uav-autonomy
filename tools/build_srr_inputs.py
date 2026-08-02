#!/usr/bin/env python3
"""Build the runtime inputs for the Stop-Replan-Resume node, from public files.

*Stop-Replan-Resume* (SRR) is this project's name for the obstacle-response
sequence: stop, plan a local detour, resume the original mission.

The SRR node takes five inputs. Two are shipped in the repository and two are
maps that have to be rasterised; this tool assembles all of them into one
directory so the documented run command has everything it needs.

============================  ===============================================
``prior/map.yaml`` + ``.pgm``  Execution prior, rasterised from the world SDF
                               at canopy radii. The map the replanner searches.
                               Does NOT contain the SRR obstacle.
``patched/map.yaml`` + ``.pgm`` The prior with the ground-truth obstacle
                               stamped in. Read only by ``--oracle-mode replan``.
``obstacle.yaml``              Copied from ``sim/obstacles/obstacle.yaml``.
``mission_reference.csv``      Copied from
                               ``results/mission_reference/mission_reference.csv``
                               - the complete five-segment mission.
``setpoints_ned.csv``          Built by ``tools/build_ned_setpoints.py``.
============================  ===============================================

The two shipped files are copied rather than referenced so the input set is one
self-contained directory with its own hashes, and so a run can be reconstructed
from exactly what it consumed.

The generated stream starts at the vehicle spawn point and covers all five
segments, so these inputs drive a complete armed mission - including the
obstacle encounter on segment 1.

This tool touches no simulator. Putting the physical obstacle into Gazebo is a
separate step: ``tools/spawn_srr_obstacle.py``.

Usage::

    python3 tools/build_srr_inputs.py --output-dir /tmp/srr_inputs
"""

from __future__ import annotations

import argparse
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_autonomy import grid as G  # noqa: E402
from uav_autonomy import mapping as M  # noqa: E402
from uav_autonomy import replanning as R  # noqa: E402
from uav_autonomy.artifacts import sha256_of_file, write_hashes, write_yaml  # noqa: E402

MISSION_CSV = REPO_ROOT / "results" / "mission_reference" / "mission_reference.csv"
OBSTACLE_YAML = REPO_ROOT / "sim" / "obstacles" / "obstacle.yaml"

MISSION_SEGMENTS = ("start_to_poi1", "orbit_poi1", "poi1_to_poi2",
                    "orbit_poi2", "poi2_to_start")


def build_maps(out: Path, world_sdf: Path, obstacle: dict):
    """Execution prior, and the same prior with the obstacle stamped in."""
    image, geom, _ = M.execution_prior(world_sdf)

    prior = out / "prior"
    prior.mkdir(parents=True, exist_ok=True)
    M.write_pgm(image, prior / "map.pgm")
    M.write_map_yaml(prior / "map.yaml", geom, "map.pgm")
    write_hashes(prior / "hashes.txt", [prior / "map.pgm", prior / "map.yaml"])

    ox = float(obstacle["position_world_xy_m"][0])
    oy = float(obstacle["position_world_xy_m"][1])
    full, meta = G.load_full_pgm(prior / "map.yaml")
    crop = R.crop_world_bounds(
        full, meta,
        (geom.origin_x, geom.origin_y,
         geom.origin_x + geom.width_cells * geom.resolution,
         geom.origin_y + geom.height_cells * geom.resolution),
    )
    flipped = R.stamp_obstacle_into_crop(crop, ox, oy, float(obstacle["radius_m"]))

    patched = out / "patched"
    patched.mkdir(parents=True, exist_ok=True)
    M.write_pgm(crop.image, patched / "map.pgm")
    M.write_map_yaml(patched / "map.yaml", geom, "map.pgm")
    write_hashes(patched / "hashes.txt", [patched / "map.pgm", patched / "map.yaml"])
    return geom, flipped


def build_setpoints(out: Path, mission_csv: Path, max_duration_s) -> Path:
    """Delegate, so there is exactly one implementation of the transform."""
    path = out / "setpoints_ned.csv"
    cmd = [sys.executable, str(REPO_ROOT / "tools" / "build_ned_setpoints.py"),
           "--trajectory-csv", str(mission_csv), "--output-csv", str(path),
           "--require-segments", ",".join(MISSION_SEGMENTS)]
    if max_duration_s is not None:
        cmd += ["--max-duration-s", str(max_duration_s)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        sys.stdout.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        raise SystemExit(f"build_ned_setpoints.py failed (exit {proc.returncode})")
    return path


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--world-sdf", type=Path,
                    default=REPO_ROOT / "sim/worlds/forest_map2.sdf")
    ap.add_argument("--mission-csv", type=Path, default=MISSION_CSV)
    ap.add_argument("--obstacle-yaml", type=Path, default=OBSTACLE_YAML)
    ap.add_argument("--max-setpoint-duration-s", type=float, default=None,
                    help="Truncate the setpoint stream; for short SITL tests. "
                         "A truncated stream still starts at the spawn point, "
                         "but note that the obstacle is met at about 30 s.")
    args = ap.parse_args()

    for p in (args.world_sdf, args.mission_csv, args.obstacle_yaml):
        if not p.is_file():
            print(f"FAIL: missing input {p}", file=sys.stderr)
            return 2

    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    print(f"repository : {REPO_ROOT}")
    print(f"output     : {out.resolve()}")

    obstacle = yaml.safe_load(args.obstacle_yaml.read_text(encoding="utf-8"))
    if obstacle.get("verdict") != "PASS":
        print(f"FAIL: {args.obstacle_yaml} verdict is "
              f"{obstacle.get('verdict')!r}, not 'PASS'", file=sys.stderr)
        return 2

    obstacle_out = out / "obstacle.yaml"
    shutil.copyfile(args.obstacle_yaml, obstacle_out)
    mission_out = out / "mission_reference.csv"
    shutil.copyfile(args.mission_csv, mission_out)

    geom, flipped = build_maps(out, args.world_sdf, obstacle)
    print(f"  prior map      : {geom.width_cells} x {geom.height_cells} @ "
          f"{geom.resolution} m  -> {out / 'prior' / 'map.yaml'}")
    print(f"  patched map    : obstacle stamped, {flipped} cells flipped  -> "
          f"{out / 'patched' / 'map.yaml'}")
    print(f"  obstacle       : ({obstacle['position_world_xy_m'][0]:.3f}, "
          f"{obstacle['position_world_xy_m'][1]:.3f}) "
          f"r={obstacle['radius_m']} m  -> {obstacle_out}")

    with mission_out.open() as f:
        n_mission = sum(1 for _ in f) - 1
    print(f"  mission ref    : {n_mission} samples, "
          f"{len(MISSION_SEGMENTS)} segments  -> {mission_out}")

    setpoints = build_setpoints(out, mission_out, args.max_setpoint_duration_s)
    with setpoints.open() as f:
        n_sp = sum(1 for _ in f) - 1
    print(f"  setpoints (NED): {n_sp} samples  -> {setpoints}")

    write_yaml(out / "manifest.yaml", {
        "tool": "tools/build_srr_inputs.py",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "python_version": platform.python_version(),
        "purpose": "Runtime inputs for the Stop-Replan-Resume node, assembled "
                   "from public files only.",
        "inputs_used": {
            "world_sdf": "sim/worlds/forest_map2.sdf",
            "world_sdf_sha256": sha256_of_file(args.world_sdf),
            "mission_reference": "results/mission_reference/mission_reference.csv",
            "mission_reference_sha256": sha256_of_file(args.mission_csv),
            "obstacle": "sim/obstacles/obstacle.yaml",
            "obstacle_sha256": sha256_of_file(args.obstacle_yaml),
        },
        "coverage": {
            "mission_samples": n_mission,
            "setpoint_samples": n_sp,
            "segments": list(MISSION_SEGMENTS),
            "truncated": args.max_setpoint_duration_s is not None,
            "starts_at_spawn_point": True,
        },
        "not_included": "The physical Gazebo obstacle. Spawn it separately with "
                        "tools/spawn_srr_obstacle.py before arming.",
        "outputs": {
            "prior_map_yaml": "prior/map.yaml",
            "patched_map_yaml": "patched/map.yaml",
            "obstacle_yaml": "obstacle.yaml",
            "mission_csv": "mission_reference.csv",
            "setpoints_csv": "setpoints_ned.csv",
        },
    })

    print("\nNext: spawn the physical obstacle, then run the node.\n")
    print("  python3 tools/spawn_srr_obstacle.py --world forest_map2\n")
    print(f"  python3 ros2/nodes/stop_replan_resume_node.py \\\n"
          f"      --setpoints-csv    {setpoints} \\\n"
          f"      --mission-csv      {mission_out} \\\n"
          f"      --prior-map-yaml   {out / 'prior' / 'map.yaml'} \\\n"
          f"      --patched-map-yaml {out / 'patched' / 'map.yaml'} \\\n"
          f"      --obstacle-yaml    {obstacle_out} \\\n"
          f"      --oracle-mode off --out-dir /tmp/run_srr")
    return 0


if __name__ == "__main__":
    sys.exit(main())
