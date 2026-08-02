#!/usr/bin/env python3
"""Spawn the Stop-Replan-Resume obstacle into a running Gazebo world.

*Stop-Replan-Resume* (SRR) is this project's name for the obstacle-response
sequence: stop, plan a local detour, resume the original mission. The scenario
only means anything if the obstacle physically exists in the simulation - the
2D LiDAR has to be able to see it. This tool puts it there.

It talks to Gazebo and nothing else. It never contacts PX4, never publishes a
ROS topic, and never touches the vehicle. It cannot arm anything.

What it spawns
--------------
``sim/obstacles/srr_unknown_obstacle.sdf`` - a static cylinder, r = 1.5 m,
h = 3.0 m, at world (-21.33, -43.71), base on the ground. Geometry and pose
come from ``sim/obstacles/obstacle.yaml``, which is the single source of truth;
they are not hard-coded here.

Exactly once
------------
Spawning the same model twice would give the LiDAR two overlapping returns and
quietly corrupt the detection metrics. So the tool lists the world's models
first, and:

* model absent  -> spawn, then re-list to confirm it appeared
* model present -> spawn nothing, report ``already_present``, exit 0
* ``--fail-if-exists`` -> exit 4 instead, for scripts that need a virgin world

``spawn_record.yaml`` records ``spawned_this_invocation: true|false`` and the
pre/post model counts, so "exactly once" is checkable after the fact rather
than assumed.

Four representations of one obstacle - keep them apart
------------------------------------------------------
============================  =============================================
Physical Gazebo model          What this tool spawns. The only thing the
                               LiDAR can see.
Prior occupancy map            Does NOT contain the obstacle. This is what
                               the replanner searches.
Sensed obstacle                Built online from LiDAR clusters plus a
                               novelty test against the prior. This is what
                               gets stamped into the local ROI.
Ground truth (obstacle.yaml)   In perception mode it sits in a separate
                               evaluation-only namespace: no decision reads
                               it, diagnostics and post-run metrics may. The
                               oracle modes read it for decisions too.
Patched map                    Prior with ground truth stamped in. Used only
                               by ``--oracle-mode replan`` mechanics checks.
============================  =============================================

Usage::

    python3 tools/spawn_srr_obstacle.py --world forest_map2
    python3 tools/spawn_srr_obstacle.py --verify-only     # no spawn, just check
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_autonomy.artifacts import sha256_of_file, write_yaml  # noqa: E402

DEFAULT_WORLD = "forest_map2"
DEFAULT_OBSTACLE_YAML = REPO_ROOT / "sim/obstacles/obstacle.yaml"
POSE_TOLERANCE_M = 0.05


class SpawnError(Exception):
    """Something went wrong that must stop the run."""


# ----------------------------------------------------------------------
# Gazebo CLI
# ----------------------------------------------------------------------

def require_gz() -> str:
    gz = shutil.which("gz")
    if gz is None:
        raise SpawnError(
            "the 'gz' CLI is not on PATH. Source the Gazebo environment, or "
            "install Gazebo Harmonic.")
    return gz


def list_models(timeout_s: float = 10.0) -> list:
    """Model names visible to ``gz model --list``."""
    try:
        out = subprocess.check_output(
            [require_gz(), "model", "--list"],
            stderr=subprocess.STDOUT, timeout=timeout_s).decode()
    except subprocess.TimeoutExpired as e:
        raise SpawnError(
            f"'gz model --list' timed out after {timeout_s} s. Is the Gazebo "
            f"server running?") from e
    except subprocess.CalledProcessError as e:
        raise SpawnError(
            f"'gz model --list' failed:\n{e.output.decode(errors='replace')}") from e

    names: list = []
    capture = False
    for line in out.splitlines():
        s = line.strip()
        if s.startswith("Available models:"):
            capture = True
            continue
        if capture:
            m = re.match(r"^-\s*(\S+)\s*$", s)
            if m:
                names.append(m.group(1))
    if not names:
        raise SpawnError(
            "'gz model --list' returned no models. The Gazebo server is not "
            "running, or no world is loaded.\nOutput was:\n" + out)
    return names


def model_pose(name: str, timeout_s: float = 10.0):
    """Measured (x, y, z) of a model, or None if it cannot be parsed.

    Gazebo Harmonic prints::

        Pose [ XYZ (m) ] [ RPY (rad) ]:
          [x y z]
          [r p y]

    Older releases used ``[Pos: x | y | z, Rot: ...]``. Both are handled; the
    first Pose block is the model pose, link and inertial poses follow.
    """
    try:
        out = subprocess.check_output(
            [require_gz(), "model", "-m", name],
            stderr=subprocess.STDOUT, timeout=timeout_s).decode()
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None
    m = re.search(
        r"Pose\s*\[\s*XYZ.*?\]\s*\[\s*RPY.*?\]:\s*\n\s*\[\s*"
        r"([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)\s*\]", out, re.DOTALL)
    if m:
        return (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    m = re.search(r"\[Pos:\s*([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", out)
    if m:
        return (float(m.group(1)), float(m.group(2)), float(m.group(3)))
    return None


def gz_create(world: str, sdf_path: Path, pose_xyz, timeout_s: float = 8.0):
    """Call ``/world/<world>/create`` with an EntityFactory request."""
    req = (f'sdf_filename: "{sdf_path}" '
           f'pose: {{position: {{x: {pose_xyz[0]:.6f}, y: {pose_xyz[1]:.6f}, '
           f'z: {pose_xyz[2]:.6f}}}}} allow_renaming: false')
    proc = subprocess.run(
        [require_gz(), "service", "-s", f"/world/{world}/create",
         "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
         "--timeout", str(int(timeout_s * 1000.0)), "--req", req],
        capture_output=True, text=True, timeout=timeout_s + 5.0)
    out = (proc.stdout + proc.stderr).strip()
    return (proc.returncode == 0 and "data: true" in out), out


# ----------------------------------------------------------------------

def load_obstacle(path: Path) -> dict:
    if not path.is_file():
        raise SpawnError(f"obstacle definition not found: {path}")
    d = yaml.safe_load(path.read_text(encoding="utf-8"))
    for key in ("model_name", "position_world_xy_m", "radius_m", "height_m",
                "z_base_world_m", "sdf"):
        if key not in d:
            raise SpawnError(f"{path}: missing required key {key!r}")
    if d.get("verdict") != "PASS":
        raise SpawnError(
            f"{path}: verdict is {d.get('verdict')!r}, not 'PASS'. An obstacle "
            f"placement that failed its own checks must not drive a run.")
    return d


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default=DEFAULT_WORLD,
                    help="Gazebo world name (default: %(default)s).")
    ap.add_argument("--obstacle-yaml", type=Path, default=DEFAULT_OBSTACLE_YAML,
                    help="Obstacle definition; source of truth for pose and size.")
    ap.add_argument("--sdf", type=Path, default=None,
                    help="Override the SDF path from the obstacle definition.")
    ap.add_argument("--out-dir", type=Path, default=None,
                    help="Where to write spawn_record.yaml (default: alongside "
                         "nothing - the record goes to the current directory).")
    ap.add_argument("--fail-if-exists", action="store_true",
                    help="Exit 4 if the model is already in the world.")
    ap.add_argument("--verify-only", action="store_true",
                    help="Check presence and pose; never spawn.")
    ap.add_argument("--pose-tolerance-m", type=float, default=POSE_TOLERANCE_M)
    args = ap.parse_args()

    out_dir = (args.out_dir or Path.cwd()).resolve()
    record_path = out_dir / "spawn_record.yaml"

    try:
        obs = load_obstacle(args.obstacle_yaml)
        sdf_path = (args.sdf or (REPO_ROOT / obs["sdf"])).resolve()
        if not sdf_path.is_file():
            raise SpawnError(f"obstacle SDF not found: {sdf_path}")

        name = str(obs["model_name"])
        x, y = float(obs["position_world_xy_m"][0]), float(obs["position_world_xy_m"][1])
        z_center = float(obs["z_base_world_m"]) + 0.5 * float(obs["height_m"])

        # The SDF's own <pose> and the definition must not disagree - if they
        # did, the spawned model would silently differ from the documented one.
        sdf_text = sdf_path.read_text(encoding="utf-8")
        m = re.search(r"<pose>\s*([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", sdf_text)
        if m:
            sdf_pose = tuple(float(m.group(i)) for i in (1, 2, 3))
            want = (x, y, z_center)
            if any(abs(a - b) > 1e-6 for a, b in zip(sdf_pose, want)):
                raise SpawnError(
                    f"pose disagreement: {args.obstacle_yaml.name} says "
                    f"{want}, {sdf_path.name} says {sdf_pose}")
        if f'name="{name}"' not in sdf_text:
            raise SpawnError(
                f"{sdf_path.name} does not declare model name {name!r}; the "
                f"spawned model would not match the obstacle definition.")

        pre = list_models()
        already = name in pre
        print(f"[spawn] world           = {args.world}")
        print(f"[spawn] model           = {name}")
        print(f"[spawn] sdf             = {sdf_path}")
        print(f"[spawn] target pose     = ({x:.3f}, {y:.3f}, {z_center:.3f}) m, "
              f"r={float(obs['radius_m']):.3f} h={float(obs['height_m']):.3f}")
        print(f"[spawn] models before   = {len(pre)} (obstacle present: {already})")

        if already and args.fail_if_exists:
            raise SpawnError(
                f"model {name!r} is already in the world and --fail-if-exists "
                f"was given. Restart the simulator for a virgin world.")

        spawned_now = False
        service_output = ""
        if args.verify_only:
            status = "verify_only"
            if not already:
                raise SpawnError(
                    f"--verify-only: model {name!r} is NOT in the world.")
        elif already:
            status = "already_present"
            print("[spawn] model already present -> not spawning again")
        else:
            ok, service_output = gz_create(args.world, sdf_path, (x, y, z_center))
            if not ok:
                raise SpawnError(
                    f"gz service create failed:\n{service_output}\n"
                    f"Check that the world name {args.world!r} is right "
                    f"(gz topic -l | head).")
            spawned_now = True
            status = "spawned"

        post = list_models()
        present = name in post
        print(f"[spawn] models after    = {len(post)} (obstacle present: {present})")
        if not present:
            raise SpawnError(
                f"model {name!r} is not visible after the spawn request. "
                f"Gazebo accepted the request but produced no model.")
        if spawned_now and len(post) != len(pre) + 1:
            raise SpawnError(
                f"expected exactly one new model, went from {len(pre)} to "
                f"{len(post)}. Something else changed the world.")
        if post.count(name) != 1:
            raise SpawnError(
                f"model {name!r} appears {post.count(name)} times in the world. "
                f"Duplicate obstacles corrupt the detection metrics.")

        measured = model_pose(name)
        pose_match = None
        if measured is not None:
            dx, dy, dz = (measured[0] - x, measured[1] - y, measured[2] - z_center)
            pose_match = (abs(dx) <= args.pose_tolerance_m and
                          abs(dy) <= args.pose_tolerance_m and
                          abs(dz) <= args.pose_tolerance_m)
            print(f"[spawn] measured pose   = ({measured[0]:.3f}, "
                  f"{measured[1]:.3f}, {measured[2]:.3f}) m  "
                  f"delta=({dx:+.3f}, {dy:+.3f}, {dz:+.3f})  match={pose_match}")
            if not pose_match:
                raise SpawnError(
                    f"the model is in the world but at the wrong pose: measured "
                    f"{measured}, expected ({x}, {y}, {z_center}), tolerance "
                    f"{args.pose_tolerance_m} m.")
        else:
            print("[spawn] measured pose   = unavailable "
                  "(gz model output could not be parsed)")

    except SpawnError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
            write_yaml(record_path, {
                "verdict": "FAIL", "error": str(e),
                "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
                "world": args.world,
            })
        except OSError:
            pass
        return 4 if "already in the world" in str(e) else 3

    out_dir.mkdir(parents=True, exist_ok=True)
    write_yaml(record_path, {
        "verdict": "PASS",
        "tool": "tools/spawn_srr_obstacle.py",
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "world": args.world,
        "model_name": name,
        "status": status,
        "spawned_this_invocation": spawned_now,
        "model_count_before": len(pre),
        "model_count_after": len(post),
        "model_occurrences_after": post.count(name),
        "obstacle": {
            "position_world_xy_m": [x, y],
            "z_base_world_m": float(obs["z_base_world_m"]),
            "z_center_world_m": z_center,
            "radius_m": float(obs["radius_m"]),
            "height_m": float(obs["height_m"]),
            "static": True,
        },
        "pose_verification": {
            "method": "gz model -m <name>, first Pose block",
            "measured_world_xyz_m": list(measured) if measured else None,
            "tolerance_m": args.pose_tolerance_m,
            "match": pose_match,
        },
        "inputs": {
            "obstacle_yaml": str(args.obstacle_yaml.relative_to(REPO_ROOT)
                                 if args.obstacle_yaml.is_relative_to(REPO_ROOT)
                                 else args.obstacle_yaml),
            "obstacle_yaml_sha256": sha256_of_file(args.obstacle_yaml),
            "sdf": str(sdf_path.relative_to(REPO_ROOT)
                       if sdf_path.is_relative_to(REPO_ROOT) else sdf_path),
            "sdf_sha256": sha256_of_file(sdf_path),
        },
        "spawn_service": {
            "service": f"/world/{args.world}/create",
            "reqtype": "gz.msgs.EntityFactory",
            "response": service_output or None,
        },
        "scope": "Gazebo simulation only. No PX4 command, no ROS publication, "
                 "no vehicle interaction.",
    })
    print(f"[spawn] status          = {status}")
    print(f"[spawn] record          = {record_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
