#!/usr/bin/env python3
"""Build a PX4 local-NED setpoint stream from a world-frame mission reference.

This is the bridge between the offline planning chain and the PX4 Offboard
runtime. It applies the ENU -> NED transform (:mod:`uav_autonomy.frames`),
resamples onto a fixed rate, and writes the CSV that
``ros2/nodes/offboard_mission_executor.py`` and
``ros2/nodes/stop_replan_resume_node.py`` consume via ``--setpoints-csv``.

Input
-----
Any world-frame trajectory CSV with ``x_m``, ``y_m``, ``yaw_rad`` and
``v_ref_mps``. Two shapes are supported and auto-detected:

*Complete mission* (``results/mission_reference/mission_reference.csv``) --
carries ``segment_id`` / ``segment_type`` / ``segment_name`` and ``t_ref_s``.
All five segments are emitted in order with their labels preserved; the
reference timeline comes straight from ``t_ref_s``.

*Single leg* (``results/planning/trajectory.csv``) -- no segment columns and no
time column. The timeline is integrated from ``s_m`` and ``v_ref_mps``, and the
whole stream is labelled with ``--segment-id`` / ``--segment-name``.

Output schema
-------------
``idx, t_ref_s, segment_id, segment_type, segment_name, x_ned_m, y_ned_m,
z_ned_m, yaw_ned_rad, vx_ned_mps, vy_ned_mps, vz_ned_mps, v_ref_mps,
curvature_1_per_m``

alongside ``<output>.manifest.yaml``, ``<output>.validation.yaml`` and
``<output>.hashes.txt``.

Checks that make the build fail (exit 2) rather than fly something wrong
-----------------------------------------------------------------------
* required columns missing, non-finite values, or a non-monotonic timeline;
* segment blocks that are out of order, interleaved or repeated;
* ``--require-segments`` given and not matched exactly;
* resampling that changes the path length by more than ``--max-length-drift``;
* a first setpoint further than ``--max-start-offset-m`` from the NED origin.

That last check is the one that matters in practice. The Offboard executor's
settle guard refuses to fly to a first setpoint far from a standing start, so a
stream built from a mid-route slice cannot drive an armed mission. Failing here
is better than discovering it after arming. Pass ``--allow-offset-start`` when
you deliberately want a partial stream (dry runs, geometry work).

Usage::

    # Complete mission, ready for an armed run
    python3 tools/build_ned_setpoints.py \\
        --trajectory-csv results/mission_reference/mission_reference.csv \\
        --output-csv /tmp/setpoints_ned.csv

    # Single planning leg
    python3 tools/build_ned_setpoints.py \\
        --trajectory-csv results/planning/trajectory.csv \\
        --output-csv /tmp/leg1_ned.csv --allow-offset-start
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

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from uav_autonomy.artifacts import sha256_of_file, write_yaml  # noqa: E402
from uav_autonomy.frames import (  # noqa: E402
    X_ORIGIN_WORLD, Y_ORIGIN_WORLD, Z_REF_M, apply_enu_to_ned, wrap_pi,
)

REQUIRED_COLUMNS = ("x_m", "y_m", "yaw_rad", "v_ref_mps")
SEGMENT_COLUMNS = ("segment_id", "segment_type", "segment_name")

OUTPUT_HEADER = [
    "idx", "t_ref_s", "segment_id", "segment_type", "segment_name",
    "x_ned_m", "y_ned_m", "z_ned_m", "yaw_ned_rad",
    "vx_ned_mps", "vy_ned_mps", "vz_ned_mps",
    "v_ref_mps", "curvature_1_per_m",
]


class InputError(Exception):
    """Anything that makes the input unusable. Caught in main(), reported, exit 2."""


# ----------------------------------------------------------------------
# Load
# ----------------------------------------------------------------------

def load_trajectory(path: Path, segment_id: int, segment_name: str,
                    segment_type: str) -> dict:
    """Read a world-frame trajectory CSV into arrays, validating as we go."""
    if not path.is_file():
        raise InputError(f"trajectory CSV not found: {path}")
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames or []
        missing = [c for c in REQUIRED_COLUMNS if c not in header]
        if missing:
            raise InputError(
                f"{path}: missing required columns {missing}. Present: {header}")
        rows = list(reader)
    if len(rows) < 2:
        raise InputError(f"{path}: needs at least two samples, found {len(rows)}")

    present = set(rows[0].keys())
    has_segments = all(c in present for c in SEGMENT_COLUMNS)
    has_time = "t_ref_s" in present
    has_s = "s_m" in present

    def col(name: str) -> np.ndarray:
        return np.array([float(r[name]) for r in rows], dtype=float)

    data = {
        "x_m": col("x_m"), "y_m": col("y_m"),
        "yaw": col("yaw_rad"), "v": col("v_ref_mps"),
        "curv": (col("curvature_1_per_m") if "curvature_1_per_m" in present
                 else np.zeros(len(rows))),
        "s_m": col("s_m") if has_s else None,
    }
    for name in ("x_m", "y_m", "yaw", "v", "curv"):
        arr = data[name]
        if not np.all(np.isfinite(arr)):
            bad = int(np.argmax(~np.isfinite(arr)))
            raise InputError(f"{path}: non-finite {name} at row {bad + 2}")

    if has_segments:
        data["seg_id"] = np.array([int(r["segment_id"]) for r in rows], dtype=int)
        data["seg_type"] = [r["segment_type"] for r in rows]
        data["seg_name"] = [r["segment_name"] for r in rows]
    else:
        data["seg_id"] = np.full(len(rows), int(segment_id), dtype=int)
        data["seg_type"] = [segment_type] * len(rows)
        data["seg_name"] = [segment_name] * len(rows)
    data["has_segment_columns"] = has_segments

    if has_time:
        t = col("t_ref_s")
        if not np.all(np.diff(t) >= -1e-9):
            bad = int(np.argmax(np.diff(t) < -1e-9))
            raise InputError(
                f"{path}: t_ref_s is not monotonically non-decreasing "
                f"(row {bad + 3} goes backwards)")
        data["t"] = t
        data["time_source"] = "t_ref_s column"
    else:
        if data["s_m"] is None:
            raise InputError(
                f"{path}: neither t_ref_s nor s_m present; cannot build a "
                f"reference timeline")
        s = data["s_m"]
        if not np.all(np.diff(s) >= -1e-9):
            raise InputError(f"{path}: s_m is not monotonically non-decreasing")
        v_avg = np.maximum(0.5 * (data["v"][:-1] + data["v"][1:]), 1e-6)
        data["t"] = np.concatenate([[0.0], np.cumsum(np.diff(s) / v_avg)])
        data["time_source"] = "integrated from s_m / v_ref_mps"
    return data


def segment_blocks(seg_id, seg_name) -> list:
    """Contiguous runs of (segment_id, segment_name), in file order."""
    blocks: list = []
    for i in range(len(seg_id)):
        key = (int(seg_id[i]), seg_name[i])
        if not blocks or blocks[-1]["key"] != key:
            blocks.append({"key": key, "start": i, "end": i, "n": 1})
        else:
            blocks[-1]["end"] = i
            blocks[-1]["n"] += 1
    return blocks


def check_segment_structure(data: dict, require) -> list:
    """Segments must appear once each, as contiguous runs, in ascending order."""
    blocks = segment_blocks(data["seg_id"], data["seg_name"])
    names = [b["key"][1] for b in blocks]
    ids = [b["key"][0] for b in blocks]

    if len(set(names)) != len(names):
        dupes = sorted({n for n in names if names.count(n) > 1})
        raise InputError(
            f"segment blocks are interleaved or repeated: {names}. "
            f"Offending names: {dupes}. Each segment must be one contiguous "
            f"run of rows.")
    if ids != sorted(ids):
        raise InputError(
            f"segment_id is not ascending in file order: {ids}. "
            f"The mission would be flown out of order.")
    if require is not None and names != require:
        raise InputError(
            f"--require-segments mismatch.\n"
            f"  expected: {require}\n  found:    {names}")
    return blocks


# ----------------------------------------------------------------------
# Resample
# ----------------------------------------------------------------------

def resample(data: dict, rate_hz: float, max_duration_s) -> dict:
    """Fixed-rate resample. Yaw is unwrapped first; labels are nearest-left.

    Nearest-left label lookup, rather than interpolation, is what keeps segment
    boundaries crisp: a grid point inside segment 3 is labelled 3, never a blend
    of 2 and 3.
    """
    t_orig = data["t"]
    t_end = float(t_orig[-1])
    if max_duration_s is not None:
        t_end = min(t_end, float(max_duration_s))
    if t_end <= 0.0:
        raise InputError(f"reference duration is {t_end} s; nothing to stream")

    dt = 1.0 / rate_hz
    t_grid = np.arange(int(math.floor(t_end / dt)) + 1, dtype=float) * dt
    # Snap the final sample onto the exact end so the stream covers the whole
    # reference even when the duration is not a multiple of dt.
    if t_grid[-1] < t_end - 1e-9:
        t_grid = np.concatenate([t_grid, [t_end]])

    yaw_unwrapped = np.unwrap(data["yaw"])
    out = {
        "t": t_grid,
        "x": np.interp(t_grid, t_orig, data["x_m"]),
        "y": np.interp(t_grid, t_orig, data["y_m"]),
        "yaw": wrap_pi(np.interp(t_grid, t_orig, yaw_unwrapped)),
        "v": np.interp(t_grid, t_orig, data["v"]),
        "curv": np.interp(t_grid, t_orig, data["curv"]),
    }
    idx_left = np.clip(
        np.searchsorted(t_orig, t_grid, side="right") - 1, 0, len(t_orig) - 1)
    out["seg_id"] = data["seg_id"][idx_left]
    out["seg_type"] = [data["seg_type"][i] for i in idx_left]
    out["seg_name"] = [data["seg_name"][i] for i in idx_left]
    return out


def path_length(x: np.ndarray, y: np.ndarray) -> float:
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


def write_csv(path: Path, grid: dict, ned: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(OUTPUT_HEADER)
        for i in range(len(grid["t"])):
            w.writerow([
                i,
                f"{grid['t'][i]:.6f}",
                int(grid["seg_id"][i]), grid["seg_type"][i], grid["seg_name"][i],
                f"{ned['x_ned'][i]:.6f}", f"{ned['y_ned'][i]:.6f}",
                f"{ned['z_ned'][i]:.6f}", f"{ned['yaw_ned'][i]:.9f}",
                f"{ned['vx_ned'][i]:.6f}", f"{ned['vy_ned'][i]:.6f}",
                f"{ned['vz_ned'][i]:.6f}",
                f"{grid['v'][i]:.6f}", f"{grid['curv'][i]:.9f}",
            ])


# ----------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trajectory-csv", type=Path,
                    default=REPO_ROOT / "results/mission_reference/mission_reference.csv",
                    help="World-frame reference. Default: the complete mission.")
    ap.add_argument("--output-csv", type=Path, required=True)
    ap.add_argument("--rate-hz", type=float, default=20.0)
    ap.add_argument("--z-ref-m", type=float, default=Z_REF_M,
                    help="Reference altitude above the takeoff point, in metres.")
    ap.add_argument("--origin-x", type=float, default=X_ORIGIN_WORLD,
                    help="World x of the PX4 local-NED origin (vehicle spawn).")
    ap.add_argument("--origin-y", type=float, default=Y_ORIGIN_WORLD,
                    help="World y of the PX4 local-NED origin (vehicle spawn).")
    ap.add_argument("--segment-id", type=int, default=1,
                    help="Label for inputs without segment columns.")
    ap.add_argument("--segment-name", type=str, default="start_to_poi1",
                    help="Label for inputs without segment columns.")
    ap.add_argument("--segment-type", type=str, default="leg",
                    help="Label for inputs without segment columns.")
    ap.add_argument("--require-segments", type=str, default=None,
                    help="Comma-separated segment names that must appear, in "
                         "this exact order. Fails otherwise.")
    ap.add_argument("--max-duration-s", type=float, default=None,
                    help="Truncate the stream. Useful for short smoke tests.")
    ap.add_argument("--max-start-offset-m", type=float, default=1.0,
                    help="Reject a first setpoint further than this from the "
                         "NED origin (default 1.0 m).")
    ap.add_argument("--allow-offset-start", action="store_true",
                    help="Downgrade the start-offset check to a warning. Such a "
                         "stream cannot drive an armed run from the spawn point.")
    ap.add_argument("--max-length-drift", type=float, default=0.005,
                    help="Maximum relative path-length change from resampling.")
    args = ap.parse_args()

    try:
        require = ([s.strip() for s in args.require_segments.split(",")]
                   if args.require_segments else None)
        data = load_trajectory(args.trajectory_csv, args.segment_id,
                               args.segment_name, args.segment_type)
        blocks = check_segment_structure(data, require)
        grid = resample(data, args.rate_hz, args.max_duration_s)
    except InputError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return 2

    ned = apply_enu_to_ned(
        x_world=grid["x"], y_world=grid["y"], yaw_world=grid["yaw"],
        v_world=grid["v"], origin_xy_world=(args.origin_x, args.origin_y),
        z_ref_m=args.z_ref_m,
    )

    # ---- validation -------------------------------------------------
    errors: list = []
    warnings: list = []

    start_offset = float(math.hypot(ned["x_ned"][0], ned["y_ned"][0]))
    if start_offset > args.max_start_offset_m:
        msg = (f"first setpoint is {start_offset:.3f} m from the NED origin "
               f"(limit {args.max_start_offset_m:.3f} m). The settle guard will "
               f"refuse to fly there from a standing start.")
        (warnings if args.allow_offset_start else errors).append(msg)

    len_in = path_length(data["x_m"], data["y_m"])
    len_grid = path_length(grid["x"], grid["y"])
    len_ned = path_length(ned["x_ned"], ned["y_ned"])
    drift = abs(len_grid - len_in) / max(len_in, 1e-9)
    if args.max_duration_s is None and drift > args.max_length_drift:
        errors.append(f"resampling changed the path length by {100 * drift:.3f} % "
                      f"(limit {100 * args.max_length_drift:.3f} %)")
    transform_drift = abs(len_ned - len_grid) / max(len_grid, 1e-9)
    if transform_drift > 1e-9:
        errors.append(f"ENU->NED changed the path length by {transform_drift:.3e}; "
                      f"the transform must be an isometry")

    if not np.all(ned["z_ned"] < 0.0):
        errors.append("z_ned is not strictly negative; the vehicle would be "
                      "commanded at or below the takeoff plane")
    for name, arr in (("x_ned", ned["x_ned"]), ("y_ned", ned["y_ned"]),
                      ("yaw_ned", ned["yaw_ned"]), ("vx_ned", ned["vx_ned"]),
                      ("vy_ned", ned["vy_ned"])):
        if not np.all(np.isfinite(arr)):
            errors.append(f"non-finite values in {name}")
    if not np.all(np.diff(grid["t"]) > 0.0):
        errors.append("output t_ref_s is not strictly increasing")

    # Yaw must follow the direction of travel wherever the reference moves.
    moving = grid["v"] > 0.05
    if np.count_nonzero(moving) > 2:
        course = np.arctan2(np.gradient(ned["y_ned"]), np.gradient(ned["x_ned"]))
        err = np.abs(wrap_pi(course - ned["yaw_ned"]))[moving]
        yaw_p95 = float(np.degrees(np.percentile(err, 95)))
        yaw_max = float(np.degrees(np.max(err)))
        if yaw_p95 > 5.0:
            errors.append(f"yaw does not follow the course: p95 error "
                          f"{yaw_p95:.2f} deg")
    else:
        yaw_p95 = yaw_max = float("nan")

    out_blocks = segment_blocks(grid["seg_id"], grid["seg_name"])
    if args.max_duration_s is None and \
            [b["key"] for b in out_blocks] != [b["key"] for b in blocks]:
        errors.append(
            f"resampling lost or reordered segments: "
            f"{[b['key'] for b in blocks]} -> {[b['key'] for b in out_blocks]}")

    validation = {
        "verdict": "FAIL" if errors else "PASS",
        "errors": errors,
        "warnings": warnings,
        "n_samples_input": int(len(data["t"])),
        "n_samples_output": int(len(grid["t"])),
        "rate_hz": args.rate_hz,
        "duration_s": float(grid["t"][-1]),
        "time_source": data["time_source"],
        "input_has_segment_columns": bool(data["has_segment_columns"]),
        "segments": [{"segment_id": b["key"][0], "segment_name": b["key"][1],
                      "n_samples": b["n"]} for b in out_blocks],
        "first_setpoint_ned_xy_m": [float(ned["x_ned"][0]), float(ned["y_ned"][0])],
        "first_setpoint_distance_from_origin_m": start_offset,
        "last_setpoint_ned_xy_m": [float(ned["x_ned"][-1]), float(ned["y_ned"][-1])],
        "last_segment_name": grid["seg_name"][-1],
        "ned_bounds": {
            "x_ned_min_m": float(ned["x_ned"].min()),
            "x_ned_max_m": float(ned["x_ned"].max()),
            "y_ned_min_m": float(ned["y_ned"].min()),
            "y_ned_max_m": float(ned["y_ned"].max()),
            "z_ned_m": float(ned["z_ned"][0]),
        },
        "path_length_world_input_m": len_in,
        "path_length_world_resampled_m": len_grid,
        "path_length_ned_m": len_ned,
        "path_length_rel_err_resample": drift,
        "path_length_rel_err_transform": transform_drift,
        "yaw_alignment_deg": {"p95": yaw_p95, "max": yaw_max},
    }

    if errors:
        print("FAIL: setpoint stream rejected", file=sys.stderr)
        for e in errors:
            print(f"  - {e}", file=sys.stderr)
        if not args.allow_offset_start and start_offset > args.max_start_offset_m:
            print("\n  Build the stream from the COMPLETE mission reference:\n"
                  "    --trajectory-csv results/mission_reference/mission_reference.csv\n"
                  "  or pass --allow-offset-start for a deliberately partial stream.",
                  file=sys.stderr)
        return 2

    write_csv(args.output_csv, grid, ned)

    manifest_path = args.output_csv.with_suffix(".manifest.yaml")
    validation_path = args.output_csv.with_suffix(".validation.yaml")
    hashes_path = args.output_csv.with_suffix(".hashes.txt")

    write_yaml(validation_path, validation)
    write_yaml(manifest_path, {
        "tool": "tools/build_ned_setpoints.py",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "python_version": platform.python_version(),
        "numpy_version": np.__version__,
        "input": {
            "trajectory_csv": str(args.trajectory_csv),
            "trajectory_csv_sha256": sha256_of_file(args.trajectory_csv),
            "n_samples": int(len(data["t"])),
            "time_source": data["time_source"],
        },
        "transform": {
            "formulas": "x_ned = y_world - Y0; y_ned = x_world - X0; "
                        "z_ned = -z_ref; yaw_ned = wrap_pi(pi/2 - yaw_world); "
                        "vx_ned = v*sin(yaw_world); vy_ned = v*cos(yaw_world)",
            "X0_m": args.origin_x, "Y0_m": args.origin_y,
            "z_ref_m": args.z_ref_m,
            "implementation": "uav_autonomy.frames.apply_enu_to_ned",
        },
        "stream": {
            "rate_hz": args.rate_hz,
            "n_samples": int(len(grid["t"])),
            "duration_s": float(grid["t"][-1]),
            "truncated": args.max_duration_s is not None,
            "segments": [{"segment_id": b["key"][0], "segment_name": b["key"][1]}
                         for b in out_blocks],
        },
        "validation": {"verdict": validation["verdict"],
                       "warnings": warnings,
                       "report": validation_path.name},
    })
    hashes_path.write_text(
        "".join(f"{sha256_of_file(p)}  {p.name}\n"
                for p in (args.output_csv, manifest_path, validation_path)),
        encoding="utf-8")

    seg_desc = " -> ".join(f"{b['key'][0]}:{b['key'][1]}" for b in out_blocks)
    print(f"[build_ned_setpoints] input     = {args.trajectory_csv}")
    print(f"[build_ned_setpoints] samples   = {len(grid['t'])} @ {args.rate_hz} Hz "
          f"({grid['t'][-1]:.2f} s)")
    print(f"[build_ned_setpoints] segments  = {seg_desc}")
    print(f"[build_ned_setpoints] first sp  = ({ned['x_ned'][0]:.3f}, "
          f"{ned['y_ned'][0]:.3f}) m, {start_offset:.3f} m from the NED origin")
    print(f"[build_ned_setpoints] last sp   = ({ned['x_ned'][-1]:.3f}, "
          f"{ned['y_ned'][-1]:.3f}) m in segment {grid['seg_name'][-1]!r}")
    print(f"[build_ned_setpoints] NED z     = {ned['z_ned'][0]:.2f} m (constant)")
    print(f"[build_ned_setpoints] length    = {len_ned:.3f} m "
          f"(resample drift {100 * drift:.4f} %)")
    for w in warnings:
        print(f"[build_ned_setpoints] WARNING: {w}")
    print(f"[build_ned_setpoints] output    = {args.output_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
