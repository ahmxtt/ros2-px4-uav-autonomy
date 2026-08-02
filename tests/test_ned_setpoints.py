"""Generating the PX4 setpoint stream from the complete mission reference.

The stream is the last thing between the offline chain and an armed vehicle,
so the failure modes worth pinning are the ones that are invisible until the
vehicle is in the air: a stream that starts somewhere the vehicle is not, a
segment order that does not match the mission, a transform that is not an
isometry, or a truncation that silently drops the end of the route.
"""

from __future__ import annotations

import csv
import math
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
TOOL = REPO / "tools" / "build_ned_setpoints.py"
MISSION_CSV = REPO / "results" / "mission_reference" / "mission_reference.csv"
WINDOW_CSV = REPO / "results" / "stop_replan_resume" / "mission_segment_window.csv"
TRAJECTORY_CSV = REPO / "results" / "planning" / "trajectory.csv"

SEGMENTS = ["start_to_poi1", "orbit_poi1", "poi1_to_poi2",
            "orbit_poi2", "poi2_to_start"]


def run(*args):
    return subprocess.run([sys.executable, str(TOOL), *args],
                          capture_output=True, text=True, cwd="/")


@pytest.fixture(scope="module")
def stream(tmp_path_factory):
    out = tmp_path_factory.mktemp("ned") / "setpoints_ned.csv"
    proc = run("--trajectory-csv", str(MISSION_CSV), "--output-csv", str(out),
               "--require-segments", ",".join(SEGMENTS))
    assert proc.returncode == 0, proc.stderr
    with out.open() as f:
        rows = list(csv.DictReader(f))
    return out, rows


def test_runs_from_a_foreign_working_directory(stream):
    """cwd is / in the fixture. Relative-path assumptions fail here."""
    _out, rows = stream
    assert rows


def test_sample_count_and_rate(stream):
    _out, rows = stream
    assert len(rows) == 5725
    t = [float(r["t_ref_s"]) for r in rows]
    assert t[-1] == pytest.approx(286.158823, abs=1e-5)
    # 20 Hz grid, with the final sample snapped onto the exact mission end.
    interior = [b - a for a, b in zip(t[:-2], t[1:-1])]
    assert max(interior) == pytest.approx(0.05, abs=1e-9)


def test_first_setpoint_is_at_the_settle_start(stream):
    """No artificial jump. This is the check that gates an armed run."""
    _out, rows = stream
    x, y = float(rows[0]["x_ned_m"]), float(rows[0]["y_ned_m"])
    assert math.hypot(x, y) < 0.1


def test_last_setpoint_returns_home_in_the_terminating_segment(stream):
    _out, rows = stream
    x, y = float(rows[-1]["x_ned_m"]), float(rows[-1]["y_ned_m"])
    assert math.hypot(x, y) < 0.2
    assert rows[-1]["segment_name"] == "poi2_to_start"


def test_segment_order_and_labels_survive_resampling(stream):
    _out, rows = stream
    blocks = []
    for r in rows:
        key = (int(r["segment_id"]), r["segment_name"])
        if not blocks or blocks[-1] != key:
            blocks.append(key)
    assert blocks == list(enumerate(SEGMENTS, start=1))


def test_altitude_is_the_canonical_reference_height(stream):
    _out, rows = stream
    z = {float(r["z_ned_m"]) for r in rows}
    assert z == {-2.0}


def test_transform_is_consistent_with_the_mission(stream):
    """Spot-check the ENU -> NED mapping against the source rows."""
    _out, rows = stream
    with MISSION_CSV.open() as f:
        mission = list(csv.DictReader(f))
    # t = 0 is a shared sample, so it must map exactly.
    x_world, y_world = float(mission[0]["x_m"]), float(mission[0]["y_m"])
    assert float(rows[0]["x_ned_m"]) == pytest.approx(y_world + 70.0, abs=1e-9)
    assert float(rows[0]["y_ned_m"]) == pytest.approx(x_world + 50.0, abs=1e-9)
    yaw_world = float(mission[0]["yaw_rad"])
    expected = math.atan2(math.sin(math.pi / 2 - yaw_world),
                          math.cos(math.pi / 2 - yaw_world))
    assert float(rows[0]["yaw_ned_rad"]) == pytest.approx(expected, abs=1e-9)


def test_velocity_is_tangent_to_the_path(stream):
    _out, rows = stream
    for r in rows[::500]:
        v = float(r["v_ref_mps"])
        speed = math.hypot(float(r["vx_ned_mps"]), float(r["vy_ned_mps"]))
        # Six-decimal CSV fields, so the recomputed speed carries
        # rounding of the two components rather than of v itself.
        assert speed == pytest.approx(v, abs=2e-6)
        assert float(r["vz_ned_mps"]) == 0.0


def test_validation_report_passes_and_records_the_segments(stream):
    out, _rows = stream
    report = yaml.safe_load(out.with_suffix(".validation.yaml").read_text())
    assert report["verdict"] == "PASS"
    assert report["errors"] == []
    assert [s["segment_name"] for s in report["segments"]] == SEGMENTS
    assert report["first_setpoint_distance_from_origin_m"] < 0.1
    assert report["path_length_rel_err_transform"] < 1e-12
    assert report["path_length_rel_err_resample"] < 0.005


def test_manifest_and_hashes_are_written(stream):
    out, _rows = stream
    manifest = yaml.safe_load(out.with_suffix(".manifest.yaml").read_text())
    assert manifest["stream"]["n_samples"] == 5725
    assert manifest["input"]["time_source"] == "t_ref_s column"
    hashes = out.with_suffix(".hashes.txt").read_text().splitlines()
    assert len(hashes) == 3


def test_a_mid_route_slice_is_refused(tmp_path):
    """The published window starts 15 m downstream and cannot drive a run."""
    out = tmp_path / "window.csv"
    proc = run("--trajectory-csv", str(WINDOW_CSV), "--output-csv", str(out))
    assert proc.returncode == 2
    assert "from the NED origin" in proc.stderr
    assert not out.exists()


def test_a_mid_route_slice_is_allowed_when_asked_for(tmp_path):
    out = tmp_path / "window.csv"
    proc = run("--trajectory-csv", str(WINDOW_CSV), "--output-csv", str(out),
               "--allow-offset-start")
    assert proc.returncode == 0, proc.stderr
    assert "WARNING" in proc.stdout
    report = yaml.safe_load(out.with_suffix(".validation.yaml").read_text())
    assert report["warnings"]


def test_wrong_segment_expectation_is_refused(tmp_path):
    out = tmp_path / "s.csv"
    proc = run("--trajectory-csv", str(MISSION_CSV), "--output-csv", str(out),
               "--require-segments", "start_to_poi1,orbit_poi1")
    assert proc.returncode == 2
    assert "require-segments mismatch" in proc.stderr


def test_single_leg_input_without_segment_columns(tmp_path):
    """results/planning/trajectory.csv has neither segments nor a time column."""
    out = tmp_path / "leg.csv"
    proc = run("--trajectory-csv", str(TRAJECTORY_CSV), "--output-csv", str(out),
               "--segment-name", "start_to_poi1")
    assert proc.returncode == 0, proc.stderr
    manifest = yaml.safe_load(out.with_suffix(".manifest.yaml").read_text())
    assert manifest["input"]["time_source"] == "integrated from s_m / v_ref_mps"
    with out.open() as f:
        rows = list(csv.DictReader(f))
    assert {r["segment_name"] for r in rows} == {"start_to_poi1"}


def test_missing_columns_are_reported_clearly(tmp_path):
    bad = tmp_path / "bad.csv"
    bad.write_text("a,b\n1,2\n3,4\n")
    out = tmp_path / "o.csv"
    proc = run("--trajectory-csv", str(bad), "--output-csv", str(out))
    assert proc.returncode == 2
    assert "missing required columns" in proc.stderr


def test_out_of_order_segments_are_refused(tmp_path):
    """Segment 2 before segment 1 would fly the mission backwards."""
    with MISSION_CSV.open() as f:
        rows = list(csv.DictReader(f))
    seg1 = [dict(r) for r in rows if r["segment_id"] == "1"][:50]
    seg2 = [dict(r) for r in rows if r["segment_id"] == "2"][:50]
    swapped = seg2 + seg1
    # Keep time and arc length monotone so the ordering check is what fires,
    # not the earlier monotonicity check.
    for i, r in enumerate(swapped):
        r["t_ref_s"] = f"{i * 0.05:.6f}"
        r["s_m"] = f"{i * 0.05:.6f}"
    bad = tmp_path / "swapped.csv"
    with bad.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(swapped)
    out = tmp_path / "o.csv"
    proc = run("--trajectory-csv", str(bad), "--output-csv", str(out),
               "--allow-offset-start")
    assert proc.returncode == 2
    assert "ascending" in proc.stderr
