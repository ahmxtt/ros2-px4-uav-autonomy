"""The complete mission reference: the file that makes an armed run possible.

An armed run is only reachable if the reference starts where the vehicle
stands. A mission that begins mid-route is refused by the settle guard before
it ever reaches MISSION, and the failure surfaces as a timeout with no obvious
cause. These tests pin the properties that make the file usable, so a future
edit that quietly truncates or reorders it fails here instead of at 2 m.
"""

from __future__ import annotations

import csv
import hashlib
from pathlib import Path

import pytest
import yaml

from uav_autonomy.frames import X_ORIGIN_WORLD, Y_ORIGIN_WORLD, world_to_ned

REPO = Path(__file__).resolve().parents[1]
BUNDLE = REPO / "results" / "mission_reference"
MISSION_CSV = BUNDLE / "mission_reference.csv"
WINDOW_CSV = REPO / "results" / "stop_replan_resume" / "mission_segment_window.csv"

EXPECTED_SEGMENTS = [
    (1, "leg", "start_to_poi1", 5179),
    (2, "orbit", "orbit_poi1", 1685),
    (3, "leg", "poi1_to_poi2", 2572),
    (4, "orbit", "orbit_poi2", 1673),
    (5, "leg", "poi2_to_start", 5048),
]
EXPECTED_ROWS = 16157


@pytest.fixture(scope="module")
def rows():
    with MISSION_CSV.open() as f:
        return list(csv.DictReader(f))


def test_bundle_is_complete():
    for name in ("mission_reference.csv", "mission_metrics.yaml",
                 "segment_metrics.yaml", "transition_metrics.yaml",
                 "manifest.yaml", "hashes.txt", "README.md"):
        assert (BUNDLE / name).is_file(), f"missing {name}"


def test_hashes_verify():
    """Every file listed in hashes.txt is present and unmodified."""
    for line in (BUNDLE / "hashes.txt").read_text().splitlines():
        if not line.strip():
            continue
        digest, name = line.split(None, 1)
        target = BUNDLE / name.strip()
        assert target.is_file(), f"hashes.txt lists a missing file: {name}"
        actual = hashlib.sha256(target.read_bytes()).hexdigest()
        assert actual == digest, f"{name} does not match its recorded hash"


def test_row_count_and_columns(rows):
    assert len(rows) == EXPECTED_ROWS
    assert list(rows[0]) == [
        "segment_id", "segment_type", "segment_name", "s_m", "x_m", "y_m",
        "yaw_rad", "curvature_1_per_m", "v_ref_mps", "t_ref_s",
    ]


def test_segments_are_contiguous_and_in_order(rows):
    """Each segment is one unbroken run, and the runs ascend 1..5.

    Interleaved segments would be flown out of order; the setpoint builder
    rejects that too, but the data should never get that far.
    """
    blocks = []
    for r in rows:
        key = (int(r["segment_id"]), r["segment_type"], r["segment_name"])
        if not blocks or blocks[-1][0] != key:
            blocks.append([key, 0])
        blocks[-1][1] += 1
    got = [(k[0], k[1], k[2], n) for k, n in blocks]
    assert got == EXPECTED_SEGMENTS


def test_starts_at_the_vehicle_spawn_point(rows):
    """The whole point of publishing the complete reference."""
    x0, y0 = float(rows[0]["x_m"]), float(rows[0]["y_m"])
    north, east = world_to_ned(x0, y0, X_ORIGIN_WORLD, Y_ORIGIN_WORLD)
    offset = (north ** 2 + east ** 2) ** 0.5
    assert offset < 0.1, (
        f"the mission starts {offset:.3f} m from the NED origin; the settle "
        f"guard will refuse to fly there from a standing start")


def test_returns_to_the_start(rows):
    """It is a round trip: the last leg comes home."""
    x0, y0 = float(rows[0]["x_m"]), float(rows[0]["y_m"])
    xn, yn = float(rows[-1]["x_m"]), float(rows[-1]["y_m"])
    assert abs(xn - x0) < 0.2 and abs(yn - y0) < 0.2
    assert rows[-1]["segment_name"] == "poi2_to_start"


def test_arc_length_and_time_are_cumulative_and_monotonic(rows):
    s = [float(r["s_m"]) for r in rows]
    t = [float(r["t_ref_s"]) for r in rows]
    assert s[0] == 0.0 and t[0] == 0.0
    assert all(b >= a for a, b in zip(s, s[1:])), "s_m goes backwards"
    assert all(b >= a for a, b in zip(t, t[1:])), "t_ref_s goes backwards"
    assert s[-1] == pytest.approx(428.651364, abs=1e-5)
    assert t[-1] == pytest.approx(286.158823, abs=1e-5)


def test_speed_is_positive_everywhere(rows):
    """A zero reference speed would divide by zero in the time integration."""
    assert min(float(r["v_ref_mps"]) for r in rows) > 0.0


def test_obstacle_lies_on_segment_one(rows):
    """The Stop-Replan-Resume event has to be on the route, or it never fires."""
    obstacle = yaml.safe_load(
        (REPO / "sim" / "obstacles" / "obstacle.yaml").read_text())
    ox, oy = (float(v) for v in obstacle["position_world_xy_m"])
    seg1 = [r for r in rows if r["segment_id"] == "1"]
    d = min((float(r["x_m"]) - ox) ** 2 + (float(r["y_m"]) - oy) ** 2
            for r in seg1) ** 0.5
    assert d < 0.05, f"the obstacle is {d:.3f} m off the segment-1 centreline"


def test_window_is_an_exact_slice_of_the_full_mission(rows):
    """results/stop_replan_resume/mission_segment_window.csv must not drift.

    The offline geometry reproduction reads the window; the runtime reads the
    full file. If they ever disagree, the reproduced geometry stops describing
    the run it claims to describe.
    """
    with WINDOW_CSV.open() as f:
        window = list(csv.DictReader(f))
    seg1 = [r for r in rows if r["segment_id"] == "1"]
    starts = [i for i, r in enumerate(seg1) if r["s_m"] == window[0]["s_m"]]
    assert starts, "the window's first arc length is not in segment 1"
    i0 = starts[0]
    sliced = seg1[i0:i0 + len(window)]
    assert len(sliced) == len(window)
    for a, b in zip(sliced, window):
        for col in ("s_m", "x_m", "y_m", "yaw_rad",
                    "curvature_1_per_m", "v_ref_mps"):
            assert a[col] == b[col], f"window diverges from the mission at {col}"


def test_manifest_matches_the_data(rows):
    manifest = yaml.safe_load((BUNDLE / "manifest.yaml").read_text())
    assert manifest["samples"]["total"] == len(rows)
    per = manifest["samples"]["per_segment"]
    for seg_id, _typ, name, count in EXPECTED_SEGMENTS:
        assert per[f"{seg_id}_{name}"] == count


def test_manifest_does_not_call_derived_data_raw():
    text = (BUNDLE / "manifest.yaml").read_text().lower()
    assert "derived mission product" in text
    assert "not raw data" in text
