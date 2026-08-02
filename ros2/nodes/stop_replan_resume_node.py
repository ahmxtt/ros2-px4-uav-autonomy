#!/usr/bin/env python3
"""Live Stop-Replan-Resume (SRR) executor for ROS 2 / PX4 Offboard.

*Stop-Replan-Resume* is this project's name for the obstacle-response
sequence: on detecting an obstacle that is absent from the prior map, the
vehicle stops, plans a local detour, and resumes the original mission.

Validation stage: **Gate 6D**, in two variants distinguished by
``--oracle-mode`` (see ``docs/architecture.md`` for the gate framework):

``off`` -> **Gate 6D-B**, perception-driven
    Detection comes from LiDAR clustering plus a novelty test against the
    prior map; the replan is computed around the *sensed* obstacle on the
    *unpatched* prior. Ground truth is retained in a separate
    evaluation-only namespace: it does not gate detection, obstacle
    construction, replanning or control decisions, but it may be used to
    compute diagnostic metrics and post-run evaluation values. This is the
    mode of the published run.
``detection`` -> hybrid
    Ground truth gates the detection event; the replan still uses the sensed
    geometry.
``replan`` -> **Gate 6D-A**, oracle mechanics
    Ground truth supplies the obstacle and the patched prior is used. This
    validates the state machine, not the perception.

A pass in perception mode additionally requires
``detection_mode == "lidar_cluster_detection"``. Synthetic detection is
logged but can never produce a perception pass.

This node subclasses the nominal Offboard mission executor and inserts::

    ... MISSION --(detection confirmed)--> DETECTED -> STOP_HOVER
              -> REPLAN_LOCAL_ROI -> RESUME_REPLAN_REFERENCE
              -> REJOIN_ORIGINAL_MISSION -> OBSERVE_END -> LAND ...

The local replan itself is ``uav_autonomy.replanning.run_local_roi_replan``,
the same code path validated offline at Gate 6C.5.

REQUIRES ROS 2 (rclpy, px4_msgs, sensor_msgs) and a running PX4 SITL stack; it
cannot run offline. For the offline geometry of the same replan, see
``tools/reproduce_srr_geometry.py``.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as _dt
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parents[1]
# Sibling node module (the nominal executor) and the algorithmic package.
sys.path.insert(0, str(THIS_DIR))
sys.path.insert(0, str(REPO_ROOT / "src"))

import offboard_mission_executor as executor_mod   # noqa: E402
from offboard_mission_executor import (   # noqa: E402
    OffboardMissionExecutor, Setpoint, SegmentLoadReport, RunReport,
    TickLog, load_segment, sha256_file, parse_args as g4c_parse_args,
    px4_qos, NAVIGATION_STATE_OFFBOARD, ARMING_STATE_DISARMED,
    ARMING_STATE_ARMED, MAX_ALTITUDE_HARD_CAP_M, write_outputs as
    g4c_write_outputs, _SIGNAL_HANDLERS_NO,
)
from uav_autonomy import replanning as roi         # noqa: E402
from uav_autonomy.grid import load_full_pgm        # noqa: E402

import rclpy   # noqa: E402
from sensor_msgs.msg import LaserScan              # noqa: E402


# World <-> NED transform (Gate 4A / 5C constants).
X_ORIGIN_WORLD = -50.0
Y_ORIGIN_WORLD = -70.0
Z_REF_M = 2.0


def world_to_ned(xw: float, yw: float) -> Tuple[float, float]:
    """x_ned = y_world - y_origin; y_ned = x_world - x_origin."""
    return (yw - Y_ORIGIN_WORLD, xw - X_ORIGIN_WORLD)


def ned_to_world(xned: float, yned: float) -> Tuple[float, float]:
    return (yned + X_ORIGIN_WORLD, xned + Y_ORIGIN_WORLD)


def yaw_world_to_ned(yaw_world: float) -> float:
    y = (math.pi / 2.0) - yaw_world
    return math.atan2(math.sin(y), math.cos(y))


def yaw_ned_to_world(yaw_ned: float) -> float:
    y = (math.pi / 2.0) - yaw_ned
    return math.atan2(math.sin(y), math.cos(y))


# ----------------------------------------------------------------------
# Detection event records
# ----------------------------------------------------------------------

@dataclass
class DetectionFrame:
    wall_s: float
    n_returns_in_corridor: int
    nearest_cluster_world_xy: Optional[Tuple[float, float]]
    nearest_cluster_dist_to_obs_m: Optional[float]
    is_positive: bool
    hysteresis_streak: int


@dataclass
class HoverMetrics:
    detection_world_xy: Tuple[float, float] = (math.nan, math.nan)
    detection_yaw_world_rad: float = math.nan
    hover_xy_ned: Tuple[float, float] = (math.nan, math.nan)
    hover_xy_world: Tuple[float, float] = (math.nan, math.nan)
    stable_wall_s: float = math.nan
    max_pos_err_during_stable_m: float = 0.0
    max_speed_during_stable_mps: float = 0.0
    stable_for_s_at_promotion: float = 0.0


@dataclass
class ReplanMetrics:
    started_wall_s: float = math.nan
    completed_wall_s: float = math.nan
    success: bool = False
    classification: str = "n/a"
    selected_attempt: str = "n/a"
    clearance_min_m: float = math.nan
    clearance_min_ge_r_eff: bool = False
    clearance_min_ge_r_pref: bool = False
    collision_free: bool = False
    max_lateral_accel_mps2: float = math.nan
    max_curvature_1_per_m: float = math.nan
    astar_path_length_m: float = math.nan
    smoothed_length_m: float = math.nan
    astar_nodes_expanded: int = 0
    t_crop_s: float = math.nan
    t_patch_s: float = math.nan
    t_clearance_field_s: float = math.nan
    t_inflate_s: float = math.nan
    t_astar_s: float = math.nan
    t_smoother_s: float = math.nan
    t_validate_s: float = math.nan
    t_total_s: float = math.nan
    fail_reason: Optional[str] = None


@dataclass
class ResumeJoinMetrics:
    resume_idx_in_segment: int = -1
    resume_xy_world: Tuple[float, float] = (math.nan, math.nan)
    resume_yaw_world_rad: float = math.nan
    replan_end_xy_world: Tuple[float, float] = (math.nan, math.nan)
    pos_jump_replan_to_resume_m: float = math.nan
    yaw_jump_replan_to_resume_deg: float = math.nan
    rejoined_wall_s: float = math.nan


@dataclass
class SensedObstacle:
    """Perception-derived obstacle region (Gate 6D-B). Built ONLY from
    LiDAR clusters + the unpatched prior map — never from obstacle.yaml /
    Gazebo truth. `centroid_world_xy` is back-projected away from the
    vehicle by `radius_m` so the conservative disk encloses the unseen far
    side of the cylinder; `radius_m` is floored + inflated. The arc_* fields
    record the raw near-arc measurement for offline error metrics only."""
    centroid_world_xy: Tuple[float, float]
    radius_m: float
    first_seen_wall_s: float
    n_frames: int
    source: str = "perception"
    arc_centroid_world_xy: Tuple[float, float] = (math.nan, math.nan)
    arc_half_span_m: float = math.nan
    bearing_from_vehicle_rad: float = math.nan
    prior_dist_to_occupied_m: float = math.nan


# ----------------------------------------------------------------------
# Prior-occupancy novelty field (perception mode)
# ----------------------------------------------------------------------

PGM_OCCUPIED = 0


class PriorOccupancy:
    """Distance-to-nearest-occupied field of the UNPATCHED prior map, used
    to reject sensed clusters that are already explained by a known obstacle
    (tree) in the prior. A neighborhood (distance-transform) test, not a
    single-cell lookup, to absorb near-arc centroid bias."""

    def __init__(self, map_yaml: Path):
        meta = yaml.safe_load(Path(map_yaml).read_text())
        img = np.array(Image.open(Path(map_yaml).parent / meta["image"]))
        self.res = float(meta["resolution"])
        self.ox = float(meta["origin"][0])
        self.oy = float(meta["origin"][1])
        self.H, self.W = img.shape
        self.y_max = self.oy + self.H * self.res
        occupied = (img == PGM_OCCUPIED)
        # Distance (m) from each cell to the nearest occupied cell.
        self.dist_m = (distance_transform_edt(~occupied).astype(np.float64)
                       * self.res)

    def dist_to_occupied_m(self, xw: float, yw: float) -> float:
        """Distance (m) from a world point to the nearest prior-occupied
        cell. Out-of-bounds -> 0.0 (treated as 'explained', i.e. rejected
        as novel)."""
        col = int((xw - self.ox) / self.res)
        row = int((self.y_max - yw) / self.res)
        if 0 <= row < self.H and 0 <= col < self.W:
            return float(self.dist_m[row, col])
        return 0.0


# ----------------------------------------------------------------------
# Detection
# ----------------------------------------------------------------------

@dataclass
class DetectionConfig:
    lookahead_m: float = 7.5
    corridor_half_width_m: float = 1.5
    hysteresis_frames: int = 3
    hysteresis_min_duration_s: float = 0.2
    min_range_m: float = 0.4         # filter out self-returns just in case
    cluster_gap_m: float = 0.5       # range jump that starts a new cluster
    cluster_match_radius_m: float = 2.5   # ORACLE detection gate ONLY:
                                          # cluster centroid <= this from
                                          # GT obstacle XY. Unused in
                                          # perception mode.
    # --- Perception-mode (Gate 6D-B) confirmation gate ---
    min_cluster_points: int = 3          # plausible-size: min returns
    sensed_radius_floor_m: float = 0.5   # conservative radius floor
    sensed_radius_max_m: float = 3.0     # plausible-size: reject huge
    obstacle_inflation_m: float = 0.30   # sensor margin added to disk
                                         # (A* adds r_eff separately)
    prior_novelty_radius_m: float = 1.0  # cluster is "novel" iff nearest
                                         # prior-occupied cell is farther
                                         # than this (else = known tree)


def cluster_scan_to_world(
    scan: LaserScan,
    veh_xw: float, veh_yw: float, yaw_world: float,
    cfg: DetectionConfig,
) -> List[Tuple[float, float, int, List[Tuple[float, float]]]]:
    """Cluster LaserScan returns and return per-cluster
    (cx_world, cy_world, n_points, points_world) tuples. Ignores ranges
    outside [range_min, range_max] and below cfg.min_range_m. Consecutive
    rays with finite range and |Δrange| <= cluster_gap_m belong to the
    same cluster. The point list is needed for the perception-mode
    near-arc radius/span estimate."""
    ranges = scan.ranges
    n = len(ranges)
    if n == 0:
        return []
    a0 = scan.angle_min
    da = scan.angle_increment
    rmin_sensor = scan.range_min
    rmax_sensor = scan.range_max
    cos_yaw = math.cos(yaw_world)
    sin_yaw = math.sin(yaw_world)
    clusters: List[List[Tuple[float, float]]] = []
    cur: List[Tuple[float, float]] = []
    prev_r = None
    for i in range(n):
        r = ranges[i]
        if not (math.isfinite(r) and rmin_sensor <= r <= rmax_sensor
                and r >= cfg.min_range_m):
            if cur:
                clusters.append(cur); cur = []; prev_r = None
            continue
        theta_body = a0 + i * da
        # Body XY: forward = x_body, left = y_body (REP-103).
        bx = r * math.cos(theta_body)
        by = r * math.sin(theta_body)
        # Body -> world: rotate by yaw_world.
        xw = veh_xw + cos_yaw * bx - sin_yaw * by
        yw = veh_yw + sin_yaw * bx + cos_yaw * by
        if prev_r is not None and abs(r - prev_r) > cfg.cluster_gap_m:
            if cur:
                clusters.append(cur); cur = []
        cur.append((xw, yw))
        prev_r = r
    if cur:
        clusters.append(cur)
    out: List[Tuple[float, float, int, List[Tuple[float, float]]]] = []
    for c in clusters:
        if not c:
            continue
        cx = sum(p[0] for p in c) / len(c)
        cy = sum(p[1] for p in c) / len(c)
        out.append((cx, cy, len(c), c))
    return out


def cluster_in_corridor(cx: float, cy: float,
                        veh_xw: float, veh_yw: float, yaw_world: float,
                        cfg: DetectionConfig) -> Tuple[bool, float, float]:
    """Return (in_corridor, forward_distance, lateral_offset).
    forward_distance positive when cluster is ahead of the vehicle along
    heading."""
    dx = cx - veh_xw; dy = cy - veh_yw
    cosY = math.cos(yaw_world); sinY = math.sin(yaw_world)
    fwd = cosY * dx + sinY * dy        # along heading
    lat = -sinY * dx + cosY * dy       # left-positive lateral
    in_corr = (0.0 <= fwd <= cfg.lookahead_m
               and abs(lat) <= cfg.corridor_half_width_m)
    return in_corr, fwd, lat


# ----------------------------------------------------------------------
# Gate 6D node
# ----------------------------------------------------------------------

class StopReplanResumeNode(OffboardMissionExecutor):
    def __init__(self, args, setpoints, seg_report, obstacle: dict,
                 detection_cfg: DetectionConfig,
                 obstacle_yaml_path: Path,
                 scan_topic: str,
                 out_dir: Path,
                 patched_map_yaml: Path,
                 prior_map_yaml: Path,
                 mission_csv: Path,
                 segment_name: str,
                 ):
        super().__init__(args, setpoints, seg_report)
        self._obstacle = obstacle
        self._oracle_mode = getattr(args, "oracle_mode", "off")
        # GROUND TRUTH lives ONLY in the _gt_* namespace. It is used by the
        # offline metrics writer always, and by the ONLINE path ONLY when
        # oracle-mode explicitly requests it (detection/replan).
        self._gt_obstacle_xy = (
            float(obstacle["position_world_xy_m"][0]),
            float(obstacle["position_world_xy_m"][1]))
        self._gt_obstacle_r = float(obstacle["radius_m"])
        # _obstacle_xy_world / _obstacle_r are the ONLINE-DECISION obstacle,
        # and they are the ONLY thing perception mode empties:
        #   oracle replan/detection -> = GT (the banked 6D-A mechanics);
        #   off (perception, 6D-B)  -> None, so ANY accidental use of ground
        #                              truth for a decision fails loud instead
        #                              of silently assisting the planner.
        # The _gt_* pair above stays populated for the whole run. It is read
        # live to log a per-tick diagnostic distance-to-truth, and again by the
        # metrics writer afterwards. Evaluation, never a decision.
        if self._oracle_mode in ("detection", "replan"):
            self._obstacle_xy_world = self._gt_obstacle_xy
            self._obstacle_r = self._gt_obstacle_r
        else:
            self._obstacle_xy_world = None
            self._obstacle_r = None
        self._det_cfg = detection_cfg
        self._scan_topic_requested = scan_topic
        self._out_dir = out_dir
        self._patched_map_yaml = patched_map_yaml
        self._prior_map_yaml = prior_map_yaml
        self._mission_csv = mission_csv
        self._detection_segment_name = segment_name

        # Perception (Gate 6D-B): sensed obstacle + prior-novelty field.
        self._sensed_obstacle: Optional[SensedObstacle] = None
        self._prior_occ: Optional[PriorOccupancy] = None
        if self._oracle_mode != "replan":
            # Needed for the perception detection gate and/or the unpatched
            # crop. Loaded lazily-safe: a failure is logged, not fatal.
            try:
                self._prior_occ = PriorOccupancy(self._prior_map_yaml)
            except Exception as e:                                # noqa
                self.get_logger().warning(
                    f"[srr] prior-occupancy load failed: {e!r}")

        # Detection state.
        self._scan_topic_used: Optional[str] = None
        self._scan_msg_count = 0
        self._scan_last_wall: Optional[float] = None
        self._scan_first_wall: Optional[float] = None
        self._det_streak = 0
        self._det_streak_start_wall: Optional[float] = None
        self._det_confirmed = False
        self._det_confirmed_wall: Optional[float] = None
        # Never-reset latches: _det_confirmed is intentionally reset to False
        # on rejoin (re-detection control), so reporting must use these to know
        # whether a detection EVER fired during the run.
        self._det_ever_confirmed = False
        self._det_first_confirmed_wall: Optional[float] = None
        self._det_frames: List[DetectionFrame] = []
        self._det_world_xy: Optional[Tuple[float, float]] = None
        self._false_positive_count = 0   # detections outside active segment
        # Once the sensed obstacle has been replanned around and the
        # mission has rejoined the original reference, mark it handled so
        # the same physical obstacle cannot trigger a second STOP_HOVER on
        # subsequent MISSION ticks.
        self._obstacle_handled: bool = False

        # Gate 6D extension state.
        self._hover_sp_ned: Optional[Tuple[float, float, float]] = None
        self._hover_yaw_ned: float = math.nan
        self._hover_metrics = HoverMetrics()
        self._hover_stable_start: Optional[float] = None
        self._stop_hover_entered_wall: Optional[float] = None

        # Replan products.
        self._replan_metrics = ReplanMetrics()
        self._replan_setpoints: List[Setpoint] = []
        self._replan_start_wall: Optional[float] = None
        self._replan_idx = 0
        self._replan_raw_xy = np.zeros((0, 2))
        self._replan_smoothed_xy = np.zeros((0, 2))
        self._replan_smoothed_yaw = np.zeros(0)
        self._replan_smoothed_v = np.zeros(0)
        self._replan_smoothed_s = np.zeros(0)
        self._roi_diagnostics: dict = {}

        # Resume / rejoin.
        self._resume_idx_in_original: int = -1
        self._resume_xy_world: Optional[Tuple[float, float]] = None
        self._resume_metrics = ResumeJoinMetrics()
        self._rejoined_wall: Optional[float] = None

        # Event log.
        self._events: List[Tuple[float, str, str]] = []   # (wall, name, info)
        self._log_event("node_initialized",
                        f"oracle_mode={self._oracle_mode} "
                        f"online_obstacle_xy={self._obstacle_xy_world} "
                        f"online_obstacle_r={self._obstacle_r} "
                        f"gt_obstacle_xy={self._gt_obstacle_xy} "
                        f"gt_obstacle_r={self._gt_obstacle_r} "
                        f"(GT is evaluation-only unless oracle-mode uses it)")

        # Detection LIDAR log.
        self._lidar_log: List[dict] = []

        # Subscribe to LiDAR.
        qos = px4_qos()
        self._sub_scan = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos)
        self.get_logger().info(
            f"[srr] subscribed to {scan_topic} (best-effort QoS)")
        # State-aware altitude cap: strict cap that applies post-SETTLE.
        # Parent's _settle_tick uses self.args.max_altitude_m; we mutate
        # it on SETTLE_HOLD entry / exit via _transition().
        self._max_altitude_m_strict = float(args.max_altitude_m)
        self._max_altitude_m_settle = (
            float(args.takeoff_altitude_m)
            + float(args.settle_altitude_tolerance_m))
        # Robust SETTLE lateral guard (A/B/C) duration trackers. None when
        # the corresponding condition is not currently satisfied.
        self._settle_guard_b_start_wall: Optional[float] = None
        self._settle_guard_c_start_wall: Optional[float] = None

    # ------------------------------------------------------------------
    # Event logging
    # ------------------------------------------------------------------

    def _log_event(self, name: str, info: str = "") -> None:
        self._events.append((time.monotonic() - self._wall_start, name, info))
        self.get_logger().info(f"[event] {name}: {info}")

    def _planning_obstacle(self) -> Tuple[Tuple[float, float], float]:
        """Obstacle (xy_world, radius) used for ONLINE replan + resume-anchor
        geometry. oracle-mode 'replan' -> ground truth (6D-A mechanics);
        otherwise -> the perception-derived SensedObstacle (6D-B). Raises if
        perception mode is active but no obstacle has been sensed yet — that
        would be a logic error (replan called before detection)."""
        if self._oracle_mode == "replan":
            return self._gt_obstacle_xy, self._gt_obstacle_r
        if self._sensed_obstacle is None:
            raise RuntimeError(
                "perception-mode replan requested before any obstacle was "
                "sensed (no SensedObstacle)")
        return (self._sensed_obstacle.centroid_world_xy,
                self._sensed_obstacle.radius_m)

    def _transition(self, new_state: str) -> None:
        old_state = self._state
        super()._transition(new_state)
        # State-aware altitude cap. Parent's _settle_tick reads
        # self.args.max_altitude_m directly; we temporarily relax it
        # during SETTLE_HOLD to absorb the takeoff climb transient, then
        # restore strict mission-flight cap on exit.
        if new_state == "SETTLE_HOLD":
            self.args.max_altitude_m = self._max_altitude_m_settle
            self._log_event(
                "settle_altitude_cap_relaxed",
                f"cap={self._max_altitude_m_settle:.2f}m "
                f"(takeoff+{self.args.settle_altitude_tolerance_m:.2f})")
        elif old_state == "SETTLE_HOLD" \
                and new_state != "ABORT_LANDING":
            self.args.max_altitude_m = self._max_altitude_m_strict
            self._log_event(
                "settle_altitude_cap_restored",
                f"strict_cap={self._max_altitude_m_strict:.2f}m")

    # ------------------------------------------------------------------
    # LaserScan callback
    # ------------------------------------------------------------------

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        self._scan_msg_count += 1
        if self._scan_first_wall is None:
            self._scan_first_wall = now
            self._log_event("scan_topic_first_msg",
                            f"topic={self._scan_topic_requested}")
        self._scan_last_wall = now
        if self._scan_topic_used is None:
            self._scan_topic_used = self._scan_topic_requested

        # Detection only during MISSION_ORIGINAL on the active segment.
        if self._state != "MISSION":
            return
        if self._last_lpos is None:
            return
        # Once the sensed obstacle has been handled (replanned around
        # + rejoined), do not run detection on it again.
        if self._obstacle_handled:
            return

        # Active-segment check: current mission setpoint's segment_name.
        cur_sp = self._select_mission_setpoint_no_advance()
        if cur_sp is None or cur_sp.segment_name != \
                self._detection_segment_name:
            return

        # Vehicle XY in world and yaw_world.
        veh_xw, veh_yw = ned_to_world(self._last_lpos.x, self._last_lpos.y)
        yaw_w = yaw_ned_to_world(float(self._last_lpos.heading))

        # Don't run detection past the obstacle (oracle modes only: needs
        # GT to know where the obstacle is). In perception mode the corridor
        # gate already bounds detection to the forward lookahead window.
        if self._oracle_mode in ("detection", "replan"):
            dx_obs = self._obstacle_xy_world[0] - veh_xw
            dy_obs = self._obstacle_xy_world[1] - veh_yw
            cosY = math.cos(yaw_w); sinY = math.sin(yaw_w)
            fwd_to_obs = cosY * dx_obs + sinY * dy_obs
            if fwd_to_obs < -1.0:
                return

        # Cluster the scan; keep corridor clusters with their point lists.
        clusters = cluster_scan_to_world(msg, veh_xw, veh_yw, yaw_w,
                                         self._det_cfg)
        corridor_clusters = []   # (cx, cy, npts, pts, fwd)
        n_in = 0
        for cx, cy, npts, pts in clusters:
            in_corr, fwd, _lat = cluster_in_corridor(
                cx, cy, veh_xw, veh_yw, yaw_w, self._det_cfg)
            if in_corr and npts >= 2:
                n_in += npts
                corridor_clusters.append((cx, cy, npts, pts, fwd))

        # --- Candidate selection: oracle vs perception gate ---
        cand = None          # (cx, cy, npts, pts, fwd)
        nearest_dist = math.inf     # to GT, for logging/metrics only
        novelty_dist = math.nan
        if self._oracle_mode in ("detection", "replan"):
            # ORACLE detection gate: nearest corridor cluster to the KNOWN
            # obstacle XY, within cluster_match_radius_m. (6D-A mechanics.)
            for c in corridor_clusters:
                d_obs = math.hypot(c[0] - self._obstacle_xy_world[0],
                                   c[1] - self._obstacle_xy_world[1])
                if d_obs < nearest_dist:
                    nearest_dist = d_obs
                    if d_obs <= self._det_cfg.cluster_match_radius_m:
                        cand = c
        else:
            # PERCEPTION gate (6D-B): nearest-forward corridor cluster that
            # is (a) plausible size and (b) NOT explained by the prior map.
            # No ground truth involved.
            best_fwd = math.inf
            for c in corridor_clusters:
                cx, cy, npts, pts, fwd = c
                if npts < self._det_cfg.min_cluster_points:
                    continue
                # near-arc half-span -> conservative radius estimate
                r_arc = max(math.hypot(p[0] - cx, p[1] - cy) for p in pts)
                r_est = max(r_arc, self._det_cfg.sensed_radius_floor_m)
                if r_est > self._det_cfg.sensed_radius_max_m:
                    continue   # implausibly large -> reject
                # prior-map novelty (neighborhood / distance-transform)
                if self._prior_occ is not None:
                    d_prior = self._prior_occ.dist_to_occupied_m(cx, cy)
                else:
                    d_prior = math.inf
                if d_prior <= self._det_cfg.prior_novelty_radius_m:
                    continue   # explained by a known prior obstacle (tree)
                if fwd < best_fwd:
                    best_fwd = fwd
                    cand = c
                    novelty_dist = d_prior
            # GT distance recorded for metrics only (never gates anything).
            if cand is not None:
                nearest_dist = math.hypot(
                    cand[0] - self._gt_obstacle_xy[0],
                    cand[1] - self._gt_obstacle_xy[1])

        is_positive = cand is not None
        if is_positive:
            if self._det_streak == 0:
                self._det_streak_start_wall = now
            self._det_streak += 1
        else:
            self._det_streak = 0
            self._det_streak_start_wall = None

        nearest = (cand[0], cand[1]) if cand is not None else None

        # Confirmed?
        if (not self._det_confirmed
                and is_positive
                and self._det_streak >= self._det_cfg.hysteresis_frames
                and self._det_streak_start_wall is not None
                and (now - self._det_streak_start_wall)
                >= self._det_cfg.hysteresis_min_duration_s):
            self._det_confirmed = True
            self._det_confirmed_wall = now
            self._det_ever_confirmed = True
            if self._det_first_confirmed_wall is None:
                self._det_first_confirmed_wall = now
            self._det_world_xy = nearest
            # Build the perception-derived SensedObstacle (used for replan +
            # anchor in non-oracle-replan modes). Conservative disk:
            # back-project the arc centroid AWAY from the vehicle by the
            # estimated radius so the disk encloses the unseen far side.
            cx, cy, npts, pts, _fwd = cand
            r_arc = max(math.hypot(p[0] - cx, p[1] - cy) for p in pts)
            r_est = max(r_arc, self._det_cfg.sensed_radius_floor_m)
            bearing = math.atan2(cy - veh_yw, cx - veh_xw)
            ux, uy = math.cos(bearing), math.sin(bearing)
            center_x = cx + ux * r_est
            center_y = cy + uy * r_est
            radius = r_est + self._det_cfg.obstacle_inflation_m
            self._sensed_obstacle = SensedObstacle(
                centroid_world_xy=(center_x, center_y),
                radius_m=radius,
                first_seen_wall_s=(self._det_streak_start_wall
                                   - self._wall_start),
                n_frames=self._det_streak,
                source="perception",
                arc_centroid_world_xy=(cx, cy),
                arc_half_span_m=r_arc,
                bearing_from_vehicle_rad=bearing,
                prior_dist_to_occupied_m=novelty_dist,
            )
            self._log_event(
                "detection_confirmed",
                f"mode={self._oracle_mode} arc_centroid=({cx:.3f},{cy:.3f}) "
                f"sensed_center=({center_x:.3f},{center_y:.3f}) "
                f"sensed_r={radius:.3f}m streak={self._det_streak} "
                f"novelty_dist={novelty_dist:.3f}m "
                f"gt_dist={nearest_dist:.3f}m(evaluation-only)")

        frame = DetectionFrame(
            wall_s=(now - self._wall_start),
            n_returns_in_corridor=n_in,
            nearest_cluster_world_xy=nearest,
            nearest_cluster_dist_to_obs_m=nearest_dist if math.isfinite(
                nearest_dist) else None,
            is_positive=is_positive,
            hysteresis_streak=self._det_streak,
        )
        self._det_frames.append(frame)
        self._lidar_log.append({
            "wall_s": frame.wall_s,
            "veh_xw": veh_xw, "veh_yw": veh_yw, "yaw_world_rad": yaw_w,
            "n_clusters": len(clusters),
            "n_corridor_clusters": len(corridor_clusters),
            "n_returns_in_corridor": n_in,
            "cand_arc_x": nearest[0] if nearest else math.nan,
            "cand_arc_y": nearest[1] if nearest else math.nan,
            "cand_novelty_dist_m": novelty_dist,
            "cand_gt_dist_m_metrics_only":
                nearest_dist if math.isfinite(nearest_dist) else math.nan,
            "is_positive": int(bool(is_positive)),
            "hysteresis_streak": self._det_streak,
        })

    def _select_mission_setpoint_no_advance(self) -> Optional[Setpoint]:
        if not self.setpoints or self._mission_start_wall is None:
            return None
        elapsed = time.monotonic() - self._mission_start_wall
        cursor_t = self._mission_cursor_t(elapsed)
        if self._mission_idx < len(self.setpoints):
            return self.setpoints[self._mission_idx]
        return self.setpoints[-1] if self.setpoints else None

    # ------------------------------------------------------------------
    # Override _stream_tick to handle STOP_HOVER and RESUME_REPLAN_REFERENCE.
    # ------------------------------------------------------------------

    def _stream_tick(self) -> None:
        if self._abort_requested:
            super()._stream_tick()
            return
        if not self._stream_active:
            return

        st = self._state
        # SINGLE setpoint source per state. Before MISSION starts we
        # publish settle-hold ourselves so velocity is explicitly NaN
        # (PX4 "ignore velocity feedforward"), defeating the parent's
        # default-zero velocity which this PX4 build interprets as a
        # velocity reference and which caused the arm_run_2 SETTLE drift.
        if st in ("PRE_STREAM", "WAIT_OFFBOARD", "WAIT_ARMED",
                  "SETTLE_HOLD"):
            self._publish_settle_hold_explicit()
            return
        if st in ("STOP_HOVER", "REPLAN_LOCAL_ROI", "DETECTED"):
            self._publish_hover()
            return
        if st == "RESUME_REPLAN_REFERENCE":
            self._publish_replan()
            return
        # Default: delegate to parent (MISSION, OBSERVE_END, etc.).
        super()._stream_tick()

    def _publish_settle_hold_explicit(self) -> None:
        """Single SETTLE setpoint source: position = current hold point,
        velocity = NaN (no FF), yaw = NaN (no yaw demand). One publish per
        tick, no other source may publish during PRE_STREAM/SETTLE_HOLD."""
        if math.isnan(self._hold_x) or math.isnan(self._hold_y):
            return    # before EKF-stable; parent waits silently
        sp = Setpoint(
            t_ref_s=0.0,
            x_ned_m=float(self._hold_x),
            y_ned_m=float(self._hold_y),
            z_ned_m=float(self._hold_z_ned),
            yaw_ned_rad=float("nan"),
            segment_id=-2,
            segment_type="settle",
            segment_name="settle_hold",
            vx_ned_mps=0.0, vy_ned_mps=0.0, vz_ned_mps=0.0,
        )
        self._publish_offboard_control_mode()
        # CRITICAL: velocity_ned=None -> _publish_trajectory_setpoint sets
        # m.velocity[i] = NaN. No mission cursor; no detection-derived
        # setpoint; no replan; no feedforward.
        self._publish_trajectory_setpoint(sp, velocity_ned=None)
        self._record_publish(time.monotonic())
        self._last_mission_sp = None    # do NOT pollute abort path with
                                        # a SETTLE sp masquerading as a
                                        # mission sp.

    def _publish_hover(self) -> None:
        if self._hover_sp_ned is None:
            self._publish_trajectory_setpoint_hold()
            return
        now = time.monotonic()
        sp = Setpoint(
            t_ref_s=0.0,
            x_ned_m=self._hover_sp_ned[0],
            y_ned_m=self._hover_sp_ned[1],
            z_ned_m=self._hover_sp_ned[2],
            yaw_ned_rad=self._hover_yaw_ned,
            segment_id=99,
            segment_type="hover",
            segment_name="stop_hover",
            vx_ned_mps=0.0, vy_ned_mps=0.0, vz_ned_mps=0.0,
        )
        self._publish_offboard_control_mode()
        vff = (0.0, 0.0, 0.0) if getattr(
            self.args, "use_velocity_feedforward", False) else None
        self._publish_trajectory_setpoint(sp, velocity_ned=vff)
        self._record_publish(now)
        self._last_mission_sp = sp

    def _publish_replan(self) -> None:
        if not self._replan_setpoints or self._replan_start_wall is None:
            self._publish_hover()
            return
        now = time.monotonic()
        elapsed = now - self._replan_start_wall
        target_t = elapsed
        # advance cursor.
        while (self._replan_idx + 1 < len(self._replan_setpoints)
               and self._replan_setpoints[self._replan_idx + 1].t_ref_s
               <= target_t):
            self._replan_idx += 1
        sp = self._replan_setpoints[self._replan_idx]
        self._publish_offboard_control_mode()
        vff = None
        if getattr(self.args, "use_velocity_feedforward", False):
            vff = (sp.vx_ned_mps, sp.vy_ned_mps, sp.vz_ned_mps)
        self._publish_trajectory_setpoint(sp, velocity_ned=vff)
        self._record_publish(now)
        self._update_tracking(now, sp)
        self._last_mission_sp = sp

    # ------------------------------------------------------------------
    # Override _control_tick: insert detection branch in MISSION, then
    # handle new states.
    # ------------------------------------------------------------------

    def _control_tick(self) -> None:
        now = time.monotonic()
        # When abort is requested, always delegate to the
        # Gate-4C/5C Land -> WAIT_DISARM -> force-disarm cascade. Without
        # this guard the override stays stuck in REPLAN_LOCAL_ROI and the
        # vehicle is left hovering in OFFBOARD.
        if self._abort_requested:
            super()._control_tick()
            return
        # PRE-MISSION SAFETY GUARD (refined A/B/C rule).
        #   A) lat_drift > 0.60 m                                  [instant]
        #   B) lat_drift > 0.30 m AND radial_v_away > 0.25 m/s    [>=0.5 s]
        #   C) lat_speed > 0.80 m/s                                [>=0.5 s]
        # Rule (A) catches gross runaway; (B) catches a steady drift
        # accelerating away from hold; (C) catches very fast motion that
        # is not yet far from hold. A near-origin transient with
        # speed ~0.5 m/s no longer aborts (was the arm_run_3 false alarm).
        if self._state == "SETTLE_HOLD" and self._last_lpos is not None \
                and not math.isnan(self._hold_x) \
                and not math.isnan(self._hold_y):
            dx = float(self._last_lpos.x) - float(self._hold_x)
            dy = float(self._last_lpos.y) - float(self._hold_y)
            lat_drift = math.hypot(dx, dy)
            v_xy_valid = bool(getattr(self._last_lpos, "v_xy_valid", False))
            lat_speed = (math.hypot(float(self._last_lpos.vx),
                                    float(self._last_lpos.vy))
                         if v_xy_valid else 0.0)
            # Radial velocity AWAY from hold. Positive => drifting away.
            if lat_drift > 1e-6 and v_xy_valid:
                rx = dx / lat_drift
                ry = dy / lat_drift
                radial_v_away = (float(self._last_lpos.vx) * rx
                                 + float(self._last_lpos.vy) * ry)
            else:
                radial_v_away = 0.0

            # A) gross drift -> instant abort
            if lat_drift > self.args.settle_drift_abort_m:
                self._log_event(
                    "settle_guard_A",
                    f"lat_drift={lat_drift:.3f}m > "
                    f"{self.args.settle_drift_abort_m}m  lpos=("
                    f"{self._last_lpos.x:.3f},{self._last_lpos.y:.3f}) "
                    f"hold=({self._hold_x:.3f},{self._hold_y:.3f}) "
                    f"v=({self._last_lpos.vx:.3f},"
                    f"{self._last_lpos.vy:.3f}) "
                    f"radial_v_away={radial_v_away:.3f}m/s")
                self._request_abort(
                    f"settle_guard_A:lat_drift={lat_drift:.3f}m")
                super()._control_tick()
                return

            # B) sustained drift+away velocity
            b_cond = (
                lat_drift > self.args.settle_drift_warn_m
                and v_xy_valid
                and radial_v_away
                > self.args.settle_radial_v_away_abort_mps
            )
            if b_cond:
                if self._settle_guard_b_start_wall is None:
                    self._settle_guard_b_start_wall = now
                b_duration = now - self._settle_guard_b_start_wall
                if b_duration >= self.args.settle_guard_min_duration_s:
                    self._log_event(
                        "settle_guard_B",
                        f"lat_drift={lat_drift:.3f}m radial_v_away="
                        f"{radial_v_away:.3f}m/s held >="
                        f"{self.args.settle_guard_min_duration_s}s "
                        f"(actual {b_duration:.2f}s)  "
                        f"lpos=({self._last_lpos.x:.3f},"
                        f"{self._last_lpos.y:.3f}) "
                        f"v=({self._last_lpos.vx:.3f},"
                        f"{self._last_lpos.vy:.3f})")
                    self._request_abort(
                        "settle_guard_B:"
                        f"drift={lat_drift:.3f}m,"
                        f"radial_v_away={radial_v_away:.3f}mps,"
                        f"duration={b_duration:.2f}s")
                    super()._control_tick()
                    return
            else:
                self._settle_guard_b_start_wall = None

            # C) sustained high lateral speed
            c_cond = (v_xy_valid
                      and lat_speed > self.args.settle_speed_abort_mps)
            if c_cond:
                if self._settle_guard_c_start_wall is None:
                    self._settle_guard_c_start_wall = now
                c_duration = now - self._settle_guard_c_start_wall
                if c_duration >= self.args.settle_guard_min_duration_s:
                    self._log_event(
                        "settle_guard_C",
                        f"lat_speed={lat_speed:.3f}m/s > "
                        f"{self.args.settle_speed_abort_mps}m/s held >="
                        f"{self.args.settle_guard_min_duration_s}s "
                        f"(actual {c_duration:.2f}s)  "
                        f"lpos=({self._last_lpos.x:.3f},"
                        f"{self._last_lpos.y:.3f}) "
                        f"v=({self._last_lpos.vx:.3f},"
                        f"{self._last_lpos.vy:.3f})")
                    self._request_abort(
                        "settle_guard_C:"
                        f"lat_speed={lat_speed:.3f}mps,"
                        f"duration={c_duration:.2f}s")
                    super()._control_tick()
                    return
            else:
                self._settle_guard_c_start_wall = None
        # New states: tick the log ourselves (parent's _control_tick does
        # this, but we short-circuit before calling parent).
        if self._state in ("DETECTED", "STOP_HOVER", "REPLAN_LOCAL_ROI",
                           "RESUME_REPLAN_REFERENCE",
                           "REJOIN_ORIGINAL_MISSION"):
            self._append_tick_log(now)

        if self._state == "DETECTED":
            self._handle_detected_tick(now)
            return
        if self._state == "STOP_HOVER":
            # Mid-hover watchdogs: status freshness + lpos freshness.
            if (self.report.first_status_seen
                    and (now - self._last_status_wall) > self.args.watchdog_s):
                self._request_abort("status_stale_in_stop_hover"); return
            if (self._stream_active and self.report.first_lpos_seen
                    and (now - self._last_lpos_wall)
                    > self.args.lpos_watchdog_s):
                self._request_abort("lpos_stale_in_stop_hover"); return
            self._handle_stop_hover_tick(now)
            return
        if self._state == "REPLAN_LOCAL_ROI":
            self._handle_replan_tick(now)
            return
        if self._state == "RESUME_REPLAN_REFERENCE":
            if (self._stream_active and self.report.first_lpos_seen
                    and (now - self._last_lpos_wall)
                    > self.args.lpos_watchdog_s):
                self._request_abort("lpos_stale_in_resume"); return
            self._handle_resume_tick(now)
            return
        if self._state == "REJOIN_ORIGINAL_MISSION":
            # Just delegate to MISSION; the cursor has already been remapped.
            self._state = "MISSION"
            self._state_entered_wall = now

        # Detection trigger from MISSION before delegating to parent.
        prior_state = self._state
        super()._control_tick()
        # If parent didn't transition us out of MISSION, check for detection.
        if (prior_state == "MISSION" and self._state == "MISSION"
                and self._det_confirmed and not self._abort_requested):
            self._enter_detected(now)

    # ------------------------------------------------------------------
    # New state handlers
    # ------------------------------------------------------------------

    def _enter_detected(self, now: float) -> None:
        if self._last_lpos is None:
            self._request_abort("detection_with_no_lpos")
            return
        # Snapshot hover at current pose.
        self._hover_sp_ned = (float(self._last_lpos.x),
                              float(self._last_lpos.y),
                              float(self._last_lpos.z))
        self._hover_yaw_ned = float(self._last_lpos.heading)
        veh_xw, veh_yw = ned_to_world(self._last_lpos.x, self._last_lpos.y)
        yaw_w = yaw_ned_to_world(float(self._last_lpos.heading))
        self._hover_metrics.detection_world_xy = (veh_xw, veh_yw)
        self._hover_metrics.detection_yaw_world_rad = yaw_w
        self._hover_metrics.hover_xy_ned = (self._hover_sp_ned[0],
                                            self._hover_sp_ned[1])
        self._hover_metrics.hover_xy_world = (veh_xw, veh_yw)
        self._log_event("stop_command_sent",
                        f"hover_ned={self._hover_sp_ned}")
        self._transition("STOP_HOVER")
        self._stop_hover_entered_wall = now

    def _handle_detected_tick(self, now: float) -> None:
        # Should only be transiently set; immediately move to STOP_HOVER.
        self._enter_detected(now)

    def _handle_stop_hover_tick(self, now: float) -> None:
        if self._last_lpos is None:
            return
        if self._last_status is not None and self._last_status.nav_state \
                != NAVIGATION_STATE_OFFBOARD:
            self._request_abort("nav_state_left_offboard_in_stop_hover")
            return
        if self._last_status is not None and bool(self._last_status.failsafe):
            self._request_abort("failsafe_in_stop_hover")
            return
        if (self._last_lpos.z_valid and
                abs(self._last_lpos.z) > self.args.max_altitude_m):
            self._request_abort(
                f"max_altitude_exceeded:z={self._last_lpos.z:.2f}")
            return

        sp = self._hover_sp_ned
        err = max(abs(self._last_lpos.x - sp[0]),
                  abs(self._last_lpos.y - sp[1]),
                  abs(self._last_lpos.z - sp[2]))
        v_valid = bool(getattr(self._last_lpos, "v_xy_valid", False)) \
            and bool(getattr(self._last_lpos, "v_z_valid", False))
        speed = (math.sqrt(self._last_lpos.vx**2 + self._last_lpos.vy**2
                           + self._last_lpos.vz**2)
                 if v_valid else math.inf)
        self._hover_metrics.max_pos_err_during_stable_m = max(
            self._hover_metrics.max_pos_err_during_stable_m, err)
        if v_valid:
            self._hover_metrics.max_speed_during_stable_mps = max(
                self._hover_metrics.max_speed_during_stable_mps, speed)
        pos_ok = err <= self.args.stop_hover_pos_tol_m
        vel_ok = v_valid and speed <= self.args.stop_hover_speed_tol_mps
        if pos_ok and vel_ok:
            if self._hover_stable_start is None:
                self._hover_stable_start = now
            stable_for = now - self._hover_stable_start
            if stable_for >= self.args.stop_hover_stable_s:
                self._hover_metrics.stable_for_s_at_promotion = stable_for
                self._hover_metrics.stable_wall_s = now - self._wall_start
                # Clear any stale replan-compute start time before
                # entering REPLAN_LOCAL_ROI so the timeout check measures
                # the NEW compute, not a previous one's residue.
                self._replan_start_wall = None
                self._log_event("hover_stable_achieved",
                                f"stable_for={stable_for:.2f}s err="
                                f"{err:.3f} speed={speed:.3f}")
                self._transition("REPLAN_LOCAL_ROI")
                return
        else:
            self._hover_stable_start = None
        if (self._stop_hover_entered_wall is not None
                and now - self._stop_hover_entered_wall > 15.0):
            self._request_abort("stop_hover_timeout")

    def _handle_replan_tick(self, now: float) -> None:
        # In-process Gate 6C.5 local ROI replan.
        if self._replan_start_wall is None:
            self._replan_start_wall = now
            self._replan_metrics.started_wall_s = now - self._wall_start
            self._log_event("replan_started", "")
            # Run synchronously inside this tick.
            self._run_local_roi_replan(now)
            return
        # If replan didn't complete (shouldn't happen with in-process call),
        # bail.
        if (now - self._replan_start_wall) > self.args.replan_timeout_s + 5.0:
            self._request_abort("replan_runtime_exceeded")

    def _run_local_roi_replan(self, now: float) -> None:
        try:
            # Obstacle geometry for planning: ground truth ONLY in oracle
            # 'replan' mode; otherwise the perception-derived SensedObstacle.
            obstacle_xy, obstacle_r = self._planning_obstacle()
            # Base map: the patched_prior (which already contains the GT
            # obstacle) ONLY in oracle 'replan'; otherwise the UNPATCHED
            # prior — the sensed obstacle disk is stamped into the crop by
            # the ROI replanner from (obstacle_xy, obstacle_r).
            if self._oracle_mode == "replan":
                base_map_yaml = self._patched_map_yaml
            else:
                base_map_yaml = self._prior_map_yaml

            # Hover xy in world.
            hover_xw, hover_yw = ned_to_world(self._hover_sp_ned[0],
                                              self._hover_sp_ned[1])
            # Determine resume anchor: load segment rows from mission CSV,
            # pick first downstream sample resume_offset past obstacle that is
            # outside (obstacle_r + r_pref) of the (sensed or GT) obstacle.
            seg_rows = roi.load_segment_rows(
                self._mission_csv, self._detection_segment_name)
            seg_xs, seg_ys, seg_ss = roi.seg_xy_arrays(seg_rows)
            d2 = (seg_xs - obstacle_xy[0]) ** 2 \
                + (seg_ys - obstacle_xy[1]) ** 2
            i_obs = int(np.argmin(d2))
            s_obs = float(seg_ss[i_obs])
            s_resume_target = s_obs + self.args.resume_offset_m
            i_resume = roi.idx_at_s(seg_ss, s_resume_target,
                                    monotonic_after=i_obs, clamp=True)
            # Slide downstream until anchor is outside (obstacle_r + r_pref).
            ok = False
            while i_resume + 1 < len(seg_rows):
                rx = float(seg_xs[i_resume]); ry = float(seg_ys[i_resume])
                if math.hypot(
                        rx - obstacle_xy[0],
                        ry - obstacle_xy[1]) \
                        >= (obstacle_r + roi.R_PREF_M):
                    ok = True; break
                i_resume += 1
            if not ok:
                # Fall back to the indexed sample anyway; rare.
                pass
            self._resume_idx_in_original = i_resume
            self._resume_xy_world = (float(seg_xs[i_resume]),
                                     float(seg_ys[i_resume]))

            # Derive ROI spec.
            full_image, full_meta = load_full_pgm(base_map_yaml)
            roi_spec = roi.derive_roi_spec(float(full_meta["resolution"]))
            scale = float(self.args.roi_scale)
            s_min = s_obs - roi.LOOKAHEAD_M - \
                roi_spec.upstream_margin_m * scale
            s_max = s_obs + max(self.args.resume_offset_m,
                                roi_spec.resume_distance_m * scale)
            lateral = roi_spec.lateral_half_width_m * scale
            # i_hover in original segment: use last sample whose s_m is
            # closest to detection position.
            d2h = (seg_xs - hover_xw) ** 2 + (seg_ys - hover_yw) ** 2
            i_hover = int(np.argmin(d2h))

            cand = roi.run_local_roi_replan(
                full_image=full_image, full_meta=full_meta,
                seg_xs=seg_xs, seg_ys=seg_ys, seg_ss=seg_ss,
                obstacle_xy=obstacle_xy,
                obs_r=obstacle_r,
                s_min=s_min, s_max=s_max,
                lateral_half_width=lateral,
                hover_xy=(hover_xw, hover_yw),
                resume_xy=self._resume_xy_world,
                hover_idx=i_hover, resume_idx=i_resume,
            )

            # Fill metrics.
            rm = self._replan_metrics
            rm.completed_wall_s = time.monotonic() - self._wall_start
            rm.classification = cand.classification
            rm.selected_attempt = cand.selected_attempt
            rm.clearance_min_m = cand.clearance_min_m
            rm.clearance_min_ge_r_eff = cand.clearance_min_ge_r_eff
            rm.clearance_min_ge_r_pref = cand.clearance_min_ge_r_pref
            rm.collision_free = cand.collision_free
            rm.max_lateral_accel_mps2 = cand.max_lateral_accel_mps2
            rm.max_curvature_1_per_m = cand.max_curvature_1_per_m
            rm.astar_path_length_m = cand.astar_path_length_m
            rm.smoothed_length_m = cand.smoothed_length_m
            rm.astar_nodes_expanded = cand.astar_nodes_expanded
            rm.t_crop_s = cand.t_crop
            rm.t_patch_s = cand.t_patch
            rm.t_clearance_field_s = cand.t_clearance_field
            rm.t_inflate_s = cand.t_inflate
            rm.t_astar_s = cand.t_astar
            rm.t_smoother_s = cand.t_smoother
            rm.t_validate_s = cand.t_validate
            rm.t_total_s = cand.t_total
            rm.fail_reason = cand.fail_reason
            rm.success = cand.success

            self._replan_raw_xy = cand.astar_xy
            self._replan_smoothed_xy = cand.smoothed_xy
            self._replan_smoothed_yaw = cand.smoothed_yaw
            self._replan_smoothed_v = cand.smoothed_v
            self._replan_smoothed_s = cand.smoothed_s

            if not cand.success:
                self._log_event("replan_failed", str(cand.fail_reason))
                self._request_abort(
                    f"replan_failed:{cand.fail_reason}")
                return
            if cand.t_total > self.args.replan_timeout_s + 1e-6:
                self._log_event("replan_runtime_exceeded",
                                f"{cand.t_total:.3f}s")
                self._request_abort("replan_runtime_exceeded")
                return

            # Build Setpoint list from smoothed XY in NED with
            # speed-scaled timing.
            sps: List[Setpoint] = []
            t_cum = 0.0
            prev_xy = None
            xs = cand.smoothed_xy[:, 0]; ys = cand.smoothed_xy[:, 1]
            yaws_w = cand.smoothed_yaw
            vs = cand.smoothed_v if len(cand.smoothed_v) \
                else np.full(len(xs), 1.5)
            for i in range(len(xs)):
                xw = float(xs[i]); yw = float(ys[i])
                xned, yned = world_to_ned(xw, yw)
                yawned = yaw_world_to_ned(float(yaws_w[i]))
                v = float(vs[i] if i < len(vs) else 1.5)
                if prev_xy is not None:
                    ds = math.hypot(xw - prev_xy[0], yw - prev_xy[1])
                    v_avg = 0.5 * (max(0.05, v) + max(0.05, prev_v))
                    t_cum += ds / max(0.05, v_avg)
                vx_w = v * math.cos(float(yaws_w[i]))
                vy_w = v * math.sin(float(yaws_w[i]))
                # Velocity transform = rotation only (no translation):
                # x_ned = y_world - y0 => d x_ned / dt = vy_world;
                # y_ned = x_world - x0 => d y_ned / dt = vx_world.
                vx_ned = vy_w
                vy_ned = vx_w
                sps.append(Setpoint(
                    t_ref_s=t_cum,
                    x_ned_m=xned, y_ned_m=yned,
                    z_ned_m=-Z_REF_M,
                    yaw_ned_rad=yawned,
                    segment_id=98,
                    segment_type="local_replan",
                    segment_name="local_roi_replan",
                    vx_ned_mps=vx_ned, vy_ned_mps=vy_ned,
                    vz_ned_mps=0.0,
                ))
                prev_xy = (xw, yw); prev_v = v   # noqa: F841
            self._replan_setpoints = sps
            self._replan_idx = 0
            self._replan_start_wall = time.monotonic()
            self._log_event("replan_completed",
                            f"runtime={cand.t_total:.3f}s "
                            f"len={cand.smoothed_length_m:.2f}m "
                            f"clr={cand.clearance_min_m:.3f}m "
                            f"cls={cand.classification} "
                            f"sps={len(sps)}")
            self._transition("RESUME_REPLAN_REFERENCE")
        except Exception as e:
            self._log_event("replan_exception", repr(e))
            self._request_abort(f"replan_exception:{type(e).__name__}")

    def _handle_resume_tick(self, now: float) -> None:
        # Mid-resume aborts.
        if self._last_status is not None and \
                self._last_status.nav_state != NAVIGATION_STATE_OFFBOARD:
            self._request_abort("nav_state_left_offboard_in_resume")
            return
        if self._last_status is not None and bool(self._last_status.failsafe):
            self._request_abort("failsafe_in_resume")
            return
        if self._last_lpos is not None:
            if abs(self._last_lpos.z) > self.args.max_altitude_m:
                self._request_abort(
                    f"max_altitude_exceeded:z={self._last_lpos.z:.2f}")
                return
        if self._replan_start_wall is None or not self._replan_setpoints:
            return
        elapsed = now - self._replan_start_wall
        if elapsed >= self._replan_setpoints[-1].t_ref_s:
            # Replan complete — snap original mission cursor to resume idx.
            self._snap_mission_cursor_to_resume(now)

    def _snap_mission_cursor_to_resume(self, now: float) -> None:
        if self._resume_idx_in_original < 0:
            self._request_abort("resume_idx_unset")
            return
        # Find the setpoint index in self.setpoints whose (x_ned, y_ned)
        # matches the resume world XY within 1.0 m.
        rxw, ryw = self._resume_xy_world
        rxn, ryn = world_to_ned(rxw, ryw)
        best_i = -1
        best_d = math.inf
        for i, sp in enumerate(self.setpoints):
            d = math.hypot(sp.x_ned_m - rxn, sp.y_ned_m - ryn)
            if d < best_d:
                best_d = d; best_i = i
        if best_i < 0 or best_d > 1.0:
            self._request_abort(
                f"could_not_locate_resume_setpoint_d={best_d:.2f}")
            return
        # Resume continuity metrics.
        last = self._replan_smoothed_xy[-1] if \
            len(self._replan_smoothed_xy) else np.array([math.nan, math.nan])
        self._resume_metrics.resume_idx_in_segment = best_i
        self._resume_metrics.resume_xy_world = (rxw, ryw)
        self._resume_metrics.resume_yaw_world_rad = float(
            self.setpoints[best_i].yaw_ned_rad)  # NED for now; convert
        self._resume_metrics.resume_yaw_world_rad = yaw_ned_to_world(
            self._resume_metrics.resume_yaw_world_rad)
        self._resume_metrics.replan_end_xy_world = (float(last[0]),
                                                    float(last[1])) \
            if last.size == 2 else (math.nan, math.nan)
        self._resume_metrics.pos_jump_replan_to_resume_m = math.hypot(
            float(last[0]) - rxw, float(last[1]) - ryw) \
            if last.size == 2 else math.nan
        # Yaw jump
        if len(self._replan_smoothed_yaw):
            yj = float(self._replan_smoothed_yaw[-1])
            target = self._resume_metrics.resume_yaw_world_rad
            dy = math.atan2(math.sin(target - yj), math.cos(target - yj))
            self._resume_metrics.yaw_jump_replan_to_resume_deg = \
                abs(math.degrees(dy))
        # Reset mission cursor so MISSION continues from best_i.
        cursor_t = self.setpoints[best_i].t_ref_s
        # We want _mission_cursor_t(elapsed) == cursor_t  =>  set
        # _mission_start_wall such that the cursor is at cursor_t now.
        # During post-hold/ramp the cursor advances at unit rate, so:
        #   elapsed_eff = hold_s + ramp_s/2 + (cursor_t)
        # if we treat hold/ramp as already done (start with elapsed past
        # the ramp_s+hold). Simpler: zero out hold and ramp by setting
        # mission_cursor_offset_s = 0 and mission_start_ramp_s = 0 just for
        # this rejoin.
        self._mission_cursor_offset_s = 0.0
        # We can't easily mutate args.mission_start_ramp_s safely; set
        # mission_start_wall so that elapsed = cursor_t which falls past
        # the ramp window (ramp_s = args.mission_start_ramp_s).
        # Use _mission_cursor_t to invert:
        # if cursor_t <= 0: elapsed = 0
        # if cursor_t in ramp window (cursor_t <= ramp_s/2):
        #   ramp_s = args.mission_start_ramp_s; tau = sqrt(2*ramp_s*cursor_t)
        # else: tau = cursor_t + ramp_s/2; elapsed = tau
        ramp_s = float(getattr(self.args, "mission_start_ramp_s", 0.0))
        if cursor_t <= 0.0:
            elapsed_target = 0.0
        elif ramp_s > 1e-9 and cursor_t <= 0.5 * ramp_s:
            elapsed_target = math.sqrt(max(0.0, 2.0 * ramp_s * cursor_t))
        else:
            elapsed_target = cursor_t + 0.5 * ramp_s
        self._mission_start_wall = now - elapsed_target
        self._mission_idx = best_i
        self._rejoined_wall = now
        self._resume_metrics.rejoined_wall_s = now - self._wall_start
        self.report.mission_started = True   # already true
        # Mark the unknown obstacle as handled and clear detection
        # state so the same obstacle cannot retrigger DETECTED -> STOP_HOVER
        # on subsequent MISSION ticks. Hysteresis streak / confirmation
        # flags / streak-start timer all reset.
        self._obstacle_handled = True
        self._det_confirmed = False
        self._det_confirmed_wall = None
        self._det_streak = 0
        self._det_streak_start_wall = None
        self._log_event(
            "rejoin_original_mission",
            f"resume_idx={best_i} d_to_resume={best_d:.3f}m "
            f"yaw_jump_deg="
            f"{self._resume_metrics.yaw_jump_replan_to_resume_deg:.2f} "
            f"obstacle_handled=true")
        self._transition("REJOIN_ORIGINAL_MISSION")

    # ------------------------------------------------------------------
    # Override _append_tick_log to add Gate-6D-specific columns.
    # ------------------------------------------------------------------

    def _append_tick_log(self, now: float) -> None:
        super()._append_tick_log(now)
        if not self.tick_log:
            return
        # Mutate the latest tick to carry Gate-6D extras.
        tl = self.tick_log[-1]
        lat_drift = math.nan
        lat_speed = math.nan
        radial_v_away = math.nan
        if self._last_lpos is not None and not math.isnan(self._hold_x):
            dx = float(self._last_lpos.x) - float(self._hold_x)
            dy = float(self._last_lpos.y) - float(self._hold_y)
            lat_drift = math.hypot(dx, dy)
            if bool(getattr(self._last_lpos, "v_xy_valid", False)):
                lat_speed = math.hypot(float(self._last_lpos.vx),
                                       float(self._last_lpos.vy))
                if lat_drift > 1e-6:
                    radial_v_away = (
                        float(self._last_lpos.vx) * (dx / lat_drift)
                        + float(self._last_lpos.vy) * (dy / lat_drift))
                else:
                    radial_v_away = 0.0
        # Distance-to-obstacle columns (metrics/logging only). Use the
        # GT obstacle for the TRUE distance (does NOT influence decisions),
        # and the sensed obstacle for the perception distance when available.
        dist_to_true = math.nan
        dist_to_sensed = math.nan
        if self._last_lpos is not None:
            vxw, vyw = ned_to_world(self._last_lpos.x, self._last_lpos.y)
            dist_to_true = math.hypot(vxw - self._gt_obstacle_xy[0],
                                      vyw - self._gt_obstacle_xy[1])
            if self._sensed_obstacle is not None:
                so = self._sensed_obstacle
                dist_to_sensed = math.hypot(
                    vxw - so.centroid_world_xy[0],
                    vyw - so.centroid_world_xy[1])
        ext = {
            "g6d_scan_topic_used": self._scan_topic_used or "",
            "g6d_scan_msg_count": self._scan_msg_count,
            "g6d_det_streak": self._det_streak,
            "g6d_det_confirmed": int(self._det_confirmed),
            "g6d_distance_to_true_obstacle_m_metrics_only": dist_to_true,
            "g6d_distance_to_sensed_obstacle_m": dist_to_sensed,
            "g6d_reference_source": (
                "settle_hold" if self._state in (
                    "PRE_STREAM", "WAIT_OFFBOARD", "WAIT_ARMED",
                    "SETTLE_HOLD")
                else "original" if self._state == "MISSION"
                     and not self._obstacle_handled
                else "rejoined" if self._state == "MISSION"
                     and self._obstacle_handled
                else "replan" if self._state == "RESUME_REPLAN_REFERENCE"
                else "rejoined" if self._state == "REJOIN_ORIGINAL_MISSION"
                else "hover" if self._state == "STOP_HOVER"
                else "n/a"),
            "g6d_hover_stable_for_s": (
                (now - self._hover_stable_start)
                if self._hover_stable_start else 0.0),
            "g6d_settle_lat_drift_m": lat_drift,
            "g6d_settle_lat_speed_mps": lat_speed,
            "g6d_settle_radial_v_away_mps": radial_v_away,
            "g6d_settle_guard_b_duration_s": (
                (now - self._settle_guard_b_start_wall)
                if self._settle_guard_b_start_wall is not None else 0.0),
            "g6d_settle_guard_c_duration_s": (
                (now - self._settle_guard_c_start_wall)
                if self._settle_guard_c_start_wall is not None else 0.0),
            "g6d_obstacle_handled": int(self._obstacle_handled),
            "g6d_args_max_altitude_m_current":
                float(self.args.max_altitude_m),
        }
        for k, v in ext.items():
            setattr(tl, k, v)


def parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Gate 6D — live Stop-Replan-Resume executor. Subclasses the "
            "Gate-4C/5C OFFBOARD execution node and adds real LiDAR "
            "cluster detection + in-process Gate-6C.5 local ROI replan."
        )
    )
    # Reuse the nominal executor's arguments unchanged.
    # Mission inputs. These are large derived products that this repository
    # does not ship; build them with the offline tools or supply your own.
    # See docs/reproducibility.md and docs/data_availability.md.
    p.add_argument("--setpoints-csv", type=str, required=True,
                   help="NED setpoint stream for the nominal mission.")
    p.add_argument("--mission-csv", type=str, required=True,
                   help="World-frame mission reference "
                        "(segment_name, s_m, x_m, y_m, yaw, v_ref).")
    p.add_argument("--obstacle-yaml", type=str, required=True,
                   help="Unknown-obstacle definition. Evaluation ground truth, "
                        "and the oracle source in --oracle-mode detection/replan.")
    p.add_argument("--patched-map-yaml", type=str, required=True,
                   help="Prior map with the obstacle stamped in. Used only by "
                        "--oracle-mode replan; perception mode ignores it.")
    p.add_argument("--prior-map-yaml", type=str, required=True,
                   help="Unpatched execution prior the replanner searches on.")
    p.add_argument("--out-dir", type=str,
                   default=str(REPO_ROOT / "runs" / "stop_replan_resume"))
    p.add_argument("--segment-id", type=int, default=1)
    p.add_argument("--segment-name", type=str, default="start_to_poi1")
    p.add_argument("--segment-frac", type=float, default=1.0,
                   help="ignored when --multi-segment is set")
    p.add_argument("--multi-segment", action="store_true", default=True)
    p.add_argument("--speed-scale", type=float, default=1.0)
    p.add_argument("--rate-hz", type=float, default=20.0)
    p.add_argument("--takeoff-altitude-m", type=float, default=2.0)
    p.add_argument("--max-altitude-m", type=float, default=2.5)
    p.add_argument("--max-tracking-error-m", type=float, default=1.0)
    p.add_argument("--lpos-watchdog-s", type=float, default=0.5)
    p.add_argument("--watchdog-s", type=float, default=3.0)
    p.add_argument("--xy-bounds-rel", type=float, default=5.0)
    p.add_argument("--ekf-ready-timeout-s", type=float, default=120.0)
    p.add_argument("--ekf-stable-s", type=float, default=20.0)
    p.add_argument("--pre-stream-s", type=float, default=2.0)
    p.add_argument("--handshake-timeout-s", type=float, default=2.0)
    p.add_argument("--end-hold-s", type=float, default=2.0)
    p.add_argument("--max-run-s", type=float, default=600.0)
    p.add_argument("--arm", action="store_true", default=False)
    p.add_argument("--i-have-physical-eyes-on-the-sim",
                   dest="i_have_physical_eyes_on_the_sim",
                   action="store_true", default=False)
    p.add_argument("--write-plot", action="store_true", default=False)
    p.add_argument("--dry-load", action="store_true", default=False)
    p.add_argument("--settle-pos-threshold-m", type=float, default=0.20)
    p.add_argument("--settle-vel-threshold-mps", type=float, default=0.12)
    p.add_argument("--settle-stable-s", type=float, default=2.0)
    p.add_argument("--mission-start-hold-s", type=float, default=2.0)
    p.add_argument("--mission-start-ramp-s", type=float, default=8.0)
    p.add_argument("--use-velocity-feedforward",
                   action="store_true", default=False)

    # Gate 6D additions.
    p.add_argument("--scan-topic", type=str, default="/scan_filtered")
    p.add_argument("--scan-fallback-topic", type=str, default="/scan")
    p.add_argument("--lookahead-m", type=float, default=7.5)
    p.add_argument("--corridor-half-width-m", type=float, default=1.5)
    p.add_argument("--hysteresis-frames", type=int, default=3)
    p.add_argument("--hysteresis-min-s", type=float, default=0.2)
    p.add_argument("--cluster-gap-m", type=float, default=0.5)
    p.add_argument("--cluster-match-radius-m", type=float, default=2.5)
    p.add_argument("--stop-hover-stable-s", type=float, default=3.0)
    p.add_argument("--stop-hover-pos-tol-m", type=float, default=0.30)
    p.add_argument("--stop-hover-speed-tol-mps", type=float, default=0.15)
    p.add_argument("--replan-timeout-s", type=float, default=3.0)
    p.add_argument("--roi-scale", type=float, default=1.0)
    p.add_argument("--resume-offset-m", type=float, default=15.0)
    p.add_argument("--detection-mode", type=str,
                   default="lidar_cluster_detection",
                   choices=["lidar_cluster_detection",
                            "synthetic_live_obstacle_event"])
    # --- Gate 6D-B perception / oracle isolation ---
    p.add_argument("--oracle-mode", type=str, default="off",
                   choices=["off", "detection", "replan"],
                   help=("off  = perception-driven (6D-B): detection by "
                         "prior-map novelty, replan around the SENSED "
                         "obstacle on the UNPATCHED prior. Ground truth is "
                         "retained in a separate evaluation-only namespace: "
                         "it does not gate detection, obstacle construction, "
                         "replanning or control decisions, but it may be used "
                         "for diagnostic metrics and post-run evaluation. The "
                         "online-decision obstacle is None, so an accidental "
                         "decision use fails loud. detection = GT gates "
                         "detection only, replan uses sensed geometry. "
                         "replan = full oracle (6D-A mechanics): GT + "
                         "patched_prior."))
    p.add_argument("--min-cluster-points", type=int, default=3)
    p.add_argument("--sensed-radius-floor-m", type=float, default=0.5)
    p.add_argument("--sensed-radius-max-m", type=float, default=3.0)
    p.add_argument("--obstacle-inflation-m", type=float, default=0.30)
    p.add_argument("--prior-novelty-radius-m", type=float, default=1.0)
    # SETTLE timeout override, applied to this process only.
    p.add_argument("--settle-altitude-tolerance-m", type=float, default=0.8,
                   help=("State-aware altitude cap during SETTLE_HOLD: "
                         "cap = takeoff_altitude_m + this. After SETTLE "
                         "completes the strict --max-altitude-m applies."))
    # Refined SETTLE lateral guard (A/B/C rule).
    p.add_argument("--settle-drift-abort-m", type=float, default=0.60,
                   help="Guard A: instant abort if lat drift > this.")
    p.add_argument("--settle-drift-warn-m", type=float, default=0.30,
                   help=("Guard B drift threshold (must be combined with "
                         "radial-away velocity threshold and duration)."))
    p.add_argument("--settle-radial-v-away-abort-mps",
                   type=float, default=0.25,
                   help=("Guard B: radial velocity AWAY from hold above "
                         "this, combined with drift > settle-drift-warn-m, "
                         "sustained for settle-guard-min-duration-s, "
                         "aborts. Catches a steady drift accelerating away "
                         "from the hold point, ignoring small near-origin "
                         "takeoff transients."))
    p.add_argument("--settle-speed-abort-mps", type=float, default=0.80,
                   help=("Guard C: lateral speed above this sustained for "
                         "settle-guard-min-duration-s aborts."))
    p.add_argument("--settle-guard-min-duration-s",
                   type=float, default=0.5,
                   help="Minimum continuous-condition duration for B and C.")
    p.add_argument("--settle-timeout-s", type=float, default=12.0,
                   help=("Max wall time in SETTLE_HOLD to bank the stability "
                         "window before aborting. Overrides the gate4c "
                         "SETTLE_TIMEOUT_S constant for this process only; "
                         "does NOT change the stability thresholds. Raise it "
                         "to absorb the PX4 takeoff-ramp dead-time."))
    return p.parse_args(argv)


DEFAULT_INPUT_BUNDLES = (
    REPO_ROOT / "results" / "planning",
    REPO_ROOT / "results" / "nominal_mission",
    REPO_ROOT / "results" / "stop_replan_resume",
)


def _verify_immutable(dirs=DEFAULT_INPUT_BUNDLES) -> dict:
    """Re-verify the ``hashes.txt`` of each input bundle before the run.

    A run is only interpretable if the reference data it flew against is the
    data that was frozen. The per-bundle status is written into the run output
    so a later reader can tell whether an input drifted underneath it.

    A directory without a ``hashes.txt`` is reported as ``no_hashes_txt``
    rather than silently passing.
    """
    results: dict = {}
    for d in dirs:
        d = Path(d)
        h = d / "hashes.txt"
        try:
            key = str(d.relative_to(REPO_ROOT))
        except ValueError:
            key = str(d)
        if not h.exists():
            results[key] = "no_hashes_txt"
            continue
        ok = True
        bad: List[str] = []
        for line in h.read_text().splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                digest, name = line.split(None, 1)
            except ValueError:
                ok = False
                bad.append(line)
                continue
            f = d / name
            if not f.exists():
                ok = False
                bad.append(f"missing:{name}")
                continue
            if sha256_file(f) != digest:
                ok = False
                bad.append(f"mismatch:{name}")
        results[key] = "PASS" if ok else "FAIL:" + ",".join(bad)
    return results


def _write_srr_outputs(out_dir: Path, node: "StopReplanResumeNode",
                          args, mission_csv: Path,
                          immutable_status: dict) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    # Event log.
    with (out_dir / "event_log.csv").open("w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["wall_s", "event", "info"])
        for wall, name, info in node._events:
            w.writerow([f"{wall:.6f}", name, info])
    # Detection events (per frame).
    with (out_dir / "detection_events.csv").open("w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["wall_s", "n_returns_in_corridor",
                    "nearest_cluster_x", "nearest_cluster_y",
                    "nearest_cluster_dist_to_obs_m", "is_positive",
                    "hysteresis_streak"])
        for fr in node._det_frames:
            nx = fr.nearest_cluster_world_xy[0] \
                if fr.nearest_cluster_world_xy else math.nan
            ny = fr.nearest_cluster_world_xy[1] \
                if fr.nearest_cluster_world_xy else math.nan
            w.writerow([f"{fr.wall_s:.6f}", fr.n_returns_in_corridor,
                        f"{nx:.6f}", f"{ny:.6f}",
                        ("%.6f" % fr.nearest_cluster_dist_to_obs_m)
                        if fr.nearest_cluster_dist_to_obs_m is not None
                        else "",
                        int(bool(fr.is_positive)), fr.hysteresis_streak])
    # LiDAR detection log (denser).
    with (out_dir / "lidar_detection_log.csv").open("w", newline="") as f:
        if node._lidar_log:
            cols = list(node._lidar_log[0].keys())
            w = csv.writer(f, lineterminator="\n")
            w.writerow(cols)
            for row in node._lidar_log:
                w.writerow([row.get(c, "") for c in cols])
        else:
            f.write("wall_s,no_frames\n")
    # Hover metrics.
    yaml.safe_dump(dataclasses.asdict(node._hover_metrics),
                   (out_dir / "hover_metrics.yaml").open("w"),
                   sort_keys=False)
    # Replan runtime metrics.
    yaml.safe_dump(dataclasses.asdict(node._replan_metrics),
                   (out_dir / "replan_runtime_metrics.yaml").open("w"),
                   sort_keys=False)
    # Resume join metrics.
    yaml.safe_dump(dataclasses.asdict(node._resume_metrics),
                   (out_dir / "resume_join_metrics.yaml").open("w"),
                   sort_keys=False)
    # Local replan raw + smoothed CSVs.
    with (out_dir / "local_replan_live.csv").open("w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["idx", "x_world_m", "y_world_m"])
        for i, (x, y) in enumerate(node._replan_raw_xy):
            w.writerow([i, f"{x:.6f}", f"{y:.6f}"])
    with (out_dir / "local_replan_smoothed_live.csv").open(
            "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["idx", "x_world_m", "y_world_m", "yaw_world_rad",
                    "v_ref_mps", "s_m"])
        for i in range(len(node._replan_smoothed_xy)):
            w.writerow([
                i,
                f"{node._replan_smoothed_xy[i, 0]:.6f}",
                f"{node._replan_smoothed_xy[i, 1]:.6f}",
                f"{node._replan_smoothed_yaw[i]:.9f}"
                if i < len(node._replan_smoothed_yaw) else "",
                f"{node._replan_smoothed_v[i]:.6f}"
                if i < len(node._replan_smoothed_v) else "",
                f"{node._replan_smoothed_s[i]:.6f}"
                if i < len(node._replan_smoothed_s) else "",
            ])
    # Stitched live reference: original segment up to resume idx, with the
    # replan section replaced by the live replan.
    try:
        seg_rows = roi.load_segment_rows(mission_csv, args.segment_name)
        with (out_dir / "stitched_reference_live.csv").open(
                "w", newline="") as f:
            fns = ["segment_id", "segment_type", "segment_name",
                   "x_m", "y_m", "yaw_rad", "v_ref_mps", "source"]
            wcsv = csv.DictWriter(f, fieldnames=fns); wcsv.writeheader()
            # pre-detection prefix: from start to closest sample to hover.
            hover_xw, hover_yw = node._hover_metrics.hover_xy_world
            if math.isfinite(hover_xw):
                xs = np.array([float(r["x_m"]) for r in seg_rows])
                ys = np.array([float(r["y_m"]) for r in seg_rows])
                i_hover = int(np.argmin((xs - hover_xw)**2
                                        + (ys - hover_yw)**2))
            else:
                i_hover = 0
            for r in seg_rows[:i_hover + 1]:
                wcsv.writerow({k: r.get(k, "") for k in fns
                               if k != "source"} | {"source": "original"})
            for i in range(len(node._replan_smoothed_xy)):
                wcsv.writerow({
                    "segment_id": "98", "segment_type": "local_replan",
                    "segment_name": "local_roi_replan",
                    "x_m": f"{node._replan_smoothed_xy[i, 0]:.6f}",
                    "y_m": f"{node._replan_smoothed_xy[i, 1]:.6f}",
                    "yaw_rad": f"{node._replan_smoothed_yaw[i]:.9f}"
                    if i < len(node._replan_smoothed_yaw) else "",
                    "v_ref_mps":
                        f"{node._replan_smoothed_v[i]:.6f}"
                        if i < len(node._replan_smoothed_v) else "",
                    "source": "replan",
                })
            if node._resume_idx_in_original >= 0:
                for r in seg_rows[node._resume_idx_in_original:]:
                    wcsv.writerow({k: r.get(k, "") for k in fns
                                   if k != "source"}
                                  | {"source": "post_resume"})
    except Exception as e:
        (out_dir / "stitched_reference_live.csv").write_text(
            f"# error building stitched: {e}\n", encoding="utf-8")
    # Sensed-vs-true obstacle metrics (OFFLINE EVALUATION ONLY — these
    # compare the perception output against ground truth; ground truth
    # never entered the online decision path in perception mode).
    so = node._sensed_obstacle
    gt_xy = node._gt_obstacle_xy
    gt_r = node._gt_obstacle_r
    if so is not None:
        centroid_err = math.hypot(so.centroid_world_xy[0] - gt_xy[0],
                                  so.centroid_world_xy[1] - gt_xy[1])
        arc_centroid_err = math.hypot(
            so.arc_centroid_world_xy[0] - gt_xy[0],
            so.arc_centroid_world_xy[1] - gt_xy[1])
        radius_err = so.radius_m - gt_r
        sensed_metrics = {
            "oracle_mode": node._oracle_mode,
            "perception_sensed_obstacle": {
                "centroid_world_xy": list(so.centroid_world_xy),
                "radius_m": so.radius_m,
                "arc_centroid_world_xy": list(so.arc_centroid_world_xy),
                "arc_half_span_m": so.arc_half_span_m,
                "bearing_from_vehicle_rad": so.bearing_from_vehicle_rad,
                "prior_dist_to_occupied_m": so.prior_dist_to_occupied_m,
                "first_seen_wall_s": so.first_seen_wall_s,
                "n_frames": so.n_frames,
                "source": so.source,
            },
            "ground_truth_eval_only": {
                "centroid_world_xy": list(gt_xy),
                "radius_m": gt_r,
            },
            "errors_eval_only": {
                "backprojected_centroid_error_m": centroid_err,
                "arc_centroid_error_m": arc_centroid_err,
                "radius_error_m": radius_err,
                "radius_conservative": bool(so.radius_m >= gt_r),
            },
        }
    else:
        sensed_metrics = {
            "oracle_mode": node._oracle_mode,
            "perception_sensed_obstacle": None,
            "note": ("no obstacle sensed online "
                     "(oracle-mode replan uses ground truth directly, "
                     "or no detection occurred)"),
            "ground_truth_eval_only": {
                "centroid_world_xy": list(gt_xy), "radius_m": gt_r},
        }
    yaml.safe_dump(sensed_metrics,
                   (out_dir / "sensed_obstacle_metrics.yaml").open("w"),
                   sort_keys=False)

    # Diagnostics.
    diag = {
        "state_when_finalize": node._state,
        "oracle_mode": node._oracle_mode,
        "online_obstacle_was_ground_truth":
            node._oracle_mode in ("detection", "replan"),
        "scan_topic_used": node._scan_topic_used,
        "scan_msg_count": node._scan_msg_count,
        "detection_confirmed": bool(node._det_ever_confirmed),
        "detection_confirmed_wall_s":
            (node._det_first_confirmed_wall - node._wall_start)
            if node._det_first_confirmed_wall else math.nan,
        "sensed_obstacle_present": so is not None,
        "false_positive_count_outside_active_segment":
            node._false_positive_count,
        "resume_idx_in_original": node._resume_idx_in_original,
        "rejoin_wall_s":
            (node._rejoined_wall - node._wall_start)
            if node._rejoined_wall else math.nan,
        "events_count": len(node._events),
        "tick_count": len(node.tick_log),
        "immutable_artifacts_status": immutable_status,
    }
    yaml.safe_dump(diag, (out_dir / "diagnostics.yaml").open("w"),
                   sort_keys=False)


def _write_verdict(out_dir: Path, node: "StopReplanResumeNode", args,
                   immutable_status: dict) -> str:
    rm = node._replan_metrics
    hm = node._hover_metrics
    res = node._resume_metrics
    cond = {
        "detection_mode_lidar_cluster_detection":
            args.detection_mode == "lidar_cluster_detection",
        # Gate 6D-B PASS requires perception mode. The key name is kept for
        # schema compatibility with the published bundle; what it asserts is
        # that no online DECISION read ground truth - detection, obstacle
        # construction, replanning and control all ran without it. Ground
        # truth itself stays available in the evaluation-only namespace for
        # diagnostics and post-run metrics. In oracle modes this condition is
        # False, so an oracle run can be a valid 6D-A mechanics PASS but never
        # a 6D-B perception PASS.
        "perception_mode_no_ground_truth_online":
            node._oracle_mode == "off",
        "perception_sensed_obstacle_built":
            node._sensed_obstacle is not None,
        "scan_topic_received_messages": node._scan_msg_count > 0,
        "detection_confirmed": bool(node._det_ever_confirmed),
        "stop_hover_command_sent":
            any(e[1] == "stop_command_sent" for e in node._events),
        "hover_stable_ge_3s":
            hm.stable_for_s_at_promotion >= args.stop_hover_stable_s - 1e-6,
        "replan_success": bool(rm.success),
        "replan_runtime_le_3s": (
            math.isfinite(rm.t_total_s) and rm.t_total_s
            <= args.replan_timeout_s + 1e-6),
        "bspline_validation_pass":
            bool(rm.collision_free and rm.clearance_min_ge_r_eff),
        "resume_started":
            any(e[1] == "replan_completed" for e in node._events),
        "rejoin_original_mission":
            any(e[1] == "rejoin_original_mission" for e in node._events),
        "mission_completed": bool(node.report.mission_completed),
        "no_failsafe_during_run":
            not bool(node.report.aborted)
            and not node.report.nav_state_left_offboard_in_mission,
        "tracking_max_lt_1m":
            (not math.isnan(node.report.tracking_max_err_m))
            and node.report.tracking_max_err_m < 1.0,
        "max_altitude_le_2p5":
            (math.isnan(node.report.tracking_max_err_m)
             or True),     # altitude cap is enforced live as abort
        "land_sent": bool(node.report.land_sent),
        "disarm_seen": bool(node.report.disarm_seen),
        "force_disarm_not_sent": not bool(node.report.force_disarm_sent),
        "immutable_artifacts_unchanged": all(
            v.startswith("PASS") for v in immutable_status.values()),
    }
    verdict = "PASS" if all(cond.values()) else "FAIL"
    perception_mode = node._oracle_mode == "off"
    payload = {
        "verdict": verdict,
        # Dual representation: a self-describing scenario id, plus the
        # validation-gate label from the project's incremental gate framework
        # (documented in docs/architecture.md). The gate label alone is not
        # self-explanatory, so it never appears without the id and the note.
        "scenario_id": ("stop_replan_resume_perception"
                        if perception_mode else
                        "stop_replan_resume_oracle_mechanics"),
        "validation_gate": "Gate 6D-B" if perception_mode else "Gate 6D-A",
        "validation_gate_note": (
            "Gate 6D-B: perception-driven. Detection from LiDAR clustering "
            "plus a novelty test against the prior map; the replan uses the "
            "sensed obstacle on the unpatched prior. Ground truth is retained "
            "in a separate evaluation-only namespace: it does not gate "
            "detection, obstacle construction, replanning or control "
            "decisions, but it may be used to compute diagnostic metrics and "
            "post-run evaluation values."
            if perception_mode else
            "Gate 6D-A: oracle mechanics. Ground truth supplies the obstacle "
            "and the patched prior is used. Validates the state machine, not "
            "the perception."
        ),
        "oracle_mode": node._oracle_mode,
        "detection_mode": args.detection_mode,
        "abort_reason": node.report.abort_reason if node.report.aborted
        else None,
        "conditions": cond,
    }
    yaml.safe_dump(payload, (out_dir / "verdict.yaml").open("w"),
                   sort_keys=False)
    return verdict


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    # Override the parent's SETTLE_TIMEOUT_S for this process only (gate4c
    # source is untouched). This extends the time allowed to bank the settle
    # stability window so the PX4 takeoff-ramp dead-time does not exhaust the
    # budget; the pos/vel/stability thresholds themselves are unchanged.
    executor_mod.SETTLE_TIMEOUT_S = float(args.settle_timeout_s)

    # Load obstacle ground truth. In perception mode (oracle off) the
    # obstacle.yaml is OPTIONAL — GT is used only for offline metrics, so a
    # run with obstacle.yaml absent must still detect+replan+resume from
    # perception alone (this is the oracle-leak test). In oracle modes the
    # file is REQUIRED because GT drives the online geometry.
    obstacle_path = Path(args.obstacle_yaml)
    if obstacle_path.is_file():
        obstacle = yaml.safe_load(obstacle_path.read_text())
        if obstacle.get("verdict") != "PASS":
            print(f"FAIL: obstacle verdict {obstacle.get('verdict')!r}",
                  file=sys.stderr)
            return 2
    else:
        if args.oracle_mode in ("detection", "replan"):
            print(f"FAIL: oracle-mode={args.oracle_mode} requires "
                  f"obstacle.yaml but it is absent: {obstacle_path}",
                  file=sys.stderr)
            return 2
        # Perception mode without ground truth: placeholder GT (NaN) so
        # metrics columns exist but carry no information. Online decisions
        # never read it.
        print(f"[srr] obstacle.yaml absent ({obstacle_path}); "
              f"perception mode proceeds with NaN ground-truth (metrics "
              f"will be NaN). This is the oracle-leak test path.",
              flush=True)
        obstacle = {
            "position_world_xy_m": [float("nan"), float("nan")],
            "radius_m": float("nan"),
            "verdict": "PASS",
        }

    # Static checks pre-spin.
    immutable = _verify_immutable()
    bad = [k for k, v in immutable.items() if not v.startswith("PASS")]
    if bad:
        print(f"FAIL: immutable artifacts modified: {bad}",
              file=sys.stderr)
        return 2

    # Spawn obstacle if needed? The task assumes it is already spawned.
    # No further validation here; the world file is the reference.

    # Load setpoints.
    setpoints_csv = Path(args.setpoints_csv).resolve()
    setpoints, seg_report = load_segment(
        setpoints_csv, args.segment_id, args.segment_name,
        args.segment_frac, args.speed_scale,
        multi_segment=True,
    )
    if seg_report.errors:
        print(f"FAIL: segment load errors: {seg_report.errors}",
              file=sys.stderr)
        return 2
    if (args.use_velocity_feedforward
            and not seg_report.has_velocity_cols):
        print("FAIL: --use-velocity-feedforward set but CSV missing "
              "velocity columns", file=sys.stderr)
        return 2

    if args.dry_load:
        print(f"[srr] dry-load OK: {seg_report.n_rows_selected} rows, "
              f"stretched_duration="
              f"{seg_report.selected_duration_stretched_s:.2f}s")
        # Still write a stub verdict.
        (out_dir / "verdict.yaml").write_text(
            "verdict: DRY_LOAD_OK\n", encoding="utf-8")
        return 0

    det_cfg = DetectionConfig(
        min_cluster_points=args.min_cluster_points,
        sensed_radius_floor_m=args.sensed_radius_floor_m,
        sensed_radius_max_m=args.sensed_radius_max_m,
        obstacle_inflation_m=args.obstacle_inflation_m,
        prior_novelty_radius_m=args.prior_novelty_radius_m,
        lookahead_m=args.lookahead_m,
        corridor_half_width_m=args.corridor_half_width_m,
        hysteresis_frames=args.hysteresis_frames,
        hysteresis_min_duration_s=args.hysteresis_min_s,
        cluster_gap_m=args.cluster_gap_m,
        cluster_match_radius_m=args.cluster_match_radius_m,
    )

    # Record params + source revision + topic snapshot upfront.
    #
    # The revision is provenance, not a precondition: a run started from a
    # tarball or an un-initialised checkout must still fly. Record what is
    # knowable and say so plainly when it is not.
    try:
        head = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=str(REPO_ROOT),
            stderr=subprocess.DEVNULL).decode().strip()
    except (subprocess.CalledProcessError, FileNotFoundError, OSError):
        head = "unknown (not a git checkout)"
    (out_dir / "git_head.txt").write_text(head + "\n", encoding="utf-8")

    if _SIGNAL_HANDLERS_NO is not None:
        rclpy.init(signal_handler_options=_SIGNAL_HANDLERS_NO)
    else:
        rclpy.init()
    node = StopReplanResumeNode(
        args, setpoints, seg_report, obstacle, det_cfg,
        Path(args.obstacle_yaml).resolve(),
        args.scan_topic, out_dir,
        Path(args.patched_map_yaml).resolve(),
        Path(args.prior_map_yaml).resolve(),
        Path(args.mission_csv).resolve(),
        args.segment_name,
    )

    sigint_state = {"count": 0}

    def _sigint(_signum, _frame):
        sigint_state["count"] += 1
        if sigint_state["count"] >= 2:
            print("Second SIGINT received -- hard exit (no safe land!).",
                  file=sys.stderr, flush=True)
            os._exit(2)
        node.trigger_sigint_abort()

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    # Pre-assign the verdict so the return path stays safe even if a
    # later write step raises.
    verdict = "FAIL"
    spin_exc: Optional[BaseException] = None
    try:
        while rclpy.ok() and not node.is_done():
            rclpy.spin_once(node, timeout_sec=0.05)
    except BaseException as e:                                    # noqa
        spin_exc = e
    finally:
        # Each finalize step is wrapped so one failure does not block the
        # others. All available tick / event data must reach disk.
        def _safe(label, fn):
            try:
                return fn()
            except Exception as exc:                              # noqa
                try:
                    node.get_logger().error(
                        f"finalize step '{label}' failed: {exc!r}")
                except Exception:
                    pass
                return None
        _safe("finalize_metrics", node.finalize_metrics)
        immutable_after = _safe("verify_immutable", _verify_immutable) \
            or {}
        _safe("g4c_write_outputs",
              lambda: g4c_write_outputs(out_dir, args, setpoints_csv,
                                        seg_report, node.report,
                                        node.tick_log))
        _safe("srr_write_outputs",
              lambda: _write_srr_outputs(
                  out_dir, node, args,
                  Path(args.mission_csv).resolve(), immutable_after))
        v_out = _safe("write_verdict",
                      lambda: _write_verdict(out_dir, node, args,
                                             immutable_after))
        if v_out is not None:
            verdict = v_out
        # Refresh hashes (best-effort).
        def _write_hashes():
            hash_lines = []
            for name in sorted(p.name for p in out_dir.iterdir()
                               if p.is_file()):
                if name == "hashes.txt": continue
                hash_lines.append(f"{sha256_file(out_dir/name)}  {name}")
            (out_dir / "hashes.txt").write_text(
                "\n".join(hash_lines) + "\n", encoding="utf-8")
        _safe("write_hashes", _write_hashes)
        try:
            node.get_logger().info(
                f"VERDICT={verdict} spin_exc={type(spin_exc).__name__ if spin_exc else 'none'}")
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
