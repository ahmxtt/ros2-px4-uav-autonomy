#!/usr/bin/env python3
"""Nominal PX4 Offboard mission executor (ROS 2).

Validation stage: **Gate 4C** (short defensive Offboard segment), extended
through **Gate 5B/5C** to full multi-segment mission execution. The gate
framework is described in ``docs/architecture.md``.

Streams a pre-computed NED setpoint reference to PX4 over uXRCE-DDS at a fixed
rate and supervises the run through an explicit state machine::

    INIT -> PRE_STREAM -> DECIDE_ARM -> HANDSHAKE -> ARM
         -> SETTLE -> MISSION -> LAND -> WAIT_DISARM -> DONE

Arming is guarded by two independent flags. The autopilot is commanded to
OFFBOARD and armed only when BOTH ``--arm`` and
``--i-have-physical-eyes-on-the-sim`` are given; either flag alone keeps the
run in dry-run mode and no mode-change or arm command is ever constructed.
The default is dry-run.

Safety supervision is continuous rather than end-of-run. The executor watches
the PX4 arming and navigation state, the failsafe flags, an absolute altitude
cap and a rolling tracking-error window, and commands a controlled land on
violation. A run that leaves OFFBOARD, breaches the tracking window or trips a
failsafe is recorded as a failure with its reason, not silently truncated. If
a commanded disarm is not observed, a force-disarm fallback runs.

Every run writes a self-describing bundle: the full per-tick log, run metrics,
a verdict with per-condition booleans, an input manifest and SHA-256 hashes.

REQUIRES ROS 2 (rclpy, px4_msgs) and a running PX4 SITL stack; it cannot run
offline. The setpoint cursor advances on wall-clock time, so the simulator
must run at a real-time factor near 1.0 for the flown reference to match the
planned one.
"""

import argparse
import csv
import hashlib
import math
import os
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

try:
    from rclpy.signals import SignalHandlerOptions

    _SIGNAL_HANDLERS_NO = SignalHandlerOptions.NO
except ImportError:
    _SIGNAL_HANDLERS_NO = None

# Gate 4C is the first armed/OFFBOARD-capable node, so VehicleCommand IS
# imported here (unlike Gate 4B). It is wired exclusively to the two-flag
# armed path; see _send_offboard_mode_request, _send_arm_command, _send_land.
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

try:
    from px4_msgs.msg import FailsafeFlags as _FailsafeFlagsType
except ImportError:
    _FailsafeFlagsType = None

# Diagnostic-only optional subscriptions. Each is wrapped in its
# own try/except so we degrade gracefully on PX4 builds that lack the type.
try:
    from px4_msgs.msg import VehicleControlMode as _VehicleControlModeType
except ImportError:
    _VehicleControlModeType = None
try:
    from px4_msgs.msg import BatteryStatus as _BatteryStatusType
except ImportError:
    _BatteryStatusType = None
try:
    from px4_msgs.msg import EstimatorStatusFlags as _EstimatorStatusFlagsType
except ImportError:
    _EstimatorStatusFlagsType = None
try:
    from px4_msgs.msg import EstimatorStatus as _EstimatorStatusType
except ImportError:
    _EstimatorStatusType = None

# Curated subset of FailsafeFlags fields logged per-tick (high signal-to-noise
# for the OFFBOARD-drop / failsafe investigation). The full message snapshot
# is captured in run_metrics.yaml's first_failsafe_tick + offboard_drop blocks.
_FAILSAFE_FLAGS_TICK_FIELDS = (
    "offboard_control_signal_lost",
    "manual_control_signal_lost",
    "gcs_connection_lost",
    "local_position_invalid",
    "local_position_invalid_relaxed",
    "local_velocity_invalid",
    "global_position_invalid",
    "battery_warning",
    "battery_unhealthy",
    "battery_low_remaining_time",
    "geofence_breached",
    "mission_failure",
    "navigator_failure",
    "position_accuracy_low",
    "wind_limit_exceeded",
)
_VEHICLE_CONTROL_MODE_TICK_FIELDS = (
    "flag_armed",
    "flag_control_offboard_enabled",
    "flag_control_position_enabled",
    "flag_control_velocity_enabled",
    "flag_control_attitude_enabled",
)
_BATTERY_STATUS_TICK_FIELDS = (
    "voltage_v",
    "remaining",
    "warning",
)


# PX4 MAVLink command IDs.
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

# PX4 navigation_state values.
NAVIGATION_STATE_OFFBOARD = 14

# PX4 arming_state values.
ARMING_STATE_DISARMED = 1
ARMING_STATE_ARMED = 2

# PX4 magic constant for force-disarm bypass on COMPONENT_ARM_DISARM param2.
# Used only in the abort path after a Land attempt has not produced a disarm
# within --disarm-timeout-s.
FORCE_DISARM_MAGIC = 21196.0

# Default paths.
_REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_SETPOINTS_CSV = (
    ""  # no default: the setpoint stream must be supplied explicitly
)
DEFAULT_OUT_DIR = str(_REPO_ROOT / "runs" / "offboard_mission")

# Required CSV columns (Gate 4A schema).
REQUIRED_CSV_COLUMNS = (
    "t_ref_s",
    "x_ned_m",
    "y_ned_m",
    "z_ned_m",
    "yaw_ned_rad",
    "segment_id",
    "segment_type",
    "segment_name",
)
# Velocity columns. NOT in the hard-required set: older single-segment CSVs
# may omit them, in which case velocity feedforward is unavailable but the
# rest of the pipeline still works. _check_velocity_columns_or_stop is called
# explicitly when --use-velocity-feedforward is set.
VELOCITY_CSV_COLUMNS = ("vx_ned_mps", "vy_ned_mps", "vz_ned_mps")

# Default selected segment (Gate 3 leg 1).
DEFAULT_SEGMENT_ID = 1
DEFAULT_SEGMENT_NAME = "start_to_poi1"

# Safety caps applied to user-provided arguments.
SPEED_SCALE_HARD_CAP = 0.7
# Multi-segment (Gate 5C full-mission) raises the cap to the CSV's native
# speed: the reference trajectory was already smoothed inside the v_nominal /
# a_lat_max budget, so a multi-segment run at the native v_nominal is the
# intended "normal speed". Going above 1.0 is still disallowed (no extrapolation).
SPEED_SCALE_HARD_CAP_MULTI_SEGMENT = 1.0
MAX_ALTITUDE_HARD_CAP_M = 3.5

# Settle thresholds (Gate-0B-equivalent).
SETTLE_THRESHOLD_M = 0.3
SETTLE_TIMEOUT_S = 12.0

# Rolling-window for tracking-error abort during MISSION.
TRACKING_WINDOW_S = 1.0

# Disarm watchdog after Land (Gate 0B uses 30 s).
DISARM_TIMEOUT_S = 30.0
ABORT_DISARM_TIMEOUT_S = 15.0


def px4_qos() -> QoSProfile:
    # Same profile Gate 0B / Gate 4B use for the PX4 uXRCE-DDS bridge.
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


@dataclass
class Setpoint:
    t_ref_s: float
    x_ned_m: float
    y_ned_m: float
    z_ned_m: float
    yaw_ned_rad: float
    segment_id: int
    segment_type: str
    segment_name: str
    # Velocity feedforward. Stored as the CSV-source velocity at
    # the original v_ref (not stretched by speed_scale). _stream_tick MISSION
    # multiplies these by speed_scale * cursor_rate before publishing.
    # Default 0.0 so older CSVs without velocity columns still load cleanly
    # under --no-use-velocity-feedforward.
    vx_ned_mps: float = 0.0
    vy_ned_mps: float = 0.0
    vz_ned_mps: float = 0.0


@dataclass
class SegmentLoadReport:
    n_rows_total: int = 0
    n_rows_segment: int = 0
    n_rows_selected: int = 0
    csv_duration_s: float = math.nan
    segment_duration_s: float = math.nan
    selected_duration_s: float = math.nan
    selected_duration_stretched_s: float = math.nan
    segment_id: int = -1
    segment_name: str = ""
    x_min: float = math.nan
    x_max: float = math.nan
    y_min: float = math.nan
    y_max: float = math.nan
    z_min: float = math.nan
    z_max: float = math.nan
    has_velocity_cols: bool = False
    errors: List[str] = field(default_factory=list)


def load_segment(
    path: Path,
    segment_id: int,
    segment_name: str,
    segment_frac: float,
    speed_scale: float,
    multi_segment: bool = False,
) -> Tuple[List[Setpoint], SegmentLoadReport]:
    report = SegmentLoadReport()
    out: List[Setpoint] = []
    if not path.is_file():
        report.errors.append(f"setpoints_csv_not_found:{path}")
        return out, report

    all_rows: List[Setpoint] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        missing = [c for c in REQUIRED_CSV_COLUMNS if c not in reader.fieldnames]
        if missing:
            report.errors.append(f"missing_columns:{','.join(missing)}")
            return out, report
        has_velocity_cols = all(
            c in reader.fieldnames for c in VELOCITY_CSV_COLUMNS
        )
        report.has_velocity_cols = has_velocity_cols  # type: ignore[attr-defined]
        for i, row in enumerate(reader):
            try:
                t = float(row["t_ref_s"])
                x = float(row["x_ned_m"])
                y = float(row["y_ned_m"])
                z = float(row["z_ned_m"])
                yaw = float(row["yaw_ned_rad"])
                seg_id = int(row["segment_id"])
                seg_type = row["segment_type"]
                seg_name = row["segment_name"]
                if has_velocity_cols:
                    vx = float(row["vx_ned_mps"])
                    vy = float(row["vy_ned_mps"])
                    vz = float(row["vz_ned_mps"])
                else:
                    vx = vy = vz = 0.0
            except (TypeError, ValueError, KeyError) as e:
                report.errors.append(f"row_{i}_parse_error:{e}")
                return out, report
            for fv, fname in (
                (t, "t_ref_s"),
                (x, "x_ned_m"),
                (y, "y_ned_m"),
                (z, "z_ned_m"),
                (yaw, "yaw_ned_rad"),
            ):
                if math.isnan(fv) or math.isinf(fv):
                    report.errors.append(f"row_{i}_nonfinite:{fname}={fv}")
                    return out, report
            if has_velocity_cols:
                for vv, vname in ((vx, "vx_ned_mps"), (vy, "vy_ned_mps"),
                                  (vz, "vz_ned_mps")):
                    if math.isnan(vv) or math.isinf(vv):
                        report.errors.append(
                            f"row_{i}_nonfinite:{vname}={vv}"
                        )
                        return out, report
            all_rows.append(
                Setpoint(t, x, y, z, yaw, seg_id, seg_type, seg_name,
                         vx, vy, vz)
            )

    if not all_rows:
        report.errors.append("empty_csv")
        return out, report

    report.n_rows_total = len(all_rows)
    report.csv_duration_s = all_rows[-1].t_ref_s - all_rows[0].t_ref_s

    if multi_segment:
        # Multi-segment mode (Gate 5C full-mission). Consume ALL rows without
        # filtering. segment_id / segment_name CLI defaults are ignored;
        # per-row labels are preserved on each Setpoint and surface in the
        # tick log and per-segment metrics. segment_frac is also ignored
        # here (truncation is a single-segment concept). speed_scale still
        # applies to the entire timeline below.
        if not all_rows:
            report.errors.append("empty_csv_multi_segment")
            return out, report
        seg_rows = all_rows
        report.segment_id = -1
        report.segment_name = "MULTI_SEGMENT"
    else:
        seg_rows = [
            r for r in all_rows
            if r.segment_id == segment_id and r.segment_name == segment_name
        ]
        if not seg_rows:
            report.errors.append(
                f"segment_not_found:segment_id={segment_id} "
                f"segment_name={segment_name}"
            )
            return out, report
        report.segment_id = segment_id
        report.segment_name = segment_name

    # Re-base time to t0 = 0 across the selected rows.
    t0 = seg_rows[0].t_ref_s
    seg_rows = [
        Setpoint(
            t_ref_s=(r.t_ref_s - t0),
            x_ned_m=r.x_ned_m,
            y_ned_m=r.y_ned_m,
            z_ned_m=r.z_ned_m,
            yaw_ned_rad=r.yaw_ned_rad,
            segment_id=r.segment_id,
            segment_type=r.segment_type,
            segment_name=r.segment_name,
            vx_ned_mps=r.vx_ned_mps,
            vy_ned_mps=r.vy_ned_mps,
            vz_ned_mps=r.vz_ned_mps,
        )
        for r in seg_rows
    ]
    report.n_rows_segment = len(seg_rows)
    report.segment_duration_s = seg_rows[-1].t_ref_s

    if multi_segment:
        # No fractional truncation in multi-segment mode.
        selected = seg_rows
    else:
        frac = max(0.0, min(1.0, float(segment_frac)))
        cutoff_t = report.segment_duration_s * frac
        selected = [r for r in seg_rows if r.t_ref_s <= cutoff_t + 1e-9]
        if not selected:
            report.errors.append(
                f"segment_frac_truncated_to_empty:frac={frac}"
            )
            return out, report

    report.n_rows_selected = len(selected)
    report.selected_duration_s = selected[-1].t_ref_s

    # Verify monotonic time and z negative.
    prev_t = -math.inf
    for r in selected:
        if r.t_ref_s < prev_t - 1e-9:
            report.errors.append("non_monotonic_t_in_selected")
            return out, report
        prev_t = r.t_ref_s
        if r.z_ned_m > 0.0:
            report.errors.append(f"z_ned_non_negative:{r.z_ned_m}")
            return out, report

    # Apply speed_scale by stretching t_ref ONLY. Geometry untouched.
    scale = float(speed_scale)
    if scale <= 0.0:
        report.errors.append(f"speed_scale_non_positive:{scale}")
        return out, report
    speed_cap = (
        SPEED_SCALE_HARD_CAP_MULTI_SEGMENT if multi_segment
        else SPEED_SCALE_HARD_CAP
    )
    if scale > speed_cap + 1e-9:
        report.errors.append(
            f"speed_scale_exceeds_hard_cap:{scale}>{speed_cap}"
            + ("(multi_segment cap)" if multi_segment else "")
        )
        return out, report

    stretch = 1.0 / scale
    stretched = [
        Setpoint(
            t_ref_s=(r.t_ref_s * stretch),
            x_ned_m=r.x_ned_m,
            y_ned_m=r.y_ned_m,
            z_ned_m=r.z_ned_m,
            yaw_ned_rad=r.yaw_ned_rad,
            segment_id=r.segment_id,
            segment_type=r.segment_type,
            segment_name=r.segment_name,
            # Velocities are NOT stretched at load time; speed_scale is
            # applied at publish time so the same scaled CSV value gives the
            # correct wall-time velocity at any speed_scale.
            vx_ned_mps=r.vx_ned_mps,
            vy_ned_mps=r.vy_ned_mps,
            vz_ned_mps=r.vz_ned_mps,
        )
        for r in selected
    ]
    report.selected_duration_stretched_s = stretched[-1].t_ref_s

    xs = [r.x_ned_m for r in stretched]
    ys = [r.y_ned_m for r in stretched]
    zs = [r.z_ned_m for r in stretched]
    report.x_min, report.x_max = min(xs), max(xs)
    report.y_min, report.y_max = min(ys), max(ys)
    report.z_min, report.z_max = min(zs), max(zs)

    return stretched, report


# -------------- Reports --------------


@dataclass
class RunReport:
    timestamp_iso: str = ""
    dry_run: bool = True
    arm_flag: bool = False
    eyes_flag: bool = False
    speed_scale: float = math.nan
    takeoff_altitude_m: float = math.nan

    topics_visible: bool = False
    first_status_seen: bool = False
    first_lpos_seen: bool = False
    first_lpos_valid: bool = False
    failsafe_flags_subscribed: bool = False

    ekf_ready_reached: bool = False
    ekf_ready_time_s: float = math.nan
    hold_x: float = math.nan
    hold_y: float = math.nan
    hold_z_ned: float = math.nan

    nav_state_after_handshake: int = -1
    nav_state_offboard_reached: bool = False
    arm_attempted: bool = False
    arm_reached: bool = False
    settle_completed: bool = False
    settle_time_s: float = math.nan
    settle_max_abs_err_m: float = math.nan

    mission_started: bool = False
    mission_started_wall_s: float = math.nan
    mission_stopped_wall_s: float = math.nan
    mission_completed: bool = False
    mission_messages_sent: int = 0
    achieved_rate_p50_hz: float = math.nan
    achieved_rate_p95_hz: float = math.nan
    achieved_rate_p99_hz: float = math.nan
    tracking_max_err_m: float = math.nan
    tracking_rolling_max_err_m: float = math.nan
    tracking_window_breach: bool = False
    nav_state_left_offboard_in_mission: bool = False

    land_sent: bool = False
    disarm_seen: bool = False
    force_disarm_sent: bool = False
    aborted: bool = False
    abort_reason: str = ""
    refusal_reasons: List[str] = field(default_factory=list)
    nav_states_observed: List[int] = field(default_factory=list)
    arming_states_observed: List[int] = field(default_factory=list)

    # Diagnostics. Populated by the node before write_outputs.
    first_failsafe_snapshot: Optional[dict] = None
    offboard_drop_snapshot: Optional[dict] = None
    optional_topics_status: Optional[dict] = None

    verdict: str = "PENDING"


@dataclass
class TickLog:
    wall_s: float
    state: str
    cmd_t_ref_s: float
    cur_segment_id: int
    cur_segment_name: str
    cur_segment_type: str
    cursor_t: float
    cursor_rate: float
    sp_x: float
    sp_y: float
    sp_z: float
    sp_yaw: float
    # Published velocity feedforward components (NaN if FF disabled this tick).
    sp_vx_ff: float
    sp_vy_ff: float
    sp_vz_ff: float
    vff_enabled: int
    lpos_x: float
    lpos_y: float
    lpos_z: float
    lpos_vx: float
    lpos_vy: float
    lpos_vz: float
    lpos_speed_mps: float
    lpos_xy_valid: int
    lpos_z_valid: int
    lpos_v_xy_valid: int
    lpos_v_z_valid: int
    nav_state: int
    arming_state: int
    failsafe: int
    err_x: float
    err_y: float
    err_z: float
    err_abs_max: float
    # Curated FailsafeFlags subset (-1 = topic/msg missing this tick).
    ff_offboard_control_signal_lost: int
    ff_manual_control_signal_lost: int
    ff_gcs_connection_lost: int
    ff_local_position_invalid: int
    ff_local_position_invalid_relaxed: int
    ff_local_velocity_invalid: int
    ff_global_position_invalid: int
    ff_battery_warning: int
    ff_battery_unhealthy: int
    ff_battery_low_remaining_time: int
    ff_geofence_breached: int
    ff_mission_failure: int
    ff_navigator_failure: int
    ff_position_accuracy_low: int
    ff_wind_limit_exceeded: int
    # VehicleControlMode (-1 if missing).
    vcm_flag_armed: int
    vcm_flag_control_offboard_enabled: int
    vcm_flag_control_position_enabled: int
    vcm_flag_control_velocity_enabled: int
    vcm_flag_control_attitude_enabled: int
    # BatteryStatus (NaN if missing).
    bat_voltage_v: float
    bat_remaining: float
    bat_warning: int


TICK_LOG_FIELDS = (
    "wall_s", "state", "cmd_t_ref_s",
    "cur_segment_id", "cur_segment_name", "cur_segment_type",
    "cursor_t", "cursor_rate",
    "sp_x", "sp_y", "sp_z", "sp_yaw",
    "sp_vx_ff", "sp_vy_ff", "sp_vz_ff", "vff_enabled",
    "lpos_x", "lpos_y", "lpos_z",
    "lpos_vx", "lpos_vy", "lpos_vz", "lpos_speed_mps",
    "lpos_xy_valid", "lpos_z_valid", "lpos_v_xy_valid", "lpos_v_z_valid",
    "nav_state", "arming_state", "failsafe",
    "err_x", "err_y", "err_z", "err_abs_max",
    "ff_offboard_control_signal_lost",
    "ff_manual_control_signal_lost",
    "ff_gcs_connection_lost",
    "ff_local_position_invalid",
    "ff_local_position_invalid_relaxed",
    "ff_local_velocity_invalid",
    "ff_global_position_invalid",
    "ff_battery_warning",
    "ff_battery_unhealthy",
    "ff_battery_low_remaining_time",
    "ff_geofence_breached",
    "ff_mission_failure",
    "ff_navigator_failure",
    "ff_position_accuracy_low",
    "ff_wind_limit_exceeded",
    "vcm_flag_armed",
    "vcm_flag_control_offboard_enabled",
    "vcm_flag_control_position_enabled",
    "vcm_flag_control_velocity_enabled",
    "vcm_flag_control_attitude_enabled",
    "bat_voltage_v", "bat_remaining", "bat_warning",
)


def _fmt_csv_field(v) -> str:
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return ""
        return f"{v:.6f}"
    return str(v).replace(",", ";")


# -------------- Node --------------


class OffboardMissionExecutor(Node):
    def __init__(
        self,
        args,
        setpoints: List[Setpoint],
        seg_report: SegmentLoadReport,
    ):
        super().__init__("offboard_mission_executor")
        self.args = args
        self.setpoints = setpoints
        self.seg_report = seg_report

        self.report = RunReport(
            timestamp_iso=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            arm_flag=bool(args.arm),
            eyes_flag=bool(args.i_have_physical_eyes_on_the_sim),
            dry_run=(not (args.arm and args.i_have_physical_eyes_on_the_sim)),
            speed_scale=float(args.speed_scale),
            takeoff_altitude_m=float(args.takeoff_altitude_m),
        )
        self.tick_log: List[TickLog] = []

        qos = px4_qos()

        # Subscribers.
        self._sub_status = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, qos
        )
        self._sub_lpos = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._on_lpos,
            qos,
        )
        if _FailsafeFlagsType is not None:
            self._sub_failsafe_flags = self.create_subscription(
                _FailsafeFlagsType,
                "/fmu/out/failsafe_flags",
                self._on_failsafe_flags,
                qos,
            )
            self.report.failsafe_flags_subscribed = True
        else:
            self._sub_failsafe_flags = None

        # Diagnostic subscriptions (best-effort). State for the
        # latest message of each type; None if the type or stream is missing.
        self._last_vehicle_control_mode = None
        self._last_battery_status = None
        self._last_estimator_status_flags = None
        self._last_estimator_status = None
        self._optional_topics_status: dict = {
            "failsafe_flags": "msg_type_missing"
            if _FailsafeFlagsType is None else "subscribed",
            "vehicle_control_mode": "msg_type_missing"
            if _VehicleControlModeType is None else "subscribed",
            # PX4 SITL exposes battery telemetry as the
            # versioned topic /fmu/out/battery_status_v1 (the unversioned
            # /fmu/out/battery_status is not bridged in this build).
            "battery_status_v1": "msg_type_missing"
            if _BatteryStatusType is None else "subscribed",
            "estimator_status_flags": "msg_type_missing"
            if _EstimatorStatusFlagsType is None else "subscribed",
            # Legacy /fmu/out/estimator_status is not exported by this PX4
            # SITL build. estimator_status_flags above covers the modern
            # field set; we record the legacy topic as unavailable rather
            # than subscribing and never receiving.
            "estimator_status": "not_exported_by_this_px4_build",
        }
        if _VehicleControlModeType is not None:
            self._sub_vehicle_control_mode = self.create_subscription(
                _VehicleControlModeType,
                "/fmu/out/vehicle_control_mode",
                self._on_vehicle_control_mode,
                qos,
            )
        if _BatteryStatusType is not None:
            # Use the versioned topic name that PX4 SITL exports.
            self._sub_battery_status = self.create_subscription(
                _BatteryStatusType,
                "/fmu/out/battery_status_v1",
                self._on_battery_status,
                qos,
            )
        if _EstimatorStatusFlagsType is not None:
            self._sub_estimator_status_flags = self.create_subscription(
                _EstimatorStatusFlagsType,
                "/fmu/out/estimator_status_flags",
                self._on_estimator_status_flags,
                qos,
            )
        # Legacy /fmu/out/estimator_status is not bridged by this PX4 SITL
        # build. We keep _on_estimator_status and _last_estimator_status as
        # harmless no-ops so the diagnostic snapshot dump (which calls
        # _msg_dump(self._last_estimator_status)) stays None-safe.
        # Diagnostic snapshots populated by callbacks / control_tick on
        # transitions. Written verbatim to run_metrics.yaml in write_outputs.
        self._first_failsafe_snapshot: Optional[dict] = None
        self._offboard_drop_snapshot: Optional[dict] = None
        self._prev_nav_state: Optional[int] = None
        self._prev_failsafe: Optional[bool] = None

        # Publishers. VehicleCommand publisher exists because Gate 4C is the
        # first armed/OFFBOARD-capable node — but it is only ever exercised
        # through _send_offboard_mode_request / _send_arm_command / _send_land
        # which are all gated by self._armed_path() (two-flag check) at their
        # only call sites in the state machine (see _control_tick).
        self._pub_oc_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self._pub_trajsp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self._pub_vcmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        # State.
        self._last_status: Optional[VehicleStatus] = None
        self._last_status_wall = 0.0
        self._last_lpos: Optional[VehicleLocalPosition] = None
        self._last_lpos_wall = 0.0
        self._last_failsafe_flags = None
        # Last MISSION setpoint actually published, used to keep streaming
        # alive through OBSERVE_END so PX4's COM_OF_LOSS_T (~1 s + 0.5 s)
        # does not trip OFFBOARD setpoint-loss failsafe between mission end
        # and NAV_LAND. Set in _stream_tick when MISSION publishes, then
        # re-published once per tick in OBSERVE_END.
        self._last_mission_sp: Optional[Setpoint] = None

        self._state = "WAIT_STATUS"
        self._state_entered_wall = time.monotonic()
        self._wall_start = time.monotonic()
        self._abort_requested = False

        # Hold point snapped at EKF-ready entry. The first MISSION setpoint
        # is (0, 0, -takeoff_alt) by design; PRE_STREAM / SETTLE_HOLD use
        # this hold point.
        self._takeoff_alt = min(
            abs(args.takeoff_altitude_m), MAX_ALTITUDE_HARD_CAP_M
        )
        self._hold_x = math.nan
        self._hold_y = math.nan
        self._hold_z_ned = -self._takeoff_alt
        self._stream_active = False

        # Mission cursor.
        self._mission_start_wall: Optional[float] = None
        self._mission_idx = 0
        self._last_publish_wall: Optional[float] = None
        self._publish_intervals_s: List[float] = []

        # Tracking-error rolling window.
        self._tracking_history: List[Tuple[float, float]] = []  # (wall_s, |err|)
        self._tracking_abs_max = 0.0

        # EKF-stable-window tracking (Gate-0B-equivalent).
        self._ekf_stable_start_wall: Optional[float] = None
        self._ekf_last_xy_reset_cnt = None
        self._ekf_last_z_reset_cnt = None
        self._ekf_last_heading_reset_cnt = None

        # Settle / disarm timing.
        self._settle_started_wall: Optional[float] = None
        self._settle_max_abs_err = 0.0
        # Gate 5C velocity-aware settle: continuously-stable window tracking.
        # Reset whenever the pos error or the local speed exceeds threshold,
        # or velocity validity is missing. Settle completes only when this
        # window has been continuously open for >= settle_stable_s seconds.
        self._settle_stable_start_wall: Optional[float] = None
        self._settle_max_speed_mps = 0.0
        self._land_sent_wall: Optional[float] = None
        self._force_disarm_sent = False
        # Gate 5C mission-start hold: after SETTLE passes, the cursor stays
        # at setpoints[0] for mission_start_hold_s before advancing.
        self._mission_cursor_offset_s: float = 0.0

        # Segment XY envelope for bounds-check.
        self._seg_x_lo = seg_report.x_min - args.xy_bounds_rel
        self._seg_x_hi = seg_report.x_max + args.xy_bounds_rel
        self._seg_y_lo = seg_report.y_min - args.xy_bounds_rel
        self._seg_y_hi = seg_report.y_max + args.xy_bounds_rel

        period_s = 1.0 / max(1.0, args.rate_hz)
        self._stream_timer = self.create_timer(period_s, self._stream_tick)
        self._control_timer = self.create_timer(0.1, self._control_tick)

        self._log_startup_banner()

    # ---------- Gating ----------

    def _armed_path(self) -> bool:
        # SINGLE PLACE that decides whether arm/OFFBOARD commands are allowed.
        # Both flags must be true. Every caller of _send_offboard_mode_request,
        # _send_arm_command, and (during the success path) _send_land is
        # gated by this check, either directly or via the DECIDE_ARM ->
        # SEND_OFFBOARD branch which itself is taken only when this returns
        # True.
        return bool(self.args.arm) and bool(
            self.args.i_have_physical_eyes_on_the_sim
        )

    # Gate 5C dirty-state guard. Returns None if pre-arm state is acceptable;
    # otherwise a short reason string. Called from DECIDE_ARM AFTER PRE_STREAM
    # has given PX4 ~pre_stream_s seconds to consume our setpoint stream and
    # clear any fresh-boot offboard_control_signal_lost flag.
    #
    # Rejection conditions:
    #   * nav_state == OFFBOARD (14) BEFORE we requested it — means the stack
    #     is in a residual OFFBOARD state from a prior aborted run.
    #   * nav_state in {4 AUTO_RTL, 5 AUTO_LANDGS, 18 AUTO_LAND, 19 AUTO_RTGS}
    #     — failsafe-like residual state.
    #   * failsafe_flags.offboard_control_signal_lost still true after
    #     PRE_STREAM — PX4 has not registered our stream.
    #
    # Anything else (incl. fresh-boot offboard_control_signal_lost cleared
    # during PRE_STREAM) is fine.
    _PX4_FAILSAFE_LIKE_NAV_STATES = (4, 5, 18, 19)

    def _check_dirty_pre_arm_state(self) -> Optional[str]:
        if self._last_status is not None:
            ns = int(self._last_status.nav_state)
            if ns == NAVIGATION_STATE_OFFBOARD:
                return f"nav_state_already_offboard:{ns}"
            if ns in self._PX4_FAILSAFE_LIKE_NAV_STATES:
                return f"nav_state_failsafe_like:{ns}"
        if self._last_failsafe_flags is not None:
            if bool(self._last_failsafe_flags.offboard_control_signal_lost):
                return "offboard_control_signal_lost_still_true_post_prestream"
        return None

    # ---------- Logging ----------

    def _log_startup_banner(self) -> None:
        a = self.args
        self.get_logger().info(
            "offboard_mission_executor: "
            f"arm_flag={a.arm} "
            f"eyes_flag={a.i_have_physical_eyes_on_the_sim} "
            f"armed_path={self._armed_path()} "
            f"speed_scale={a.speed_scale:.3f} (cap {SPEED_SCALE_HARD_CAP:.2f}) "
            f"takeoff_alt_m={self._takeoff_alt:.2f} "
            f"max_alt_m={a.max_altitude_m:.2f} (cap "
            f"{MAX_ALTITUDE_HARD_CAP_M:.2f}) "
            f"max_track_err_m={a.max_tracking_error_m:.2f} "
            f"lpos_watchdog_s={a.lpos_watchdog_s:.2f} "
            f"xy_bounds_rel_m={a.xy_bounds_rel:.2f} "
            f"rate_hz={a.rate_hz:.1f} "
            f"max_run_s={a.max_run_s:.1f} "
            f"segment_id={self.seg_report.segment_id} "
            f"segment_name={self.seg_report.segment_name} "
            f"segment_frac={a.segment_frac:.3f} "
            f"selected_rows={self.seg_report.n_rows_selected} "
            f"stretched_dur_s="
            f"{self.seg_report.selected_duration_stretched_s:.2f}"
        )
        env_rmw = os.environ.get("RMW_IMPLEMENTATION", "<unset>")
        try:
            from rclpy.utilities import get_rmw_implementation_identifier
            active_rmw = get_rmw_implementation_identifier()
        except Exception:
            # Fall back to the env string only if the active RMW cannot be
            # queried (diagnostic-only; no effect on the middleware in use).
            active_rmw = env_rmw
        self.get_logger().info(
            f"RMW active={active_rmw} (env RMW_IMPLEMENTATION={env_rmw})"
        )
        if active_rmw != "rmw_fastrtps_cpp":
            self.get_logger().warning(
                "PX4 /fmu/* deserialization expects rmw_fastrtps_cpp; "
                f"active RMW is {active_rmw}. Expect issues unless "
                "cross-vendor DDS interop is verified."
            )
        if not self._armed_path():
            self.get_logger().info(
                "ARM-GATE: armed path NOT enabled. Run will stop after "
                "PRE_STREAM; no DO_SET_MODE / ARM / LAND will be sent. "
                "Pass BOTH --arm and --i-have-physical-eyes-on-the-sim to "
                "enable the armed/OFFBOARD/short-segment path."
            )
        else:
            self.get_logger().warning(
                "ARM-GATE: BOTH flags set. Run WILL attempt OFFBOARD + ARM + "
                "execute the selected short segment. SIGINT triggers Land + "
                "(after timeout) force-disarm."
            )

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds // 1000)

    # ---------- Subscribers ----------

    def _on_status(self, msg: VehicleStatus) -> None:
        self._last_status = msg
        self._last_status_wall = time.monotonic()
        if not self.report.first_status_seen:
            self.report.first_status_seen = True
            self.get_logger().info(
                "First VehicleStatus: "
                f"nav_state={msg.nav_state} "
                f"arming_state={msg.arming_state} "
                f"failsafe={msg.failsafe}"
            )
        if int(msg.nav_state) not in self.report.nav_states_observed:
            self.report.nav_states_observed.append(int(msg.nav_state))
        if int(msg.arming_state) not in self.report.arming_states_observed:
            self.report.arming_states_observed.append(int(msg.arming_state))

    def _on_lpos(self, msg: VehicleLocalPosition) -> None:
        self._last_lpos = msg
        self._last_lpos_wall = time.monotonic()
        if (
            not self.report.first_lpos_seen
            and bool(msg.xy_valid)
            and bool(msg.z_valid)
        ):
            self.report.first_lpos_seen = True
            self.report.first_lpos_valid = True
            self.get_logger().info(
                "First valid VehicleLocalPosition: "
                f"x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f}"
            )
        elif not self.report.first_lpos_seen:
            self.report.first_lpos_seen = True

    def _on_failsafe_flags(self, msg) -> None:
        self._last_failsafe_flags = msg
        # Mark stream as live (not just msg_type_missing).
        if self._optional_topics_status.get("failsafe_flags") != "active":
            self._optional_topics_status["failsafe_flags"] = "active"

    def _on_vehicle_control_mode(self, msg) -> None:
        self._last_vehicle_control_mode = msg
        self._optional_topics_status["vehicle_control_mode"] = "active"

    def _on_battery_status(self, msg) -> None:
        self._last_battery_status = msg
        self._optional_topics_status["battery_status_v1"] = "active"

    def _on_estimator_status_flags(self, msg) -> None:
        self._last_estimator_status_flags = msg
        self._optional_topics_status["estimator_status_flags"] = "active"

    def _on_estimator_status(self, msg) -> None:
        self._last_estimator_status = msg
        self._optional_topics_status["estimator_status"] = "active"

    # ---------- Publisher helpers ----------

    def _publish_offboard_control_mode(self) -> None:
        m = OffboardControlMode()
        m.timestamp = self._now_us()
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self._pub_oc_mode.publish(m)

    def _publish_trajectory_setpoint_hold(self) -> None:
        m = TrajectorySetpoint()
        m.timestamp = self._now_us()
        m.position[0] = float(self._hold_x)
        m.position[1] = float(self._hold_y)
        m.position[2] = float(self._hold_z_ned)
        m.yaw = float("nan")
        m.yawspeed = float("nan")
        self._pub_trajsp.publish(m)

    def _publish_trajectory_setpoint(
        self,
        sp: Setpoint,
        velocity_ned: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        m = TrajectorySetpoint()
        m.timestamp = self._now_us()
        m.position[0] = float(sp.x_ned_m)
        m.position[1] = float(sp.y_ned_m)
        m.position[2] = float(sp.z_ned_m)
        # Velocity feedforward. NaN => "ignore this field" so PX4
        # uses position-only control (pre-v4 behavior). Numeric => use the
        # provided velocity as a feedforward setpoint, layered on top of the
        # position track.
        if velocity_ned is not None:
            m.velocity[0] = float(velocity_ned[0])
            m.velocity[1] = float(velocity_ned[1])
            m.velocity[2] = float(velocity_ned[2])
        else:
            m.velocity[0] = float("nan")
            m.velocity[1] = float("nan")
            m.velocity[2] = float("nan")
        m.yaw = float(sp.yaw_ned_rad)
        m.yawspeed = float("nan")
        self._pub_trajsp.publish(m)

    def _publish_vehicle_command(self, command: int, **kwargs) -> None:
        # Internal helper. ONLY called by:
        #   _send_offboard_mode_request  (gated by self._armed_path())
        #   _send_arm_command            (gated by self._armed_path())
        #   _send_land                   (only after arming, abort path)
        #   _send_force_disarm           (abort path, after Land timeout)
        m = VehicleCommand()
        m.timestamp = self._now_us()
        m.command = int(command)
        m.param1 = float(kwargs.get("param1", 0.0))
        m.param2 = float(kwargs.get("param2", 0.0))
        m.param3 = float(kwargs.get("param3", 0.0))
        m.param4 = float(kwargs.get("param4", 0.0))
        m.param5 = float(kwargs.get("param5", 0.0))
        m.param6 = float(kwargs.get("param6", 0.0))
        m.param7 = float(kwargs.get("param7", 0.0))
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self._pub_vcmd.publish(m)

    def _send_offboard_mode_request(self) -> None:
        # Two-flag gate enforced here AND at the only call site.
        if not self._armed_path():
            self.get_logger().error(
                "REFUSED: _send_offboard_mode_request called without both "
                "--arm and --i-have-physical-eyes-on-the-sim. No command "
                "sent."
            )
            return
        self.get_logger().warning(
            "Sending VEHICLE_CMD_DO_SET_MODE param1=1.0 param2=6.0 (OFFBOARD)."
        )
        self._publish_vehicle_command(
            VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )

    def _send_arm_command(self) -> None:
        # Two-flag gate enforced here AND at the only call site.
        if not self._armed_path():
            self.get_logger().error(
                "REFUSED: _send_arm_command called without both --arm and "
                "--i-have-physical-eyes-on-the-sim. No command sent."
            )
            return
        self.report.arm_attempted = True
        self.get_logger().warning("Sending VEHICLE_CMD_COMPONENT_ARM_DISARM.")
        self._publish_vehicle_command(
            VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )

    def _send_land(self) -> None:
        # Land is sent only from the abort/end-of-mission path AFTER the
        # vehicle was successfully armed (report.arm_reached or
        # arm_attempted). On the dry-run path neither flag is true, so this
        # method is never reached.
        self.get_logger().warning("Sending VEHICLE_CMD_NAV_LAND.")
        self._publish_vehicle_command(VEHICLE_CMD_NAV_LAND)
        self.report.land_sent = True
        self._land_sent_wall = time.monotonic()

    def _send_force_disarm(self) -> None:
        if self._force_disarm_sent:
            return
        self._force_disarm_sent = True
        self.report.force_disarm_sent = True
        self.get_logger().error("Sending force-disarm (param2 magic).")
        self._publish_vehicle_command(
            VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
            param2=FORCE_DISARM_MAGIC,
        )

    # ---------- Stream tick ----------

    def _stream_tick(self) -> None:
        if self._abort_requested:
            # During abort: keep OFFBOARD setpoint stream alive through
            # NAV_LAND so PX4 does not drop OFFBOARD via setpoint-loss
            # failsafe before AUTO_LAND takes over. Prefer the last MISSION
            # setpoint: the vehicle holds its abort position
            # while AUTO_LAND descends) rather than the takeoff hold, which
            # would order a fly-back to (0,0,-takeoff_alt) and fight the
            # landing controller. Falls back to the hold setpoint if abort
            # fires before MISSION ever started (no _last_mission_sp yet).
            if self._stream_active:
                self._publish_offboard_control_mode()
                sp = self._last_mission_sp
                vff_hold = (0.0, 0.0, 0.0) if getattr(
                    self.args, "use_velocity_feedforward", False
                ) else None
                if sp is not None:
                    self._publish_trajectory_setpoint(sp, velocity_ned=vff_hold)
                else:
                    self._publish_trajectory_setpoint_hold()
                self._record_publish(time.monotonic())
            return
        if not self._stream_active:
            return

        # PRE_STREAM / SETTLE_HOLD: hold setpoint.
        if self._state in ("PRE_STREAM", "WAIT_OFFBOARD", "WAIT_ARMED",
                           "SETTLE_HOLD"):
            self._publish_offboard_control_mode()
            self._publish_trajectory_setpoint_hold()
            self._record_publish(time.monotonic())
            return

        # MISSION: pick setpoint by elapsed * 1 (timing already stretched at
        # load time; stretched t_ref is the wall timeline).
        if self._state == "MISSION":
            now = time.monotonic()
            elapsed = (
                (now - self._mission_start_wall)
                if self._mission_start_wall is not None
                else 0.0
            )
            sp = self._select_mission_setpoint(elapsed)
            if sp is None:
                # End of segment: hold at last setpoint while transitioning.
                if self.setpoints:
                    last = self.setpoints[-1]
                    self._publish_offboard_control_mode()
                    # Hold => zero velocity feedforward if enabled, so PX4
                    # actively damps any residual motion at the segment end.
                    vff_hold = (0.0, 0.0, 0.0) if getattr(
                        self.args, "use_velocity_feedforward", False
                    ) else None
                    self._publish_trajectory_setpoint(
                        last, velocity_ned=vff_hold,
                    )
                    self._record_publish(now)
                    self._last_mission_sp = last
                return
            self._publish_offboard_control_mode()
            # Velocity feedforward when enabled. Effective
            # velocity = csv_velocity * speed_scale * cursor_rate, where
            # cursor_rate is the instantaneous d(cursor_t)/d(wall_t) (0
            # during hold, linear 0->1 during ramp, 1 after ramp). This
            # keeps the published velocity exactly consistent with the
            # actual wall-time advance of the cursor through the path.
            vff: Optional[Tuple[float, float, float]] = None
            if getattr(self.args, "use_velocity_feedforward", False):
                rate = self._mission_cursor_rate(elapsed)
                ss = float(self.args.speed_scale)
                scale = ss * rate
                vff = (
                    float(sp.vx_ned_mps) * scale,
                    float(sp.vy_ned_mps) * scale,
                    float(sp.vz_ned_mps) * scale,
                )
            self._publish_trajectory_setpoint(sp, velocity_ned=vff)
            self._record_publish(now)
            self._update_tracking(now, sp)
            self._last_mission_sp = sp
            return

        # OBSERVE_END: bridge MISSION -> NAV_LAND by continuing to stream
        # OffboardControlMode + the last published MISSION setpoint until
        # _send_land() has been issued (control_tick sets _stream_active =
        # False AFTER the Land call). Without this bridge, PX4's OFFBOARD
        # setpoint-loss watchdog (~COM_OF_LOSS_T) trips before the
        # control_tick reaches the Land branch, kicking the vehicle into
        # failsafe / out of OFFBOARD. This was observed in arm_run_1 and
        # is the only post-MISSION adjustment.
        if self._state == "OBSERVE_END":
            now = time.monotonic()
            sp = self._last_mission_sp
            if sp is None and self.setpoints:
                sp = self.setpoints[-1]
            if sp is not None:
                self._publish_offboard_control_mode()
                vff_hold = (0.0, 0.0, 0.0) if getattr(
                    self.args, "use_velocity_feedforward", False
                ) else None
                self._publish_trajectory_setpoint(sp, velocity_ned=vff_hold)
                self._record_publish(now)
            return

    def _record_publish(self, now: float) -> None:
        if self._last_publish_wall is not None:
            dt = now - self._last_publish_wall
            if dt > 0:
                self._publish_intervals_s.append(dt)
        self._last_publish_wall = now

    def _mission_cursor_rate(self, mission_elapsed_s: float) -> float:
        """d(cursor_t)/d(wall_time) at this elapsed time. Used for the Gate
        5C v4 velocity-feedforward scale during ramp.
          * during hold:  0
          * during ramp:  tau / ramp_s   (linear 0 -> 1)
          * after ramp:   1.0
        """
        hold_s = float(self._mission_cursor_offset_s)
        ramp_s = float(getattr(self.args, "mission_start_ramp_s", 0.0))
        if mission_elapsed_s < hold_s:
            return 0.0
        tau = mission_elapsed_s - hold_s
        if ramp_s > 1e-9 and tau < ramp_s:
            return tau / ramp_s
        return 1.0

    def _mission_cursor_t(self, mission_elapsed_s: float) -> float:
        """Map wall time since MISSION entry to the cursor time used to look
        up setpoints[].t_ref_s.

        Profile:
          * [0, hold_s):                cursor_t = 0  (hold at setpoints[0])
          * [hold_s, hold_s + ramp_s):  quadratic ramp, cursor_t advances
                                        from 0 to ramp_s/2 as the time
                                        derivative grows linearly from 0
                                        to 1 across the start ramp.
          * [hold_s + ramp_s, ...):     unit-rate advance, cursor_t =
                                        ramp_s/2 + (tau - ramp_s).

        With ramp_s = 0 the ramp branch is skipped and behavior matches
        pre-v3 (hold then unit-rate cursor).
        """
        hold_s = float(self._mission_cursor_offset_s)
        ramp_s = float(getattr(self.args, "mission_start_ramp_s", 0.0))
        if mission_elapsed_s < hold_s:
            return 0.0
        tau = mission_elapsed_s - hold_s
        if ramp_s > 1e-9 and tau < ramp_s:
            return (tau * tau) / (2.0 * ramp_s)
        return 0.5 * max(ramp_s, 0.0) + (tau - max(ramp_s, 0.0))

    def _select_mission_setpoint(self, elapsed_s: float) -> Optional[Setpoint]:
        if not self.setpoints:
            return None
        # Gate 5C mission-start hold + ramp: for the first mission_cursor_offset_s
        # wall seconds, the cursor stays at setpoints[0]. Then for the next
        # mission_start_ramp_s wall seconds, the cursor's time-derivative
        # ramps linearly from 0 to 1 (slow start). See _mission_cursor_t.
        target_t = self._mission_cursor_t(elapsed_s)
        if target_t <= 0.0:
            # Still inside hold window; lock at first setpoint.
            self._mission_idx = 0
            return self.setpoints[0]
        while (
            self._mission_idx + 1 < len(self.setpoints)
            and self.setpoints[self._mission_idx + 1].t_ref_s <= target_t
        ):
            self._mission_idx += 1
        if self._mission_idx >= len(self.setpoints) - 1:
            last = self.setpoints[-1]
            if target_t > last.t_ref_s + 1.0 / max(1.0, self.args.rate_hz):
                return None
        if self._state == "MISSION":
            self.report.mission_messages_sent += 1
        return self.setpoints[self._mission_idx]

    def _update_tracking(self, now: float, sp: Setpoint) -> None:
        lpos = self._last_lpos
        if lpos is None:
            return
        err_x = lpos.x - sp.x_ned_m
        err_y = lpos.y - sp.y_ned_m
        err_z = lpos.z - sp.z_ned_m
        err_abs = max(abs(err_x), abs(err_y), abs(err_z))
        if err_abs > self._tracking_abs_max:
            self._tracking_abs_max = err_abs
            self.report.tracking_max_err_m = err_abs

        # Rolling 1 s window.
        self._tracking_history.append((now, err_abs))
        cutoff = now - TRACKING_WINDOW_S
        while self._tracking_history and self._tracking_history[0][0] < cutoff:
            self._tracking_history.pop(0)
        if self._tracking_history:
            window_max = max(e for _, e in self._tracking_history)
            if (
                math.isnan(self.report.tracking_rolling_max_err_m)
                or window_max > self.report.tracking_rolling_max_err_m
            ):
                self.report.tracking_rolling_max_err_m = window_max
            if window_max > self.args.max_tracking_error_m:
                self.report.tracking_window_breach = True
                self._request_abort(
                    f"tracking_window_breach_max={window_max:.3f}m"
                )

    # ---------- Control tick ----------

    def _control_tick(self) -> None:
        now = time.monotonic()

        # Always log a tick so the log is dense enough to plot.
        self._append_tick_log(now)

        if self._abort_requested:
            self._step_abort_path(now)
            return

        post_pre = self._state not in (
            "WAIT_STATUS", "WAIT_LPOS", "WAIT_EKF_READY", "DONE", "ABORT_LANDING"
        )
        if post_pre and (now - self._wall_start) > self.args.max_run_s:
            self.get_logger().error(
                f"max_run_s exceeded ({self.args.max_run_s:.1f} s)."
            )
            self._request_abort("max_run_s_exceeded")
            return

        # Status freshness watchdog (always once first status seen).
        if (
            self.report.first_status_seen
            and (now - self._last_status_wall) > self.args.watchdog_s
        ):
            self._request_abort("status_stale")
            return

        # Lpos freshness watchdog once we are streaming.
        if (
            self._stream_active
            and self.report.first_lpos_seen
            and (now - self._last_lpos_wall) > self.args.lpos_watchdog_s
        ):
            self._request_abort("lpos_stale")
            return

        if self._state == "WAIT_STATUS":
            if self.report.first_status_seen:
                self.report.topics_visible = True
                self._transition("WAIT_LPOS")
            elif (now - self._state_entered_wall) > 10.0:
                self._request_abort("no_status")
            return

        if self._state == "WAIT_LPOS":
            if self.report.first_lpos_seen:
                self._transition("WAIT_EKF_READY")
            elif (now - self._state_entered_wall) > 10.0:
                self._request_abort("no_lpos")
            return

        if self._state == "WAIT_EKF_READY":
            self._wait_ekf_ready_tick(now)
            return

        if self._state == "PRE_STREAM":
            if (now - self._state_entered_wall) >= self.args.pre_stream_s:
                self._transition("DECIDE_ARM")
            return

        if self._state == "DECIDE_ARM":
            # CRITICAL: this is the only branch in the entire node that
            # leads to _send_offboard_mode_request / _send_arm_command. Any
            # path where _armed_path() returns False MUST end the run here.
            if not self._armed_path():
                self.get_logger().info(
                    "Dry-run stop: BOTH --arm and "
                    "--i-have-physical-eyes-on-the-sim are required to "
                    "exercise the OFFBOARD/ARM/segment path. No "
                    "VEHICLE_CMD_DO_SET_MODE or VEHICLE_CMD_COMPONENT_ARM_"
                    "DISARM has been or will be sent."
                )
                self.report.verdict = "DRY_RUN_OK"
                self._stream_active = False
                self._transition("DONE")
                return
            # Gate 5C dirty-state guard. Refuses to start the armed path on
            # top of residual OFFBOARD / failsafe-like nav_states or a
            # still-latched offboard_control_signal_lost. The flag is allowed
            # to be true at boot but MUST clear during PRE_STREAM once we
            # publish setpoints; if it's still true at DECIDE_ARM, the
            # stack has not recovered and the user should restart it.
            dirty = self._check_dirty_pre_arm_state()
            if dirty is not None:
                self.get_logger().error(
                    "REFUSING TO ARM: dirty pre-arm state "
                    f"'{dirty}'. The PX4 SITL / MicroXRCEAgent / Gazebo "
                    "stack must be restarted manually before retrying. Do "
                    "NOT retry on top of a residual OFFBOARD / RTL / "
                    "failsafe state."
                )
                self._request_abort(f"dirty_state_pre_arm:{dirty}")
                return
            self._send_offboard_mode_request()
            self._transition("WAIT_OFFBOARD")
            return

        if self._state == "WAIT_OFFBOARD":
            ns = (
                self._last_status.nav_state if self._last_status else -1
            )
            self.report.nav_state_after_handshake = int(ns)
            if ns == NAVIGATION_STATE_OFFBOARD:
                self.report.nav_state_offboard_reached = True
                self.get_logger().info(f"OFFBOARD reached: nav_state={ns}")
                self._send_arm_command()
                self._transition("WAIT_ARMED")
            elif (now - self._state_entered_wall) > self.args.handshake_timeout_s:
                self.get_logger().error(
                    f"OFFBOARD not reached within "
                    f"{self.args.handshake_timeout_s:.1f} s (nav_state={ns})."
                )
                self._request_abort("offboard_handshake_failed")
            return

        if self._state == "WAIT_ARMED":
            armed = (
                self._last_status is not None
                and self._last_status.arming_state == ARMING_STATE_ARMED
            )
            if armed:
                self.report.arm_reached = True
                self.get_logger().warning(
                    "ARMED. Climbing to PX4-NED "
                    f"({self._hold_x:.2f}, {self._hold_y:.2f}, "
                    f"{self._hold_z_ned:.2f})."
                )
                self._settle_started_wall = now
                self._settle_max_abs_err = 0.0
                self._transition("SETTLE_HOLD")
            elif (now - self._state_entered_wall) > self.args.handshake_timeout_s:
                self._request_abort("arm_not_confirmed")
            return

        if self._state == "SETTLE_HOLD":
            self._settle_tick(now)
            return

        if self._state == "MISSION":
            self._mission_tick(now)
            return

        if self._state == "OBSERVE_END":
            # End-of-segment hold: _stream_tick keeps publishing the last
            # MISSION setpoint throughout this window so OFFBOARD stays alive
            # right up to the NAV_LAND emission. Once Land has been sent,
            # PX4 transitions to AUTO_LAND (nav_state 18) and no longer
            # consumes /fmu/in/trajectory_setpoint, so it is safe to stop
            # the stream then.
            if (now - self._state_entered_wall) >= self.args.end_hold_s:
                self.report.mission_completed = True
                self.report.mission_stopped_wall_s = now - self._wall_start
                self.get_logger().info("Segment complete. Sending Land.")
                self._send_land()
                self._stream_active = False
                self._transition("WAIT_DISARM")
            return

        if self._state == "WAIT_DISARM":
            disarmed = (
                self._last_status is not None
                and self._last_status.arming_state == ARMING_STATE_DISARMED
            )
            if disarmed:
                self.report.disarm_seen = True
                self._finalize_verdict()
                self.get_logger().info(
                    f"DISARMED. verdict={self.report.verdict}"
                )
                self._transition("DONE")
            elif (
                self._land_sent_wall is not None
                and (now - self._land_sent_wall) > DISARM_TIMEOUT_S
            ):
                self.get_logger().error(
                    f"No disarm within {DISARM_TIMEOUT_S:.1f} s of Land. "
                    "Force-disarm fallback."
                )
                self._send_force_disarm()
                self._request_abort("no_disarm_after_land")
            return

    # ---------- State helpers ----------

    def _transition(self, new_state: str) -> None:
        self.get_logger().info(f"state {self._state} -> {new_state}")
        self._state = new_state
        self._state_entered_wall = time.monotonic()

    def _wait_ekf_ready_tick(self, now: float) -> None:
        lpos = self._last_lpos
        status = self._last_status
        ff = self._last_failsafe_flags

        failed_req: Optional[str] = None
        if lpos is None:
            failed_req = "no_pose"
        elif not bool(lpos.xy_valid):
            failed_req = "xy_invalid"
        elif not bool(lpos.z_valid):
            failed_req = "z_invalid"
        elif not bool(lpos.xy_global):
            failed_req = "xy_not_global"
        elif not bool(lpos.z_global):
            failed_req = "z_not_global"
        elif status is None:
            failed_req = "no_status"
        elif bool(status.failsafe):
            failed_req = "in_failsafe"
        elif ff is not None and bool(ff.local_position_invalid):
            failed_req = "ff_local_position_invalid"
        elif ff is not None and bool(ff.local_velocity_invalid):
            failed_req = "ff_local_velocity_invalid"

        if failed_req is not None:
            self._ekf_stable_start_wall = None
            self._ekf_last_xy_reset_cnt = None
            self._ekf_last_z_reset_cnt = None
            self._ekf_last_heading_reset_cnt = None
        else:
            if (
                self._ekf_stable_start_wall is None
                or self._ekf_last_xy_reset_cnt != lpos.xy_reset_counter
                or self._ekf_last_z_reset_cnt != lpos.z_reset_counter
                or self._ekf_last_heading_reset_cnt
                != lpos.heading_reset_counter
            ):
                self._ekf_stable_start_wall = now
                self._ekf_last_xy_reset_cnt = lpos.xy_reset_counter
                self._ekf_last_z_reset_cnt = lpos.z_reset_counter
                self._ekf_last_heading_reset_cnt = lpos.heading_reset_counter
            elif (now - self._ekf_stable_start_wall) >= self.args.ekf_stable_s:
                # Snap hold point at the FIRST mission setpoint (0,0,-alt)
                # rather than the lpos position: this aligns SETTLE_HOLD with
                # MISSION row 0, avoiding a transient at MISSION entry. The
                # vehicle spawns near (0,0,0); takeoff is a near-vertical
                # climb to z=-takeoff_alt.
                first_sp = self.setpoints[0]
                self._hold_x = float(first_sp.x_ned_m)
                self._hold_y = float(first_sp.y_ned_m)
                # Hold-z uses takeoff_alt as -|alt|. The first MISSION sp
                # has z_ned == -takeoff_alt by construction (Gate-4A constant
                # altitude with z_ref_m == 3.0; we cap takeoff to that).
                self._hold_z_ned = -min(
                    self._takeoff_alt, abs(first_sp.z_ned_m)
                )
                self.report.hold_x = self._hold_x
                self.report.hold_y = self._hold_y
                self.report.hold_z_ned = self._hold_z_ned
                self.report.ekf_ready_reached = True
                self.report.ekf_ready_time_s = (
                    now - self._wall_start
                )
                self.get_logger().info(
                    f"EKF stable. Hold setpoint: "
                    f"x={self._hold_x:.3f} y={self._hold_y:.3f} "
                    f"z={self._hold_z_ned:.3f}"
                )
                self._stream_active = True
                self._transition("PRE_STREAM")
                return

        if (now - self._state_entered_wall) > self.args.ekf_ready_timeout_s:
            self._request_abort(
                f"ekf_not_ready_{failed_req or 'no_stable_window'}"
            )

    def _settle_tick(self, now: float) -> None:
        if self._last_lpos is None:
            return
        err_x = self._last_lpos.x - self._hold_x
        err_y = self._last_lpos.y - self._hold_y
        err_z = self._last_lpos.z - self._hold_z_ned
        cur_err = max(abs(err_x), abs(err_y), abs(err_z))
        if cur_err > self._settle_max_abs_err:
            self._settle_max_abs_err = cur_err
        # nav_state must remain OFFBOARD throughout SETTLE.
        if (
            self._last_status is not None
            and self._last_status.nav_state != NAVIGATION_STATE_OFFBOARD
        ):
            self.report.nav_state_left_offboard_in_mission = True
            self._request_abort("nav_state_left_offboard_in_settle")
            return
        # Altitude cap.
        if (
            self._last_lpos.z_valid
            and abs(self._last_lpos.z) > self.args.max_altitude_m
        ):
            self._request_abort(
                f"max_altitude_exceeded:z={self._last_lpos.z:.2f}"
            )
            return
        # Failsafe.
        if (
            self._last_status is not None
            and bool(self._last_status.failsafe)
        ):
            self._request_abort("failsafe_in_settle")
            return

        # Gate 5C velocity-aware SETTLE: require pos_err < pos_threshold AND
        # speed < vel_threshold continuously for settle_stable_s seconds.
        # Velocity must be valid (v_xy_valid AND v_z_valid). If validity is
        # missing the window is void; we do NOT advance to MISSION.
        pos_thr = float(self.args.settle_pos_threshold_m)
        vel_thr = float(self.args.settle_vel_threshold_mps)
        stable_s = float(self.args.settle_stable_s)
        lpos = self._last_lpos
        v_valid = bool(getattr(lpos, "v_xy_valid", False)) and \
            bool(getattr(lpos, "v_z_valid", False))
        speed = math.sqrt(lpos.vx ** 2 + lpos.vy ** 2 + lpos.vz ** 2) \
            if v_valid else float("inf")
        if v_valid and speed > self._settle_max_speed_mps:
            self._settle_max_speed_mps = speed
        pos_ok = cur_err < pos_thr
        vel_ok = v_valid and speed < vel_thr
        if pos_ok and vel_ok:
            if self._settle_stable_start_wall is None:
                self._settle_stable_start_wall = now
            stable_for = now - self._settle_stable_start_wall
            if stable_for >= stable_s:
                self.report.settle_completed = True
                self.report.settle_time_s = (
                    now - self._settle_started_wall
                )
                self.report.settle_max_abs_err_m = self._settle_max_abs_err
                self.get_logger().warning(
                    f"Settle complete in {self.report.settle_time_s:.2f} s "
                    f"(max_err={self._settle_max_abs_err:.3f} m, "
                    f"final_err={cur_err:.3f} m, "
                    f"final_speed={speed:.3f} m/s, "
                    f"stable_for={stable_for:.2f} s "
                    f"thresholds pos<{pos_thr} vel<{vel_thr}). "
                    f"Mission-start hold {self.args.mission_start_hold_s:.2f} s "
                    f"then ramp 0->1 over "
                    f"{self.args.mission_start_ramp_s:.2f} s before unit-rate."
                )
                self._mission_start_wall = now
                self._mission_idx = 0
                # Cursor offset: hold first setpoint for mission_start_hold_s.
                self._mission_cursor_offset_s = float(
                    self.args.mission_start_hold_s
                )
                self.report.mission_started = True
                self.report.mission_started_wall_s = now - self._wall_start
                self._transition("MISSION")
                return
        else:
            # Reset stability window on any condition fail.
            self._settle_stable_start_wall = None

        if (now - self._settle_started_wall) > SETTLE_TIMEOUT_S:
            self.report.settle_max_abs_err_m = self._settle_max_abs_err
            self.get_logger().error(
                f"Settle timed out after {SETTLE_TIMEOUT_S:.1f} s. "
                f"final_err={cur_err:.3f} (thr {pos_thr:.2f}), "
                f"final_speed={speed:.3f} m/s (thr {vel_thr:.2f}), "
                f"v_valid={v_valid}."
            )
            self._request_abort("settle_timeout")

    def _mission_tick(self, now: float) -> None:
        # Mid-mission abort triggers.
        if (
            self._last_status is not None
            and self._last_status.nav_state != NAVIGATION_STATE_OFFBOARD
        ):
            self.report.nav_state_left_offboard_in_mission = True
            self._request_abort("nav_state_left_offboard_in_mission")
            return
        if (
            self._last_status is not None
            and bool(self._last_status.failsafe)
        ):
            self._request_abort("failsafe_in_mission")
            return
        if (
            self._last_failsafe_flags is not None
            and bool(self._last_failsafe_flags.local_position_invalid)
        ):
            self._request_abort("ff_local_position_invalid_in_mission")
            return
        if (
            self._last_failsafe_flags is not None
            and bool(self._last_failsafe_flags.local_velocity_invalid)
        ):
            self._request_abort("ff_local_velocity_invalid_in_mission")
            return
        if self._last_lpos is not None:
            if not bool(self._last_lpos.xy_valid):
                self._request_abort("lpos_xy_invalid_in_mission")
                return
            if not bool(self._last_lpos.z_valid):
                self._request_abort("lpos_z_invalid_in_mission")
                return
            if abs(self._last_lpos.z) > self.args.max_altitude_m:
                self._request_abort(
                    f"max_altitude_exceeded:z={self._last_lpos.z:.2f}"
                )
                return
            if (
                self._last_lpos.x < self._seg_x_lo
                or self._last_lpos.x > self._seg_x_hi
                or self._last_lpos.y < self._seg_y_lo
                or self._last_lpos.y > self._seg_y_hi
            ):
                self._request_abort(
                    f"xy_outside_segment_envelope:x={self._last_lpos.x:.2f},"
                    f"y={self._last_lpos.y:.2f}"
                )
                return

        # End of mission?
        if self._mission_start_wall is not None and self.setpoints:
            elapsed = now - self._mission_start_wall
            # Account for hold AND ramp via the cursor-time helper.
            cursor_t = self._mission_cursor_t(elapsed)
            if cursor_t > self.setpoints[-1].t_ref_s:
                self.report.mission_stopped_wall_s = now - self._wall_start
                self.get_logger().info(
                    "Segment cursor reached end. Entering OBSERVE_END."
                )
                self._transition("OBSERVE_END")

    # ---------- Abort ----------

    def _request_abort(self, reason: str) -> None:
        if self._abort_requested:
            return
        self._abort_requested = True
        self.report.aborted = True
        self.report.abort_reason = reason
        if reason and reason not in self.report.refusal_reasons:
            self.report.refusal_reasons.append(reason)
        self.get_logger().error(f"abort requested: {reason}")

    def _step_abort_path(self, now: float) -> None:
        if self._state != "ABORT_LANDING":
            armed_now = (
                self._last_status is not None
                and self._last_status.arming_state == ARMING_STATE_ARMED
            )
            had_armed = (
                self.report.arm_attempted or self.report.arm_reached
            )
            if (armed_now or had_armed) and not self.report.land_sent:
                self._send_land()
                self._transition("ABORT_LANDING")
                return
            self._stream_active = False
            self._transition("DONE")
            return

        disarmed = (
            self._last_status is not None
            and self._last_status.arming_state == ARMING_STATE_DISARMED
        )
        if disarmed:
            self.report.disarm_seen = True
            self.get_logger().info("Disarm confirmed after abort-Land.")
            self._stream_active = False
            self._transition("DONE")
            return
        if (now - self._state_entered_wall) > ABORT_DISARM_TIMEOUT_S:
            self._send_force_disarm()
            self._stream_active = False
            self._transition("DONE")

    def trigger_sigint_abort(self) -> None:
        if not self._abort_requested:
            self.get_logger().warning(
                "SIGINT received. Initiating safe abort."
            )
        self._request_abort("sigint")

    def is_done(self) -> bool:
        return self._state == "DONE"

    # ---------- Tick log ----------

    def _append_tick_log(self, now: float) -> None:
        lpos = self._last_lpos
        status = self._last_status
        sp = None
        cmd_t = math.nan
        cursor_t_val = math.nan
        cursor_rate_val = math.nan
        if self._state == "MISSION" and self._mission_start_wall is not None:
            elapsed = now - self._mission_start_wall
            if self._mission_idx < len(self.setpoints):
                sp = self.setpoints[self._mission_idx]
                cmd_t = sp.t_ref_s
            cursor_t_val = self._mission_cursor_t(elapsed)
            cursor_rate_val = self._mission_cursor_rate(elapsed)
        elif self._state in ("PRE_STREAM", "WAIT_OFFBOARD", "WAIT_ARMED",
                             "SETTLE_HOLD") and not math.isnan(self._hold_x):
            sp = Setpoint(
                t_ref_s=math.nan,
                x_ned_m=self._hold_x,
                y_ned_m=self._hold_y,
                z_ned_m=self._hold_z_ned,
                yaw_ned_rad=math.nan,
                segment_id=-1,
                segment_type="hold",
                segment_name="hold",
            )

        if sp is not None and lpos is not None:
            err_x = lpos.x - sp.x_ned_m
            err_y = lpos.y - sp.y_ned_m
            err_z = lpos.z - sp.z_ned_m
            err_abs = max(abs(err_x), abs(err_y), abs(err_z))
        else:
            err_x = err_y = err_z = err_abs = math.nan

        # Velocity feedforward (post-scaling) — only meaningful during MISSION.
        vff_enabled = int(bool(
            getattr(self.args, "use_velocity_feedforward", False)
        ))
        if (vff_enabled and self._state == "MISSION" and sp is not None
                and not math.isnan(cursor_rate_val)):
            scale = float(self.args.speed_scale) * float(cursor_rate_val)
            sp_vx_ff = float(getattr(sp, "vx_ned_mps", 0.0)) * scale
            sp_vy_ff = float(getattr(sp, "vy_ned_mps", 0.0)) * scale
            sp_vz_ff = float(getattr(sp, "vz_ned_mps", 0.0)) * scale
        else:
            sp_vx_ff = sp_vy_ff = sp_vz_ff = math.nan

        # lpos extras.
        if lpos is not None:
            lpos_speed = math.sqrt(lpos.vx ** 2 + lpos.vy ** 2 + lpos.vz ** 2)
            v_xy_valid = int(bool(getattr(lpos, "v_xy_valid", False)))
            v_z_valid = int(bool(getattr(lpos, "v_z_valid", False)))
        else:
            lpos_speed = math.nan
            v_xy_valid = v_z_valid = 0

        # FailsafeFlags curated subset (-1 if message absent).
        ff = self._last_failsafe_flags
        def _ff(name):
            return int(bool(getattr(ff, name))) if ff is not None else -1
        ff_vals = {f"ff_{n}": _ff(n) for n in _FAILSAFE_FLAGS_TICK_FIELDS}

        # VehicleControlMode (-1 if absent).
        vcm = self._last_vehicle_control_mode
        def _vcm(name):
            return int(bool(getattr(vcm, name))) if vcm is not None else -1
        vcm_vals = {f"vcm_{n}": _vcm(n) for n in _VEHICLE_CONTROL_MODE_TICK_FIELDS}

        # BatteryStatus.
        bat = self._last_battery_status
        if bat is not None:
            bat_voltage = float(getattr(bat, "voltage_v", math.nan))
            bat_rem = float(getattr(bat, "remaining", math.nan))
            bat_warn = int(getattr(bat, "warning", 0))
        else:
            bat_voltage = math.nan
            bat_rem = math.nan
            bat_warn = -1

        self.tick_log.append(
            TickLog(
                wall_s=now - self._wall_start,
                state=self._state,
                cmd_t_ref_s=cmd_t,
                cur_segment_id=(sp.segment_id if sp else -1),
                cur_segment_name=(sp.segment_name if sp else ""),
                cur_segment_type=(sp.segment_type if sp else ""),
                cursor_t=cursor_t_val,
                cursor_rate=cursor_rate_val,
                sp_x=(sp.x_ned_m if sp else math.nan),
                sp_y=(sp.y_ned_m if sp else math.nan),
                sp_z=(sp.z_ned_m if sp else math.nan),
                sp_yaw=(sp.yaw_ned_rad if sp else math.nan),
                sp_vx_ff=sp_vx_ff,
                sp_vy_ff=sp_vy_ff,
                sp_vz_ff=sp_vz_ff,
                vff_enabled=vff_enabled,
                lpos_x=(lpos.x if lpos else math.nan),
                lpos_y=(lpos.y if lpos else math.nan),
                lpos_z=(lpos.z if lpos else math.nan),
                lpos_vx=(lpos.vx if lpos else math.nan),
                lpos_vy=(lpos.vy if lpos else math.nan),
                lpos_vz=(lpos.vz if lpos else math.nan),
                lpos_speed_mps=lpos_speed,
                lpos_xy_valid=(
                    int(bool(lpos.xy_valid)) if lpos else 0
                ),
                lpos_z_valid=(int(bool(lpos.z_valid)) if lpos else 0),
                lpos_v_xy_valid=v_xy_valid,
                lpos_v_z_valid=v_z_valid,
                nav_state=(int(status.nav_state) if status else -1),
                arming_state=(int(status.arming_state) if status else -1),
                failsafe=(int(bool(status.failsafe)) if status else 0),
                err_x=err_x,
                err_y=err_y,
                err_z=err_z,
                err_abs_max=err_abs,
                **ff_vals,
                **vcm_vals,
                bat_voltage_v=bat_voltage,
                bat_remaining=bat_rem,
                bat_warning=bat_warn,
            )
        )

        # First-failsafe rising edge + OFFBOARD-drop transition detection
        # capture full diagnostic snapshots once each, written to
        # run_metrics.yaml by write_outputs.
        cur_fs = bool(status.failsafe) if status is not None else None
        cur_ns = int(status.nav_state) if status is not None else None
        if (cur_fs is not None and self._prev_failsafe is not None
                and self._first_failsafe_snapshot is None
                and (not self._prev_failsafe) and cur_fs):
            self._first_failsafe_snapshot = self._build_diag_snapshot(
                now, trigger="first_failsafe_rising_edge",
                extra={"prev_failsafe": False, "new_failsafe": True,
                       "prev_nav_state": self._prev_nav_state,
                       "new_nav_state": cur_ns},
            )
        if (cur_ns is not None and self._prev_nav_state is not None
                and self._offboard_drop_snapshot is None
                and self._state == "MISSION"
                and self._prev_nav_state == NAVIGATION_STATE_OFFBOARD
                and cur_ns != NAVIGATION_STATE_OFFBOARD):
            self._offboard_drop_snapshot = self._build_diag_snapshot(
                now, trigger="nav_state_leaves_offboard_in_mission",
                extra={"prev_nav_state": self._prev_nav_state,
                       "new_nav_state": cur_ns,
                       "prev_failsafe": self._prev_failsafe,
                       "new_failsafe": cur_fs},
            )
        if cur_fs is not None:
            self._prev_failsafe = cur_fs
        if cur_ns is not None:
            self._prev_nav_state = cur_ns

    def _build_diag_snapshot(self, now: float, trigger: str,
                              extra: dict) -> dict:
        """Full diagnostic snapshot at this tick. Used for both first_failsafe
        and offboard_drop blocks in run_metrics.yaml."""
        lpos = self._last_lpos
        status = self._last_status
        ff = self._last_failsafe_flags
        vcm = self._last_vehicle_control_mode
        bat = self._last_battery_status
        esf = self._last_estimator_status_flags
        es = self._last_estimator_status

        # Dump ALL FailsafeFlags fields if present.
        ff_full: dict = {}
        if ff is not None:
            for attr in dir(ff):
                if attr.startswith("_") or callable(getattr(ff, attr, None)):
                    continue
                if attr in ("SLOT_TYPES",) or attr.isupper():
                    continue
                try:
                    v = getattr(ff, attr)
                    if hasattr(v, "__len__") and not isinstance(v, str):
                        continue  # skip arrays
                    if isinstance(v, bool):
                        ff_full[attr] = bool(v)
                    elif isinstance(v, (int, float)):
                        ff_full[attr] = (int(v) if isinstance(v, int)
                                         else float(v))
                except (AttributeError, TypeError):
                    continue

        def _msg_dump(m, fields=None):
            if m is None:
                return None
            out = {}
            if fields is None:
                fields = [a for a in dir(m)
                          if not a.startswith("_")
                          and not callable(getattr(m, a, None))
                          and a not in ("SLOT_TYPES",) and not a.isupper()]
            for a in fields:
                try:
                    v = getattr(m, a)
                    if hasattr(v, "__len__") and not isinstance(v, str):
                        continue
                    if isinstance(v, bool):
                        out[a] = bool(v)
                    elif isinstance(v, (int, float)):
                        out[a] = (int(v) if isinstance(v, int) else float(v))
                except (AttributeError, TypeError):
                    pass
            return out

        # Surrounding tick window (~2 s = ~20 ticks at 10 Hz control rate).
        cur_wall = now - self._wall_start
        before = [t for t in self.tick_log
                  if (cur_wall - 2.0) <= t.wall_s <= cur_wall]
        # next-2-s slice will be appended after write_outputs has the full
        # tick log; we approximate here by taking the last 20 logged ticks.
        # The "next 2 s" block is filled in finalize_metrics if we keep
        # logging past the trigger event.
        snap = {
            "trigger": trigger,
            "wall_s": cur_wall,
            "node_state": self._state,
            "extra": extra,
            "vehicle_status": {
                "nav_state": int(status.nav_state) if status is not None else -1,
                "arming_state": int(status.arming_state)
                if status is not None else -1,
                "failsafe": bool(status.failsafe)
                if status is not None else None,
            },
            "vehicle_local_position": _msg_dump(
                lpos,
                fields=("x", "y", "z", "vx", "vy", "vz",
                        "xy_valid", "z_valid", "v_xy_valid", "v_z_valid",
                        "xy_global", "z_global",
                        "xy_reset_counter", "z_reset_counter",
                        "heading_reset_counter",
                        "heading_good_for_control", "heading_var",
                        "dist_bottom", "dist_bottom_valid"),
            ),
            "failsafe_flags_full": ff_full,
            "vehicle_control_mode": _msg_dump(vcm),
            "battery_status": _msg_dump(bat),
            "estimator_status_flags": _msg_dump(esf),
            "estimator_status": _msg_dump(es),
            "prev_2s_summary": {
                "n_ticks": len(before),
                "nav_states": sorted({t.nav_state for t in before}),
                "arming_states": sorted({t.arming_state for t in before}),
                "failsafe_ticks": sum(1 for t in before if t.failsafe == 1),
                "err_max_m": max(
                    (t.err_abs_max for t in before
                     if not math.isnan(t.err_abs_max)),
                    default=math.nan,
                ),
                "lpos_speed_max_mps": max(
                    (t.lpos_speed_mps for t in before
                     if not math.isnan(t.lpos_speed_mps)),
                    default=math.nan,
                ),
            },
        }
        return snap

    # ---------- Verdict ----------

    def _finalize_verdict(self) -> None:
        if self.report.aborted and self.report.abort_reason != "":
            self.report.verdict = "FAIL"
            return
        if self.report.dry_run:
            self.report.verdict = "DRY_RUN_OK"
            return
        ok = (
            self.report.topics_visible
            and self.report.ekf_ready_reached
            and self.report.nav_state_offboard_reached
            and self.report.arm_reached
            and self.report.settle_completed
            and self.report.mission_started
            and self.report.mission_completed
            and (not self.report.tracking_window_breach)
            and (not self.report.nav_state_left_offboard_in_mission)
            and self.report.disarm_seen
        )
        self.report.verdict = "PASS" if ok else "FAIL"

    def finalize_metrics(self) -> None:
        if self._publish_intervals_s:
            hz = sorted(
                1.0 / dt for dt in self._publish_intervals_s if dt > 0
            )
            if hz:
                self.report.achieved_rate_p50_hz = _percentile(hz, 50.0)
                self.report.achieved_rate_p95_hz = _percentile(hz, 95.0)
                self.report.achieved_rate_p99_hz = _percentile(hz, 99.0)

        # Fill the next-2-s summary for each diagnostic snapshot
        # (we only have data up to "now" while building the snapshot; the
        # follow-up window is filled in here after the tick log is complete).
        def _fill_next_window(snap: Optional[dict]) -> None:
            if snap is None:
                return
            t0 = float(snap.get("wall_s", math.nan))
            if math.isnan(t0):
                snap["next_2s_summary"] = {"n_ticks": 0}
                return
            after = [t for t in self.tick_log
                     if t0 < t.wall_s <= t0 + 2.0]
            snap["next_2s_summary"] = {
                "n_ticks": len(after),
                "nav_states": sorted({t.nav_state for t in after}),
                "arming_states": sorted({t.arming_state for t in after}),
                "failsafe_ticks": sum(1 for t in after if t.failsafe == 1),
                "err_max_m": max(
                    (t.err_abs_max for t in after
                     if not math.isnan(t.err_abs_max)),
                    default=math.nan,
                ),
                "lpos_speed_max_mps": max(
                    (t.lpos_speed_mps for t in after
                     if not math.isnan(t.lpos_speed_mps)),
                    default=math.nan,
                ),
                "node_states": sorted({t.state for t in after}),
            }
        _fill_next_window(self._first_failsafe_snapshot)
        _fill_next_window(self._offboard_drop_snapshot)

        # Surface snapshots + topic-status into the RunReport so write_outputs
        # picks them up via the existing runtime argument.
        self.report.first_failsafe_snapshot = self._first_failsafe_snapshot
        self.report.offboard_drop_snapshot = self._offboard_drop_snapshot
        self.report.optional_topics_status = dict(self._optional_topics_status)

        if self.report.verdict == "PENDING":
            self._finalize_verdict()


def _percentile(sorted_values: List[float], pct: float) -> float:
    if not sorted_values:
        return math.nan
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    k = (len(sorted_values) - 1) * (pct / 100.0)
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return float(sorted_values[int(k)])
    frac = k - lo
    return float(
        sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * frac
    )


def _per_segment_metrics(tick_log: List[TickLog]) -> List[dict]:
    """Aggregate MISSION-state tick rows per (segment_id, segment_name).

    Returns a list in segment_id order, each entry:
      segment_id, segment_name, segment_type, n_rows, messages_sent (proxy),
      start_wall_s, end_wall_s, rmse_m, max_err_m, rolling_1s_max_err_m,
      mean_err_m, p95_err_m, segment_completed (heuristic: had any tick).
    Out-of-mission rows (hold, abort) are excluded; segment_id == -1 also
    skipped. Empty result means single-segment / pre-mission only.
    """
    by_key: dict = {}
    by_order: List[tuple] = []
    for t in tick_log:
        if t.state != "MISSION":
            continue
        sid = int(t.cur_segment_id)
        if sid < 0:
            continue
        key = (sid, t.cur_segment_name)
        if key not in by_key:
            by_key[key] = {
                "segment_id": sid,
                "segment_name": t.cur_segment_name,
                "segment_type": t.cur_segment_type,
                "n_rows": 0,
                "start_wall_s": t.wall_s,
                "end_wall_s": t.wall_s,
                "errs": [],
            }
            by_order.append(key)
        bucket = by_key[key]
        bucket["n_rows"] += 1
        bucket["end_wall_s"] = t.wall_s
        if not math.isnan(t.err_abs_max):
            bucket["errs"].append(float(t.err_abs_max))

    out: List[dict] = []
    for key in sorted(by_order):
        b = by_key[key]
        errs = b["errs"]
        if errs:
            rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
            emax = max(errs)
            emean = sum(errs) / len(errs)
            esorted = sorted(errs)
            p95 = esorted[int(0.95 * (len(esorted) - 1))]
        else:
            rmse = emax = emean = p95 = math.nan
        # Rolling 1 s max over the segment's tick window. Tick log is at
        # roughly 10 Hz from _control_tick; use a sliding window of ~10 ticks.
        window = max(1, int(TRACKING_WINDOW_S * 10))
        rolling_max = math.nan
        if errs:
            running = 0.0
            for i in range(len(errs)):
                lo = max(0, i - window + 1)
                w = max(errs[lo:i + 1])
                if math.isnan(rolling_max) or w > rolling_max:
                    rolling_max = w
            running = 0.0  # noqa: F841
        out.append({
            "segment_id": b["segment_id"],
            "segment_name": b["segment_name"],
            "segment_type": b["segment_type"],
            "n_rows": b["n_rows"],
            "messages_sent_approx": b["n_rows"],
            "start_wall_s": _finite_or_none(b["start_wall_s"]),
            "end_wall_s": _finite_or_none(b["end_wall_s"]),
            "duration_s": _finite_or_none(b["end_wall_s"] - b["start_wall_s"]),
            "tracking_rmse_m": _finite_or_none(rmse),
            "tracking_max_err_m": _finite_or_none(emax),
            "tracking_rolling_1s_max_err_m": _finite_or_none(rolling_max),
            "tracking_mean_err_m": _finite_or_none(emean),
            "tracking_p95_err_m": _finite_or_none(p95),
            "segment_completed": b["n_rows"] > 0,
        })
    return out


# -------------- Output writers --------------


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def write_outputs(
    out_dir: Path,
    args,
    setpoints_csv_path: Path,
    seg_report: SegmentLoadReport,
    runtime: RunReport,
    tick_log: List[TickLog],
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    # run_log.csv (per-tick).
    log_path = out_dir / "run_log.csv"
    with log_path.open("w", newline="") as f:
        f.write(",".join(TICK_LOG_FIELDS) + "\n")
        for t in tick_log:
            row = [_fmt_csv_field(getattr(t, k)) for k in TICK_LOG_FIELDS]
            f.write(",".join(row) + "\n")

    # run_metrics.yaml.
    metrics = {
        "verdict": runtime.verdict,
        "timestamp_iso": runtime.timestamp_iso,
        "mode": "armed_short_segment" if not runtime.dry_run else "dry_run",
        "arm_flag": runtime.arm_flag,
        "eyes_flag": runtime.eyes_flag,
        "armed_path_enabled": bool(runtime.arm_flag and runtime.eyes_flag),
        "topics_visible": runtime.topics_visible,
        "first_status_seen": runtime.first_status_seen,
        "first_lpos_seen": runtime.first_lpos_seen,
        "first_lpos_valid": runtime.first_lpos_valid,
        "failsafe_flags_subscribed": runtime.failsafe_flags_subscribed,
        "ekf_ready": {
            "reached": runtime.ekf_ready_reached,
            "time_s": _finite_or_none(runtime.ekf_ready_time_s),
        },
        "hold_setpoint_ned_m": {
            "x": _finite_or_none(runtime.hold_x),
            "y": _finite_or_none(runtime.hold_y),
            "z": _finite_or_none(runtime.hold_z_ned),
        },
        "handshake": {
            "nav_state_after_handshake": runtime.nav_state_after_handshake,
            "nav_state_offboard_reached": runtime.nav_state_offboard_reached,
            "arm_attempted": runtime.arm_attempted,
            "arm_reached": runtime.arm_reached,
        },
        "settle": {
            "completed": runtime.settle_completed,
            "time_s": _finite_or_none(runtime.settle_time_s),
            "max_abs_err_m": _finite_or_none(runtime.settle_max_abs_err_m),
        },
        "mission": {
            "started": runtime.mission_started,
            "completed": runtime.mission_completed,
            "started_wall_s": _finite_or_none(runtime.mission_started_wall_s),
            "stopped_wall_s": _finite_or_none(runtime.mission_stopped_wall_s),
            "messages_sent": runtime.mission_messages_sent,
            "achieved_rate_hz": {
                "p50": _finite_or_none(runtime.achieved_rate_p50_hz),
                "p95": _finite_or_none(runtime.achieved_rate_p95_hz),
                "p99": _finite_or_none(runtime.achieved_rate_p99_hz),
                "target": float(args.rate_hz),
            },
            "tracking": {
                "max_err_m": _finite_or_none(runtime.tracking_max_err_m),
                "rolling_1s_max_err_m": _finite_or_none(
                    runtime.tracking_rolling_max_err_m
                ),
                "window_breach": runtime.tracking_window_breach,
                "max_tracking_error_m_setting": float(
                    args.max_tracking_error_m
                ),
            },
            "nav_state_left_offboard_in_mission": (
                runtime.nav_state_left_offboard_in_mission
            ),
        },
        "land_disarm": {
            "land_sent": runtime.land_sent,
            "disarm_seen": runtime.disarm_seen,
            "force_disarm_sent": runtime.force_disarm_sent,
        },
        "nav_states_observed": sorted(runtime.nav_states_observed),
        "arming_states_observed": sorted(runtime.arming_states_observed),
        "aborted": runtime.aborted,
        "abort_reason": runtime.abort_reason,
        "refusal_reasons": list(runtime.refusal_reasons),
        "segment": {
            "segment_id": seg_report.segment_id,
            "segment_name": seg_report.segment_name,
            "n_rows_selected": seg_report.n_rows_selected,
            "selected_duration_s": _finite_or_none(
                seg_report.selected_duration_s
            ),
            "selected_duration_stretched_s": _finite_or_none(
                seg_report.selected_duration_stretched_s
            ),
            "bounds_ned_m": {
                "x_min": _finite_or_none(seg_report.x_min),
                "x_max": _finite_or_none(seg_report.x_max),
                "y_min": _finite_or_none(seg_report.y_min),
                "y_max": _finite_or_none(seg_report.y_max),
                "z_min": _finite_or_none(seg_report.z_min),
                "z_max": _finite_or_none(seg_report.z_max),
            },
        },
        "safety_caps": {
            "speed_scale_hard_cap": SPEED_SCALE_HARD_CAP,
            "max_altitude_hard_cap_m": MAX_ALTITUDE_HARD_CAP_M,
            "tracking_window_s": TRACKING_WINDOW_S,
        },
        "multi_segment_enabled": bool(getattr(args, "multi_segment", False)),
        "per_segment": _per_segment_metrics(tick_log),
        "diagnostics": {
            "optional_topics_status": runtime.optional_topics_status or {},
            "first_failsafe_tick": runtime.first_failsafe_snapshot,
            "offboard_drop_tick": runtime.offboard_drop_snapshot,
        },
    }
    metrics_path = out_dir / "run_metrics.yaml"
    metrics_path.write_text(
        yaml.safe_dump(metrics, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )

    # verdict.yaml — minimal top-level for quick eyeballing.
    verdict_path = out_dir / "verdict.yaml"
    verdict_path.write_text(
        yaml.safe_dump(
            {
                "verdict": runtime.verdict,
                "mode": "armed_short_segment"
                if not runtime.dry_run
                else "dry_run",
                "armed_path_enabled": bool(
                    runtime.arm_flag and runtime.eyes_flag
                ),
                "aborted": runtime.aborted,
                "abort_reason": runtime.abort_reason,
                "mission_completed": runtime.mission_completed,
                "disarm_seen": runtime.disarm_seen,
                "tracking_window_breach": runtime.tracking_window_breach,
            },
            sort_keys=False,
            default_flow_style=False,
        ),
        encoding="utf-8",
    )

    # manifest.yaml.
    manifest = {
        "baseline": "offboard_mission_execution",
        "purpose": (
            "Gate 4C short-segment OFFBOARD execution of Gate-3 leg 1 "
            "(start_to_poi1) using Gate-4A transformed setpoints. First "
            "armed/OFFBOARD node in the pipeline; default behavior is "
            "dry-run."
        ),
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "host": {
            "user": os.environ.get("USER", "unknown"),
            "hostname": _safe_hostname(),
        },
        "git": {"repository": _git_info()},
        "inputs": {
            "setpoints_csv": str(setpoints_csv_path),
            "setpoints_csv_sha256": (
                sha256_file(setpoints_csv_path)
                if setpoints_csv_path.is_file()
                else None
            ),
        },
        "parameters": {
            "arm_flag": bool(args.arm),
            "eyes_flag": bool(args.i_have_physical_eyes_on_the_sim),
            "armed_path_enabled": bool(
                args.arm and args.i_have_physical_eyes_on_the_sim
            ),
            "segment_id": int(args.segment_id),
            "segment_name": str(args.segment_name),
            "segment_frac": float(args.segment_frac),
            "speed_scale": float(args.speed_scale),
            "rate_hz": float(args.rate_hz),
            "takeoff_altitude_m": float(args.takeoff_altitude_m),
            "max_altitude_m": float(args.max_altitude_m),
            "max_tracking_error_m": float(args.max_tracking_error_m),
            "lpos_watchdog_s": float(args.lpos_watchdog_s),
            "xy_bounds_rel": float(args.xy_bounds_rel),
            "max_run_s": float(args.max_run_s),
        },
        "safety": {
            "vehicle_command_publisher_present": True,
            "armed_path_two_flag_gate": [
                "--arm",
                "--i-have-physical-eyes-on-the-sim",
            ],
            "armed_path_actually_enabled": bool(
                args.arm and args.i_have_physical_eyes_on_the_sim
            ),
            "publishes": [
                "/fmu/in/offboard_control_mode",
                "/fmu/in/trajectory_setpoint",
                "/fmu/in/vehicle_command (only on armed path)",
            ],
            "subscribes": [
                "/fmu/out/vehicle_status_v1",
                "/fmu/out/vehicle_local_position",
                "/fmu/out/failsafe_flags",
            ],
            "abort_path": (
                "Land + WAIT_DISARM (timeout "
                f"{DISARM_TIMEOUT_S:.0f} s) -> force-disarm fallback. SIGINT "
                "triggers same abort path if armed; just stops streaming "
                "otherwise."
            ),
        },
        "result_verdict": runtime.verdict,
    }
    manifest_path = out_dir / "manifest.yaml"
    manifest_path.write_text(
        yaml.safe_dump(manifest, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )

    # Optional tracking plot.
    if args.write_plot:
        _maybe_write_plot(out_dir, tick_log)

    # hashes.txt.
    hashes_path = out_dir / "hashes.txt"
    lines = []
    for name in (
        "run_metrics.yaml",
        "verdict.yaml",
        "run_log.csv",
        "manifest.yaml",
        "tracking_plot.png",
    ):
        p = out_dir / name
        if p.is_file():
            lines.append(f"{sha256_file(p)}  {name}")
    hashes_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _maybe_write_plot(out_dir: Path, tick_log: List[TickLog]) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    if not tick_log:
        return
    ts = [t.wall_s for t in tick_log]
    sp_x = [t.sp_x for t in tick_log]
    sp_y = [t.sp_y for t in tick_log]
    lp_x = [t.lpos_x for t in tick_log]
    lp_y = [t.lpos_y for t in tick_log]
    err = [t.err_abs_max for t in tick_log]
    fig, (ax_xy, ax_err) = plt.subplots(2, 1, figsize=(10, 8))
    ax_xy.plot(sp_x, sp_y, "b-", label="setpoint NED xy")
    ax_xy.plot(lp_x, lp_y, "r.", markersize=2, label="lpos NED xy")
    ax_xy.set_xlabel("x_ned [m]")
    ax_xy.set_ylabel("y_ned [m]")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True)
    ax_xy.legend()
    ax_xy.set_title("Offboard execution - setpoint vs. estimated position")
    ax_err.plot(ts, err, "k-", label="|err| (max abs)")
    ax_err.set_xlabel("wall [s]")
    ax_err.set_ylabel("tracking error [m]")
    ax_err.grid(True)
    ax_err.legend()
    fig.tight_layout()
    fig.savefig(str(out_dir / "tracking_plot.png"), dpi=120)
    plt.close(fig)


def _finite_or_none(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, int):
        return v
    if v is None:
        return None
    try:
        fv = float(v)
    except (TypeError, ValueError):
        return None
    if math.isnan(fv) or math.isinf(fv):
        return None
    return fv


def _safe_hostname() -> str:
    try:
        return socket.gethostname()
    except OSError:
        return "unknown"


def _git_info() -> dict:
    info = {"commit": None, "branch": None}
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=os.getcwd(),
            stderr=subprocess.DEVNULL,
        )
        info["commit"] = commit.decode().strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    try:
        branch = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=os.getcwd(),
            stderr=subprocess.DEVNULL,
        )
        info["branch"] = branch.decode().strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    return info


# -------------- argparse + main --------------


def parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Gate 4C short-segment OFFBOARD execution node. Default behavior "
            "is dry-run. The OFFBOARD/ARM/segment path is only taken when "
            "BOTH --arm and --i-have-physical-eyes-on-the-sim are set."
        )
    )
    p.add_argument(
        "--setpoints-csv",
        type=str,
        default=DEFAULT_SETPOINTS_CSV,
        help="Path to Gate-4A transformed_setpoints.csv.",
    )
    p.add_argument(
        "--out-dir",
        type=str,
        default=DEFAULT_OUT_DIR,
        help="Output artifact directory.",
    )
    p.add_argument(
        "--segment-id",
        type=int,
        default=DEFAULT_SEGMENT_ID,
        help="segment_id to execute (default 1 = leg start_to_poi1).",
    )
    p.add_argument(
        "--segment-name",
        type=str,
        default=DEFAULT_SEGMENT_NAME,
        help="segment_name to execute (default start_to_poi1).",
    )
    p.add_argument(
        "--segment-frac",
        type=float,
        default=0.2,
        help=(
            "Fraction of the segment (0..1] to execute. Defaults to 0.2 "
            "for a truly short Gate-4C run."
        ),
    )
    p.add_argument(
        "--speed-scale",
        type=float,
        default=0.5,
        help=(
            "Time-stretch factor applied to t_ref ONLY (geometry untouched). "
            f"Default 0.5. Hard cap {SPEED_SCALE_HARD_CAP:.2f}."
        ),
    )
    p.add_argument(
        "--rate-hz",
        type=float,
        default=20.0,
        help="Setpoint publish rate (Hz). Default 20.",
    )
    p.add_argument(
        "--takeoff-altitude-m",
        type=float,
        default=3.0,
        help="Takeoff altitude AGL (NED-z target is -|value|). Default 3.0.",
    )
    p.add_argument(
        "--max-altitude-m",
        type=float,
        default=MAX_ALTITUDE_HARD_CAP_M,
        help=(
            "Hard cap on |z| during the run. Default and absolute hard cap: "
            f"{MAX_ALTITUDE_HARD_CAP_M:.2f} m."
        ),
    )
    p.add_argument(
        "--max-tracking-error-m",
        type=float,
        default=1.5,
        help=(
            "Rolling 1 s max(|lpos - setpoint|) abort threshold during "
            "MISSION. Default 1.5 m."
        ),
    )
    p.add_argument(
        "--lpos-watchdog-s",
        type=float,
        default=0.5,
        help="Max allowed gap on /fmu/out/vehicle_local_position (seconds).",
    )
    p.add_argument(
        "--watchdog-s",
        type=float,
        default=3.0,
        help="Max allowed gap on /fmu/out/vehicle_status_v1 (seconds).",
    )
    p.add_argument(
        "--xy-bounds-rel",
        type=float,
        default=5.0,
        help=(
            "Tolerance around the segment XY envelope (meters) before "
            "aborting in MISSION."
        ),
    )
    p.add_argument(
        "--ekf-ready-timeout-s",
        type=float,
        default=120.0,
        help="Max wait for EKF readiness + stability.",
    )
    p.add_argument(
        "--ekf-stable-s",
        type=float,
        default=20.0,
        help="Rolling EKF stability window (Gate-0B-equivalent).",
    )
    p.add_argument(
        "--pre-stream-s",
        type=float,
        default=2.0,
        help="Pre-OFFBOARD setpoint streaming duration (seconds).",
    )
    p.add_argument(
        "--handshake-timeout-s",
        type=float,
        default=2.0,
        help="Max wait for OFFBOARD / ARM confirmation (seconds).",
    )
    p.add_argument(
        "--end-hold-s",
        type=float,
        default=2.0,
        help="Hold-at-last-setpoint window after segment end before Land.",
    )
    p.add_argument(
        "--max-run-s",
        type=float,
        default=120.0,
        help="Wallclock cap for the whole run (seconds).",
    )
    p.add_argument(
        "--arm",
        action="store_true",
        default=False,
        help=(
            "Enable the OFFBOARD/ARM/segment path. STILL requires "
            "--i-have-physical-eyes-on-the-sim — alone, this flag does "
            "nothing."
        ),
    )
    p.add_argument(
        "--i-have-physical-eyes-on-the-sim",
        dest="i_have_physical_eyes_on_the_sim",
        action="store_true",
        default=False,
        help=(
            "Operator-watch acknowledgement. Required together with --arm "
            "to actually arm + go OFFBOARD."
        ),
    )
    p.add_argument(
        "--write-plot",
        action="store_true",
        default=False,
        help="If set, write tracking_plot.png (matplotlib required).",
    )
    p.add_argument(
        "--dry-load",
        action="store_true",
        default=False,
        help=(
            "Load + validate the segment, write a report, and exit without "
            "spinning ROS at all."
        ),
    )
    p.add_argument(
        "--multi-segment",
        action="store_true",
        default=False,
        help=(
            "Gate 5C full-mission mode. When set, do NOT filter by "
            "--segment-id / --segment-name; consume all CSV rows in order. "
            "Per-row segment_id / segment_name / segment_type are preserved "
            "on each setpoint and surface in run_log.csv as "
            "cur_segment_id / cur_segment_name / cur_segment_type, and a "
            "per_segment block is added to run_metrics.yaml. --segment-frac "
            "is ignored in this mode."
        ),
    )
    # ---- Gate 5C velocity-aware SETTLE + mission-start hold ----
    p.add_argument(
        "--settle-pos-threshold-m",
        type=float, default=0.20,
        help=(
            "SETTLE position-error threshold (m). max(|err_x|,|err_y|,|err_z|) "
            "must drop below this AND velocity below --settle-vel-threshold-mps "
            "continuously for --settle-stable-s before MISSION starts. "
            "Default 0.20 (tighter than the pre-Gate-5C 0.30 to absorb "
            "residual climb velocity)."
        ),
    )
    p.add_argument(
        "--settle-vel-threshold-mps",
        type=float, default=0.12,
        help=(
            "SETTLE local-speed threshold (m/s). sqrt(vx^2+vy^2+vz^2) must "
            "drop below this together with the pos threshold for "
            "--settle-stable-s seconds. v_xy_valid AND v_z_valid are required."
        ),
    )
    p.add_argument(
        "--settle-stable-s",
        type=float, default=2.0,
        help=(
            "Required continuous-stable duration (s) inside the pos+vel "
            "thresholds before SETTLE completes. Resets on any threshold "
            "exceedance. Default 2.0."
        ),
    )
    p.add_argument(
        "--mission-start-hold-s",
        type=float, default=2.0,
        help=(
            "After SETTLE passes, hold the cursor at setpoints[0] for this "
            "many seconds before advancing. Lets the controller catch up to "
            "the (potentially nonzero-yaw) first mission setpoint without "
            "the cursor running ahead. Default 2.0."
        ),
    )
    p.add_argument(
        "--mission-start-ramp-s",
        type=float, default=8.0,
        help=(
            "After the mission-start hold, ramp the cursor advance rate "
            "smoothly from 0 to the requested speed_scale over this many "
            "seconds. The cursor time-derivative goes linearly "
            "from 0 to 1, so cursor_t(tau) = tau^2/(2*ramp_s) during the "
            "ramp and (ramp_s/2) + (tau - ramp_s) after. Total mission wall "
            "time grows by ramp_s/2. End-of-mission detection accounts for "
            "this. Set to 0 to disable the ramp and keep pre-v3 behavior. "
            "Default 8.0."
        ),
    )
    p.add_argument(
        "--use-velocity-feedforward",
        action="store_true",
        default=False,
        help=(
            "When set, publish TrajectorySetpoint.velocity = "
            "csv_velocity * speed_scale * cursor_rate (NaN otherwise). "
            "cursor_rate is 0 during the mission-start hold, ramps linearly "
            "from 0 to 1 during the start ramp, and is 1 thereafter. The "
            "CSV must contain vx_ned_mps / vy_ned_mps / vz_ned_mps columns; "
            "otherwise main() refuses to spin. Geometry, yaw, two-flag arm "
            "gate, dirty-state guard, and abort/land/disarm logic are "
            "unchanged."
        ),
    )
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    setpoints_csv_path = Path(args.setpoints_csv).resolve()
    out_dir = Path(args.out_dir).resolve()

    armed_path = bool(args.arm and args.i_have_physical_eyes_on_the_sim)
    print(
        f"[offboard] armed_path={armed_path} speed_scale={args.speed_scale} "
        f"multi_segment={args.multi_segment} "
        f"segment_id={args.segment_id} segment_name={args.segment_name} "
        f"segment_frac={args.segment_frac} csv={setpoints_csv_path} "
        f"out_dir={out_dir}",
        flush=True,
    )

    setpoints, seg_report = load_segment(
        setpoints_csv_path,
        args.segment_id,
        args.segment_name,
        args.segment_frac,
        args.speed_scale,
        multi_segment=bool(args.multi_segment),
    )
    # Refuse to spin if --use-velocity-feedforward is set but the
    # CSV lacks vx_ned_mps / vy_ned_mps / vz_ned_mps. Geometry-only fallback
    # would silently publish zero velocity feedforward, defeating the flag.
    if (
        getattr(args, "use_velocity_feedforward", False)
        and not getattr(seg_report, "has_velocity_cols", False)
    ):
        seg_report.errors.append(
            "use_velocity_feedforward_set_but_csv_missing_velocity_columns:"
            f"{','.join(VELOCITY_CSV_COLUMNS)}"
        )
    if seg_report.errors:
        print(
            f"[offboard] segment load errors: {seg_report.errors}",
            file=sys.stderr,
            flush=True,
        )
        runtime = RunReport(
            timestamp_iso=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            arm_flag=bool(args.arm),
            eyes_flag=bool(args.i_have_physical_eyes_on_the_sim),
            dry_run=(not armed_path),
            speed_scale=float(args.speed_scale),
            takeoff_altitude_m=float(args.takeoff_altitude_m),
        )
        runtime.aborted = True
        runtime.abort_reason = "segment_load_failed"
        runtime.verdict = "FAIL"
        write_outputs(
            out_dir, args, setpoints_csv_path, seg_report, runtime, []
        )
        return 2

    if args.dry_load:
        runtime = RunReport(
            timestamp_iso=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            arm_flag=bool(args.arm),
            eyes_flag=bool(args.i_have_physical_eyes_on_the_sim),
            dry_run=(not armed_path),
            speed_scale=float(args.speed_scale),
            takeoff_altitude_m=float(args.takeoff_altitude_m),
        )
        runtime.verdict = "DRY_LOAD_OK"
        write_outputs(
            out_dir, args, setpoints_csv_path, seg_report, runtime, []
        )
        print(
            f"[offboard] dry-load OK: {seg_report.n_rows_selected} rows, "
            f"stretched_duration={seg_report.selected_duration_stretched_s:.2f} "
            "s",
            flush=True,
        )
        return 0

    if _SIGNAL_HANDLERS_NO is not None:
        rclpy.init(signal_handler_options=_SIGNAL_HANDLERS_NO)
    else:
        rclpy.init()

    node = OffboardMissionExecutor(args, setpoints, seg_report)

    sigint_state = {"count": 0}

    def _sigint(_signum, _frame):
        sigint_state["count"] += 1
        if sigint_state["count"] >= 2:
            print(
                "Second SIGINT received -- hard exit (no safe land!).",
                file=sys.stderr,
                flush=True,
            )
            os._exit(2)
        node.trigger_sigint_abort()

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    try:
        while rclpy.ok() and not node.is_done():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.finalize_metrics()
        write_outputs(
            out_dir,
            args,
            setpoints_csv_path,
            seg_report,
            node.report,
            node.tick_log,
        )
        node.get_logger().info(f"VERDICT={node.report.verdict}")
        node.destroy_node()
        rclpy.shutdown()

    return 0 if node.report.verdict in ("PASS", "DRY_RUN_OK", "DRY_LOAD_OK") else 1


if __name__ == "__main__":
    sys.exit(main())
