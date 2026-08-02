#!/usr/bin/env bash
# =============================================================================
# PX4 SITL + Gazebo stack for this repository.  SIMULATION ONLY.
# =============================================================================
# Brings the simulator up in the order that works, checks that each layer is
# actually alive before starting the next, and tears everything down again.
#
# This script never arms anything. It starts no mission node. Arming stays an
# explicit, separate, two-flag decision made by the operator.
#
#   bash tools/sitl_stack.sh start     # agent -> px4+gazebo -> bridge -> filter
#   bash tools/sitl_stack.sh status    # what is up, what topics are live, RTF
#   bash tools/sitl_stack.sh params    # apply the documented PX4 parameters
#   bash tools/sitl_stack.sh prearm    # leave AUTO_LOITER for Position mode
#   bash tools/sitl_stack.sh rtf       # measure the real-time factor over 12 s
#   bash tools/sitl_stack.sh stop      # kill everything, verify nothing remains
#
# WHY THE ORDER MATTERS
#
#   1. Micro XRCE-DDS Agent FIRST. PX4's uXRCE-DDS client tries to connect once,
#      early. If the agent is not listening yet the client gives up and no
#      /fmu/... topic ever appears - which looks exactly like "the simulator is
#      not publishing".
#   2. PX4 SECOND, and PX4 starts Gazebo itself. This matters: PX4's simulated
#      IMU, barometer, magnetometer and GPS are driven by Gazebo's sensor
#      systems through PX4's own gz_bridge, and that pipeline expects PX4 to
#      own the simulator's lifecycle. Attaching PX4 to a Gazebo server that was
#      already free-running produces "No valid data from Baro 0 / Gyro 0" and
#      an EKF that diverges by hundreds of metres while the vehicle sits still.
#
#      PX4 resolves worlds from $PX4_GZ_WORLDS, which its own gz_env.sh
#      normally overwrites. We give PX4 a scratch working directory with our
#      own gz_env.sh beside it, pointing PX4_GZ_WORLDS at this repository's
#      sim/worlds. So the world being simulated is provably the SDF in this
#      repository, and nothing is written into the PX4 tree.
#   3. Bridge THIRD. It can only discover the Gazebo LiDAR topic once the
#      vehicle model exists.
#   4. Self-filter LAST. It republishes /scan as /scan_filtered, which is the
#      perception input the Stop-Replan-Resume node subscribes to by default.
#
# ONE ARMED RUN PER STACK. A run that ends in a force-disarm leaves PX4 in a
# state where the next arming attempt reliably misbehaves (settle timeouts,
# altitude-guard aborts). Run `stop` and `start` again between armed runs.
# PX4 is launched with its working directory inside RUN_DIR, so its parameter
# store and every other runtime file live there and nowhere else. `stop`
# removes RUN_DIR and `start` recreates it, which is what makes the next stack
# genuinely virgin. Nothing under the PX4 build tree is touched.
# =============================================================================
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${PX4_DIR:=$HOME/PX4-Autopilot}"
: "${WORLD:=forest_map2}"
: "${WORLD_SDF:=$REPO/sim/worlds/${WORLD}.sdf}"
: "${PX4_SIM_MODEL:=gz_x500_lidar_2d}"
: "${SPAWN_POSE:=-50,-70,0.5,0,0,0}"     # world ENU; also the PX4 local-NED origin
: "${LOG_DIR:=/tmp/sitl_stack}"
: "${RUN_DIR:=/tmp/sitl_stack/px4_run}"   # PX4 scratch working directory
: "${ROS_DOMAIN_ID:=0}"
# Bracketed so this script's own command line never matches the pattern
# and pkill cannot take out the shell that is running it.
PX4_PATTERN="px4_sitl_default/bin/px[4] -d"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # the stack is validated on FastDDS only
export ROS_DOMAIN_ID
export GZ_FUEL_OFFLINE=1                     # tree meshes come from the local cache
# Without this, `ros2 topic list` answers from a cached daemon and reports
# topics belonging to a PX4 instance that has already been killed - which makes
# every readiness check below pass instantly against a dead stack.
export ROS_DISABLE_ROS2CLI_DAEMON=1
unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_LOCALHOST_ONLY ROS_STATIC_PEERS 2>/dev/null || true

sanitize_env() {
  # Gazebo renders the 2D LiDAR offscreen through EGL. Two things on a typical
  # desktop break that, and when they do the render blocks the server's main
  # loop: the simulator then stops answering service calls, the real-time
  # factor collapses, PX4 reports "No valid data from Baro 0", and the EKF
  # diverges by hundreds of metres while the vehicle sits on the ground.
  #
  #  1. A terminal launched from a snap-packaged editor (VS Code, for example)
  #     exports SNAP*, GTK_*, GDK_*, LOCPATH and a snap-prefixed
  #     LD_LIBRARY_PATH. Child processes then load the snap's GUI libraries
  #     instead of the system ones.
  #  2. On a hybrid-graphics laptop, glvnd tries the NVIDIA EGL vendor first.
  #     If its DRI driver is not installed, Mesa logs "failed to create dri2
  #     screen" and silently falls back to software rendering.
  #
  # Both are fixed here rather than documented as caveats.
  local v
  for v in $(compgen -v | grep -E '^(SNAP|GTK_|GDK_|GIO_|GSETTINGS_|VSCODE_|LOCPATH)' || true); do
    unset "$v" 2>/dev/null || true
  done
  unset LD_PRELOAD 2>/dev/null || true
  if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    LD_LIBRARY_PATH="$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' \
      | grep -v '^/snap/' | paste -sd: -)"
    export LD_LIBRARY_PATH
  fi
  if [ -f /usr/share/glvnd/egl_vendor.d/50_mesa.json ] \
     && [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
  fi
  # The offscreen (GBM / surfaceless) EGL path enumerates DRM devices and takes
  # the first one, which on a hybrid-graphics laptop is the discrete GPU whose
  # driver may not be usable. The X11 EGL platform binds to the display's own
  # GPU instead, which is by construction one that works. Only when there is a
  # display to bind to.
  if [ -n "${DISPLAY:-}" ]; then
    export EGL_PLATFORM=x11
  fi
}

log() { printf '[stack] %s\n' "$*"; }
die() { printf '[stack] FAIL: %s\n' "$*" >&2; exit 1; }

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash 2>/dev/null || die "ROS 2 Jazzy not found at /opt/ros/jazzy"
  # shellcheck disable=SC1091
  [ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
  set -u
}

wait_for() {  # wait_for <seconds> <description> <command...>
  local deadline=$1 desc=$2; shift 2
  local i=0
  while [ "$i" -lt "$deadline" ]; do
    if "$@" >/dev/null 2>&1; then log "ready: $desc (${i}s)"; return 0; fi
    sleep 1; i=$((i + 1))
  done
  return 1
}

topic_flowing() {  # topic_flowing <ros topic> <pkg.msg module> <MsgClass>
  TOPIC="$1" MODULE="$2" CLASS="$3" python3 - <<'PROBE' >/dev/null 2>&1
import importlib, os, sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
topic = os.environ["TOPIC"]
msg = getattr(importlib.import_module(os.environ["MODULE"]), os.environ["CLASS"])
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               durability=DurabilityPolicy.VOLATILE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
rclpy.init(); n = Node("topic_probe"); got = []
n.create_subscription(msg, topic, lambda m: got.append(m), q)
end = time.monotonic() + 3.0
while time.monotonic() < end and rclpy.ok() and len(got) < 3:
    rclpy.spin_once(n, timeout_sec=0.05)
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if len(got) >= 3 else 1)
PROBE
}

px4_telemetry_flowing() {
  python3 - <<'PROBE' >/dev/null 2>&1
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               durability=DurabilityPolicy.VOLATILE,
               history=HistoryPolicy.KEEP_LAST, depth=10)
rclpy.init(); n = Node("px4_flow_probe"); got = []
n.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                      lambda m: got.append(m), q)
end = time.monotonic() + 3.0
while time.monotonic() < end and rclpy.ok() and len(got) < 20:
    rclpy.spin_once(n, timeout_sec=0.05)
n.destroy_node(); rclpy.shutdown()
ok = len(got) >= 20 and abs(got[-1].z) < 5.0 and got[-1].z_valid and got[-1].xy_valid
sys.exit(0 if ok else 1)
PROBE
}

# ---------------------------------------------------------------------------
cmd_start() {
  sanitize_env
  source_ros
  [ -f "$WORLD_SDF" ] || die "world not found: $WORLD_SDF"
  [ -d "$PX4_DIR" ]   || die "PX4 not found: $PX4_DIR (set PX4_DIR)"
  local px4_bin="$PX4_DIR/build/px4_sitl_default/bin/px4"
  [ -x "$px4_bin" ] || die "PX4 binary not built: $px4_bin
  Build it once with:  cd $PX4_DIR && make px4_sitl"

  pgrep -f "$PX4_PATTERN" >/dev/null && die "PX4 is already running. Run 'stop' first."
  pgrep -f "gz sim"   >/dev/null && die "Gazebo is already running. Run 'stop' first."

  mkdir -p "$LOG_DIR"
  log "logs -> $LOG_DIR"
  log "world -> $WORLD_SDF"

  # -- 1. Micro XRCE-DDS Agent, before PX4 -----------------------------------
  log "1/4 starting Micro XRCE-DDS Agent"
  command -v MicroXRCEAgent >/dev/null || die "MicroXRCEAgent not on PATH"
  setsid MicroXRCEAgent udp4 -p 8888 >"$LOG_DIR/agent.log" 2>&1 &
  sleep 2
  pgrep -f MicroXRCEAgent >/dev/null || die "agent died; see $LOG_DIR/agent.log"

  # -- 2. PX4, which starts Gazebo on THIS repository's world ----------------
  log "2/4 starting PX4 SITL, which starts Gazebo on the repository world"
  # Recreated from scratch every time: PX4's parameter store and runtime files
  # live in here, so a stale one is exactly what makes a second armed run
  # misbehave. Nothing is written into the PX4 build tree.
  local rundir="$RUN_DIR/rootfs"
  rm -rf "$RUN_DIR"; mkdir -p "$rundir"
  # PX4's rcS sources ../gz_env.sh from beside its working directory and lets
  # that file define where worlds and plugins live. Writing our own is what
  # points PX4 at this repository's world without touching the PX4 tree.
  cat >"$RUN_DIR/gz_env.sh" <<EOF
export PX4_GZ_MODELS=$PX4_DIR/Tools/simulation/gz/models
export PX4_GZ_WORLDS=$REPO/sim/worlds
export PX4_GZ_PLUGINS=$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins
export PX4_GZ_SERVER_CONFIG=$PX4_DIR/src/modules/simulation/gz_bridge/server.config
export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\$PX4_GZ_MODELS:\$PX4_GZ_WORLDS
export GZ_SIM_SYSTEM_PLUGIN_PATH=\$GZ_SIM_SYSTEM_PLUGIN_PATH:\$PX4_GZ_PLUGINS
export GZ_SIM_SERVER_CONFIG_PATH=\$PX4_GZ_SERVER_CONFIG
EOF
  (
    cd "$rundir" || exit 1
    export PX4_SIM_MODEL PX4_GZ_WORLD="$WORLD" PX4_GZ_MODEL_POSE="$SPAWN_POSE"
    export HEADLESS=1     # no gz GUI client: it costs real-time factor
    setsid "$px4_bin" -d "$PX4_DIR/build/px4_sitl_default/etc" \
      -s etc/init.d-posix/rcS -w . >"$LOG_DIR/px4.log" 2>&1 &
  )
  wait_for 90 "gazebo world /$WORLD" \
    bash -c "gz topic -l 2>/dev/null | grep -q '^/world/$WORLD/clock$'" \
    || die "Gazebo did not come up; see $LOG_DIR/px4.log"
  # Three separate things have to become true, in this order. Checking only
  # the last one is how a half-started stack slips through.
  wait_for 90 "vehicle model in Gazebo" \
    bash -c "gz model --list 2>/dev/null | grep -q x500" \
    || die "PX4 never spawned the vehicle; see $LOG_DIR/px4.log"
  wait_for 60 "Gazebo LiDAR topic" \
    bash -c "gz topic -l 2>/dev/null | grep -q '/sensor/lidar_2d_v2/scan$'" \
    || die "the vehicle has no LiDAR sensor topic; see $LOG_DIR/px4.log"
  # Topic *presence* is not enough - PX4 must actually be publishing, and its
  # EKF must be sane. A diverging EKF here means the sensor pipeline is broken.
  wait_for 120 "PX4 telemetry flowing with a sane EKF" px4_telemetry_flowing \
    || die "PX4 publishes no usable local position; see $LOG_DIR/px4.log
  Look for 'No valid data from Baro 0' - that is the sensor pipeline failing."

  # -- 3. Gazebo -> ROS bridge -----------------------------------------------
  log "3/4 starting the Gazebo -> ROS bridge"
  local gz_scan gz_clock gz_pose cfg="$LOG_DIR/bridge.yaml"
  gz_scan="$(gz topic -l 2>/dev/null | grep '/sensor/lidar_2d_v2/scan$' | head -n1)"
  [ -n "$gz_scan" ] || die "no Gazebo LiDAR topic; the vehicle model has no sensor"
  gz_clock="/world/$WORLD/clock"
  gz_pose="/world/$WORLD/dynamic_pose/info"
  # Only /scan by default. Relaying Gazebo's 1 kHz /clock and the whole-world
  # pose array costs simulator time, and neither ROS node in this repository
  # reads them - they use wall time and PX4 telemetry. Set BRIDGE_FULL=1 when
  # you want RViz or the ground-truth recorder, which do need them.
  cat >"$cfg" <<EOF
- gz_topic_name: "$gz_scan"
  ros_topic_name: "/scan"
  gz_type_name: "gz.msgs.LaserScan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  direction: GZ_TO_ROS
  qos: { reliability: best_effort, durability: volatile, history: keep_last, depth: 10 }
EOF
  if [ "${BRIDGE_FULL:-0}" = "1" ]; then
    cat >>"$cfg" <<EOF

- gz_topic_name: "$gz_clock"
  ros_topic_name: "/clock"
  gz_type_name: "gz.msgs.Clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  direction: GZ_TO_ROS

- gz_topic_name: "$gz_pose"
  ros_topic_name: "/gz_pose_array"
  gz_type_name: "gz.msgs.Pose_V"
  ros_type_name: "geometry_msgs/msg/PoseArray"
  direction: GZ_TO_ROS
EOF
    log "BRIDGE_FULL=1: also relaying /clock and /gz_pose_array"
  fi
  setsid ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p config_file:="$cfg" >"$LOG_DIR/bridge.log" 2>&1 &
  # Not `ros2 topic list`: with the CLI daemon disabled it can report an empty
  # list while the topic is publishing at full rate. Subscribe for real.
  wait_for 45 "/scan carrying data" \
    topic_flowing /scan sensor_msgs.msg LaserScan \
    || die "bridge produced no /scan; see $LOG_DIR/bridge.log"

  # -- 4. LiDAR self-filter --------------------------------------------------
  log "4/4 starting the LiDAR self-filter (/scan -> /scan_filtered)"
  # Sim time needs /clock, which the minimal bridge does not relay.
  local SIM_TIME=false
  [ "${BRIDGE_FULL:-0}" = "1" ] && SIM_TIME=true
  setsid python3 "$REPO/ros2/nodes/scan_self_filter.py" --ros-args \
    -p use_sim_time:=$SIM_TIME -p input_topic:=/scan -p output_topic:=/scan_filtered \
    -p self_radius:=0.35 -p mask_sectors_deg:=-35:-8,8:35 -p mask_max_range:=0.70 \
    >"$LOG_DIR/scan_filter.log" 2>&1 &
  wait_for 30 "/scan_filtered carrying data" \
    topic_flowing /scan_filtered sensor_msgs.msg LaserScan \
    || die "self-filter produced no /scan_filtered; see $LOG_DIR/scan_filter.log"

  if grep -q "failed to create dri2 screen" "$LOG_DIR/px4.log" 2>/dev/null; then
    log "WARNING: Gazebo fell back to software rendering (see px4.log)."
    log "         Expect a low real-time factor and an unstable armed run."
  fi
  log "stack is up. Next: 'params', 'rtf', then spawn the obstacle."
  cmd_status
}

# Why each parameter is needed:
#   MPC_TKO_RAMP_T / MPC_TKO_SPEED - the default takeoff ramp is slow enough
#     that the settle stability window expires before the vehicle reaches the
#     reference altitude, and the run aborts on a settle timeout.
#   COM_RCL_EXCEPT 4 - no RC transmitter exists in SITL. Without the Offboard
#     exception PX4 declares RC loss and fails safe the moment it arms.
PX4_PARAMS=("MPC_TKO_RAMP_T 1.0" "MPC_TKO_SPEED 1.0" "COM_RCL_EXCEPT 4")

cmd_params() {
  # px4-param is the client half of the running SITL instance; it talks to the
  # daemon over its IPC socket, so no interactive PX4 console is needed.
  local px4_param="$PX4_DIR/build/px4_sitl_default/bin/px4-param"
  local rundir="$PX4_DIR/build/px4_sitl_default/rootfs"
  pgrep -f "$PX4_PATTERN" >/dev/null || die "PX4 is not running. Run 'start' first."
  [ -x "$px4_param" ] || die "px4-param not found: $px4_param"
  log "applying the documented PX4 SITL parameters"
  local kv name value
  for kv in "${PX4_PARAMS[@]}"; do
    name="${kv%% *}"; value="${kv##* }"
    (cd "$rundir" && "$px4_param" set "$name" "$value") >/dev/null 2>&1 \
      || die "param set $name failed"
  done
  (cd "$rundir" && "$px4_param" save) >/dev/null 2>&1
  for kv in "${PX4_PARAMS[@]}"; do
    name="${kv%% *}"
    (cd "$rundir" && "$px4_param" show "$name") 2>/dev/null \
      | grep -E "^[x+* ]*$name" | tail -1 | sed 's/^/[stack]   /'
  done
}

cmd_prearm() {
  # PX4 SITL boots into AUTO_LOITER (nav_state 4). The mission executor refuses
  # to arm on top of that, and it is right to: arming into a residual
  # OFFBOARD / RTL / failsafe state is how you get a run that does something
  # other than what you asked for. Position mode is the clean starting point.
  local px4_cmd="$PX4_DIR/build/px4_sitl_default/bin/px4-commander"
  local rundir="$PX4_DIR/build/px4_sitl_default/rootfs"
  pgrep -f "$PX4_PATTERN" >/dev/null || die "PX4 is not running. Run 'start' first."
  [ -x "$px4_cmd" ] || die "px4-commander not found: $px4_cmd"
  log "putting PX4 into Position mode (from its AUTO_LOITER boot state)"
  (cd "$rundir" && "$px4_cmd" mode posctl) >/dev/null 2>&1 \
    || die "could not switch PX4 to Position mode"
  sleep 2
  local mode
  mode="$( (cd "$rundir" && "$px4_cmd" status) 2>&1 \
           | grep -i "navigation mode:" | head -1 | sed 's/.*navigation mode: *//')"
  log "  navigation mode = ${mode:-unknown}"
  case "$mode" in
    Position*) log "  ready to arm" ;;
    *) die "expected Position, got '${mode:-unknown}'" ;;
  esac
}

cmd_rtf() {
  sanitize_env
  source_ros
  # Gazebo's own statistics topic is the authority on how fast physics runs.
  # The /clock-derived figure below measures something different and useful:
  # how well the ROS bridge keeps up. They can disagree by a lot, and only the
  # first one tells you whether an armed run will hold its tracking budget.
  log "Gazebo physics real-time factor (authoritative), 8 s"
  timeout 8 gz topic -e -t "/world/$WORLD/stats" 2>/dev/null \
    | grep -oP 'real_time_factor:\s*\K[0-9.]+' \
    | python3 -c "
import sys
v = [float(x) for x in sys.stdin]
if not v:
    print('[stack] no /world/$WORLD/stats - is Gazebo running?')
else:
    v.sort()
    med = v[len(v) // 2]
    # Median, not mean: a single multi-second stall (a subscriber attaching, a
    # page fault) drags the mean down while sustained pacing is fine. The p05
    # is printed so those stalls stay visible rather than being averaged away.
    print(f'[stack] physics RTF: median {med:.3f}  p05 {v[len(v)//20]:.3f}  '
          f'min {v[0]:.3f}  max {v[-1]:.3f}  (n={len(v)})')
    print('[stack] ' + ('OK - at or near real time.' if med >= 0.95 else
          'TOO SLOW. Armed runs pace on wall-clock; below ~0.95 the settle '
          'window and the tracking budget are both at risk. Close GUI clients '
          '(rviz, QGroundControl, gz sim -g) and retry.'))
"
  log "ROS /clock relay throughput over 12 s (bridge health, not physics speed)"
  python3 - <<'PY'
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
               durability=DurabilityPolicy.VOLATILE,
               history=HistoryPolicy.KEEP_LAST, depth=50)
rclpy.init(); n = Node("rtf_probe"); samples = []
n.create_subscription(Clock, "/clock",
                      lambda m: samples.append((time.monotonic(),
                                                m.clock.sec + m.clock.nanosec * 1e-9)), q)
end = time.monotonic() + 12
while time.monotonic() < end and rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.05)
if len(samples) > 20:
    w = [a for a, _ in samples]; c = [b for _, b in samples]
    overall = (c[-1] - c[0]) / (w[-1] - w[0])
    worst, i = 9.0, 0
    while i < len(samples) - 1:
        j = i
        while j < len(samples) - 1 and w[j] - w[i] < 1.0:
            j += 1
        if w[j] > w[i]:
            worst = min(worst, (c[j] - c[i]) / (w[j] - w[i]))
        i = j
    print(f"[stack] /clock relay: overall {overall:.3f}, worst 1 s window {worst:.3f}")
    print("[stack] A low value here means the ros_gz_bridge is not keeping up "
          "with the 1 kHz /clock stream. It does NOT by itself mean physics is "
          "slow - check the physics RTF above for that.")
else:
    print(f"[stack] only {len(samples)} /clock messages - is the bridge running?")
n.destroy_node(); rclpy.shutdown()
PY
}

cmd_status() {
  sanitize_env
  source_ros
  echo "--- processes ---"
  for p in "MicroXRCEAgent" "gz sim" "$PX4_PATTERN" "parameter_bridge" "scan_self_filter"; do
    if pgrep -f "$p" >/dev/null; then printf '  up    %s\n' "$p"
    else printf '  DOWN  %s\n' "$p"; fi
  done
  echo "--- topics (subscribed, not listed) ---"
  local spec t mod cls
  for spec in "/clock rosgraph_msgs.msg Clock" \
              "/scan sensor_msgs.msg LaserScan" \
              "/scan_filtered sensor_msgs.msg LaserScan" \
              "/gz_pose_array geometry_msgs.msg PoseArray" \
              "/fmu/out/vehicle_local_position px4_msgs.msg VehicleLocalPosition" \
              "/fmu/out/vehicle_status_v1 px4_msgs.msg VehicleStatus"; do
    read -r t mod cls <<<"$spec"
    if topic_flowing "$t" "$mod" "$cls"; then printf '  ok    %s\n' "$t"
    else printf '  NO DATA  %s\n' "$t"; fi
  done
  echo "--- gazebo models ---"
  gz model --list 2>/dev/null | sed -n '1,3p'
  gz model --list 2>/dev/null | grep -cE '^\s*-' | xargs -I{} echo "  {} models in the world"
}

cmd_stop() {
  log "stopping the stack"
  for p in scan_self_filter.py parameter_bridge ros_gz_bridge "$PX4_PATTERN" "gz sim" MicroXRCEAgent; do
    pkill -f "$p" 2>/dev/null
  done
  sleep 3
  for p in scan_self_filter.py parameter_bridge "$PX4_PATTERN" "gz sim" MicroXRCEAgent; do
    pkill -9 -f "$p" 2>/dev/null
  done
  sleep 1
  # PX4 ran with its working directory inside RUN_DIR, so its parameter store
  # is RUN_DIR/rootfs/eeprom and not the one under the PX4 build tree. A run
  # that ended in a force-disarm leaves parameters there that break the next
  # arming attempt, so the whole scratch directory goes. cmd_start recreates
  # it, which is what makes the next stack virgin.
  rm -rf "$RUN_DIR" 2>/dev/null
  local left=""
  for p in MicroXRCEAgent "gz sim" "$PX4_PATTERN" parameter_bridge scan_self_filter; do
    pgrep -f "$p" >/dev/null && left="$left $p"
  done
  if [ -n "$left" ]; then
    log "WARNING: still running:$left"
    pgrep -af "MicroXRCEAgent|gz sim|${PX4_PATTERN}|parameter_bridge|scan_self_filter"
    return 1
  fi
  log "clean: no PX4, Gazebo, DDS agent, bridge or filter process remains"
  log "scratch run directory removed ($RUN_DIR) - the next start is virgin"
}

case "${1:-}" in
  start)         cmd_start ;;
  status)        cmd_status ;;
  params)        cmd_params ;;
  prearm)        cmd_prearm ;;
  rtf)           cmd_rtf ;;
  stop)          cmd_stop ;;
  *) sed -n '2,42p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
