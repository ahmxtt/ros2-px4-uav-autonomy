#!/usr/bin/env python3
"""A minimal MAVLink ground station: send heartbeats, print nothing else.

Why this exists
---------------
PX4 expects a ground control station to be connected. When none is, two things
happen that break an unattended SITL run:

* the pre-flight checks report "No connection to the ground control station";
* ``vehicle_status`` is republished irregularly. Measured on an idle PX4 SITL
  with no GCS: median gap 0.74 s but p95 2.8 s and worst case 3.9 s, against a
  0.001 s median on ``vehicle_local_position``. The mission executor's status
  watchdog fires at 3.0 s, so a run aborts with ``status_stale`` while sitting
  on the ground waiting for the EKF.

Running QGroundControl fixes both, and that is what the original validation
stack did. But a GUI costs real-time factor, which an armed run cannot spare.
This is the 60 lines of QGroundControl that actually matter here: a GCS
heartbeat at 1 Hz.

What it does NOT do
-------------------
It sends **only** ``HEARTBEAT``, with component type ``MAV_TYPE_GCS``. It never
sends a command, never arms, never changes a mode, never uploads a mission. It
cannot make the vehicle do anything. Point it at PX4 SITL and leave it running
for the duration of a session.

Usage::

    python3 tools/gcs_heartbeat.py &            # listens on udp:14550
    python3 tools/gcs_heartbeat.py --port 14550 --rate-hz 1.0 --verbose
"""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=14550,
                    help="UDP port PX4 broadcasts its MAVLink stream to.")
    ap.add_argument("--rate-hz", type=float, default=1.0,
                    help="Heartbeat rate (default: %(default)s).")
    ap.add_argument("--duration-s", type=float, default=0.0,
                    help="Stop after this long. 0 means run until killed.")
    ap.add_argument("--verbose", action="store_true",
                    help="Report once when the link comes up, then every 30 s.")
    args = ap.parse_args()

    try:
        from pymavlink import mavutil
    except ImportError:
        print("FAIL: pymavlink is not installed.\n"
              "  pip install pymavlink", file=sys.stderr)
        return 2

    conn = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{args.port}", source_system=255, source_component=190)

    period = 1.0 / max(args.rate_hz, 0.1)
    deadline = time.monotonic() + args.duration_s if args.duration_s > 0 else None
    next_beat = 0.0
    next_report = time.monotonic() + 30.0
    seen = False
    received = 0

    print(f"[gcs] heartbeating on udp:{args.port} at {args.rate_hz} Hz "
          f"(HEARTBEAT only; sends no commands)", flush=True)
    try:
        while True:
            now = time.monotonic()
            if deadline is not None and now > deadline:
                break
            # Draining the inbound stream is what teaches pymavlink where to
            # send: it replies to whoever last spoke to it.
            msg = conn.recv_match(blocking=False)
            if msg is not None:
                received += 1
                if not seen:
                    seen = True
                    print(f"[gcs] link up: PX4 system {msg.get_srcSystem()}",
                          flush=True)
            if now >= next_beat:
                next_beat = now + period
                try:
                    conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                except Exception:
                    # No peer yet: PX4 has not broadcast, so there is nobody to
                    # answer. Keep listening.
                    pass
            if args.verbose and now >= next_report:
                next_report = now + 30.0
                print(f"[gcs] alive, {received} messages received", flush=True)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    print(f"[gcs] stopped after receiving {received} messages", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
