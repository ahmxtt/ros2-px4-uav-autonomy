#!/usr/bin/env python3
import math
import sys
import time
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


@dataclass
class SectorDeg:
    start: float
    end: float


def normalize_deg(value: float) -> float:
    while value <= -180.0:
        value += 360.0
    while value > 180.0:
        value -= 360.0
    return value


def parse_sectors(raw: str) -> List[SectorDeg]:
    out: List[SectorDeg] = []
    text = raw.strip()
    if not text:
        return out

    for part in text.split(","):
        token = part.strip()
        if not token:
            continue
        if ":" not in token:
            raise ValueError(f"invalid sector '{token}', expected start:end in degrees")
        a_raw, b_raw = token.split(":", 1)
        out.append(SectorDeg(normalize_deg(float(a_raw)), normalize_deg(float(b_raw))))
    return out


def in_sector(deg: float, sector: SectorDeg) -> bool:
    d = normalize_deg(deg)
    a = normalize_deg(sector.start)
    b = normalize_deg(sector.end)
    if a <= b:
        return a <= d <= b
    return d >= a or d <= b


class ScanSelfFilter(Node):
    def __init__(self) -> None:
        super().__init__("scan_self_filter")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_filtered")
        self.declare_parameter("self_radius", 0.35)
        self.declare_parameter("mask_sectors_deg", "-35:-8,8:35")
        self.declare_parameter("mask_max_range", 0.70)
        self.declare_parameter("replace_with_inf", True)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.self_radius = max(0.0, float(self.get_parameter("self_radius").value))
        self.mask_text = str(self.get_parameter("mask_sectors_deg").value)
        self.mask_max_range = max(0.0, float(self.get_parameter("mask_max_range").value))
        self.replace_with_inf = bool(self.get_parameter("replace_with_inf").value)

        try:
            self.sectors = parse_sectors(self.mask_text)
        except ValueError as exc:
            self.get_logger().error(f"mask_sectors_deg parse error: {exc}")
            raise

        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos)
        self.create_subscription(LaserScan, self.input_topic, self._callback, qos)

        self.msg_count = 0
        self.last_log = 0.0

        self.get_logger().info(
            f"Filtering {self.input_topic} -> {self.output_topic}; "
            f"self_radius={self.self_radius:.3f}m mask_sectors_deg='{self.mask_text}' "
            f"mask_max_range={self.mask_max_range:.3f}m replace_with_inf={self.replace_with_inf}"
        )

    def _callback(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        ranges = list(msg.ranges)
        removed = 0

        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue

            angle_rad = msg.angle_min + (i * msg.angle_increment)
            angle_deg = math.degrees(angle_rad)

            near_hit = r < self.self_radius
            masked_hit = (
                r <= self.mask_max_range
                and any(in_sector(angle_deg, sector) for sector in self.sectors)
            )

            if near_hit or masked_hit:
                ranges[i] = math.inf if self.replace_with_inf else float("nan")
                removed += 1

        out.ranges = ranges
        out.intensities = list(msg.intensities)
        self.pub.publish(out)

        self.msg_count += 1
        now = time.monotonic()
        if now - self.last_log >= 2.0:
            self.get_logger().info(
                f"filtered_msg={self.msg_count} removed_points={removed}/{len(ranges)}"
            )
            self.last_log = now


def main() -> int:
    rclpy.init(args=sys.argv)
    node = ScanSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
