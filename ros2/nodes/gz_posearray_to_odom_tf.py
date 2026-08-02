#!/usr/bin/env python3
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GzPoseArrayToOdomTF(Node):
    def __init__(self) -> None:
        super().__init__("gz_posearray_to_odom_tf")
        self.declare_parameter("pose_index", 0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("topic", "/gz_pose_array")
        self.declare_parameter("projection_mode", "full")

        self.odom_pub = self.create_publisher(Odometry, "/ground_truth/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_bad_index_warn = 0.0
        self.last_bad_mode_warn = 0.0

        topic = str(self.get_parameter("topic").value)
        self.create_subscription(PoseArray, topic, self._callback, 10)
        self.get_logger().info(
            f"Listening on {topic}; publishing /ground_truth/odom and TF odom->base_link"
        )

    def _callback(self, msg: PoseArray) -> None:
        pose_index = int(self.get_parameter("pose_index").value)
        odom_frame = str(self.get_parameter("odom_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        projection_mode = str(self.get_parameter("projection_mode").value).strip().lower()

        if pose_index < 0 or pose_index >= len(msg.poses):
            now = time.monotonic()
            if now - self.last_bad_index_warn > 2.0:
                self.get_logger().warn(
                    f"pose_index={pose_index} out of range (size={len(msg.poses)})"
                )
                self.last_bad_index_warn = now
            return

        pose = msg.poses[pose_index]
        pose_out = Pose()
        pose_out.position.x = pose.position.x
        pose_out.position.y = pose.position.y
        pose_out.position.z = pose.position.z

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        if projection_mode == "yaw_only":
            # Keep translation unchanged and keep yaw while forcing roll/pitch to 0.
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            qx = 0.0
            qy = 0.0
            qz = math.sin(yaw * 0.5)
            qw = math.cos(yaw * 0.5)
        elif projection_mode != "full":
            now = time.monotonic()
            if now - self.last_bad_mode_warn > 2.0:
                self.get_logger().warn(
                    f"Unsupported projection_mode='{projection_mode}', using 'full'"
                )
                self.last_bad_mode_warn = now

        pose_out.orientation.x = qx
        pose_out.orientation.y = qy
        pose_out.orientation.z = qz
        pose_out.orientation.w = qw

        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = odom_frame
        tf_msg.child_frame_id = base_frame
        tf_msg.transform.translation.x = pose_out.position.x
        tf_msg.transform.translation.y = pose_out.position.y
        tf_msg.transform.translation.z = pose_out.position.z
        tf_msg.transform.rotation = pose_out.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose = pose_out
        self.odom_pub.publish(odom)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert PoseArray index to /ground_truth/odom and TF."
    )
    _, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = GzPoseArrayToOdomTF()
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
