#!/usr/bin/env python3
"""Deterministic fake ROS topic stream for autonomy_metrics integration testing."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String


class FakeAutonomyMetricsStream(Node):
    def __init__(self):
        super().__init__("fake_autonomy_metrics_stream")

        self.autonomous_mode_pub = self.create_publisher(Bool, "/autonomous_mode", 10)
        self.autonomy_checks_pub = self.create_publisher(
            Bool, "/autonomy_checks/heartbeat", 10
        )
        self.navigation_safe_pub = self.create_publisher(
            Bool, "/navigation_safe/heartbeat", 10
        )
        self.system_check_pub = self.create_publisher(Bool, "/system_check/heartbeat", 10)
        self.robot_state_pub = self.create_publisher(String, "/robot_state", 10)
        self.odom_pub = self.create_publisher(
            Odometry, "/gophar_vehicle_controller/odometry", 10
        )
        self.area_pub = self.create_publisher(String, "/robot_navigation_area", 10)
        self.path_pub = self.create_publisher(Path, "/plan", 10)
        self.global_odom_pub = self.create_publisher(Odometry, "/odometry/global", 10)

        self.start_time = self.get_clock().now()
        self.tick_count = 0
        self.done = False
        self.timer = self.create_timer(0.2, self._tick)

        self._publish_path()

    def _tick(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.tick_count += 1

        autonomous_mode = False
        autonomy_checks = True
        navigation_safe = True
        system_check = True
        robot_state = "Disabled"
        area = "OUTSIDE_POLYTUNNEL"
        odom_x = 0.0
        odom_y = 0.0
        global_y = 0.05
        speed = 0.0

        if elapsed < 1.0:
            pass
        elif elapsed < 4.0:
            autonomous_mode = True
            robot_state = "Active"
            area = "OUTSIDE_POLYTUNNEL"
            odom_x = 0.8 * (elapsed - 1.0)
            speed = 0.8
            global_y = 0.10
        elif elapsed < 6.0:
            autonomous_mode = True
            robot_state = "Active"
            area = "TRANSITION_INTO_POLYTUNNEL"
            odom_x = 2.4 + 0.6 * (elapsed - 4.0)
            speed = 0.6
            global_y = 0.25
        elif elapsed < 7.0:
            autonomous_mode = True
            robot_state = "Disabled"
            area = "INSIDE_POLYTUNNEL"
            odom_x = 3.6 + 0.2 * (elapsed - 6.0)
            speed = 0.2
            global_y = 0.35
        elif elapsed < 8.0:
            autonomous_mode = True
            system_check = False
            robot_state = "Active"
            area = "INSIDE_POLYTUNNEL"
            odom_x = 3.8 + 0.4 * (elapsed - 7.0)
            speed = 0.4
            global_y = 0.45
        elif elapsed < 9.5:
            autonomous_mode = False
            system_check = False
            robot_state = "Enabled"
            area = "INSIDE_POLYTUNNEL"
            odom_x = 4.2 + 0.3 * (elapsed - 8.0)
            speed = 0.3
            global_y = 0.40
        elif elapsed < 12.0:
            autonomous_mode = True
            robot_state = "Active"
            area = "INSIDE_POLYTUNNEL"
            odom_x = 4.65 + 0.9 * (elapsed - 9.5)
            speed = 0.9
            global_y = 0.20
        elif elapsed < 13.5:
            autonomous_mode = True
            autonomy_checks = False
            robot_state = "Active"
            area = "OUTSIDE_POLYTUNNEL"
            odom_x = 6.9 + 0.5 * (elapsed - 12.0)
            speed = 0.5
            global_y = 0.60
        elif elapsed < 15.0:
            autonomous_mode = False
            autonomy_checks = False
            robot_state = "Enabled"
            area = "OUTSIDE_POLYTUNNEL"
            odom_x = 7.65 + 0.2 * (elapsed - 13.5)
            speed = 0.2
            global_y = 0.55
        else:
            self.get_logger().info("Fake data scenario complete.")
            self.timer.cancel()
            self.done = True
            return

        self._publish_bool(self.autonomous_mode_pub, autonomous_mode)
        self._publish_bool(self.autonomy_checks_pub, autonomy_checks)
        self._publish_bool(self.navigation_safe_pub, navigation_safe)
        self._publish_bool(self.system_check_pub, system_check)
        self._publish_string(self.robot_state_pub, robot_state)
        self._publish_string(self.area_pub, area)
        self._publish_odometry(self.odom_pub, odom_x, odom_y, speed)
        self._publish_odometry(self.global_odom_pub, odom_x, global_y, speed)

    def _publish_path(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self._stamp()
        for index in range(0, 21):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self._stamp()
            pose.pose.position.x = float(index) * 0.5
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _publish_odometry(self, publisher, x: float, y: float, speed: float):
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = self._stamp()
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = float(speed)
        publisher.publish(odom)

    def _publish_bool(self, publisher, value: bool):
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    def _publish_string(self, publisher, value: str):
        msg = String()
        msg.data = value
        publisher.publish(msg)

    def _stamp(self) -> Time:
        now = self.get_clock().now().to_msg()
        return Time(sec=now.sec, nanosec=now.nanosec)


def main():
    rclpy.init()
    node = FakeAutonomyMetricsStream()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if node.context.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
