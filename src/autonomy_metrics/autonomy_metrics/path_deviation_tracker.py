#!/usr/bin/env python3
"""
PathDeviationTracker
Author: Ibrahim Hroob - JABASAI

Subscribes (via the main node) to a global-path topic (nav_msgs/msg/Path) and
computes the cross-track error from the robot's current odometry position.

Published topics
----------------
- mdbi_logger/path_deviation_error  (Float32, metres)
"""

import math

from std_msgs.msg import Float32


class PathDeviationTracker:
    """Computes the minimum distance from the robot to the global path."""

    def __init__(self, node):
        """
        Parameters
        ----------
        node : rclpy.node.Node
            Parent ROS 2 node (used for publishers, clock, logging).
        """
        self._node = node
        self._global_path: list[tuple[float, float]] = []
        self.deviation: float = 0.0

        self.deviation_pub = node.create_publisher(
            Float32, "mdbi_logger/path_deviation_error", 10
        )

        node.get_logger().info("[PathDevTracker] Initialised")

    # ------------------------------------------------------------------
    # Public update methods
    # ------------------------------------------------------------------

    def update_path(self, path_msg) -> None:
        """
        Called when a new nav_msgs/msg/Path arrives.

        Extracts the (x, y) waypoints and stores them for deviation
        computation.
        """
        self._global_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in path_msg.poses
        ]
        self._node.get_logger().info(
            f"[PathDevTracker] Received global path with "
            f"{len(self._global_path)} poses"
        )

    def update_position(self, x: float, y: float) -> None:
        """
        Called from the odometry callback with the robot's current position.

        Computes the minimum distance to any segment of the stored global path
        and publishes the result.
        """
        if not self._global_path or len(self._global_path) < 2:
            return

        min_dist = float("inf")
        for i in range(len(self._global_path) - 1):
            d = self._point_to_segment_dist(
                x, y, self._global_path[i], self._global_path[i + 1]
            )
            if d < min_dist:
                min_dist = d

        self.deviation = min_dist

        msg = Float32()
        msg.data = float(self.deviation)
        self.deviation_pub.publish(msg)

    # ------------------------------------------------------------------
    # Geometry helper
    # ------------------------------------------------------------------

    @staticmethod
    def _point_to_segment_dist(
        px: float,
        py: float,
        seg_a: tuple[float, float],
        seg_b: tuple[float, float],
    ) -> float:
        """Minimum distance from point (px, py) to line segment (seg_a → seg_b)."""
        ax, ay = seg_a
        bx, by = seg_b
        dx, dy = bx - ax, by - ay
        len_sq = dx * dx + dy * dy
        if len_sq == 0.0:
            return math.sqrt((px - ax) ** 2 + (py - ay) ** 2)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
        proj_x = ax + t * dx
        proj_y = ay + t * dy
        return math.sqrt((px - proj_x) ** 2 + (py - proj_y) ** 2)

    # ------------------------------------------------------------------
    # Snapshot (for system_snapshot / MongoDB)
    # ------------------------------------------------------------------

    def get_snapshot(self) -> dict:
        """Return the tracker state as a plain dict."""
        return {
            "path_deviation_error": self.deviation,
            "global_path_length": len(self._global_path),
        }
