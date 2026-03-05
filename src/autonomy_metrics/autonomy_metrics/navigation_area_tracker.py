#!/usr/bin/env python3
"""
NavigationAreaTracker
Author: Ibrahim Hroob - JABASAI

Tracks the robot's navigation area (INSIDE_POLYTUNNEL / OUTSIDE_POLYTUNNEL /
TRANSITION_INTO_POLYTUNNEL), derives per-area average speeds, time-in-current-mode,
and a high-level robot status string.

Published topics
----------------
- mdbi_logger/avg_speed_inside_polytunnel  (Float32)
- mdbi_logger/avg_speed_outside_polytunnel (Float32)
- mdbi_logger/time_in_current_mode         (Float32, seconds)
- mdbi_logger/robot_status                 (String)
- mdbi_logger/navigation_area              (String)
"""

from std_msgs.msg import Float32, String


class NavigationAreaTracker:
    """Manages navigation-area state, per-area speed averages, and status derivation."""

    # Area constants (match the enum on /robot_navigation_area)
    INSIDE = "INSIDE_POLYTUNNEL"
    OUTSIDE = "OUTSIDE_POLYTUNNEL"
    TRANSITION = "TRANSITION_INTO_POLYTUNNEL"

    # Derived status labels
    STATUS_IN_ROW = "In-row"
    STATUS_HEADLAND = "Headland"
    STATUS_ROW_ENTRY = "Row-entry"
    STATUS_PAUSED_OBSTACLE = "Paused due to obstacle"
    STATUS_HELD_BY_PICKER = "Held by picker"
    STATUS_UNKNOWN = "Unknown"

    def __init__(self, node, *, paused_speed_threshold: float = 0.01):
        """
        Parameters
        ----------
        node : rclpy.node.Node
            Parent ROS 2 node (used for publishers, clock, logging).
        paused_speed_threshold : float
            Speed (m/s) below which the robot is considered stopped.
        """
        self._node = node
        self._paused_speed_thr = paused_speed_threshold

        # Current state
        self.current_area: str | None = None
        self.execution_ui_state: str = "idle"  # "idle" or "paused"
        self.current_status: str = self.STATUS_UNKNOWN

        # Running speed averages per area
        self._speed_sum = {self.INSIDE: 0.0, self.OUTSIDE: 0.0}
        self._speed_count = {self.INSIDE: 0, self.OUTSIDE: 0}

        # Time-in-current-mode (resets on every area transition)
        self._mode_start_time = node.get_clock().now()

        # ---- Publishers ----
        self.avg_speed_inside_pub = node.create_publisher(
            Float32, "mdbi_logger/avg_speed_inside_polytunnel", 10
        )
        self.avg_speed_outside_pub = node.create_publisher(
            Float32, "mdbi_logger/avg_speed_outside_polytunnel", 10
        )
        self.time_in_mode_pub = node.create_publisher(
            Float32, "mdbi_logger/time_in_current_mode", 10
        )
        self.status_pub = node.create_publisher(
            String, "mdbi_logger/robot_status", 10
        )
        self.nav_area_pub = node.create_publisher(
            String, "mdbi_logger/navigation_area", 10
        )

        node.get_logger().info("[NavAreaTracker] Initialised")

    # ------------------------------------------------------------------
    # Public update methods (called from the main node's callbacks)
    # ------------------------------------------------------------------

    def update_area(self, area_str: str) -> None:
        """Handle a new /robot_navigation_area message."""
        area = area_str.strip()
        if area != self.current_area:
            self._node.get_logger().info(
                f"[NavAreaTracker] Area changed: {self.current_area} -> {area}"
            )
            self.current_area = area
            # Reset time-in-current-mode on every area transition
            self._mode_start_time = self._node.get_clock().now()

        # Republish on a logger-namespaced topic
        msg = String()
        msg.data = area
        self.nav_area_pub.publish(msg)

    def update_execution_ui(self, state_str: str) -> None:
        """Handle a new /roboflow/execution_ui message."""
        state = state_str.strip().lower()
        if state != self.execution_ui_state:
            self._node.get_logger().info(
                f"[NavAreaTracker] Execution UI: {self.execution_ui_state} -> {state}"
            )
        self.execution_ui_state = state

    def update_speed(self, speed: float, is_autonomous: bool) -> None:
        """
        Feed the current speed (called from odom / timer).

        * Accumulates speed samples for per-area averages.
        * Derives the high-level robot status.
        * Publishes all metrics.
        """
        # Accumulate speed for the current area
        if self.current_area in (self.INSIDE, self.OUTSIDE):
            self._speed_sum[self.current_area] += speed
            self._speed_count[self.current_area] += 1

        self._derive_status(speed, is_autonomous)
        self._publish_all()

    # ------------------------------------------------------------------
    # Status derivation
    # ------------------------------------------------------------------

    def _derive_status(self, speed: float, is_autonomous: bool) -> None:
        """
        Priority order:
        1. Held by picker  (execution_ui == "paused")
        2. Paused due to obstacle  (autonomous, speed ≈ 0, not held by picker)
        3. Row-entry  (TRANSITION_INTO_POLYTUNNEL)
        4. In-row      (INSIDE_POLYTUNNEL)
        5. Headland    (OUTSIDE_POLYTUNNEL)
        6. Unknown
        """
        if self.execution_ui_state == "paused":
            self.current_status = self.STATUS_HELD_BY_PICKER
        elif is_autonomous and speed < self._paused_speed_thr and self.current_area is not None:
            self.current_status = self.STATUS_PAUSED_OBSTACLE
        elif self.current_area == self.TRANSITION:
            self.current_status = self.STATUS_ROW_ENTRY
        elif self.current_area == self.INSIDE:
            self.current_status = self.STATUS_IN_ROW
        elif self.current_area == self.OUTSIDE:
            self.current_status = self.STATUS_HEADLAND
        else:
            self.current_status = self.STATUS_UNKNOWN

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _publish_all(self) -> None:
        # Average speed INSIDE
        msg = Float32()
        msg.data = (
            float(self._speed_sum[self.INSIDE] / self._speed_count[self.INSIDE])
            if self._speed_count[self.INSIDE] > 0
            else 0.0
        )
        self.avg_speed_inside_pub.publish(msg)

        # Average speed OUTSIDE
        msg = Float32()
        msg.data = (
            float(self._speed_sum[self.OUTSIDE] / self._speed_count[self.OUTSIDE])
            if self._speed_count[self.OUTSIDE] > 0
            else 0.0
        )
        self.avg_speed_outside_pub.publish(msg)

        # Time in current mode
        now = self._node.get_clock().now()
        elapsed_sec = (now - self._mode_start_time).nanoseconds * 1e-9
        msg = Float32()
        msg.data = float(elapsed_sec)
        self.time_in_mode_pub.publish(msg)

        # Status string
        msg = String()
        msg.data = self.current_status
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Snapshot (for system_snapshot / MongoDB)
    # ------------------------------------------------------------------

    def get_snapshot(self) -> dict:
        """Return the tracker state as a plain dict."""
        avg_in = (
            self._speed_sum[self.INSIDE] / self._speed_count[self.INSIDE]
            if self._speed_count[self.INSIDE] > 0
            else 0.0
        )
        avg_out = (
            self._speed_sum[self.OUTSIDE] / self._speed_count[self.OUTSIDE]
            if self._speed_count[self.OUTSIDE] > 0
            else 0.0
        )
        now = self._node.get_clock().now()
        elapsed_sec = (now - self._mode_start_time).nanoseconds * 1e-9

        return {
            "navigation_area": self.current_area,
            "execution_ui_state": self.execution_ui_state,
            "robot_status": self.current_status,
            "avg_speed_inside_polytunnel": avg_in,
            "avg_speed_outside_polytunnel": avg_out,
            "time_in_current_mode_sec": elapsed_sec,
            "speed_samples_inside": self._speed_count[self.INSIDE],
            "speed_samples_outside": self._speed_count[self.OUTSIDE],
        }
