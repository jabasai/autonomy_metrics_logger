#!/usr/bin/env python3
"""Config-driven ROS 2 autonomy metrics logger."""

from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path

import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import Bool, Float32, Int32, String

from autonomy_metrics.db_mgr import DatabaseMgr
from autonomy_metrics.git_utils import collect_git_repos_info
from autonomy_metrics.topic_utils import import_msg_type, ros_msg_to_dict


AUTONOMOUS_LABEL = "Autonomous"
MANUAL_LABEL = "Manual"
ROBOT_STATE_DISABLED = "Disabled"
ROBOT_STATE_ACTIVE = "Active"
KNOWN_AREAS = (
    "INSIDE_POLYTUNNEL",
    "OUTSIDE_POLYTUNNEL",
    "TRANSITION_INTO_POLYTUNNEL",
)


class AutonomyMetricsLogger(Node):
    """Main ROS 2 node for autonomy/session metrics and MongoDB logging."""

    def __init__(self):
        super().__init__("autonomy_metrics_logger")

        default_config_path = os.path.join(
            get_package_share_directory("autonomy_metrics"),
            "config",
            "metrics_full.yaml",
        )

        self.declare_parameter("config_yaml", default_config_path)
        self.declare_parameter("mongodb_host", "localhost")
        self.declare_parameter("mongodb_port", 27018)
        self.declare_parameter("remote_mongodb_host", "")
        self.declare_parameter("remote_mongodb_port", 27017)
        self.declare_parameter("enable_remote_logging", False)
        self.declare_parameter("database_name", "robot_incidents")

        self.config_path = self.get_parameter("config_yaml").value
        self.database_name = self.get_parameter("database_name").value
        self.mongo_host = self.get_parameter("mongodb_host").value
        self.mongo_port = self.get_parameter("mongodb_port").value
        self.remote_mongo_host = self.get_parameter("remote_mongodb_host").value
        self.remote_mongo_port = self.get_parameter("remote_mongodb_port").value
        self.enable_remote_logging = self.get_parameter("enable_remote_logging").value

        self.config = self._load_config(self.config_path)
        self.runtime_cfg = self.config.get("runtime", {})
        self.required_roles = defaultdict(list)
        self.topic_cfg_map = {}

        self.heartbeat_period_sec = float(
            self.runtime_cfg.get("heartbeat_period_sec", 1.0)
        )
        self.snapshot_interval_sec = float(
            self.runtime_cfg.get("snapshot_interval_sec", 10.0)
        )
        self.distance_epsilon_m = float(self.runtime_cfg.get("distance_epsilon_m", 0.01))
        self.stale_speed_timeout_sec = float(
            self.runtime_cfg.get("stale_speed_timeout_sec", 2.0)
        )
        self.session_summary_topic = str(
            self.runtime_cfg.get(
                "session_summary_topic", "/autonomy_metrics/session_summary_json"
            )
        )
        self.heartbeat_topic = str(
            self.runtime_cfg.get("heartbeat_topic", "/autonomy_metrics/heartbeat")
        )

        self.autonomous_mode = False
        self.mode_entered_at = self._now_sec()
        self.mode_totals_sec = {
            AUTONOMOUS_LABEL: 0.0,
            MANUAL_LABEL: 0.0,
        }
        self.intervention_count = 0
        self.intervention_heartbeat_states = {}
        self.last_intervention_details = None

        self.robot_state = ROBOT_STATE_DISABLED
        self.active_to_disabled_in_autonomous_count = 0

        self.total_distance_m = 0.0
        self.distance_by_mode_m = {
            AUTONOMOUS_LABEL: 0.0,
            MANUAL_LABEL: 0.0,
        }
        self.last_motion_position = None
        self.first_motion_time_sec = None
        self.last_motion_time_sec = None
        self.last_speed_update_sec = None
        self.current_speed_mps = 0.0

        self.current_area = None
        self.area_entered_at = None
        self.area_totals_sec = defaultdict(float)
        self.area_visit_count = defaultdict(int)
        self.area_distance_m = defaultdict(float)

        self.current_path = []
        self.current_path_deviation_m = None
        self.max_path_deviation_m = 0.0
        self.path_deviation_sum_m = 0.0
        self.path_deviation_samples = 0

        self.latest_topic_values = {}
        self.snapshot_topics = set()
        self.topic_subscriptions = []

        self._setup_publishers()
        self._setup_databases()
        self._setup_subscriptions()
        self._init_session()

        self.heartbeat_timer = self.create_timer(
            self.heartbeat_period_sec, self._heartbeat_timer_callback
        )
        self.snapshot_timer = self.create_timer(
            self.snapshot_interval_sec, self._snapshot_timer_callback
        )

        self.get_logger().info(
            f"Autonomy metrics logger ready using config {self.config_path}"
        )

    def _load_config(self, config_path: str) -> dict:
        config_file = Path(config_path)
        if not config_file.is_file():
            raise FileNotFoundError(f"Config file not found: {config_path}")
        with config_file.open("r", encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}

    def _setup_publishers(self):
        self.heartbeat_publisher = self.create_publisher(Bool, self.heartbeat_topic, 10)
        self.mode_publisher = self.create_publisher(
            String, "/autonomy_metrics/current_mode", 10
        )
        self.interventions_publisher = self.create_publisher(
            Int32, "/autonomy_metrics/interventions", 10
        )
        self.mdbi_publisher = self.create_publisher(Float32, "/autonomy_metrics/mdbi", 10)
        self.robot_state_transition_publisher = self.create_publisher(
            Int32, "/autonomy_metrics/active_to_disabled_in_autonomous", 10
        )
        self.total_distance_publisher = self.create_publisher(
            Float32, "/autonomy_metrics/total_distance_m", 10
        )
        self.autonomous_distance_publisher = self.create_publisher(
            Float32, "/autonomy_metrics/autonomous_distance_m", 10
        )
        self.manual_distance_publisher = self.create_publisher(
            Float32, "/autonomy_metrics/manual_distance_m", 10
        )
        self.current_speed_publisher = self.create_publisher(
            Float32, "/autonomy_metrics/current_speed_mps", 10
        )
        self.average_speed_publisher = self.create_publisher(
            Float32, "/autonomy_metrics/average_speed_mps", 10
        )
        self.autonomous_time_publisher = self.create_publisher(
            Float32, "/autonomy_metrics/autonomous_time_sec", 10
        )
        self.manual_time_publisher = self.create_publisher(
            Float32, "/autonomy_metrics/manual_time_sec", 10
        )
        self.navigation_area_publisher = self.create_publisher(
            String, "/autonomy_metrics/current_navigation_area", 10
        )
        self.path_deviation_current_publisher = self.create_publisher(
            Float32, "/autonomy_metrics/path_deviation/current_m", 10
        )
        self.path_deviation_average_publisher = self.create_publisher(
            Float32, "/autonomy_metrics/path_deviation/average_m", 10
        )
        self.path_deviation_max_publisher = self.create_publisher(
            Float32, "/autonomy_metrics/path_deviation/max_m", 10
        )
        self.summary_publisher = self.create_publisher(
            String, self.session_summary_topic, 10
        )

    def _setup_databases(self):
        self.db_managers = []

        local_db = DatabaseMgr(
            database_name=self.database_name,
            host=self.mongo_host,
            port=self.mongo_port,
        )
        self.db_managers.append(local_db)

        if self.enable_remote_logging and self.remote_mongo_host:
            remote_db = DatabaseMgr(
                database_name=self.database_name,
                host=self.remote_mongo_host,
                port=self.remote_mongo_port,
            )
            self.db_managers.append(remote_db)

    def _setup_subscriptions(self):
        topics = self.config.get("topics", [])
        if not topics:
            raise RuntimeError("No topics configured in metrics YAML")

        for topic_cfg in topics:
            topic_name = topic_cfg["name"]
            topic_type = topic_cfg["type"]
            role = topic_cfg.get("role", "").strip()
            self.topic_cfg_map[topic_name] = topic_cfg

            if topic_cfg.get("snapshot", False):
                self.snapshot_topics.add(topic_name)

            msg_type = import_msg_type(topic_type)
            callback = self._build_topic_callback(topic_cfg)
            subscription = self.create_subscription(
                msg_type,
                topic_name,
                callback,
                qos_profile_sensor_data,
            )
            self.topic_subscriptions.append(subscription)
            if role:
                self.required_roles[role].append(topic_name)
            self.get_logger().info(f"Subscribed to {topic_name} with role '{role}'")

        self._validate_required_roles()

    def _validate_required_roles(self):
        required = [
            "autonomous_mode",
            "intervention_heartbeat",
            "robot_state",
            "distance_odometry",
            "navigation_area",
            "global_path",
            "global_pose_odometry",
        ]
        missing = [role for role in required if not self.required_roles.get(role)]
        if missing:
            raise RuntimeError(
                "Config is missing required topic roles: " + ", ".join(sorted(missing))
            )

        if len(self.required_roles.get("autonomous_mode", [])) != 1:
            raise RuntimeError("Exactly one topic must use role 'autonomous_mode'")
        if len(self.required_roles.get("robot_state", [])) != 1:
            raise RuntimeError("Exactly one topic must use role 'robot_state'")
        if len(self.required_roles.get("distance_odometry", [])) != 1:
            raise RuntimeError("Exactly one topic must use role 'distance_odometry'")
        if len(self.required_roles.get("global_path", [])) != 1:
            raise RuntimeError("Exactly one topic must use role 'global_path'")
        if len(self.required_roles.get("global_pose_odometry", [])) != 1:
            raise RuntimeError("Exactly one topic must use role 'global_pose_odometry'")

    def _init_session(self):
        env_variables = {
            "robot_name": os.getenv("ROBOT_NAME", "UNDEFINED"),
            "farm_name": os.getenv("FARM_NAME", "UNDEFINED"),
            "field_name": os.getenv("FIELD_NAME", "UNDEFINED"),
            "application": os.getenv("APPLICATION", "UNDEFINED"),
            "scenario_name": os.getenv("SCENARIO_NAME", "UNDEFINED"),
        }
        git_repos = collect_git_repos_info(self.config)
        initial_summary = self._build_summary()
        active_db_managers = []
        for db_manager in self.db_managers:
            try:
                db_manager.init_session(
                    env_variables=env_variables,
                    git_repos_info=git_repos,
                    metadata=initial_summary,
                )
                active_db_managers.append(db_manager)
            except Exception as exc:
                self.get_logger().warn(f"Failed to initialize MongoDB session: {exc}")
        self.db_managers = active_db_managers
        if not self.db_managers:
            self.get_logger().warn("No MongoDB targets are currently writable.")

    def _build_topic_callback(self, topic_cfg: dict):
        topic_name = topic_cfg["name"]
        role = topic_cfg.get("role", "").strip()

        def callback(msg):
            self._store_latest_topic_value(topic_name, role, msg)
            if role == "autonomous_mode":
                self._handle_autonomous_mode(msg)
            elif role == "intervention_heartbeat":
                self._handle_intervention_heartbeat(topic_cfg, msg)
            elif role == "robot_state":
                self._handle_robot_state(msg)
            elif role == "distance_odometry":
                self._handle_distance_odometry(msg)
            elif role == "navigation_area":
                self._handle_navigation_area(msg)
            elif role == "global_path":
                self._handle_global_path(msg)
            elif role == "global_pose_odometry":
                self._handle_global_pose(msg)

        return callback

    def _store_latest_topic_value(self, topic_name: str, role: str, msg):
        topic_cfg = self.topic_cfg_map[topic_name]
        data = self._serialize_topic_value(topic_cfg, msg)
        self.latest_topic_values[topic_name] = {
            "role": role,
            "received_at": self._now_datetime().isoformat(),
            "data": data,
        }

    def _serialize_topic_value(self, topic_cfg: dict, msg):
        role = topic_cfg.get("role", "")
        if role in {"autonomous_mode", "intervention_heartbeat"}:
            return {"value": bool(msg.data)}
        if role == "robot_state":
            return {"value": str(msg.data)}
        if role in {"distance_odometry", "global_pose_odometry"}:
            return {
                "frame_id": msg.header.frame_id,
                "position": {
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "z": float(msg.pose.pose.position.z),
                },
                "linear_speed_mps": float(msg.twist.twist.linear.x),
            }
        if role == "global_path":
            pose_count = len(msg.poses)
            data = {"pose_count": pose_count}
            if pose_count > 0:
                start = msg.poses[0].pose.position
                end = msg.poses[-1].pose.position
                data["start"] = {"x": float(start.x), "y": float(start.y)}
                data["end"] = {"x": float(end.x), "y": float(end.y)}
            return data
        return ros_msg_to_dict(msg)

    def _handle_autonomous_mode(self, msg: Bool):
        new_mode = bool(msg.data)
        if new_mode == self.autonomous_mode:
            return

        previous_mode = self.autonomous_mode
        failing_heartbeats = self._get_failing_intervention_heartbeats()
        self._advance_mode_clock(new_mode)

        transition_details = {
            "previous_mode": AUTONOMOUS_LABEL if previous_mode else MANUAL_LABEL,
            "new_mode": AUTONOMOUS_LABEL if new_mode else MANUAL_LABEL,
            "failing_heartbeats": failing_heartbeats,
        }
        self._log_event("mode_transition", transition_details)

        if previous_mode and not new_mode and failing_heartbeats:
            self.intervention_count += 1
            self.last_intervention_details = {
                "time": self._now_datetime().isoformat(),
                "failing_heartbeats": failing_heartbeats,
            }
            self._log_event(
                "intervention",
                {
                    "intervention_index": self.intervention_count,
                    "failing_heartbeats": failing_heartbeats,
                },
            )

    def _handle_intervention_heartbeat(self, topic_cfg: dict, msg: Bool):
        heartbeat_key = topic_cfg.get("metric_key") or topic_cfg["name"]
        self.intervention_heartbeat_states[heartbeat_key] = bool(msg.data)

    def _handle_robot_state(self, msg: String):
        new_state = self._normalize_robot_state(str(msg.data))
        previous_state = self.robot_state
        self.robot_state = new_state

        if (
            previous_state == ROBOT_STATE_ACTIVE
            and new_state == ROBOT_STATE_DISABLED
            and self.autonomous_mode
        ):
            self.active_to_disabled_in_autonomous_count += 1
            self._log_event(
                "robot_state_active_to_disabled_in_autonomous",
                {
                    "count": self.active_to_disabled_in_autonomous_count,
                    "previous_state": previous_state,
                    "new_state": new_state,
                },
            )

    def _handle_distance_odometry(self, msg: Odometry):
        now_sec = self._now_sec()
        self.last_speed_update_sec = now_sec
        self.current_speed_mps = abs(float(msg.twist.twist.linear.x))

        position = msg.pose.pose.position
        current_position = (float(position.x), float(position.y))
        if self.last_motion_position is None:
            self.last_motion_position = current_position
            self.first_motion_time_sec = now_sec
            self.last_motion_time_sec = now_sec
            return

        dx = current_position[0] - self.last_motion_position[0]
        dy = current_position[1] - self.last_motion_position[1]
        step_distance_m = math.hypot(dx, dy)

        if step_distance_m >= self.distance_epsilon_m:
            self.total_distance_m += step_distance_m
            self.distance_by_mode_m[self._mode_label()] += step_distance_m
            if self.current_area:
                self.area_distance_m[self.current_area] += step_distance_m

        self.last_motion_position = current_position
        self.last_motion_time_sec = now_sec

    def _handle_navigation_area(self, msg: String):
        new_area = str(msg.data).strip()
        now_sec = self._now_sec()

        if new_area == self.current_area:
            return

        if self.current_area is not None and self.area_entered_at is not None:
            self.area_totals_sec[self.current_area] += now_sec - self.area_entered_at

        self.current_area = new_area
        self.area_entered_at = now_sec
        self.area_visit_count[new_area] += 1
        self._log_event("navigation_area_transition", {"new_area": new_area})

    def _handle_global_path(self, msg: PathMsg):
        self.current_path = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
            )
            for pose in msg.poses
        ]

    def _handle_global_pose(self, msg: Odometry):
        if len(self.current_path) < 2:
            return

        position = msg.pose.pose.position
        x = float(position.x)
        y = float(position.y)
        current_deviation = min(
            self._point_to_segment_distance(
                x,
                y,
                self.current_path[index],
                self.current_path[index + 1],
            )
            for index in range(len(self.current_path) - 1)
        )
        self.current_path_deviation_m = current_deviation
        self.path_deviation_sum_m += current_deviation
        self.path_deviation_samples += 1
        self.max_path_deviation_m = max(self.max_path_deviation_m, current_deviation)

    def _advance_mode_clock(self, new_mode: bool):
        now_sec = self._now_sec()
        old_label = self._mode_label()
        self.mode_totals_sec[old_label] += now_sec - self.mode_entered_at
        self.autonomous_mode = new_mode
        self.mode_entered_at = now_sec

    def _mode_label(self) -> str:
        return AUTONOMOUS_LABEL if self.autonomous_mode else MANUAL_LABEL

    def _get_mode_totals_sec(self) -> dict:
        totals = dict(self.mode_totals_sec)
        totals[self._mode_label()] += self._now_sec() - self.mode_entered_at
        return totals

    def _get_area_totals_sec(self) -> dict:
        totals = dict(self.area_totals_sec)
        if self.current_area is not None and self.area_entered_at is not None:
            totals[self.current_area] = totals.get(self.current_area, 0.0) + (
                self._now_sec() - self.area_entered_at
            )
        return totals

    def _get_failing_intervention_heartbeats(self) -> list[str]:
        return sorted(
            [
                name
                for name, is_healthy in self.intervention_heartbeat_states.items()
                if is_healthy is False
            ]
        )

    def _build_summary(self) -> dict:
        now_sec = self._now_sec()
        mode_times_sec = self._get_mode_totals_sec()
        area_times_sec = self._get_area_totals_sec()

        if (
            self.last_speed_update_sec is not None
            and now_sec - self.last_speed_update_sec > self.stale_speed_timeout_sec
        ):
            live_speed_mps = 0.0
        else:
            live_speed_mps = self.current_speed_mps

        elapsed_motion_time = 0.0
        if self.first_motion_time_sec is not None and self.last_motion_time_sec is not None:
            elapsed_motion_time = max(0.0, self.last_motion_time_sec - self.first_motion_time_sec)

        average_speed_mps = (
            self.total_distance_m / elapsed_motion_time if elapsed_motion_time > 0.0 else 0.0
        )

        if self.intervention_count > 0:
            mdbi_m = self.distance_by_mode_m[AUTONOMOUS_LABEL] / self.intervention_count
        else:
            mdbi_m = None

        area_metrics = {}
        for area_name in sorted(set(KNOWN_AREAS).union(area_times_sec).union(self.area_distance_m)):
            total_time = float(area_times_sec.get(area_name, 0.0))
            total_distance = float(self.area_distance_m.get(area_name, 0.0))
            visit_count = int(self.area_visit_count.get(area_name, 0))
            area_metrics[area_name] = {
                "total_time_sec": total_time,
                "average_time_per_visit_sec": total_time / visit_count if visit_count else 0.0,
                "distance_m": total_distance,
                "average_speed_mps": total_distance / total_time if total_time > 0.0 else 0.0,
                "visits": visit_count,
            }

        path_deviation_average_m = (
            self.path_deviation_sum_m / self.path_deviation_samples
            if self.path_deviation_samples
            else 0.0
        )

        return {
            "timestamp": self._now_datetime().isoformat(),
            "mode": {
                "autonomous": self.autonomous_mode,
                "label": self._mode_label(),
                "time_sec": {
                    AUTONOMOUS_LABEL: float(mode_times_sec[AUTONOMOUS_LABEL]),
                    MANUAL_LABEL: float(mode_times_sec[MANUAL_LABEL]),
                },
                "distance_m": {
                    AUTONOMOUS_LABEL: float(self.distance_by_mode_m[AUTONOMOUS_LABEL]),
                    MANUAL_LABEL: float(self.distance_by_mode_m[MANUAL_LABEL]),
                },
            },
            "interventions": {
                "count": int(self.intervention_count),
                "failing_heartbeats": self._get_failing_intervention_heartbeats(),
                "last": self.last_intervention_details,
            },
            "mdbi_m": mdbi_m,
            "mdbi_status": "defined" if mdbi_m is not None else "undefined_until_first_intervention",
            "robot_state": {
                "current": self.robot_state,
                "active_to_disabled_in_autonomous_count": int(
                    self.active_to_disabled_in_autonomous_count
                ),
            },
            "motion": {
                "total_distance_m": float(self.total_distance_m),
                "current_speed_mps": float(live_speed_mps),
                "average_speed_mps": float(average_speed_mps),
            },
            "navigation_area": {
                "current": self.current_area,
                "states": area_metrics,
            },
            "path_deviation": {
                "current_m": float(self.current_path_deviation_m or 0.0),
                "average_m": float(path_deviation_average_m),
                "max_m": float(self.max_path_deviation_m),
                "samples": int(self.path_deviation_samples),
            },
            "heartbeats": {
                key: value for key, value in sorted(self.intervention_heartbeat_states.items())
            },
        }

    def _build_snapshot(self) -> dict:
        topics = {
            topic_name: self.latest_topic_values.get(topic_name)
            for topic_name in sorted(self.snapshot_topics)
            if topic_name in self.latest_topic_values
        }
        return {
            "time": self._now_datetime(),
            "summary": self._build_summary(),
            "topics": topics,
        }

    def _log_event(self, event_type: str, details: dict):
        event = {
            "time": self._now_datetime(),
            "event_type": event_type,
            "details": details,
            "summary": self._build_summary(),
        }

        for db_manager in self.db_managers:
            try:
                db_manager.add_event(event)
                db_manager.update_session_summary(event["summary"])
            except Exception as exc:
                self.get_logger().warn(f"Failed to write event '{event_type}' to MongoDB: {exc}")

    def _snapshot_timer_callback(self):
        snapshot = self._build_snapshot()
        for db_manager in self.db_managers:
            try:
                db_manager.add_snapshot(snapshot)
                db_manager.update_session_summary(snapshot["summary"])
            except Exception as exc:
                self.get_logger().warn(f"Failed to write periodic snapshot to MongoDB: {exc}")

    def _heartbeat_timer_callback(self):
        summary = self._build_summary()

        heartbeat_msg = Bool()
        heartbeat_msg.data = True
        self.heartbeat_publisher.publish(heartbeat_msg)

        mode_msg = String()
        mode_msg.data = summary["mode"]["label"]
        self.mode_publisher.publish(mode_msg)

        interventions_msg = Int32()
        interventions_msg.data = summary["interventions"]["count"]
        self.interventions_publisher.publish(interventions_msg)

        mdbi_msg = Float32()
        mdbi_msg.data = float(summary["mdbi_m"]) if summary["mdbi_m"] is not None else float("nan")
        self.mdbi_publisher.publish(mdbi_msg)

        transition_msg = Int32()
        transition_msg.data = summary["robot_state"][
            "active_to_disabled_in_autonomous_count"
        ]
        self.robot_state_transition_publisher.publish(transition_msg)

        self._publish_float(self.total_distance_publisher, summary["motion"]["total_distance_m"])
        self._publish_float(
            self.autonomous_distance_publisher,
            summary["mode"]["distance_m"][AUTONOMOUS_LABEL],
        )
        self._publish_float(
            self.manual_distance_publisher,
            summary["mode"]["distance_m"][MANUAL_LABEL],
        )
        self._publish_float(
            self.current_speed_publisher, summary["motion"]["current_speed_mps"]
        )
        self._publish_float(
            self.average_speed_publisher, summary["motion"]["average_speed_mps"]
        )
        self._publish_float(
            self.autonomous_time_publisher, summary["mode"]["time_sec"][AUTONOMOUS_LABEL]
        )
        self._publish_float(
            self.manual_time_publisher, summary["mode"]["time_sec"][MANUAL_LABEL]
        )

        area_msg = String()
        area_msg.data = self.current_area or "UNKNOWN"
        self.navigation_area_publisher.publish(area_msg)

        self._publish_float(
            self.path_deviation_current_publisher,
            summary["path_deviation"]["current_m"],
        )
        self._publish_float(
            self.path_deviation_average_publisher,
            summary["path_deviation"]["average_m"],
        )
        self._publish_float(
            self.path_deviation_max_publisher, summary["path_deviation"]["max_m"]
        )

        summary_msg = String()
        summary_msg.data = json.dumps(summary, sort_keys=True)
        self.summary_publisher.publish(summary_msg)

    def _publish_float(self, publisher, value: float):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def _normalize_robot_state(self, raw_state: str) -> str:
        stripped = raw_state.strip()
        lowered = stripped.lower()
        if lowered in {"disable", "disabled"}:
            return ROBOT_STATE_DISABLED
        if lowered == "active":
            return ROBOT_STATE_ACTIVE
        if lowered == "enabled":
            return "Enabled"
        return stripped or "Unknown"

    def _now_datetime(self) -> datetime:
        return datetime.now(tz=timezone.utc)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _point_to_segment_distance(px, py, start, end) -> float:
        start_x, start_y = start
        end_x, end_y = end
        delta_x = end_x - start_x
        delta_y = end_y - start_y
        segment_length_sq = delta_x * delta_x + delta_y * delta_y
        if segment_length_sq == 0.0:
            return math.hypot(px - start_x, py - start_y)
        projection = max(
            0.0,
            min(
                1.0,
                ((px - start_x) * delta_x + (py - start_y) * delta_y) / segment_length_sq,
            ),
        )
        proj_x = start_x + projection * delta_x
        proj_y = start_y + projection * delta_y
        return math.hypot(px - proj_x, py - proj_y)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyMetricsLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        final_summary = node._build_summary()
        for db_manager in node.db_managers:
            try:
                db_manager.mark_session_end(final_summary)
            except Exception as exc:
                node.get_logger().warn(f"Failed to mark session end in MongoDB: {exc}")
        if node.context.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
