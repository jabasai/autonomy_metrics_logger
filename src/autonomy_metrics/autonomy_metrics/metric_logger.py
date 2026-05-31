#!/usr/bin/env python3
"""
YAML-driven AutonomyMetricsLogger
Author: Ibrahim Hroob - JABASAI
"""

import os
import math
import subprocess
import yaml
import time
from datetime import datetime, timezone
from importlib import import_module

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

# Keep a few explicit imports used by default publishers (std msgs)
from std_msgs.msg import Bool, Float32, Int32

from autonomy_metrics.db_mgr import DatabaseMgr as DBMgr


def ros_msg_to_dict(msg):
    """
    Recursively convert a ROS 2 Python message into a plain dict / list / primitive
    so it can be safely stored in system_snapshot / Mongo.

    Works for nested messages and sequences.
    """
    # Primitive types
    if isinstance(msg, (bool, int, float, str)):
        return msg

    # Sequences (lists, tuples, etc.)
    if isinstance(msg, (list, tuple)):
        return [ros_msg_to_dict(v) for v in msg]

    # ROS 2 messages usually have get_fields_and_field_types()
    if hasattr(msg, "get_fields_and_field_types"):
        data = {}
        for field_name in msg.get_fields_and_field_types().keys():
            try:
                val = getattr(msg, field_name)
                data[field_name] = ros_msg_to_dict(val)
            except Exception:
                # Best-effort; skip problematic fields
                pass
        return data

    # Fallback: try to convert __dict__
    if hasattr(msg, "__dict__"):
        return {
            k: ros_msg_to_dict(v)
            for k, v in msg.__dict__.items()
            if not k.startswith('_')
        }

    # Last resort: return as-is
    return msg


def import_msg_type(type_str: str):
    """Dynamically imports message types based on string."""
    try:
        parts = type_str.split('/')
        if len(parts) == 2:
            pkg, cls_name = parts
            submodule = 'msg'
        elif len(parts) == 3:
            pkg, submodule, cls_name = parts
        else:
            raise ValueError("Message type must be 'pkg/msg/MessageName' or 'pkg/MessageName'")

        module = import_module(f"{pkg}.{submodule}")
        return getattr(module, cls_name)
    except Exception as e:
        raise ImportError(f"Failed to import message type '{type_str}': {e}")


def get_nested_field(obj, path: str):
    try:
        cur = obj
        for part in path.split('.'):
            if '[' in part and part.endswith(']'):
                name, idx = part[:-1].split('[')
                cur = getattr(cur, name)
                cur = cur[int(idx)]
            else:
                cur = getattr(cur, part)
        return cur
    except Exception as e:
        raise AttributeError(f"Failed to get '{path}': {e}")


class AutonomyMetricsLogger(Node):
    def __init__(self):
        super().__init__('mdbi_logger_dynamic')

        self.get_logger().info("Starting YAML-driven AutonomyMetricsLogger")

        # Parameters
        self.declare_parameter('config_yaml', '/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario/jabas/autonomy_metrics_logger/src/autonomy_metrics/config/metrics_full.yaml')
        self.declare_parameter('mongodb_host', 'localhost')
        self.declare_parameter('mongodb_port', 27018)
        self.declare_parameter('remote_mongodb_host', '')
        self.declare_parameter('remote_mongodb_port', 27017)
        self.declare_parameter('enable_remote_logging', False)
        self.declare_parameter('min_distance_threshold', 0.2)
        self.declare_parameter('stop_timeout', 2.0)
        # Collision detection tuning
        self.declare_parameter('collision_nav_threshold', 0.01)      # nav cmd must be > this
        self.declare_parameter('collision_zero_threshold', 0.001)    # collision cmd abs(linear.x) <= this
        self.declare_parameter('collision_time_window', 0.5)         # seconds: nav & collision cmds must be close in time
        self.declare_parameter('collision_log_cooldown', 1.0)        # seconds between logged collisions

        self.collision_nav_threshold = self.get_parameter('collision_nav_threshold').get_parameter_value().double_value
        self.collision_zero_threshold = self.get_parameter('collision_zero_threshold').get_parameter_value().double_value
        self.collision_time_window = self.get_parameter('collision_time_window').get_parameter_value().double_value
        self.collision_log_cooldown = self.get_parameter('collision_log_cooldown').get_parameter_value().double_value
        self.config_path = self.get_parameter('config_yaml').get_parameter_value().string_value
        self.mongo_host = self.get_parameter('mongodb_host').get_parameter_value().string_value
        self.mongo_port = self.get_parameter('mongodb_port').get_parameter_value().integer_value
        self.remote_mongo_host = self.get_parameter('remote_mongodb_host').get_parameter_value().string_value
        self.remote_mongo_port = self.get_parameter('remote_mongodb_port').get_parameter_value().integer_value
        self.enable_remote_logging = self.get_parameter('enable_remote_logging').get_parameter_value().bool_value
        self.min_distance_threshold = self.get_parameter('min_distance_threshold').get_parameter_value().double_value
        self.stop_timeout = self.get_parameter('stop_timeout').get_parameter_value().double_value

        self.get_logger().info(f"Config path: {self.config_path}")
        self.get_logger().info(
            f"Collision params: nav_thr={self.collision_nav_threshold}, "
            f"zero_thr={self.collision_zero_threshold}, time_window={self.collision_time_window}"
        )

        # DB Managers
        self.db_mgr_local = DBMgr(host=self.mongo_host, port=self.mongo_port)
        self.db_mgr_remote = None
        if self.enable_remote_logging and self.remote_mongo_host:
            try:
                self.db_mgr_remote = DBMgr(host=self.remote_mongo_host, port=self.remote_mongo_port)
                self.get_logger().info(
                    f"Remote DB logging enabled: {self.remote_mongo_host}:{self.remote_mongo_port}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to initialize remote DB manager: {e}")

        # Internal metrics state
        self.mdbi = 0.0
        self.incidents = 0
        self.distance = 0.0
        self.autonomous_distance = 0.0
        self.autonomous_time = 0.0
        self.autonomous_start_time = None
        self.AUTO = 'Autonomous'
        self.MAN = 'Manual'
        self.details = {'estop': False, 'operation_mode': self.AUTO}

        # Collision monitoring
        self.collision_incidents = 0
        self.collision_prev_has_velocity = False  # for falling-edge detection

        self.last_nav_cmd = None
        self.last_nav_time = None
        self.last_collision_cmd = None
        self.last_collision_time = None
        self.last_collision_logged_time = None

        # ----------------------------------------------------
        # Global Snapshot Storage
        # Stores the latest extracted values from all topics
        # ----------------------------------------------------
        self.system_snapshot = {}

        # Snapshot of latest values per field for change detection
        self.prev_field_values = {}

        # Current battery level (updated from any topic with 'battery_field')
        self.current_battery = None

        # Odometry / Speed state
        self.previous_x = None
        self.previous_y = None
        self.init_pose = True
        self.speed = 0.0
        self.last_odom_update_time = self.get_clock().now()

        self.dynamic_subs = []
        self.dynamic_publishers = {}
        self.topic_cfg_map = {}

        # Static Publishers (Standard Metrics)
        self.heartbeat_publisher = self.create_publisher(Bool, 'mdbi_logger/heartbeat', 10)
        self.distance_publisher = self.create_publisher(Float32, 'mdbi_logger/total_traveled_distance', 10)
        self.incidents_publisher = self.create_publisher(Int32, 'mdbi_logger/total_incidents', 10)
        self.speed_publisher = self.create_publisher(Float32, 'mdbi_logger/robot_speed', 10)
        self.collision_incidents_publisher = self.create_publisher(Int32, 'mdbi_logger/total_collision_incidents', 10)

        self.heartbeat_timer = self.create_timer(1.0, self.timer_callback)

        self.load_and_setup_config()

        # Init Session
        env_variables = {
            'robot_name': os.getenv('ROBOT_NAME', 'UNDEFINED'),
            'farm_name': os.getenv('FARM_NAME', 'UNDEFINED'),
            'field_name': os.getenv('FIELD_NAME', 'UNDEFINED'),
            'application': os.getenv('APPLICATION', 'UNDEFINED'),
            'scenario_name': os.getenv('SCENARIO_NAME', 'UNDEFINED'),
        }

        self.get_logger().info(
            f"Session env: robot={env_variables['robot_name']}, "
            f"farm={env_variables['farm_name']}, field={env_variables['field_name']}, "
            f"app={env_variables['application']}, scenario={env_variables['scenario_name']}"
        )

        git_repos = []
        if self.config:
            try:
                git_cfg = self.config.get('git_repos', {})
                for label, p in git_cfg.items():
                    repo_path = os.path.join('/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario', p)
                    repo_info = self.get_git_info(repo_path)
                    git_repos.append({label: repo_info})
                    self.get_logger().debug(
                        f"Git repo [{label}]: path={repo_info['path']}, "
                        f"branch={repo_info['branch']}, short_commit={repo_info['short_commit']}"
                    )
            except Exception as e:
                self.get_logger().warn(f"Failed to collect git info: {e}")

        self.db_mgr_local.init_session(env_variables, git_repos)
        if self.db_mgr_remote:
            self.db_mgr_remote.init_session(env_variables, git_repos)

    def load_and_setup_config(self):
        if not self.config_path:
            self.config = {}
            self.get_logger().warn("No config_yaml path provided; running with empty config.")
            return

        try:
            with open(self.config_path, 'r') as f:
                self.config = yaml.safe_load(f) or {}
            self.get_logger().info(
                f"Loaded metrics config from YAML. topics={len(self.config.get('topics', []))}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            self.config = {}
            return

        topics = self.config.get('topics', [])

        for item in topics:
            name = item.get('name')
            type_str = item.get('type')

            if not name or not type_str:
                self.get_logger().warn(f"Skipping invalid topic config: {item}")
                continue

            self.topic_cfg_map[name] = item

            # Dynamic Publisher Setup
            pub_cfg = item.get('publish', {})
            if pub_cfg.get('enable', False):
                try:
                    pub_msg_cls = import_msg_type(pub_cfg['type'])
                    pub = self.create_publisher(pub_msg_cls, pub_cfg['topic'], 10)
                    self.dynamic_publishers[name] = (pub, pub_msg_cls, pub_cfg.get('field'))
                    self.get_logger().info(
                        f"Dynamic publisher created: {pub_cfg['topic']} "
                        f"(type={pub_cfg['type']}) from topic={name}"
                    )
                except Exception as e:
                    self.get_logger().error(f"Pub creation failed for {name}: {e}")
                    self.dynamic_publishers[name] = None
            else:
                self.dynamic_publishers[name] = None

            # Subscription Setup
            try:
                msg_cls = import_msg_type(type_str)
                role = item.get('role', '').lower()

                if role == 'odometry':
                    cb = lambda msg, n=name: self.odom_role_callback(n, msg)
                elif role == 'control_mode':
                    cb = lambda msg, n=name: self.control_mode_role_callback(n, msg)
                elif role == 'estop':
                    cb = lambda msg, n=name: self.estop_role_callback(n, msg)
                elif role == 'collision_nav':
                    cb = lambda msg, n=name: self.collision_nav_callback(n, msg)
                elif role == 'collision_output':
                    cb = lambda msg, n=name: self.collision_output_callback(n, msg)
                else:
                    lf = item.get('log_fields', [])
                    cb = lambda msg, n=name, l=lf: self.generic_callback(n, msg, l)

                self.create_subscription(msg_cls, name, cb, qos_profile_sensor_data)
                self.get_logger().info(f"Subscribed to {name} (role={role})")

            except Exception as e:
                self.get_logger().error(f"Sub creation failed for {name}: {e}")

    def get_metrics_snapshot(self):
        """
        Returns a lightweight metrics summary to attach to every event.
        """
        metrics = {
            "distance": self.distance,
            "autonomous_distance": self.autonomous_distance,
            "speed": self.speed,
        }
        if self.current_battery is not None:
            metrics["battery_percentage"] = self.current_battery
        return metrics

    def trigger_intervention(self, event_type: str, extra: dict | None = None, force_count: bool = False):
        """
        Log an intervention-type event.

        - Only increments self.incidents when the robot is in Autonomous mode
          (self.details['operation_mode'] == self.AUTO), unless force_count=True.
        - Always logs the event (even in Manual) so history is preserved.
        """
        if extra is None:
            extra = {}

        current_mode = self.details.get('operation_mode', self.MAN)
        count_incident = force_count or (current_mode == self.AUTO)

        if count_incident:
            self.incidents += 1

        self.get_logger().info(
            f"[Intervention] type={event_type}, mode={current_mode}, "
            f"counted={count_incident}, total_incidents={self.incidents}"
        )

        base_details = dict(self.details)  # estop, operation_mode, etc.
        base_details.update(extra)
        base_details["system_snapshot"] = self.system_snapshot.copy()

        self.log_event(event_type, base_details)

    def handle_intervention_triggers(self, topic_name: str, data: dict, cfg: dict):
        """
        Apply YAML-configured triggers for this topic.
        - intervention_on_message: triggers on any message
        - intervention_on_change[field]: triggers when selected field changes
        """
        # 1) Trigger on any message (e.g. /cmd_vel/joy)
        msg_trig = cfg.get("intervention_on_message", {})
        if msg_trig.get("enable", False):
            evt_type = msg_trig.get("event_type", f"{topic_name}_activity")
            self.get_logger().info(
                f"[Trigger] intervention_on_message on topic '{topic_name}' -> event '{evt_type}'"
            )
            self.trigger_intervention(evt_type, extra={"topic": topic_name})

        # 2) Trigger on change of specific fields
        field_trigs = cfg.get("intervention_on_change", {})
        for field_name, trig_cfg in field_trigs.items():
            if field_name not in data:
                continue

            key = (topic_name, field_name)
            prev = self.prev_field_values.get(key)
            new = data[field_name]
            changed = (prev is None) or (prev != new)
            self.prev_field_values[key] = new

            if not changed:
                continue

            # Optional: only trigger on a specific value (e.g. True)
            trigger_value = trig_cfg.get("trigger_value", None)
            if trigger_value is not None and new != trigger_value:
                continue

            evt_type = trig_cfg.get(
                "event_type", f"{topic_name}:{field_name}_changed"
            )
            extra = {
                "topic": topic_name,
                "field": field_name,
                "new_value": new,
                "prev_value": prev,
            }
            self.get_logger().info(
                f"[Trigger] field_change on '{topic_name}.{field_name}' "
                f"from {prev} -> {new} -> event '{evt_type}'"
            )
            self.trigger_intervention(evt_type, extra=extra)

    # -------------------------
    # Callbacks
    # -------------------------
    def odom_role_callback(self, topic_name, msg):
        try:
            pos = msg.pose.pose.position
            # Update Snapshot with raw odom info
            self.system_snapshot['odometry'] = {
                'x': pos.x,
                'y': pos.y,
                'vx': msg.twist.twist.linear.x
            }
        except Exception:
            return

        current_time = self.get_clock().now()

        if self.init_pose:
            self.init_pose = False
            self.previous_x = pos.x
            self.previous_y = pos.y
            self.previous_time = current_time
            self.last_odom_update_time = current_time
            self.get_logger().info(
                f"[Odom] Initial pose set x={pos.x:.3f}, y={pos.y:.3f}"
            )
            return

        dx = pos.x - self.previous_x
        dy = pos.y - self.previous_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.min_distance_threshold:
            # Too small to count as movement
            return

        if not hasattr(self, 'previous_time'):
            self.previous_time = current_time

        time_diff = (current_time - self.previous_time).nanoseconds * 1e-9

        self.speed = dist / time_diff if time_diff > 0 else 0.0
        self.distance += dist

        if self.details.get('operation_mode') == self.AUTO:
            self.autonomous_distance += dist

        self.previous_x = pos.x
        self.previous_y = pos.y
        self.previous_time = current_time
        self.last_odom_update_time = current_time

        # Keep metrics in the snapshot too
        self.system_snapshot['metrics'] = {
            'distance': self.distance,
            'autonomous_distance': self.autonomous_distance,
            'speed': self.speed,
        }

        self.get_logger().debug(
            f"[Odom] step_dist={dist:.3f} m, total_dist={self.distance:.3f} m, "
            f"auto_dist={self.autonomous_distance:.3f} m, speed={self.speed:.3f} m/s, "
            f"mode={self.details.get('operation_mode')}"
        )

        self.publish_distance(self.distance)
        self.publish_speed(self.speed)
        self.handle_dynamic_publish(topic_name, msg)

    def control_mode_role_callback(self, topic_name, msg):
        cfg = self.topic_cfg_map.get(topic_name, {})
        mode_field = cfg.get('mode_field', 'data')
        try:
            val = get_nested_field(msg, mode_field)
        except Exception:
            return

        mapping = cfg.get('mode_mapping', {})
        new_mode = mapping.get(str(val)) or mapping.get(val)

        if new_mode is None:
            if str(val) == "3":
                new_mode = self.MAN
            else:
                new_mode = self.AUTO

        current_time = datetime.now()

        if new_mode != self.details.get('operation_mode'):
            prev_mode = self.details.get('operation_mode')
            self.details['operation_mode'] = new_mode
            self.get_logger().info(
                f"Mode changed (explicit): {prev_mode} -> {new_mode} (raw={val})"
            )

            if prev_mode == self.AUTO and new_mode == self.MAN:
                if self.autonomous_start_time:
                    delta = (current_time - self.autonomous_start_time).total_seconds()
                    self.autonomous_time += delta
                    self.autonomous_start_time = None

                # Manual override is an intervention
                self.trigger_intervention('Manual_override', force_count=True)

            elif new_mode == self.AUTO:
                self.autonomous_start_time = current_time
                # Logging autonomous mode change (not counted as intervention)
                self.log_event('Autonomous', {
                    **self.details,
                    'system_snapshot': self.system_snapshot.copy()
                })

    def estop_role_callback(self, topic_name, msg):
        cfg = self.topic_cfg_map.get(topic_name, {})
        field = cfg.get('field', 'data')
        try:
            v = bool(get_nested_field(msg, field))
            # Update Snapshot
            self.system_snapshot['estop'] = v
        except Exception:
            return

        if v != self.details.get('estop', False):
            self.details['estop'] = v
            self.get_logger().info(f"E-Stop state changed: {v}")
            if v:
                # E-stop counts as intervention
                self.trigger_intervention('EMS')

    def generic_callback(self, topic_name, msg, log_fields):
        cfg = self.topic_cfg_map.get(topic_name, {})

        data = {}
        log_all = bool(cfg.get("log_all_fields", False))

        if log_all:
            try:
                data = ros_msg_to_dict(msg)
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to serialize all fields for {topic_name}: {e}"
                )
                data = {}
        else:
            for f in log_fields:
                try:
                    data[f] = get_nested_field(msg, f)
                except Exception:
                    pass

        if data:
            # Update Snapshot with the latest values from this topic
            self.system_snapshot[topic_name] = data

            # If this topic carries battery info, update the battery state
            battery_field = cfg.get("battery_field")
            if battery_field and battery_field in data:
                old_batt = self.current_battery
                self.current_battery = data[battery_field]
                if old_batt is None or abs(self.current_battery - old_batt) >= 1.0:
                    self.get_logger().info(
                        f"[Battery] {topic_name}.{battery_field} updated: "
                        f"{old_batt} -> {self.current_battery}"
                    )

        # Apply YAML-configured intervention triggers (message & field-change)
        self.handle_intervention_triggers(topic_name, data, cfg)

        # Dynamic republishing if configured
        self.handle_dynamic_publish(topic_name, msg)

    def handle_dynamic_publish(self, topic_name, msg):
        pub_cfg = self.dynamic_publishers.get(topic_name)
        if not pub_cfg:
            return
        publisher, pub_msg_cls, pub_field = pub_cfg

        try:
            val = get_nested_field(msg, pub_field) if pub_field else msg

            out_msg = pub_msg_cls()
            if hasattr(out_msg, 'data'):
                if 'Float' in pub_msg_cls.__name__:
                    out_msg.data = float(val)
                elif 'Int' in pub_msg_cls.__name__:
                    out_msg.data = int(val)
                elif 'String' in pub_msg_cls.__name__:
                    out_msg.data = str(val)
                else:
                    out_msg.data = val
            else:
                # Try simple attribute matching
                for attr in dir(out_msg):
                    if not attr.startswith('_'):
                        try:
                            setattr(out_msg, attr, val)
                            break
                        except Exception:
                            continue

            publisher.publish(out_msg)
            self.get_logger().debug(
                f"[DynamicPublish] topic={topic_name} -> republished "
                f"value={val} ({pub_msg_cls.__name__})"
            )

        except Exception as e:
            self.get_logger().error(f"Dynamic publish failed for {topic_name}: {e}")

    def collision_nav_callback(self, topic_name, msg):
        """
        Store latest /cmd_vel/nav command.
        """
        now = self.get_clock().now()
        self.last_nav_cmd = msg
        self.last_nav_time = now

        self.system_snapshot['cmd_vel_nav'] = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
        }

        self.get_logger().debug(
            f"[Collision] NAV cmd received: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, "
            f"wz={msg.angular.z:.3f}"
        )
        # IMPORTANT: do NOT call check_collision_condition here

    def collision_output_callback(self, topic_name, msg):
        """
        Store latest /cmd_vel/collision command and check for collision condition.
        We only increment collision_incidents on the falling edge:
        - previously had velocity on /cmd_vel/collision
        - now ~zero velocity
        - while /cmd_vel/nav still commands forward motion
        """
        now = self.get_clock().now()
        self.last_collision_cmd = msg
        self.last_collision_time = now

        coll_vx = float(msg.linear.x)

        self.system_snapshot['cmd_vel_collision'] = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
        }

        self.get_logger().debug(
            f"[Collision] COLLISION cmd received: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, "
            f"wz={msg.angular.z:.3f}"
        )

        # Check for falling edge-based collision
        self.check_collision_condition(now, coll_vx)

    def check_collision_condition(self, now, coll_vx_now: float):
        """
        Detect a collision event only on falling edge:

          - previous /cmd_vel/collision had velocity (|vx| > collision_zero_threshold)
          - current /cmd_vel/collision has ~zero velocity (|vx| <= collision_zero_threshold)
          - latest /cmd_vel/nav.linear.x > collision_nav_threshold
          - nav command is recent (within collision_time_window)

        When this condition is met:
          - increment collision_incidents once
          - log a 'Collision' event with full snapshot
          - publish total collision_incidents

        We only increment again if the cycle repeats:
          velocity on /cmd_vel/collision -> zero -> velocity -> zero -> ...
        """
        # Need a nav command to interpret the collision behaviour
        if self.last_nav_cmd is None or self.last_nav_time is None:
            self.get_logger().debug(
                "[Collision] No NAV cmd yet; cannot evaluate collision condition."
            )
            collision_has_velocity_now = abs(coll_vx_now) > self.collision_zero_threshold
            self.collision_prev_has_velocity = collision_has_velocity_now
            return

        # Check nav recency
        dt_nav = (now - self.last_nav_time).nanoseconds * 1e-9
        if dt_nav > self.collision_time_window:
            self.get_logger().debug(
                f"[Collision] NAV cmd too old ({dt_nav:.3f}s > {self.collision_time_window}s); "
                "skipping collision evaluation."
            )
            collision_has_velocity_now = abs(coll_vx_now) > self.collision_zero_threshold
            self.collision_prev_has_velocity = collision_has_velocity_now
            return

        nav_vx = float(self.last_nav_cmd.linear.x)

        # Does nav actually request motion?
        if nav_vx <= self.collision_nav_threshold:
            self.get_logger().debug(
                f"[Collision] NAV vx={nav_vx:.3f} <= nav_threshold={self.collision_nav_threshold}; "
                "no forward motion requested."
            )
            collision_has_velocity_now = abs(coll_vx_now) > self.collision_zero_threshold
            self.collision_prev_has_velocity = collision_has_velocity_now
            return

        # Edge detection on /cmd_vel/collision
        collision_has_velocity_now = abs(coll_vx_now) > self.collision_zero_threshold

        self.get_logger().debug(
            f"[Collision] prev_has_vel={self.collision_prev_has_velocity}, "
            f"now_has_vel={collision_has_velocity_now}, coll_vx_now={coll_vx_now:.3f}, "
            f"nav_vx={nav_vx:.3f}"
        )

        # Falling edge: had velocity before, now ~zero
        if self.collision_prev_has_velocity and not collision_has_velocity_now:
            # This is one collision incident
            self.collision_incidents += 1
            self.get_logger().info(
                f"[Collision] Detected collision incident #{self.collision_incidents} "
                f"(nav_vx={nav_vx:.3f}, coll_vx_now={coll_vx_now:.3f})"
            )
            self.log_collision_event()

        # Update for next call
        self.collision_prev_has_velocity = collision_has_velocity_now

    # -------------------------
    # Timer & Logging
    # -------------------------
    def timer_callback(self):
        hb = Bool()
        hb.data = True
        self.heartbeat_publisher.publish(hb)

        now = self.get_clock().now()
        time_since_move = (now - self.last_odom_update_time).nanoseconds * 1e-9

        if time_since_move > self.stop_timeout and self.speed > 0.0:
            self.get_logger().info(
                f"[Timer] Robot has not moved for {time_since_move:.2f}s; "
                "forcing speed to 0.0"
            )
            self.speed = 0.0
            self.publish_speed(0.0)

    def update_db_metrics(self):
        # MBDI = autonomous_distance / incidents
        if self.incidents != 0:
            mdbi_val = float(self.autonomous_distance) / float(self.incidents)
        else:
            # No incidents → use autonomous distance as-is (no division)
            mdbi_val = float(self.autonomous_distance)

        self.get_logger().debug(
            f"[DB] metrics update: dist={self.distance:.3f}, auto_dist={self.autonomous_distance:.3f}, "
            f"incidents={self.incidents}, collisions={self.collision_incidents}, mdbi={mdbi_val:.3f}"
        )

        db_managers = [self.db_mgr_local]
        if self.db_mgr_remote:
            db_managers.append(self.db_mgr_remote)

        for dbm in db_managers:
            try:
                dbm.update_distance(self.distance)
                dbm.update_incidents(self.incidents)
                dbm.update_autonomous_distance(self.autonomous_distance)
                dbm.update_mdbi(mdbi_val)
                dbm.update_collision_incidents(self.collision_incidents)
            except Exception as e:
                self.get_logger().warn(f"DB update failed: {e}")

    def log_event(self, msg='', details=None):
        if details is None:
            details = {}

        # Attach metrics to every event
        details = {
            **details,
            'metrics': self.get_metrics_snapshot()
        }

        event_time = datetime.now(tz=timezone.utc)
        event = {'time': event_time, 'event_type': msg, 'details': details}

        self.get_logger().info(
            f"[Event] type={msg}, time={event_time.isoformat()}, "
            f"incidents={self.incidents}, collisions={self.collision_incidents}"
        )

        self.db_mgr_local.add_event(event)
        if self.db_mgr_remote:
            try:
                self.db_mgr_remote.add_event(event)
            except Exception as e:
                self.get_logger().warn(f"Remote DB event log failed: {e}")

        incidents_msg = Int32()
        incidents_msg.data = self.incidents
        self.incidents_publisher.publish(incidents_msg)

        self.update_db_metrics()

    def log_collision_event(self):
        """
        Log a 'Collision' event, capturing:
          - full system snapshot
          - nav & collision commands
          - total incidents and collision_incidents
        """
        details = {
            "collision_incident_index": self.collision_incidents,
            "incidents": self.incidents,
            "collision_incidents": self.collision_incidents,
            "total_incidents": self.incidents + self.collision_incidents,
            "nav_cmd": {
                "linear": {
                    "x": float(self.last_nav_cmd.linear.x),
                    "y": float(self.last_nav_cmd.linear.y),
                },
                "angular": {
                    "z": float(self.last_nav_cmd.angular.z),
                },
            },
            "collision_cmd": {
                "linear": {
                    "x": float(self.last_collision_cmd.linear.x),
                    "y": float(self.last_collision_cmd.linear.y),
                },
                "angular": {
                    "z": float(self.last_collision_cmd.angular.z),
                },
            },
            # Full snapshot of all system data at this moment
            "system_snapshot": self.system_snapshot.copy(),
        }

        self.get_logger().info(
            f"[Collision] Logging collision event #{self.collision_incidents} "
            f"(total_incidents={details['total_incidents']})"
        )

        # This will attach metrics (distance, speed, battery, etc.) and write to DB
        self.log_event("Collision", details)

        # Publish total collisions on a dedicated topic
        msg = Int32()
        msg.data = int(self.collision_incidents)
        self.collision_incidents_publisher.publish(msg)

    def publish_distance(self, dist):
        msg = Float32()
        msg.data = float(dist)
        self.distance_publisher.publish(msg)
        self.get_logger().debug(f"[Publish] distance={dist:.3f}")
        self.update_db_metrics()

    def publish_speed(self, speed):
        msg = Float32()
        msg.data = float(speed)
        self.speed_publisher.publish(msg)
        self.get_logger().debug(f"[Publish] speed={speed:.3f}")

    def get_git_info(self, repo_path):
        """
        Return basic git metadata for a repo.

        Example schema:
        {
            "path": "/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario",
            "exists": True,
            "remote": "git@github.com:LCAS/aoc_strawberry_scenario.git",
            "branch": "main",
            "commit": "f3a6c1b4e6f8c3b3c823f2c45e56a123456789ab",
            "short_commit": "f3a6c1b",
            "commit_message": "Fix navsat transform delay",
            "tags": ["v0.3.1"],
            "describe": "v0.3.1-2-gf3a6c1b",
            "dirty": false,
            "error": null
        }
        """
        info = {
            "path": repo_path,
            "exists": False,
            "remote": None,
            "branch": None,
            "commit": None,
            "short_commit": None,
            "commit_message": None,  # NEW
            "tags": [],              # NEW
            "describe": None,        # NEW
            "dirty": None,
            "error": None,
        }

        try:
            if not repo_path or not os.path.isdir(repo_path):
                info["error"] = "Path does not exist or is not a directory"
                return info

            def git(args):
                result = subprocess.run(
                    ["git", "-C", repo_path] + args,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                if result.returncode != 0:
                    raise RuntimeError(result.stderr.strip())
                return result.stdout.strip()

            info["exists"] = True

            # Remote
            try:
                info["remote"] = git(["config", "--get", "remote.origin.url"])
            except Exception:
                info["remote"] = None

            # Branch
            try:
                info["branch"] = git(["rev-parse", "--abbrev-ref", "HEAD"])
            except Exception:
                info["branch"] = None

            # Commit hashes
            try:
                info["commit"] = git(["rev-parse", "HEAD"])
            except Exception:
                info["commit"] = None

            try:
                info["short_commit"] = git(["rev-parse", "--short", "HEAD"])
            except Exception:
                info["short_commit"] = None

            # NEW: Last commit message (full body or just subject)
            try:
                info["commit_message"] = git(["log", "-1", "--pretty=%s"])
            except Exception:
                info["commit_message"] = None

            # NEW: Tags pointing at HEAD
            try:
                tags_str = git(["tag", "--points-at", "HEAD"])
                if tags_str:
                    info["tags"] = tags_str.splitlines()
                else:
                    info["tags"] = []
            except Exception:
                info["tags"] = []

            # NEW: git describe (nearest tag + distance)
            try:
                info["describe"] = git(["describe", "--tags", "--always"])
            except Exception:
                info["describe"] = None

            # Dirty / clean state
            try:
                status = git(["status", "--porcelain"])
                info["dirty"] = bool(status)
            except Exception:
                info["dirty"] = None

        except Exception as e:
            info["error"] = str(e)

        return info


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyMetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AutonomyMetricsLogger (KeyboardInterrupt).")
        pass
    finally:
        node.get_logger().info(
            f"Final stats: distance={node.distance:.3f}, auto_distance={node.autonomous_distance:.3f}, "
            f"incidents={node.incidents}, collisions={node.collision_incidents}"
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()