#!/usr/bin/env python3
"""
YAML-driven AutonomyMetricsLogger
Author: Ibrahim Hroob - JABASAI
"""

import os
import math
import subprocess
import yaml
from datetime import datetime, timezone
from importlib import import_module

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from std_msgs.msg import Bool, Float32, Int32, String

from autonomy_metrics.db_mgr import DatabaseMgr as DBMgr


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def ros_msg_to_dict(msg):
    """
    Recursively convert a ROS 2 Python message into a plain dict / list / primitive
    so it can be safely stored in system_snapshot / Mongo.
    """
    if isinstance(msg, (bool, int, float, str)):
        return msg

    if isinstance(msg, (list, tuple)):
        return [ros_msg_to_dict(v) for v in msg]

    if hasattr(msg, "get_fields_and_field_types"):
        data = {}
        for field_name in msg.get_fields_and_field_types().keys():
            try:
                val = getattr(msg, field_name)
                data[field_name] = ros_msg_to_dict(val)
            except Exception:
                pass
        return data

    if hasattr(msg, "__dict__"):
        return {
            k: ros_msg_to_dict(v)
            for k, v in msg.__dict__.items()
            if not k.startswith('_')
        }

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


# QoS profiles --------------------------------------------------------------
# State topics (operation mode, robot state, e-stop) MUST be RELIABLE: a
# missed message could mis-attribute distance to the wrong mode for several
# seconds, which directly affects billing.
STATE_QOS = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# Latched outputs for health (DB connectivity reporting). Subscribers that
# join late still see the last value, which is what operators want for
# alerting / dashboards.
LATCHED_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class AutonomyMetricsLogger(Node):
    def __init__(self):
        super().__init__('mdbi_logger_dynamic')

        self.get_logger().info("Starting YAML-driven AutonomyMetricsLogger")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter(
            'config_yaml',
            '/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario/'
            'jabas/autonomy_metrics_logger/src/autonomy_metrics/config/metrics_full.yaml',
        )
        self.declare_parameter('mongodb_host', 'localhost')
        self.declare_parameter('mongodb_port', 27018)
        self.declare_parameter('remote_mongodb_host', '')
        self.declare_parameter('remote_mongodb_port', 27017)
        self.declare_parameter('enable_remote_logging', False)
        self.declare_parameter('min_distance_threshold', 0.2)
        self.declare_parameter('stop_timeout', 2.0)

        # DB resilience / cadence
        self.declare_parameter('db_metrics_period', 1.0)            # s, periodic save of metrics
        self.declare_parameter('db_server_selection_timeout_ms', 1000)
        self.declare_parameter('db_connect_timeout_ms', 1000)
        self.declare_parameter('db_socket_timeout_ms', 2000)

        # Collision detection tuning
        self.declare_parameter('collision_nav_threshold', 0.01)
        self.declare_parameter('collision_zero_threshold', 0.001)
        self.declare_parameter('collision_time_window', 0.5)
        self.declare_parameter('collision_log_cooldown', 1.0)

        # Read parameters
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
        self.db_metrics_period = self.get_parameter('db_metrics_period').get_parameter_value().double_value
        self.db_server_selection_timeout_ms = self.get_parameter('db_server_selection_timeout_ms').get_parameter_value().integer_value
        self.db_connect_timeout_ms = self.get_parameter('db_connect_timeout_ms').get_parameter_value().integer_value
        self.db_socket_timeout_ms = self.get_parameter('db_socket_timeout_ms').get_parameter_value().integer_value

        self.get_logger().info(f"Config path: {self.config_path}")
        self.get_logger().info(
            f"Collision params: nav_thr={self.collision_nav_threshold}, "
            f"zero_thr={self.collision_zero_threshold}, time_window={self.collision_time_window}"
        )
        self.get_logger().info(
            f"DB params: local={self.mongo_host}:{self.mongo_port}, "
            f"remote_enabled={self.enable_remote_logging}, "
            f"remote={self.remote_mongo_host}:{self.remote_mongo_port}, "
            f"metrics_period={self.db_metrics_period}s, "
            f"server_selection_timeout={self.db_server_selection_timeout_ms}ms"
        )

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self.AUTO = 'Autonomous'
        self.MAN = 'Manual'

        self.mdbi = 0.0
        self.incidents = 0                  # only auto->manual transitions feed MDBI
        self.distance = 0.0
        self.autonomous_distance = 0.0
        self.manual_distance = 0.0
        self.autonomous_time = 0.0
        self.autonomous_start_time = None
        self.details = {'estop': False, 'operation_mode': self.AUTO, 'robot_state': None}

        # Collision monitoring
        self.collision_incidents = 0
        self.collision_prev_has_velocity = False
        self.last_nav_cmd = None
        self.last_nav_time = None
        self.last_collision_cmd = None
        self.last_collision_time = None
        self.last_collision_logged_time = None

        # System snapshot + change tracking
        self.system_snapshot = {}
        self.prev_field_values = {}
        self.current_battery = None

        # Odometry / Speed state
        self.previous_x = None
        self.previous_y = None
        self.init_pose = True
        self.speed = 0.0
        self.last_odom_update_time = self.get_clock().now()

        # Subscriptions / publishers / per-topic config
        self.dynamic_subs = []
        self.dynamic_publishers = {}
        self.topic_cfg_map = {}

        # Cached env + git info, used to retry init_session if Mongo was down at start
        self._session_env = None
        self._session_git_repos = None

        # DB health tracking ------------------------------------------------
        # Per-DB status; reason holds the latest error message (or empty)
        self._db_status = {
            'local': {'healthy': False, 'reason': 'not_initialized'},
            'remote': {'healthy': False, 'reason': 'disabled'},
        }
        # Aggregate health, latched, only republished when it changes
        self._db_healthy = None
        self._db_health_reason = None

        # ------------------------------------------------------------------
        # Static publishers
        # ------------------------------------------------------------------
        self.heartbeat_publisher = self.create_publisher(Bool, 'mdbi_logger/heartbeat', 10)
        self.distance_publisher = self.create_publisher(Float32, 'mdbi_logger/total_traveled_distance', 10)
        self.autonomous_distance_publisher = self.create_publisher(Float32, 'mdbi_logger/total_autonomous_distance', 10)
        self.manual_distance_publisher = self.create_publisher(Float32, 'mdbi_logger/total_manual_distance', 10)
        self.incidents_publisher = self.create_publisher(Int32, 'mdbi_logger/total_incidents', 10)
        self.speed_publisher = self.create_publisher(Float32, 'mdbi_logger/robot_speed', 10)
        self.collision_incidents_publisher = self.create_publisher(
            Int32, 'mdbi_logger/total_collision_incidents', 10
        )

        # Latched health
        self.db_health_publisher = self.create_publisher(Bool, 'mdbi_logger/db_health', LATCHED_QOS)
        self.db_health_reason_publisher = self.create_publisher(String, 'mdbi_logger/db_health_reason', LATCHED_QOS)

        # ------------------------------------------------------------------
        # DB managers — never raise on construction
        # ------------------------------------------------------------------
        self.db_mgr_local = self._build_db_mgr(self.mongo_host, self.mongo_port, label='local')
        if self.enable_remote_logging and self.remote_mongo_host:
            self.db_mgr_remote = self._build_db_mgr(self.remote_mongo_host, self.remote_mongo_port, label='remote')
            if self.db_mgr_remote is not None:
                self.get_logger().info(
                    f"Remote DB logging enabled: {self.remote_mongo_host}:{self.remote_mongo_port}"
                )
        else:
            self.db_mgr_remote = None
            self._db_status['remote'] = {'healthy': True, 'reason': 'disabled'}

        # Publish initial unhealthy state so subscribers know we haven't connected yet
        self._recompute_and_publish_health()

        # ------------------------------------------------------------------
        # YAML config + subscriptions
        # ------------------------------------------------------------------
        self.load_and_setup_config()

        # ------------------------------------------------------------------
        # Session metadata (env + git info) — try init now, retry on watchdog
        # ------------------------------------------------------------------
        self._session_env = {
            'robot_name': os.getenv('ROBOT_NAME', 'UNDEFINED'),
            'farm_name': os.getenv('JABAS_SITE', 'UNDEFINED'),
        }
        self.get_logger().info(
            f"Session env: robot={self._session_env['robot_name']}, "
            f"farm={self._session_env['farm_name']}, "
        )

        self._session_git_repos = self._collect_git_repos_info()
        self._try_init_sessions()  # best effort; will retry on watchdog

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self.heartbeat_timer = self.create_timer(1.0, self.timer_callback)

        # Periodic DB metrics save (billing-critical)
        period = max(0.1, float(self.db_metrics_period))
        self.db_metrics_timer = self.create_timer(period, self.db_metrics_tick)
        self.get_logger().info(f"DB metrics save period: {period:.2f}s")

    # ----------------------------------------------------------------------
    # DB construction / health
    # ----------------------------------------------------------------------
    def _build_db_mgr(self, host, port, label):
        try:
            return DBMgr(
                host=host,
                port=port,
                label=label,
                server_selection_timeout_ms=self.db_server_selection_timeout_ms,
                connect_timeout_ms=self.db_connect_timeout_ms,
                socket_timeout_ms=self.db_socket_timeout_ms,
            )
        except Exception as e:
            # MongoClient is lazy so this is unlikely, but be defensive.
            self.get_logger().error(
                f"Failed to construct DatabaseMgr({label}) for {host}:{port}: {e}"
            )
            self._db_status[label] = {'healthy': False, 'reason': f'construct_failed: {e}'}
            return None

    def _safe_db_call(self, label, fn, *args, **kwargs):
        """
        Run a DB op with full exception isolation. Returns (success, error_msg).
        On failure, marks the DB unhealthy and republishes health.
        """
        if fn is None:
            return False, 'no_op'
        try:
            fn(*args, **kwargs)
            self._mark_db_healthy(label)
            return True, ''
        except Exception as e:
            err = f"{type(e).__name__}: {e}"
            self.get_logger().warn(f"[DB:{label}] {fn.__name__} failed: {err}")
            self._mark_db_unhealthy(label, err)
            return False, err

    def _mark_db_healthy(self, label):
        prev = self._db_status.get(label, {})
        if not prev.get('healthy'):
            self.get_logger().info(f"[DB:{label}] Healthy")
        self._db_status[label] = {'healthy': True, 'reason': 'ok'}
        self._recompute_and_publish_health()

    def _mark_db_unhealthy(self, label, reason):
        self._db_status[label] = {'healthy': False, 'reason': reason}
        self._recompute_and_publish_health()

    def _recompute_and_publish_health(self):
        """
        Aggregate health = local OK AND (remote disabled OR remote OK).
        Only publishes when state or reason changes (latched topics).
        """
        local = self._db_status.get('local', {'healthy': False, 'reason': 'unknown'})
        remote = self._db_status.get('remote', {'healthy': True, 'reason': 'disabled'})

        healthy = bool(local.get('healthy')) and bool(remote.get('healthy'))

        parts = []
        if not local.get('healthy'):
            parts.append(f"local: {local.get('reason', 'unknown')}")
        if (
            self.enable_remote_logging
            and self.remote_mongo_host
            and not remote.get('healthy')
        ):
            parts.append(f"remote: {remote.get('reason', 'unknown')}")

        if healthy:
            reason = 'ok'
        else:
            reason = '; '.join(parts) if parts else 'unhealthy'

        if healthy == self._db_healthy and reason == self._db_health_reason:
            return  # no change

        self._db_healthy = healthy
        self._db_health_reason = reason

        bmsg = Bool()
        bmsg.data = bool(healthy)
        self.db_health_publisher.publish(bmsg)

        smsg = String()
        smsg.data = reason
        self.db_health_reason_publisher.publish(smsg)

        if healthy:
            self.get_logger().info(f"[DBHealth] OK ({reason})")
        else:
            self.get_logger().warn(f"[DBHealth] ISSUE - {reason}")

    def _try_init_sessions(self):
        """
        Attempt session init for each DB manager. Idempotent — safe to call
        repeatedly from a watchdog tick. Never raises.
        """
        if self._session_env is None:
            return

        if self.db_mgr_local is not None and self.db_mgr_local.session_id is None:
            self._safe_db_call(
                'local', self.db_mgr_local.init_session,
                self._session_env, self._session_git_repos or [],
            )

        if (
            self.db_mgr_remote is not None
            and self.db_mgr_remote.session_id is None
        ):
            self._safe_db_call(
                'remote', self.db_mgr_remote.init_session,
                self._session_env, self._session_git_repos or [],
            )

    def _collect_git_repos_info(self):
        repos = []
        if not self.config:
            return repos
        try:
            git_cfg = self.config.get('git_repos', {})
            for label, p in git_cfg.items():
                repo_path = os.path.join(
                    '/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario', p
                )
                repo_info = self.get_git_info(repo_path)
                repos.append({label: repo_info})
                self.get_logger().debug(
                    f"Git repo [{label}]: path={repo_info['path']}, "
                    f"branch={repo_info['branch']}, short_commit={repo_info['short_commit']}"
                )
        except Exception as e:
            self.get_logger().warn(f"Failed to collect git info: {e}")
        return repos

    # ----------------------------------------------------------------------
    # YAML config loading
    # ----------------------------------------------------------------------
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

            # Dynamic publisher
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

            # Subscription
            try:
                msg_cls = import_msg_type(type_str)
                role = item.get('role', '').lower()

                # State-style topics get RELIABLE QoS so we never miss a mode change.
                state_roles = {
                    'autonomous_mode',
                    'robot_state',
                    'control_mode',
                    'estop',
                }
                qos = STATE_QOS if role in state_roles else qos_profile_sensor_data

                if role == 'odometry':
                    cb = lambda msg, n=name: self.odom_role_callback(n, msg)
                elif role == 'autonomous_mode':
                    cb = lambda msg, n=name: self.autonomous_mode_role_callback(n, msg)
                elif role == 'robot_state':
                    cb = lambda msg, n=name: self.robot_state_role_callback(n, msg)
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

                self.create_subscription(msg_cls, name, cb, qos)
                self.get_logger().info(
                    f"Subscribed to {name} (role={role or 'generic'}, qos={'RELIABLE' if qos is STATE_QOS else 'sensor_data'})"
                )

            except Exception as e:
                self.get_logger().error(f"Sub creation failed for {name}: {e}")

    # ----------------------------------------------------------------------
    # Snapshot helpers
    # ----------------------------------------------------------------------
    def get_metrics_snapshot(self):
        metrics = {
            "distance": self.distance,
            "autonomous_distance": self.autonomous_distance,
            "manual_distance": self.manual_distance,
            "speed": self.speed,
        }
        if self.current_battery is not None:
            metrics["battery_percentage"] = self.current_battery
        return metrics

    # ----------------------------------------------------------------------
    # Intervention / event triggers
    # ----------------------------------------------------------------------
    def trigger_intervention(self, event_type: str, extra: dict | None = None, force_count: bool = False):
        """
        Increment incidents (for MDBI) only on Autonomous mode or when
        force_count=True, and log the event regardless.
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

        base_details = dict(self.details)
        base_details.update(extra)
        base_details["system_snapshot"] = self.system_snapshot.copy()

        self.log_event(event_type, base_details)

    def handle_intervention_triggers(self, topic_name: str, data: dict, cfg: dict):
        # 1) any-message trigger
        msg_trig = cfg.get("intervention_on_message", {})
        if msg_trig.get("enable", False):
            evt_type = msg_trig.get("event_type", f"{topic_name}_activity")
            self.get_logger().info(
                f"[Trigger] intervention_on_message on '{topic_name}' -> '{evt_type}'"
            )
            self.trigger_intervention(evt_type, extra={"topic": topic_name})

        # 2) field-change trigger
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
                f"[Trigger] field_change '{topic_name}.{field_name}' "
                f"{prev} -> {new} -> '{evt_type}'"
            )
            self.trigger_intervention(evt_type, extra=extra)

    # ----------------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------------
    def odom_role_callback(self, topic_name, msg):
        try:
            pos = msg.pose.pose.position
            self.system_snapshot['odometry'] = {
                'x': pos.x,
                'y': pos.y,
                'vx': msg.twist.twist.linear.x,
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

        # Debounce: very small odom deltas are noise. Note we do NOT update
        # previous_x/y here so the next iteration measures from the older
        # anchor — accumulated motion is preserved exactly.
        if dist < self.min_distance_threshold:
            return

        if not hasattr(self, 'previous_time'):
            self.previous_time = current_time

        time_diff = (current_time - self.previous_time).nanoseconds * 1e-9

        self.speed = dist / time_diff if time_diff > 0 else 0.0
        self.distance += dist

        if self.details.get('operation_mode') == self.AUTO:
            self.autonomous_distance += dist
        else:
            self.manual_distance += dist

        self.previous_x = pos.x
        self.previous_y = pos.y
        self.previous_time = current_time
        self.last_odom_update_time = current_time

        self.system_snapshot['metrics'] = {
            'distance': self.distance,
            'autonomous_distance': self.autonomous_distance,
            'manual_distance': self.manual_distance,
            'speed': self.speed,
        }

        self.get_logger().debug(
            f"[Odom] step={dist:.3f}m total={self.distance:.3f}m "
            f"auto={self.autonomous_distance:.3f}m manual={self.manual_distance:.3f}m "
            f"speed={self.speed:.3f}m/s mode={self.details.get('operation_mode')}"
        )

        # Live ROS publishes (cheap). Periodic DB save is handled separately.
        self.publish_distance_topics()
        self.publish_speed(self.speed)
        self.handle_dynamic_publish(topic_name, msg)

    def autonomous_mode_role_callback(self, topic_name, msg):
        """
        std_msgs/Bool: True -> Autonomous, False -> Manual.
        - On change Auto -> Manual: log Manual_override AND increment incidents (MDBI source).
        - On change Manual -> Auto: log Autonomous_resumed (no incident).
        """
        try:
            is_auto = bool(msg.data)
        except Exception as e:
            self.get_logger().warn(f"[autonomous_mode] could not read .data on {topic_name}: {e}")
            return

        new_mode = self.AUTO if is_auto else self.MAN
        prev_mode = self.details.get('operation_mode')

        # Always reflect in snapshot
        self.system_snapshot['autonomous_mode'] = is_auto

        if new_mode == prev_mode:
            return

        self.details['operation_mode'] = new_mode
        now = datetime.now()
        self.get_logger().info(
            f"[autonomous_mode] {prev_mode} -> {new_mode} (raw={is_auto})"
        )

        if prev_mode == self.AUTO and new_mode == self.MAN:
            # Close out the autonomous interval
            if self.autonomous_start_time:
                self.autonomous_time += (now - self.autonomous_start_time).total_seconds()
                self.autonomous_start_time = None
            # Auto -> Manual is the ONLY transition that drives MDBI.
            self.trigger_intervention(
                'Manual_override',
                extra={'source': 'autonomous_mode', 'topic': topic_name},
                force_count=True,
            )
        elif new_mode == self.AUTO:
            self.autonomous_start_time = now
            self.log_event('Autonomous_resumed', {
                **self.details,
                'source': 'autonomous_mode',
                'topic': topic_name,
                'system_snapshot': self.system_snapshot.copy(),
            })

    def robot_state_role_callback(self, topic_name, msg):
        """
        std_msgs/String: expected values "disabled" / "enabled" / "active".
        Logs an event on every change. Never increments MDBI incidents.
        """
        try:
            val = str(msg.data)
        except Exception as e:
            self.get_logger().warn(f"[robot_state] could not read .data on {topic_name}: {e}")
            return

        normalized = val.strip().lower()
        prev = self.details.get('robot_state')

        # Reflect in snapshot regardless of change
        self.system_snapshot['robot_state'] = val

        if normalized == prev:
            return

        self.details['robot_state'] = normalized
        self.get_logger().info(f"[robot_state] {prev} -> {normalized}")

        # Log event but do NOT count as MDBI incident.
        self.log_event('Robot_state_changed', {
            **self.details,
            'topic': topic_name,
            'prev_value': prev,
            'new_value': normalized,
            'raw_value': val,
            'system_snapshot': self.system_snapshot.copy(),
        })

    def control_mode_role_callback(self, topic_name, msg):
        """
        Legacy explicit-control-mode role. Kept for backward-compat with old
        configs that mapped a String/Int field to mode_mapping. The new
        recommended path is the simpler `autonomous_mode` Bool role.
        """
        cfg = self.topic_cfg_map.get(topic_name, {})
        mode_field = cfg.get('mode_field', 'data')
        try:
            val = get_nested_field(msg, mode_field)
        except Exception:
            return

        mapping = cfg.get('mode_mapping', {})
        new_mode = mapping.get(str(val)) or mapping.get(val)

        if new_mode is None:
            new_mode = self.MAN if str(val) == "3" else self.AUTO

        if new_mode == self.details.get('operation_mode'):
            return

        prev_mode = self.details.get('operation_mode')
        self.details['operation_mode'] = new_mode
        self.get_logger().info(
            f"[control_mode] {prev_mode} -> {new_mode} (raw={val})"
        )
        now = datetime.now()

        if prev_mode == self.AUTO and new_mode == self.MAN:
            if self.autonomous_start_time:
                self.autonomous_time += (now - self.autonomous_start_time).total_seconds()
                self.autonomous_start_time = None
            self.trigger_intervention('Manual_override', force_count=True)
        elif new_mode == self.AUTO:
            self.autonomous_start_time = now
            self.log_event('Autonomous', {
                **self.details,
                'system_snapshot': self.system_snapshot.copy(),
            })

    def estop_role_callback(self, topic_name, msg):
        cfg = self.topic_cfg_map.get(topic_name, {})
        field = cfg.get('field', 'data')
        try:
            v = bool(get_nested_field(msg, field))
            self.system_snapshot['estop'] = v
        except Exception:
            return

        if v != self.details.get('estop', False):
            self.details['estop'] = v
            self.get_logger().info(f"E-Stop state changed: {v}")
            if v:
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
            self.system_snapshot[topic_name] = data

            battery_field = cfg.get("battery_field")
            if battery_field and battery_field in data:
                old_batt = self.current_battery
                self.current_battery = data[battery_field]
                if old_batt is None or abs(self.current_battery - old_batt) >= 1.0:
                    self.get_logger().info(
                        f"[Battery] {topic_name}.{battery_field} updated: "
                        f"{old_batt} -> {self.current_battery}"
                    )

        self.handle_intervention_triggers(topic_name, data, cfg)
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
                for attr in dir(out_msg):
                    if not attr.startswith('_'):
                        try:
                            setattr(out_msg, attr, val)
                            break
                        except Exception:
                            continue

            publisher.publish(out_msg)
            self.get_logger().debug(
                f"[DynamicPublish] {topic_name} -> {val} ({pub_msg_cls.__name__})"
            )

        except Exception as e:
            self.get_logger().error(f"Dynamic publish failed for {topic_name}: {e}")

    def collision_nav_callback(self, topic_name, msg):
        now = self.get_clock().now()
        self.last_nav_cmd = msg
        self.last_nav_time = now

        self.system_snapshot['cmd_vel_nav'] = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
        }
        self.get_logger().debug(
            f"[Collision] NAV cmd: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, wz={msg.angular.z:.3f}"
        )

    def collision_output_callback(self, topic_name, msg):
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
            f"[Collision] COLLISION cmd: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, wz={msg.angular.z:.3f}"
        )

        self.check_collision_condition(now, coll_vx)

    def check_collision_condition(self, now, coll_vx_now: float):
        if self.last_nav_cmd is None or self.last_nav_time is None:
            collision_has_velocity_now = abs(coll_vx_now) > self.collision_zero_threshold
            self.collision_prev_has_velocity = collision_has_velocity_now
            return

        dt_nav = (now - self.last_nav_time).nanoseconds * 1e-9
        if dt_nav > self.collision_time_window:
            collision_has_velocity_now = abs(coll_vx_now) > self.collision_zero_threshold
            self.collision_prev_has_velocity = collision_has_velocity_now
            return

        nav_vx = float(self.last_nav_cmd.linear.x)

        if nav_vx <= self.collision_nav_threshold:
            collision_has_velocity_now = abs(coll_vx_now) > self.collision_zero_threshold
            self.collision_prev_has_velocity = collision_has_velocity_now
            return

        collision_has_velocity_now = abs(coll_vx_now) > self.collision_zero_threshold

        if self.collision_prev_has_velocity and not collision_has_velocity_now:
            self.collision_incidents += 1
            self.get_logger().info(
                f"[Collision] incident #{self.collision_incidents} "
                f"(nav_vx={nav_vx:.3f}, coll_vx_now={coll_vx_now:.3f})"
            )
            self.log_collision_event()

        self.collision_prev_has_velocity = collision_has_velocity_now

    # ----------------------------------------------------------------------
    # Timers
    # ----------------------------------------------------------------------
    def timer_callback(self):
        hb = Bool()
        hb.data = True
        self.heartbeat_publisher.publish(hb)

        now = self.get_clock().now()
        time_since_move = (now - self.last_odom_update_time).nanoseconds * 1e-9

        if time_since_move > self.stop_timeout and self.speed > 0.0:
            self.get_logger().info(
                f"[Timer] Robot has not moved for {time_since_move:.2f}s; speed -> 0"
            )
            self.speed = 0.0
            self.publish_speed(0.0)

    def db_metrics_tick(self):
        """
        Periodic DB watchdog:
          - Retries init_session if not yet established.
          - Saves the latest distance / mode metrics to all configured DBs.
          - Updates the latched health topic if anything changed.

        This is the billing-safety net: even if no events occur for a long
        time, the DB still gets fresh distance values at most
        ``db_metrics_period`` seconds out of date.
        """
        # 1) Retry init for any DB that hasn't been initialised yet
        self._try_init_sessions()

        # 2) Best-effort liveness ping (so health is fresh even in idle)
        if self.db_mgr_local is not None:
            if self.db_mgr_local.session_id is not None:
                # We already have a session; trust the periodic update below
                # to be the canonical health probe.
                pass
            else:
                # init_session will have set healthy=False already if it
                # failed; nothing else to do.
                pass

        # 3) Persist current metrics
        self.update_db_metrics()

    # ----------------------------------------------------------------------
    # DB writes
    # ----------------------------------------------------------------------
    def update_db_metrics(self):
        # MDBI = autonomous_distance / incidents (only auto->manual transitions
        # contribute to ``incidents``; collisions are tracked separately).
        if self.incidents != 0:
            mdbi_val = float(self.autonomous_distance) / float(self.incidents)
        else:
            mdbi_val = float(self.autonomous_distance)

        self.mdbi = mdbi_val

        self.get_logger().debug(
            f"[DB] save: dist={self.distance:.3f}, auto={self.autonomous_distance:.3f}, "
            f"manual={self.manual_distance:.3f}, incidents={self.incidents}, "
            f"collisions={self.collision_incidents}, mdbi={mdbi_val:.3f}"
        )

        for label, dbm in (('local', self.db_mgr_local), ('remote', self.db_mgr_remote)):
            if dbm is None:
                continue
            if dbm.session_id is None:
                # init_session hasn't succeeded yet (or is queued for retry).
                # Skip silently — health is already reflecting that.
                continue
            self._safe_db_call(label, dbm.update_distance, self.distance)
            self._safe_db_call(label, dbm.update_autonomous_distance, self.autonomous_distance)
            self._safe_db_call(label, dbm.update_manual_distance, self.manual_distance)
            self._safe_db_call(label, dbm.update_incidents, self.incidents)
            self._safe_db_call(label, dbm.update_mdbi, mdbi_val)
            self._safe_db_call(label, dbm.update_collision_incidents, self.collision_incidents)

    def log_event(self, msg='', details=None):
        if details is None:
            details = {}

        details = {
            **details,
            'metrics': self.get_metrics_snapshot(),
        }

        event_time = datetime.now(tz=timezone.utc)
        event = {'time': event_time, 'event_type': msg, 'details': details}

        self.get_logger().info(
            f"[Event] type={msg}, time={event_time.isoformat()}, "
            f"incidents={self.incidents}, collisions={self.collision_incidents}"
        )

        for label, dbm in (('local', self.db_mgr_local), ('remote', self.db_mgr_remote)):
            if dbm is None or dbm.session_id is None:
                continue
            self._safe_db_call(label, dbm.add_event, event)

        incidents_msg = Int32()
        incidents_msg.data = self.incidents
        self.incidents_publisher.publish(incidents_msg)

        # Push fresh metrics on every event so Mongo always reflects the
        # exact state at the moment the event happened.
        self.update_db_metrics()

    def log_collision_event(self):
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
                "angular": {"z": float(self.last_nav_cmd.angular.z)},
            },
            "collision_cmd": {
                "linear": {
                    "x": float(self.last_collision_cmd.linear.x),
                    "y": float(self.last_collision_cmd.linear.y),
                },
                "angular": {"z": float(self.last_collision_cmd.angular.z)},
            },
            "system_snapshot": self.system_snapshot.copy(),
        }

        self.get_logger().info(
            f"[Collision] Logging collision event #{self.collision_incidents} "
            f"(total_incidents={details['total_incidents']})"
        )

        self.log_event("Collision", details)

        msg = Int32()
        msg.data = int(self.collision_incidents)
        self.collision_incidents_publisher.publish(msg)

    # ----------------------------------------------------------------------
    # ROS publish helpers
    # ----------------------------------------------------------------------
    def publish_distance_topics(self):
        m = Float32(); m.data = float(self.distance)
        self.distance_publisher.publish(m)

        m = Float32(); m.data = float(self.autonomous_distance)
        self.autonomous_distance_publisher.publish(m)

        m = Float32(); m.data = float(self.manual_distance)
        self.manual_distance_publisher.publish(m)

    def publish_speed(self, speed):
        msg = Float32()
        msg.data = float(speed)
        self.speed_publisher.publish(msg)

    # ----------------------------------------------------------------------
    # Git info
    # ----------------------------------------------------------------------
    def get_git_info(self, repo_path):
        info = {
            "path": repo_path,
            "exists": False,
            "remote": None,
            "branch": None,
            "commit": None,
            "short_commit": None,
            "commit_message": None,
            "tags": [],
            "describe": None,
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

            try: info["remote"] = git(["config", "--get", "remote.origin.url"])
            except Exception: info["remote"] = None

            try: info["branch"] = git(["rev-parse", "--abbrev-ref", "HEAD"])
            except Exception: info["branch"] = None

            try: info["commit"] = git(["rev-parse", "HEAD"])
            except Exception: info["commit"] = None

            try: info["short_commit"] = git(["rev-parse", "--short", "HEAD"])
            except Exception: info["short_commit"] = None

            try: info["commit_message"] = git(["log", "-1", "--pretty=%s"])
            except Exception: info["commit_message"] = None

            try:
                tags_str = git(["tag", "--points-at", "HEAD"])
                info["tags"] = tags_str.splitlines() if tags_str else []
            except Exception:
                info["tags"] = []

            try: info["describe"] = git(["describe", "--tags", "--always"])
            except Exception: info["describe"] = None

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
    finally:
        node.get_logger().info(
            f"Final stats: distance={node.distance:.3f}, "
            f"auto_distance={node.autonomous_distance:.3f}, "
            f"manual_distance={node.manual_distance:.3f}, "
            f"incidents={node.incidents}, collisions={node.collision_incidents}"
        )
        # One last attempt to flush metrics so the DB has the most accurate
        # numbers possible at session end.
        try:
            node.update_db_metrics()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
