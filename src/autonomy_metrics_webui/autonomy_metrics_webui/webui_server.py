#!/usr/bin/env python3
"""ROS 2 + Flask backend for the autonomy metrics dashboard."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import logging
from pathlib import Path
import threading

from flask import Flask, jsonify, render_template, request
from werkzeug.serving import make_server

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, String

from autonomy_metrics.db_mgr import DatabaseMgr


class _ServerThread(threading.Thread):
    def __init__(self, host: str, port: int, app: Flask):
        super().__init__(daemon=True)
        self._server = make_server(host, port, app, threaded=True)

    def run(self):
        self._server.serve_forever()

    def shutdown(self):
        self._server.shutdown()


class AutonomyMetricsWebUiServer(Node):
    """Serve a live dashboard backed by ROS summary topics and MongoDB history."""

    def __init__(self):
        super().__init__("autonomy_metrics_webui_server")

        self.declare_parameter("ui_host", "0.0.0.0")
        self.declare_parameter("ui_port", 8081)
        self.declare_parameter("ui_access_log", False)
        self.declare_parameter("mongodb_host", "localhost")
        self.declare_parameter("mongodb_port", 27018)
        self.declare_parameter("remote_mongodb_host", "")
        self.declare_parameter("remote_mongodb_port", 27017)
        self.declare_parameter("prefer_remote_data", False)
        self.declare_parameter("database_name", "robot_incidents")
        self.declare_parameter("session_summary_topic", "/autonomy_metrics/session_summary_json")
        self.declare_parameter("heartbeat_topic", "/autonomy_metrics/heartbeat")

        self._host = self.get_parameter("ui_host").value
        self._port = int(self.get_parameter("ui_port").value)
        self._ui_access_log = bool(self.get_parameter("ui_access_log").value)
        self._summary_topic = self.get_parameter("session_summary_topic").value
        self._heartbeat_topic = self.get_parameter("heartbeat_topic").value
        self._database_name = self.get_parameter("database_name").value

        self._latest_summary = {}
        self._summary_received_at = None
        self._heartbeat_seen = False
        self._heartbeat_received_at = None
        self._lock = threading.Lock()

        self._db_reader = self._create_db_reader(
            prefer_remote=bool(self.get_parameter("prefer_remote_data").value)
        )

        self.create_subscription(String, self._summary_topic, self._summary_callback, 10)
        self.create_subscription(Bool, self._heartbeat_topic, self._heartbeat_callback, 10)

        template_folder = (
            Path(get_package_share_directory("autonomy_metrics_webui")) / "templates"
        )
        static_folder = Path(get_package_share_directory("autonomy_metrics_webui")) / "static"
        self._app = Flask(
            __name__,
            template_folder=str(template_folder),
            static_folder=str(static_folder),
        )
        self._configure_routes()

        if not self._ui_access_log:
            logging.getLogger("werkzeug").setLevel(logging.ERROR)

        self._server_thread = _ServerThread(self._host, self._port, self._app)
        self._server_thread.start()
        self.get_logger().info(
            f"Autonomy metrics dashboard available at http://{self._host}:{self._port}"
        )

    def _create_db_reader(self, prefer_remote: bool):
        ordered_targets = []
        if prefer_remote:
            ordered_targets.extend(
                [
                    (
                        self.get_parameter("remote_mongodb_host").value,
                        int(self.get_parameter("remote_mongodb_port").value),
                    ),
                    (
                        self.get_parameter("mongodb_host").value,
                        int(self.get_parameter("mongodb_port").value),
                    ),
                ]
            )
        else:
            ordered_targets.extend(
                [
                    (
                        self.get_parameter("mongodb_host").value,
                        int(self.get_parameter("mongodb_port").value),
                    ),
                    (
                        self.get_parameter("remote_mongodb_host").value,
                        int(self.get_parameter("remote_mongodb_port").value),
                    ),
                ]
            )

        for host, port in ordered_targets:
            if not host:
                continue
            try:
                reader = DatabaseMgr(
                    database_name=self._database_name,
                    host=host,
                    port=port,
                )
                reader.client.admin.command("ping")
                self.get_logger().info(f"Web UI using MongoDB reader {host}:{port}")
                return reader
            except Exception as exc:
                self.get_logger().warn(f"MongoDB reader unavailable at {host}:{port}: {exc}")
        return None

    def _configure_routes(self):
        @self._app.get("/")
        def index():
            return render_template("index.html")

        @self._app.get("/api/health")
        def api_health():
            return jsonify(
                {
                    "ok": True,
                    "server_time": self._utc_now_iso(),
                    "summary_seen": self._summary_received_at is not None,
                    "heartbeat_seen": self._heartbeat_seen,
                }
            )

        @self._app.get("/api/live")
        def api_live():
            with self._lock:
                summary = dict(self._latest_summary)
                summary_received_at = self._summary_received_at
                heartbeat_seen = self._heartbeat_seen
                heartbeat_received_at = self._heartbeat_received_at

            if not summary and self._db_reader is not None:
                latest_session = self._db_reader.fetch_latest_session()
                if latest_session:
                    summary = latest_session.get("summary", {})

            return jsonify(
                {
                    "server_time": self._utc_now_iso(),
                    "summary_received_at": summary_received_at,
                    "summary": summary,
                    "heartbeat": {
                        "seen": heartbeat_seen,
                        "received_at": heartbeat_received_at,
                        "age_sec": self._age_seconds(heartbeat_received_at),
                    },
                }
            )

        @self._app.get("/api/history")
        def api_history():
            if self._db_reader is None:
                return jsonify(
                    {
                        "session": None,
                        "events": [],
                        "snapshots": [],
                        "error": "MongoDB reader is unavailable",
                    }
                )

            event_limit = max(1, min(100, int(request.args.get("events", 25))))
            snapshot_limit = max(1, min(100, int(request.args.get("snapshots", 10))))

            latest_session = self._db_reader.fetch_latest_session()
            session_id = latest_session["_id"] if latest_session else None

            events = self._db_reader.fetch_recent_events(
                limit=event_limit,
                session_id=session_id,
            )
            snapshots = self._db_reader.fetch_recent_snapshots(
                limit=snapshot_limit,
                session_id=session_id,
            )
            return jsonify(
                {
                    "session": latest_session,
                    "events": events,
                    "snapshots": snapshots,
                    "error": None,
                }
            )

    def _summary_callback(self, msg: String):
        try:
            summary = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Failed to parse summary JSON: {exc}")
            return

        with self._lock:
            self._latest_summary = summary
            self._summary_received_at = self._utc_now_iso()

    def _heartbeat_callback(self, msg: Bool):
        with self._lock:
            self._heartbeat_seen = bool(msg.data)
            self._heartbeat_received_at = self._utc_now_iso()

    def destroy_node(self):
        if hasattr(self, "_server_thread"):
            self._server_thread.shutdown()
            self._server_thread.join(timeout=2.0)
        return super().destroy_node()

    def _age_seconds(self, timestamp: str | None):
        if not timestamp:
            return None
        try:
            seen_at = datetime.fromisoformat(timestamp)
        except ValueError:
            return None
        return max(
            0.0,
            (datetime.now(tz=timezone.utc) - seen_at.astimezone(timezone.utc)).total_seconds(),
        )

    def _utc_now_iso(self):
        return datetime.now(tz=timezone.utc).isoformat()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyMetricsWebUiServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
