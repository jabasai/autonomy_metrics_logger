# Autonomy Metrics Logger

This repository now contains two ROS 2 Python packages under `src/`:

- `autonomy_metrics`: the rewritten metrics/logger node.
- `autonomy_metrics_webui`: a web dashboard backend + frontend for live metrics and MongoDB session history.

## What The Rewrite Tracks

The logger is now driven by the YAML config and implements the requested behavior directly:

- Autonomous/manual mode comes from `/autonomous_mode` (`std_msgs/msg/Bool`).
- An intervention is counted only when `/autonomous_mode` switches `true -> false` and at least one of these heartbeat topics is `false`:
  - `/autonomy_checks/heartbeat`
  - `/navigation_safe/heartbeat`
  - `/system_check/heartbeat`
- MDBI is computed from autonomous distance and counted interventions:
  - `mdbi = autonomous_distance / interventions`
  - if interventions are still `0`, MDBI is reported as undefined until the first intervention
- `/robot_state` (`std_msgs/msg/String`) is monitored for `Active -> Disabled` transitions while in autonomous mode.
- `/gophar_vehicle_controller/odometry` is used for:
  - total travelled distance
  - manual/autonomous travelled distance
  - current linear speed
  - average speed
- `/robot_navigation_area` is used for:
  - current area
  - travelled distance by area
  - total time by area
  - average speed by area
  - average time per visit by area
- `/plan` plus `/odometry/global` are used for:
  - current path deviation
  - average path deviation
  - maximum path deviation
- Git repository metadata is still captured at session start using the same configured repo list.
- A node heartbeat is published on `/autonomy_metrics/heartbeat`.
- A database snapshot is written every `n` seconds, default `10`, for topics marked `snapshot: true` in the YAML config.
- Writes still go to the local MongoDB and optionally to the remote/cloud MongoDB.

## Config File

The system behavior is declared in `src/autonomy_metrics/config/metrics_full.yaml`.

Top-level structure:

```yaml
git_repos_base_path: "/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario"

git_repos:
  scenario: "."
  autonomy_metrics_logger: "jabas/autonomy_metrics_logger"

runtime:
  heartbeat_topic: "/autonomy_metrics/heartbeat"
  session_summary_topic: "/autonomy_metrics/session_summary_json"
  heartbeat_period_sec: 1.0
  snapshot_interval_sec: 10.0
  distance_epsilon_m: 0.01
  stale_speed_timeout_sec: 2.0

topics:
  - name: "/autonomous_mode"
    type: "std_msgs/msg/Bool"
    role: "autonomous_mode"
    snapshot: true

  - name: "/autonomy_checks/heartbeat"
    type: "std_msgs/msg/Bool"
    role: "intervention_heartbeat"
    metric_key: "autonomy_checks"
    snapshot: true
```

### Supported Roles

- `autonomous_mode`: exactly one topic, must be a boolean mode source.
- `intervention_heartbeat`: one or more boolean health gates used when mode changes from autonomous to manual.
- `robot_state`: exactly one string topic with values such as `Disabled`, `Enabled`, `Active`.
- `distance_odometry`: exactly one odometry topic for travelled distance and speed.
- `navigation_area`: exactly one string topic for `INSIDE_POLYTUNNEL`, `OUTSIDE_POLYTUNNEL`, or `TRANSITION_INTO_POLYTUNNEL`.
- `global_path`: exactly one `nav_msgs/msg/Path` topic.
- `global_pose_odometry`: exactly one odometry topic used for path deviation against the path.

### Snapshot Logging

Set `snapshot: true` on any configured topic you want included in the periodic MongoDB snapshot. The snapshot stores:

- the current computed summary
- the latest value seen for each configured snapshot topic
- a timestamp

## MongoDB Layout

The rewritten logger stores data in:

- `sessions`: one document per run with session metadata, Git information, and the latest summary
- `session_events`: event documents such as interventions, mode transitions, robot-state transitions, and area transitions
- `session_snapshots`: periodic point-in-time snapshots of metrics and selected topics

## Published ROS Topics

- `/autonomy_metrics/heartbeat`
- `/autonomy_metrics/current_mode`
- `/autonomy_metrics/interventions`
- `/autonomy_metrics/mdbi`
- `/autonomy_metrics/active_to_disabled_in_autonomous`
- `/autonomy_metrics/total_distance_m`
- `/autonomy_metrics/autonomous_distance_m`
- `/autonomy_metrics/manual_distance_m`
- `/autonomy_metrics/current_speed_mps`
- `/autonomy_metrics/average_speed_mps`
- `/autonomy_metrics/autonomous_time_sec`
- `/autonomy_metrics/manual_time_sec`
- `/autonomy_metrics/current_navigation_area`
- `/autonomy_metrics/path_deviation/current_m`
- `/autonomy_metrics/path_deviation/average_m`
- `/autonomy_metrics/path_deviation/max_m`
- `/autonomy_metrics/session_summary_json`

## Launching

Build the two packages:

```bash
colcon build --packages-select autonomy_metrics autonomy_metrics_webui
```

Launch the logger and the dashboard together:

```bash
ros2 launch autonomy_metrics autonomy_metrics.launch.py \
  config_yaml:=/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario/jabas/autonomy_metrics_logger/src/autonomy_metrics/config/metrics_full.yaml \
  mongodb_host:=localhost \
  mongodb_port:=27018 \
  enable_remote_logging:=true \
  remote_mongodb_host:=YOUR_CLOUD_MONGO_HOST \
  remote_mongodb_port:=27017 \
  use_webui:=true \
  ui_host:=0.0.0.0 \
  ui_port:=8081
```

If you only want the dashboard package:

```bash
ros2 launch autonomy_metrics_webui autonomy_metrics_webui.launch.py \
  mongodb_host:=localhost \
  mongodb_port:=27018 \
  ui_port:=8081
```

## Web UI

The web UI serves:

- live summary from `/autonomy_metrics/session_summary_json`
- heartbeat freshness from `/autonomy_metrics/heartbeat`
- recent events and snapshots from MongoDB

Default URL:

```text
http://0.0.0.0:8081
```

Open it from a browser on the host machine using the machine IP or loopback address.

## Notes On MDBI

The previous package also used the standard autonomous-distance-per-intervention formula, but its intervention counting relied on several unrelated triggers. The rewrite keeps the core metric definition and narrows the intervention rule to your requested standard:

- only count autonomous to manual transitions
- only count them when at least one configured heartbeat gate is false

That makes MDBI directly reflect the operational quality of autonomous runs instead of mixing in joystick or collision heuristics.
