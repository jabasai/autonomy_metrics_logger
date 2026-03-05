# Autonomy Metrics Logger

`autonomy_metrics_logger` is a ROS 2 node that:

- Tracks travelled distance, autonomous distance, incidents, and collisions.
- Logs interventions and collisions to MongoDB with a full snapshot of system state.
- Is configured entirely via a YAML file (topics, fields, triggers, dynamic publishers).
- Supports both explicit operation mode (`/gophar/operation_mode`) and inferred mode (odom + velocity).
- Tracks navigation area (in-row / headland / row-entry) with per-area average speed.
- Derives a high-level robot status (In-row, Headland, Row-entry, Paused due to obstacle, Held by picker).
- Computes cross-track deviation from the global path.
- Periodically logs a full system snapshot to MongoDB (`timer_event`, not counted as MDBI incident).

---

## 1. High-Level Behaviour

The node:

1. Loads a YAML config describing topics and logging behaviour.
2. Subscribes to all configured topics.
3. Maintains a live `system_snapshot` dict with the latest values from each topic.
4. Updates metrics:
   - `distance` (total odometry distance)
   - `autonomous_distance` (distance when mode = Autonomous)
   - `incidents` (interventions in autonomous mode, plus manual overrides)
   - `collision_incidents` (collision monitor only, separate from incidents)
5. Logs events to MongoDB **only** when something happens:
   - Manual override (explicit or observed)
   - E-stop
   - Faults / joy overrides / other YAML triggers
   - Collisions (nav vs collision velocity)
6. Periodically logs a `timer_event` (default every 5 s) with the full system snapshot to MongoDB. These events are **not** counted as incidents and do **not** affect MDBI.
7. Each event stored in Mongo includes:
   - `metrics` (distance, autonomous_distance, speed, battery)
   - `system_snapshot` (current values of all configured topics)
   - Event type & extra details.

MDBI is computed as:

> `mdbi = autonomous_distance / incidents`  
> (If `incidents == 0`, `mdbi = autonomous_distance`.)

---

## 2. MongoDB Integration

The node creates a document in `robot_incidents.sessions` per run:

```json
{
  "session_start_time": "...",
  "robot_name": "...",
  "farm_name": "...",
  "field_name": "...",
  "application": "...",
  "scenario_name": "...",
  "aoc_repos_info": [
    {
      "aoc_scenario_path": {
        "path": "...",
        "exists": true,
        "remote": "...",
        "branch": "...",
        "commit": "...",
        "short_commit": "...",
        "commit_message": "...",
        "tags": ["v0.1.0"],
        "describe": "v0.1.0-3-gXXXXXXX",
        "dirty": false,
        "error": null
      }
    }
  ],
  "mdbi": 123.4,
  "incidents": 5,
  "distance": 456.7,
  "autonomous_distance": 321.0,
  "collision_incidents": 2,
  "events": [
    {
      "time": "...",
      "event_type": "Manual_override",
      "details": {
        "operation_mode": "Manual",
        "estop": false,
        "metrics": {
          "distance": 12.3,
          "autonomous_distance": 12.3,
          "speed": 0.4,
          "battery_percentage": 78.0
        },
        "system_snapshot": {
          "/gophar/system_status": {
            "fault_shutdown": false,
            "hv_on": true,
            "battery_percentage": 78.0,
            "...": "..."
          },
          "/gps_base/fix": { "...": "..." },
          "odometry": { "x": 1.23, "y": 4.56, "vx": 0.4 },
          "...": "..."
        }
      }
    }
  ]
}
````

Environment variables used to tag the session:

* `ROBOT_NAME`
* `FARM_NAME`
* `FIELD_NAME`
* `APPLICATION`
* `SCENARIO_NAME`

---

## 3. Node Parameters

All parameters are exposed through the launch file.

### Core configuration

| Parameter                | Type   | Default       | Description                                            |
| ------------------------ | ------ | ------------- | ------------------------------------------------------ |
| `config_yaml`            | string | `""`          | Path to YAML config file for topics & triggers.        |
| `mongodb_host`           | string | `"localhost"` | Local MongoDB host.                                    |
| `mongodb_port`           | int    | `27017`       | Local MongoDB port.                                    |
| `remote_mongodb_host`    | string | `""`          | Remote MongoDB host (empty = disabled).                |
| `remote_mongodb_port`    | int    | `27017`       | Remote MongoDB port.                                   |
| `enable_remote_logging`  | bool   | `false`       | Enable writes to remote MongoDB.                       |
| `min_distance_threshold` | double | `0.2`         | Min odom step (m) required for distance update.        |
| `stop_timeout`           | double | `2.0`         | Time (s) after last odom update before speed is set 0. |
| `snapshot_timer_interval` | double | `5.0`        | Interval (s) between periodic `timer_event` snapshots logged to MongoDB. |

### Mode observer (fallback mode inference)

Used **only** if no topic with `role: "control_mode"` is configured.

| Parameter                       | Type   | Default | Description                                                              |
| ------------------------------- | ------ | ------- | ------------------------------------------------------------------------ |
| `mode_observer_cmd_timeout`     | double | `1.0`   | Commands on `mode_observer` topic are “recent” within this many seconds. |
| `mode_observer_speed_threshold` | double | `0.01`  | Robot is considered moving if speed > this (m/s).                        |

Logic when active:

* Topic with `role: "mode_observer"` (e.g. `/cmd_vel/collision_smoothed`) marks last command time.
* If odometry reports movement and:

  * cmd is recent → mode = Autonomous
  * cmd is not recent → mode = Manual
* When observed mode changes from Auto → Manual:

  * `Manual_override_observed` event is logged.
  * An incident is counted (MDBI).

### Collision monitor (nav vs collision velocity)

| Parameter                  | Type   | Default | Description                                                                                   |
| -------------------------- | ------ | ------- | --------------------------------------------------------------------------------------------- |
| `collision_nav_threshold`  | double | `0.01`  | `/cmd_vel/nav.linear.x` must be > this to count as a forward command.                         |
| `collision_zero_threshold` | double | `0.001` | `/cmd_vel/collision.linear.x` considered zero if abs(x) ≤ this.                               |
| `collision_moving_threshold` | double | `0.03` | `/cmd_vel/collision.linear.x` considered moving if abs(x) ≥ this (hysteresis upper bound).    |
| `collision_zero_required_count` | int | `2` | Require this many consecutive near-zero collision samples before counting a collision.         |
| `collision_time_window`    | double | `0.5`   | Nav and collision commands must both be within this time window (s).                          |
| `collision_log_cooldown`   | double | `1.0`   | Minimum time between logged collision events.                                                  |
| `intervention_event_cooldown` | double | `1.0` | Default cooldown for YAML `intervention_on_message` triggers.                                 |

Collision detection (robust falling-edge):

* Keep last `/cmd_vel/nav` and `/cmd_vel/collision`.
* Compute a collision **only when**:

  * Current mode is `Autonomous`,
  * Previous collision stream state was moving (`>= collision_moving_threshold`),
  * Current collision stream is near-zero (`<= collision_zero_threshold`) for `collision_zero_required_count` consecutive samples,
  * Latest `/cmd_vel/nav.linear.x` > `collision_nav_threshold`,
  * Commands are recent (`≤ collision_time_window`).
* Additional safeguard:

  * Events are rate-limited by `collision_log_cooldown`.
* On collision:

  * `collision_incidents` is incremented,
  * A `Collision` event is logged with snapshot,
  * Total collisions published on `mdbi_logger/total_collision_incidents` (Int32).

---

## 4. YAML Config File

### Top-level structure

```yaml
git_repos:
  aoc_scenario_path: "/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario"

topics:
  - name: "/gps_base/fix"
    type: "sensor_msgs/msg/NavSatFix"
    log_fields:
      - "latitude"
      - "longitude"
      - "altitude"
    publish:
      enable: true
      topic: "mdbi_logger/gps_alt"
      type: "std_msgs/msg/Float32"
      field: "altitude"

  - name: "/gophar_vehicle_controller/odometry"
    type: "nav_msgs/msg/Odometry"
    role: "odometry"
    log_fields:
      - "pose.pose.position.x"
      - "pose.pose.position.y"
      - "twist.twist.linear.x"
    publish:
      enable: false

  - name: "/gophar/system_status"
    type: "dynium_gophar_interfaces/msg/SystemStatus"
    role: "system_status"
    log_all_fields: false
    log_fields:
      - "fault_shutdown"
      - "hv_on"
      - "ignition_switch"
      - "charger_interlock"
      - "battery_percentage"
    battery_field: "battery_percentage"
    publish:
      enable: true
      topic: "mdbi_logger/battery_level"
      type: "std_msgs/msg/Float32"
      field: "battery_percentage"
    intervention_on_change:
      fault_shutdown:
        trigger_value: true
        event_type: "Fault_shutdown"

  - name: "/gophar/operation_mode"
    type: "std_msgs/msg/String"
    role: "control_mode"
    mode_field: "data"
    mode_mapping:
      "AUTO": "Autonomous"
      "MANUAL": "Manual"
      "1": "Autonomous"
      "0": "Manual"

  - name: "/cmd_vel/joy"
    type: "geometry_msgs/msg/Twist"
    log_fields: []
    intervention_on_message:
      enable: true
      event_type: "Joy_override"
      require_autonomous_mode: true
      on_rising_edge: true
      cooldown_sec: 1.0
      activity_field: "linear.x"
      min_abs_value: 0.02

  - name: "/cmd_vel/collision_smoothed"
    type: "geometry_msgs/msg/Twist"
    role: "mode_observer"
    log_fields: []

  - name: "/cmd_vel/nav"
    type: "geometry_msgs/msg/Twist"
    role: "collision_nav"
    log_fields: []

  - name: "/cmd_vel/collision"
    type: "geometry_msgs/msg/Twist"
    role: "collision_output"
    log_fields: []

  - name: "/gophar/steering_actuator/front"
    type: "dynium_gophar_interfaces/msg/SteeringActuatorStatus"
    log_all_fields: true
    log_fields: []

  - name: "/gophar/steering_actuator/rear"
    type: "dynium_gophar_interfaces/msg/SteeringActuatorStatus"
    log_all_fields: true
    log_fields: []

  - name: "/gophar/motor_status/motor_0"
    type: "dynium_gophar_interfaces/msg/MotorStatus"
    log_all_fields: true
    log_fields: []

  - name: "/gophar/motor_status/motor_1"
    type: "dynium_gophar_interfaces/msg/MotorStatus"
    log_all_fields: true
    log_fields: []

  # Navigation area, execution UI, and global path
  - name: "/robot_navigation_area"
    type: "std_msgs/msg/String"
    role: "navigation_area"
    log_fields:
      - "data"

  - name: "/roboflow/execution_ui"
    type: "std_msgs/msg/String"
    role: "execution_ui"
    log_fields:
      - "data"

  - name: "/plan"
    type: "nav_msgs/msg/Path"
    role: "global_path"
    log_fields: []
```

### Per-topic keys

Common fields:

* `name`
  ROS topic name to subscribe to.

* `type`
  ROS message type string. Accepts:

  * `pkg/msg/MessageName` (e.g. `"sensor_msgs/msg/NavSatFix"`), or
  * `pkg/MessageName` (e.g. `"sensor_msgs/NavSatFix"`).

* `role` (optional, lowercase in logic)
  Special roles:

  * `"odometry"`: used for distance/speed computation.
  * `"control_mode"`: explicit operation mode (`Autonomous` vs `Manual`).
  * `"estop"`: emergency stop; used to generate `EMS` incidents.
  * `"system_status"`: can carry `battery_field` and fault triggers.
  * `"mode_observer"`: used by mode observer (e.g. `/cmd_vel/collision_smoothed`).
  * `"collision_nav"`: nav velocity command.
  * `"collision_output"`: collision-limited velocity command.
  * `"navigation_area"`: navigation area enum (e.g. `/robot_navigation_area`).
  * `"execution_ui"`: execution UI state (`idle` / `paused`, e.g. `/roboflow/execution_ui`).
  * `"global_path"`: global planner path (`nav_msgs/msg/Path`) used for path-deviation error.

* `log_fields` (list of dotted paths)
  Fields to extract into `system_snapshot[topic_name]`.
  Only used if `log_all_fields` is `false`.

* `log_all_fields` (bool, default: false)
  If `true`, the full message is converted to a dict and stored in `system_snapshot[topic_name]`.
  Ignores `log_fields`.

* `battery_field` (optional, string)
  Field name under this topic that carries battery percentage; updates `current_battery`.

#### Dynamic republishing

```yaml
publish:
  enable: true
  topic: "mdbi_logger/battery_level"
  type: "std_msgs/msg/Float32"
  field: "battery_percentage"
```

* `enable`: `true`/`false`
* `topic`: output topic name
* `type`: output message type
* `field`: dotted field in input message; mapped to `.data` of the output (with type casting).

#### Control mode

For topics with `role: "control_mode"`:

* `mode_field` (string)
  Dotted field path inside message holding the raw mode value (e.g. `"data"`).

* `mode_mapping` (dict)
  Map raw values to `"Autonomous"` or `"Manual"`
  Example: `"AUTO" → "Autonomous"`, `"0" → "Manual"`.
  If unmapped and `str(value) == "3"`, forced to Manual; otherwise treated as Autonomous.

Changing from `Autonomous` to `Manual`:

* Ends an autonomy segment (time tracking).
* Logs `Manual_override` event.
* Increments `incidents` (MDBI) even if now in Manual (forced by `force_count=True`).

#### E-stop role

For `role: "estop"` topics, additional config:

* `field` (optional, default `"data"`)
  Boolean field to check.
  When it changes and becomes `True`:

  * `EMS` intervention is logged.
  * Incidents increment only if mode = Autonomous.

#### YAML-driven interventions

Per-topic:

* `intervention_on_message`:

  ```yaml
  intervention_on_message:
    enable: true
    event_type: "Joy_override"
    require_autonomous_mode: true
    on_rising_edge: true
    cooldown_sec: 1.0
    activity_field: "linear.x"
    min_abs_value: 0.02
  ```

  If enabled, events are emitted only when trigger conditions pass:

  * `require_autonomous_mode` (default `true`): skip in Manual mode.
  * `on_rising_edge` (default `true`): trigger only on inactive→active transition.
  * `cooldown_sec` (default from node param `intervention_event_cooldown`): per-trigger rate limit.
  * `activity_field` (default `linear.x`): field used to determine activity.
  * `min_abs_value`: minimum absolute value for numeric activity fields.

  This avoids noisy per-message logs and false overrides from zero/idle commands.

* `intervention_on_change`:

  ```yaml
  intervention_on_change:
    fault_shutdown:
      trigger_value: true
      event_type: "Fault_shutdown"
  ```

  * Key (`fault_shutdown`) is a field name in `data`.
  * `trigger_value` optional; if present, event fires only when field changes **and** equals `trigger_value`.
  * `event_type` is the event name stored in DB.

---

## 5. Navigation Area, Status & Path Deviation

These features are implemented in two modular tracker classes that are instantiated by the main node.

### Navigation area tracking (`NavigationAreaTracker`)

Subscribes (via `role: "navigation_area"`) to a topic publishing `std_msgs/msg/String` with one of:

* `INSIDE_POLYTUNNEL`
* `OUTSIDE_POLYTUNNEL`
* `TRANSITION_INTO_POLYTUNNEL`

Also monitors the execution UI topic (`role: "execution_ui"`) for values `idle` or `paused`.

#### Published topics

| Topic                                       | Type    | Description                                              |
| ------------------------------------------- | ------- | -------------------------------------------------------- |
| `mdbi_logger/avg_speed_inside_polytunnel`   | Float32 | Running average speed while inside a polytunnel.         |
| `mdbi_logger/avg_speed_outside_polytunnel`  | Float32 | Running average speed while outside a polytunnel.        |
| `mdbi_logger/time_in_current_mode`          | Float32 | Seconds since the last area transition (resets on change).|
| `mdbi_logger/robot_status`                  | String  | Derived high-level status (see below).                   |
| `mdbi_logger/navigation_area`               | String  | Current navigation area string.                          |

#### Robot status derivation (priority order)

| Priority | Status                  | Condition                                                     |
| -------- | ----------------------- | ------------------------------------------------------------- |
| 1        | **Held by picker**      | `/roboflow/execution_ui` is `"paused"`                       |
| 2        | **Paused due to obstacle** | Autonomous mode, speed ≈ 0, area is known                  |
| 3        | **Row-entry**           | Area = `TRANSITION_INTO_POLYTUNNEL`                           |
| 4        | **In-row**              | Area = `INSIDE_POLYTUNNEL`                                    |
| 5        | **Headland**            | Area = `OUTSIDE_POLYTUNNEL`                                   |
| 6        | **Unknown**             | No area data received yet                                     |

### Path deviation tracking (`PathDeviationTracker`)

Subscribes (via `role: "global_path"`) to a `nav_msgs/msg/Path` topic (default `/plan`).

On every odometry update the tracker computes the minimum perpendicular distance from the robot's current position to the closest segment of the received global path.

| Topic                             | Type    | Description                                |
| --------------------------------- | ------- | ------------------------------------------ |
| `mdbi_logger/path_deviation_error`| Float32 | Cross-track error to the global path (m).  |

### Periodic snapshot (`timer_event`)

A timer (configurable via `snapshot_timer_interval`, default 5 s) fires periodically and logs a `timer_event` to MongoDB. This event captures:

* Full `system_snapshot` (latest values from all subscribed topics)
* Current metrics (distance, autonomous distance, speed, battery)
* Navigation area tracker snapshot (area, status, per-area avg speeds, time-in-mode)
* Path deviation tracker snapshot (error, path length)
* Current `operation_mode` and `estop` state

**`timer_event` does NOT increment `incidents` and does NOT affect MDBI.**

---

## 6. Mode Handling and Incidents

* Default mode at startup is set to `Autonomous`:

  ```python
  self.AUTO = 'Autonomous'
  self.MAN = 'Manual'
  self.details = {'estop': False, 'operation_mode': self.AUTO}
  ```

* `incidents` are only incremented when:

  * `trigger_intervention(...)` is called and:

    * `operation_mode == "Autonomous"`, or
    * `force_count=True` (for explicit/observed manual overrides).

* E-stop, faults, joy overrides, etc. in **Manual** mode:

  * Are suppressed by default for intervention triggers,
  * Can still be explicitly logged for critical events (e.g. E-stop),
  * Do **not** increment `incidents`, thus do not affect MDBI.

### Recommended anti-noise presets

Simulation (cleaner signals):

* `mode_observer_speed_threshold: 0.01`
* `collision_nav_threshold: 0.01`
* `collision_zero_threshold: 0.001`
* `collision_moving_threshold: 0.02`
* `collision_zero_required_count: 2`
* `collision_log_cooldown: 0.6`
* `intervention_event_cooldown: 0.8`

Real robot (noisier signals):

* `mode_observer_speed_threshold: 0.02`
* `collision_nav_threshold: 0.03`
* `collision_zero_threshold: 0.01`
* `collision_moving_threshold: 0.05`
* `collision_zero_required_count: 3`
* `collision_log_cooldown: 1.5`
* `intervention_event_cooldown: 1.5`

---

## 7. Launching the Node

Example launch file (simplified; adapt package/executable names):

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_yaml_arg = DeclareLaunchArgument(
        'config_yaml',
        default_value='',
        description='Path to AutonomyMetricsLogger YAML config',
    )

    # ... declare other args (mongodb_host, ports, thresholds, etc.) ...

    config_yaml = LaunchConfiguration('config_yaml')

    metrics_logger_node = Node(
        package='autonomy_metrics',      # adjust
        executable='metric_logger',      # adjust
        name='mdbi_logger_dynamic',
        output='screen',
        parameters=[{
            'config_yaml': config_yaml,
            'mongodb_host': LaunchConfiguration('mongodb_host'),
            'mongodb_port': LaunchConfiguration('mongodb_port'),
            'remote_mongodb_host': LaunchConfiguration('remote_mongodb_host'),
            'remote_mongodb_port': LaunchConfiguration('remote_mongodb_port'),
            'enable_remote_logging': LaunchConfiguration('enable_remote_logging'),
            'min_distance_threshold': LaunchConfiguration('min_distance_threshold'),
            'stop_timeout': LaunchConfiguration('stop_timeout'),
            'mode_observer_cmd_timeout': LaunchConfiguration('mode_observer_cmd_timeout'),
            'mode_observer_speed_threshold': LaunchConfiguration('mode_observer_speed_threshold'),
            'collision_nav_threshold': LaunchConfiguration('collision_nav_threshold'),
            'collision_zero_threshold': LaunchConfiguration('collision_zero_threshold'),
            'collision_moving_threshold': LaunchConfiguration('collision_moving_threshold'),
            'collision_zero_required_count': LaunchConfiguration('collision_zero_required_count'),
            'collision_time_window': LaunchConfiguration('collision_time_window'),
            'collision_log_cooldown': LaunchConfiguration('collision_log_cooldown'),
            'intervention_event_cooldown': LaunchConfiguration('intervention_event_cooldown'),
            'snapshot_timer_interval': LaunchConfiguration('snapshot_timer_interval'),
        }],
    )

    return LaunchDescription([
        config_yaml_arg,
        # ... other DeclareLaunchArgument ...
        metrics_logger_node,
    ])
```

### Example launch usage

Minimal:

```bash
ros2 launch autonomy_metrics_logger autonomy_metrics_logger.launch.py \
  config_yaml:=/home/ros/aoc_strawberry_scenario_ws/src/aoc_strawberry_scenario/jabas/autonomy_metrics_logger/config/metrics_gophar.yaml
```

With remote DB + tuned thresholds:

```bash
ros2 launch autonomy_metrics_logger autonomy_metrics_logger.launch.py \
  config_yaml:=/path/to/metrics_gophar.yaml \
  mongodb_host:=localhost \
  mongodb_port:=27017 \
  enable_remote_logging:=true \
  remote_mongodb_host:=10.0.0.42 \
  remote_mongodb_port:=27017 \
  min_distance_threshold:=0.1 \
  stop_timeout:=1.5 \
  mode_observer_cmd_timeout:=0.8 \
  mode_observer_speed_threshold:=0.02 \
  collision_nav_threshold:=0.02 \
  collision_zero_threshold:=0.001 \
  collision_time_window:=0.4
```

---

## 8. Testing Tips

* Verify topics:

  ```bash
  ros2 topic list
  ros2 topic info /gophar_vehicle_controller/odometry
  ros2 topic info /gophar/system_status
  ```

* Trigger joystick interventions:

  ```bash
  ros2 topic pub --once /cmd_vel/joy geometry_msgs/msg/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```

* Publish a navigation area:

  ```bash
  ros2 topic pub --once /robot_navigation_area std_msgs/msg/String "{data: 'INSIDE_POLYTUNNEL'}"
  ```

* Simulate picker hold:

  ```bash
  ros2 topic pub --once /roboflow/execution_ui std_msgs/msg/String "{data: 'paused'}"
  ```

* Verify derived topics:

  ```bash
  ros2 topic echo /mdbi_logger/robot_status
  ros2 topic echo /mdbi_logger/avg_speed_inside_polytunnel
  ros2 topic echo /mdbi_logger/path_deviation_error
  ```

* Check DB content (Mongo shell or Mongo client) in `robot_incidents.sessions`.
  Look for events with `event_type: "timer_event"` for periodic snapshots.

---

## 9. Architecture Overview

The node is structured with modular tracker classes to keep concerns separated:

```
AutonomyMetricsLogger (metric_logger.py)
├── NavigationAreaTracker (navigation_area_tracker.py)
│   ├── Area tracking (INSIDE / OUTSIDE / TRANSITION)
│   ├── Per-area average speed
│   ├── Time-in-current-mode
│   └── Robot status derivation
├── PathDeviationTracker (path_deviation_tracker.py)
│   ├── Global path storage
│   └── Cross-track error computation
├── DatabaseMgr (db_mgr.py)
│   └── MongoDB read/write
└── Periodic snapshot timer (timer_event → MongoDB)
```

The README covers:

* All node parameters,
* YAML configuration options,
* How incidents, MDBI, collisions, and snapshots are computed,
* How to launch and test the system.
