# Autonomy Metrics Logger

`autonomy_metrics_logger` is a ROS 2 node that:

- Tracks travelled distance, autonomous distance, manual distance, incidents, and collisions.
- Logs interventions, mode changes, and collisions to MongoDB with a full snapshot of system state.
- Is configured entirely via a YAML file (topics, fields, triggers, dynamic publishers).
- Determines operation mode (`Autonomous` / `Manual`) from a `std_msgs/Bool` topic such as `/autonomous_mode`.
- Tracks robot lifecycle state ("disabled" / "enabled" / "active") from a `std_msgs/String` topic such as `/robot_state`.
- Is **resilient to MongoDB outages**: the node never crashes if Mongo is down, it retries connecting in the background, and it publishes its DB health on a latched topic. The travelled distance is the source of truth for billing, so accuracy and reliability are first-class concerns.

---

## 1. High-Level Behaviour

The node:

1. Loads a YAML config describing topics and logging behaviour.
2. Subscribes to all configured topics. State-style topics (`autonomous_mode`, `robot_state`, `control_mode`, `estop`) use **RELIABLE** QoS so a missed message can never mis-attribute distance.
3. Maintains a live `system_snapshot` dict with the latest values from each topic.
4. Updates metrics on every odom step:
   - `distance` — total odometry distance (`autonomous_distance + manual_distance` by construction)
   - `autonomous_distance` — distance accumulated while `/autonomous_mode == True`
   - `manual_distance` — distance accumulated while `/autonomous_mode == False`
   - `incidents` — count of Auto → Manual transitions (the only thing MDBI is divided by)
   - `collision_incidents` — collision monitor only, separate from incidents
5. Periodically (`db_metrics_period`, default 1 s) writes the latest counters to MongoDB. Worst-case loss on crash is `db_metrics_period` seconds of travel, plus any uncommitted partial odom step (≤ `min_distance_threshold`).
6. Logs events to MongoDB **immediately** when they happen:
   - `Manual_override` — `/autonomous_mode` Auto → Manual (counts toward MDBI)
   - `Autonomous_resumed` — `/autonomous_mode` Manual → Auto (does NOT count)
   - `Robot_state_changed` — every change of `/robot_state` (does NOT count)
   - `EMS` — e-stop edges
   - `Fault_shutdown` / `Joy_override` / other YAML-configured triggers
   - `Collision` — falling-edge in `/cmd_vel/collision`
7. Each event stored in Mongo includes:
   - `metrics` (distance, autonomous_distance, manual_distance, speed, battery)
   - `system_snapshot` (current values of all configured topics)
   - Event type & extra details.

MDBI is computed as:

> `mdbi = autonomous_distance / incidents`
> (If `incidents == 0`, `mdbi = autonomous_distance`.)

`incidents` here counts **only** Auto → Manual transitions. Collisions, robot-state changes, and other events are logged but do not feed MDBI.

---

## 2. MongoDB Integration

### Resilience contract

- Local Mongo down at startup: the node still starts. It publishes `db_health=false` and retries `init_session` on every periodic tick.
- Local Mongo goes down mid-session: writes start failing silently (one warn per failure), `db_health` flips to `false`, and `db_health_reason` carries a human-readable error string. When Mongo comes back, the node resumes writing automatically and `db_health` flips back to `true`.
- Remote Mongo: only attempted when `enable_remote_logging=true` AND `remote_mongodb_host` is set. Failures there never affect local logging or the node's lifetime.

### Health topics (latched, transient_local, depth=1)

- `mdbi_logger/db_health` (`std_msgs/Bool`) — `true` when all enabled DBs are reachable, `false` otherwise.
- `mdbi_logger/db_health_reason` (`std_msgs/String`) — `"ok"` when healthy, otherwise a human-readable description like `"local: ServerSelectionTimeoutError: ..."`.

Subscribers that join late always receive the latest value.

### Session document

The node creates one document in `robot_incidents.sessions` per run:

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
      "scenario": {
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
  "manual_distance": 135.7,
  "collision_incidents": 2,
  "events": [
    {
      "time": "...",
      "event_type": "Manual_override",
      "details": {
        "operation_mode": "Manual",
        "estop": false,
        "robot_state": "active",
        "source": "autonomous_mode",
        "topic": "/autonomous_mode",
        "metrics": {
          "distance": 12.3,
          "autonomous_distance": 12.3,
          "manual_distance": 0.0,
          "speed": 0.4,
          "battery_percentage": 78.0
        },
        "system_snapshot": { "...": "..." }
      }
    }
  ]
}
```

Environment variables used to tag the session:

- `ROBOT_NAME`
- `FARM_NAME`
- `FIELD_NAME`
- `APPLICATION`
- `SCENARIO_NAME`

---

## 3. Published Topics

| Topic                                        | Type                | QoS              | Description                                                                |
| -------------------------------------------- | ------------------- | ---------------- | -------------------------------------------------------------------------- |
| `mdbi_logger/heartbeat`                      | `std_msgs/Bool`     | default          | 1 Hz heartbeat.                                                             |
| `mdbi_logger/total_traveled_distance`        | `std_msgs/Float32`  | default          | Total travelled distance (m).                                              |
| `mdbi_logger/total_autonomous_distance`      | `std_msgs/Float32`  | default          | Distance travelled while `/autonomous_mode == true` (m).                   |
| `mdbi_logger/total_manual_distance`          | `std_msgs/Float32`  | default          | Distance travelled while `/autonomous_mode == false` (m).                  |
| `mdbi_logger/robot_speed`                    | `std_msgs/Float32`  | default          | Estimated speed (m/s); zeroed after `stop_timeout`.                        |
| `mdbi_logger/total_incidents`                | `std_msgs/Int32`    | default          | Auto → Manual transition count (MDBI denominator).                         |
| `mdbi_logger/total_collision_incidents`      | `std_msgs/Int32`    | default          | Collision monitor count.                                                   |
| `mdbi_logger/db_health`                      | `std_msgs/Bool`     | latched (TL,1)   | `true` when DB writes are succeeding.                                      |
| `mdbi_logger/db_health_reason`               | `std_msgs/String`   | latched (TL,1)   | `"ok"` or description of the current DB issue.                             |
| Plus any `publish:` entries declared in YAML | (configurable)      | default          | Re-publish a single field of an input message as a typed topic.            |

---

## 4. Node Parameters

All parameters are exposed through the launch file.

### Core

| Parameter                | Type   | Default       | Description                                            |
| ------------------------ | ------ | ------------- | ------------------------------------------------------ |
| `config_yaml`            | string | `""`          | Path to YAML config file for topics & triggers.        |
| `mongodb_host`           | string | `"localhost"` | Local MongoDB host.                                    |
| `mongodb_port`           | int    | `27018`       | Local MongoDB port.                                    |
| `remote_mongodb_host`    | string | `""`          | Remote MongoDB host (empty = disabled).                |
| `remote_mongodb_port`    | int    | `27017`       | Remote MongoDB port.                                   |
| `enable_remote_logging`  | bool   | `false`       | Enable writes to remote MongoDB.                       |
| `min_distance_threshold` | double | `0.2`         | Min odom step (m) required to count as movement (debounce — see Accuracy section). |
| `stop_timeout`           | double | `2.0`         | Time (s) after last odom update before published speed is forced to 0. |

### DB resilience / cadence

| Parameter                          | Type   | Default | Description                                                                  |
| ---------------------------------- | ------ | ------- | ---------------------------------------------------------------------------- |
| `db_metrics_period`                | double | `1.0`   | Period (s) between periodic `distance` / `mdbi` / `incidents` writes.        |
| `db_server_selection_timeout_ms`   | int    | `1000`  | PyMongo `serverSelectionTimeoutMS`. Keeps DB ops from hanging.               |
| `db_connect_timeout_ms`            | int    | `1000`  | PyMongo `connectTimeoutMS`.                                                  |
| `db_socket_timeout_ms`             | int    | `2000`  | PyMongo `socketTimeoutMS`.                                                   |

### Collision monitor (nav vs collision velocity)

| Parameter                  | Type   | Default | Description                                                                                   |
| -------------------------- | ------ | ------- | --------------------------------------------------------------------------------------------- |
| `collision_nav_threshold`  | double | `0.01`  | `/cmd_vel/nav.linear.x` must be > this to count as a forward command.                         |
| `collision_zero_threshold` | double | `0.001` | `/cmd_vel/collision.linear.x` considered zero if `abs(x) ≤` this.                             |
| `collision_time_window`    | double | `0.5`   | Nav and collision commands must both be within this time window (s).                          |
| `collision_log_cooldown`   | double | `1.0`   | Optional legacy cooldown between collisions.                                                  |

Collision detection is falling-edge: a collision is registered only when previous `/cmd_vel/collision` had velocity, current is ~0, nav still requests forward motion, and both commands are recent.

---

## 5. YAML Config File

### Top-level structure

```yaml
git_repos:
  scenario: "."
  # ... additional repos relative to the scenario root ...

topics:
  # GPS (snapshot only)
  - name: "/gps_base/fix"
    type: "sensor_msgs/msg/NavSatFix"
    log_all_fields: true

  # Odometry — drives distance/speed and dual-mode distance accumulators.
  - name: "/gophar_vehicle_controller/odometry"
    type: "nav_msgs/msg/Odometry"
    role: "odometry"

  # System status with a battery field and a fault trigger.
  - name: "/gophar/system_status"
    type: "dynium_gophar_interfaces/msg/SystemStatus"
    role: "system_status"
    log_all_fields: true
    battery_field: "battery_percentage"
    intervention_on_change:
      fault_shutdown:
        trigger_value: true
        event_type: "Fault_shutdown"

  # NEW: operation mode (REQUIRED for MDBI).
  - name: "/autonomous_mode"
    type: "std_msgs/msg/Bool"
    role: "autonomous_mode"

  # NEW: robot state ("disabled" / "enabled" / "active").
  - name: "/robot_state"
    type: "std_msgs/msg/String"
    role: "robot_state"

  # Joy override (logged on any message).
  - name: "/cmd_vel_joy_smoothed"
    type: "geometry_msgs/msg/Twist"
    intervention_on_message:
      enable: true
      event_type: "Joy_override"

  # Collision monitor (nav vs collision velocity).
  - name: "/cmd_vel/nav"
    type: "geometry_msgs/msg/Twist"
    role: "collision_nav"
  - name: "/cmd_vel/collision"
    type: "geometry_msgs/msg/Twist"
    role: "collision_output"
```

### Per-topic keys

- `name` — ROS topic name to subscribe to.
- `type` — ROS message type. Accepts `pkg/msg/MessageName` or `pkg/MessageName`.
- `role` (optional, lowercase). Recognised values:
  - `"odometry"` — distance & speed source. Also splits distance into autonomous vs manual.
  - `"autonomous_mode"` — `std_msgs/Bool`. `true` → Autonomous, `false` → Manual. The recommended way to drive MDBI.
  - `"robot_state"` — `std_msgs/String`. Values "disabled" / "enabled" / "active". Logs an event on every change. Never increments MDBI.
  - `"control_mode"` — legacy explicit mode (mapped via `mode_mapping`); kept for backward-compat.
  - `"estop"` — emergency stop. Boolean field on a configurable path (default `data`).
  - `"system_status"` — generic snapshot topic; can carry `battery_field` and `intervention_on_change`.
  - `"collision_nav"` — nav velocity command for collision detection.
  - `"collision_output"` — collision-limited velocity command.
- `log_fields` — list of dotted paths to extract into `system_snapshot`.
- `log_all_fields` (bool) — if true, store the full message dict instead of `log_fields`.
- `battery_field` — field that carries battery percentage on this topic.
- `publish` (dict) — see "Dynamic republishing" below.
- `intervention_on_message` (dict) — see "YAML-driven interventions".
- `intervention_on_change` (dict) — see "YAML-driven interventions".

#### Dynamic republishing

```yaml
publish:
  enable: true
  topic: "mdbi_logger/battery_level"
  type: "std_msgs/msg/Float32"
  field: "battery_percentage"
```

#### Operation mode (`autonomous_mode`)

`std_msgs/Bool` with `data == true` → Autonomous, `data == false` → Manual. On change:

- Auto → Manual: logs `Manual_override` event AND increments `incidents` (MDBI source).
- Manual → Auto: logs `Autonomous_resumed` event; does NOT increment `incidents`.

#### Robot state (`robot_state`)

`std_msgs/String` with expected values "disabled" / "enabled" / "active". Every change is logged as a `Robot_state_changed` event with `prev_value` and `new_value` in the details. **Never** affects MDBI.

#### Legacy control mode (`control_mode`)

For older robots whose mode is published as a String/Int with arbitrary values, configure:

```yaml
- name: "/gophar/operation_mode"
  type: "std_msgs/msg/String"
  role: "control_mode"
  mode_field: "data"
  mode_mapping:
    "AUTO": "Autonomous"
    "MANUAL": "Manual"
```

Use **either** `autonomous_mode` **or** `control_mode`, not both.

#### E-stop role

`field` (default `"data"`) — boolean field. Rising edge triggers an `EMS` intervention.

#### YAML-driven interventions

- `intervention_on_message: {enable: true, event_type: "..."}` — every message triggers an event.
- `intervention_on_change: {<field>: {trigger_value: <opt>, event_type: "..."}}` — field-edge trigger.

---

## 6. Distance Accuracy and Reliability

The travelled distance is the source of truth for billing, so it gets special care:

- **Dual accumulators**: every odom step contributes to either `autonomous_distance` or `manual_distance`. The total `distance` is `autonomous_distance + manual_distance` by construction.
- **Debouncing**: `min_distance_threshold` (default 0.2 m) discards individual odom-to-odom deltas below the threshold, but the previous anchor `(x, y)` is **not** updated in that branch. This means accumulated motion is preserved exactly: tiny noise gets absorbed into the next real step rather than being thrown away.
- **RELIABLE QoS** for `autonomous_mode` / `robot_state` / `control_mode` / `estop` so we never miss a mode-change message and mis-attribute distance.
- **Periodic DB save** every `db_metrics_period` seconds plus immediate save on every event. Worst-case crash loss is bounded by that period.
- **Final flush** on shutdown: `update_db_metrics()` is called from the `finally:` block in `main()` to push the last numbers to Mongo when SIGTERM/SIGINT arrives.
- **Resilient DB layer**: any DB exception is caught at the boundary, logged once, surfaced on the latched health topic, and never propagates to the odom or mode callbacks.

If you tune `min_distance_threshold` lower, distance becomes more sensitive to odom noise (and may inflate). Higher values are safer against noise but introduce up to that many metres of "uncommitted" distance during a crash. The default 0.2 m is a reasonable trade-off for typical farm robots.

---

## 7. Launching

### Minimal

```bash
ros2 launch autonomy_metrics autonomy_metrics.launch.py \
  config_yaml:=/path/to/metrics_full.yaml
```

### With remote DB and tuned cadence

```bash
ros2 launch autonomy_metrics autonomy_metrics.launch.py \
  config_yaml:=/path/to/metrics_full.yaml \
  mongodb_host:=localhost \
  mongodb_port:=27018 \
  enable_remote_logging:=true \
  remote_mongodb_host:=10.0.0.42 \
  remote_mongodb_port:=27017 \
  min_distance_threshold:=0.1 \
  stop_timeout:=1.5 \
  db_metrics_period:=1.0 \
  collision_nav_threshold:=0.02 \
  collision_zero_threshold:=0.001 \
  collision_time_window:=0.4
```

---

## 8. Testing Tips

- Discover topics:

  ```bash
  ros2 topic list
  ros2 topic info /autonomous_mode
  ros2 topic info /robot_state
  ```

- Watch DB health:

  ```bash
  ros2 topic echo /mdbi_logger/db_health
  ros2 topic echo /mdbi_logger/db_health_reason
  ```

- Watch live distance:

  ```bash
  ros2 topic echo /mdbi_logger/total_traveled_distance
  ros2 topic echo /mdbi_logger/total_autonomous_distance
  ros2 topic echo /mdbi_logger/total_manual_distance
  ```

- Force a mode transition (smoke test):

  ```bash
  ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: true}"
  ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: false}"
  ```

- Trigger a robot-state change:

  ```bash
  ros2 topic pub --once /robot_state std_msgs/msg/String "{data: 'active'}"
  ```

- Check the Mongo session document directly with `mongosh` against `robot_incidents.sessions`.
