# visual_servoing

Servo controllers that turn a tracker's output into `/cmd_vel`. Two executables
share one action-driven lifecycle (single goal at a time; `IDLE`/`RUNNING`/
`WAITING` feedback; `init_timeout`/`occlusion_timeout` failures; `stop_tracking`
on result):

| Executable | Drives off | Action | Control |
|---|---|---|---|
| `visual_servo` | `lk_tracker` (bounding box) | `ApproachTarget` | IBVS — approach until the target fills the frame |
| `pursuit_servo` | `waypoint_tracker` (ground trajectory) | `FollowTrajectory` | Pure pursuit — follow the waypoints until consumed |

```bash
colcon build --symlink-install --packages-select hint_interfaces visual_servoing
source install/setup.bash
```

---

## visual_servo

Image-based visual servo controller. Exposes an `ApproachTarget` action that drives the TurtleBot3 toward a visually tracked target until the target fills a configurable fraction of the frame.

### Usage

```bash
ros2 run visual_servoing visual_servo
```

Send a goal (requires `visual_tracker` to be running):

```bash
ros2 action send_goal /visual_servoing_node/approach_target \
  hint_interfaces/action/ApproachTarget \
  "{roi: {x_offset: 220, y_offset: 140, width: 200, height: 200, do_rectify: false}, stamp: {sec: 0, nanosec: 0}}" \
  --feedback
```

Cancel an active goal:

```bash
ros2 action cancel /visual_servoing_node/approach_target
```

### Behaviour

On goal receipt, the node calls `visual_tracker`'s `set_target` service and starts a 20 Hz control loop. On result (success or failure) it calls `stop_tracking` to clean up the tracker.

**Control law**

- **Angular**: proportional on the normalised horizontal centre error `e ∈ [−1, +1]` → `cmd_vel.angular.z`. Keeps the target centred in the frame.
- **Linear**: proportional on `(stop_area_ratio − bbox_area/image_area)`, clamped to zero. Velocity ramps naturally to zero as the target fills the frame.

**Action feedback states**

| State | Tracker state | Robot |
|---|---|---|
| `IDLE` | `UNTRACKED` | Stopped — waiting for tracker to initialise |
| `RUNNING` | `TRACKING` | Moving toward target |
| `WAITING` | `OCCLUDED` | Stopped — waiting for target to reappear |

**Action result**

| Outcome | Condition |
|---|---|
| `success = true` | `bbox_area / image_area` reached `stop_area_ratio`, **or** both linear and angular outputs fell below their dead-zone floors (`min_linear_vel`, `min_angular_vel`) |
| `success = false` | Tracker stayed `UNTRACKED` beyond `init_timeout`, target lost beyond `occlusion_timeout`, or goal cancelled |

Only one goal is accepted at a time; new goals are rejected while one is active.

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/approach_target` | `hint_interfaces/ApproachTarget` | Action server |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Sub — image width/height for normalisation |
| `/tracking/state` | `std_msgs/String` (latched) | Sub — from `visual_tracker` |
| `/tracking/bbox` | `sensor_msgs/RegionOfInterest` | Sub — from `visual_tracker` |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Pub |
| `/lk_tracker_node/set_target` | `hint_interfaces/SetTarget` | Service client |
| `/lk_tracker_node/stop_tracking` | `hint_interfaces/StopTracking` | Service client |

### Parameters

All parameters are live-adjustable via `ros2 param set`.

| Parameter | Default | Effect |
|---|---|---|
| `k_yaw` | 0.20 | Angular gain (rad/s per unit normalised error) |
| `k_lin` | 0.25 | Linear gain (m/s per unit area-ratio error) |
| `max_linear_vel` | 0.26 | m/s cap — Waffle Pi rated maximum |
| `max_angular_vel` | 1.82 | rad/s cap |
| `min_linear_vel` | 0.05 | Dead-zone floor for linear velocity; arrival declared if both `v` and `w` fall below floors |
| `min_angular_vel` | 0.05 | Dead-zone floor for angular velocity |
| `stop_area_ratio` | 0.75 | Fraction of image area at which the robot stops |
| `init_timeout` | 5.0 | Seconds to wait for tracker to reach `TRACKING` before failing |
| `occlusion_timeout` | 5.0 | Seconds in `OCCLUDED` state before aborting with failure |
| `control_rate` | 20.0 | Control loop rate in Hz |

---

## pursuit_servo

Pure-pursuit waypoint follower — the trajectory-following sibling of `visual_servo`.
Exposes a `FollowTrajectory` action that hands an ordered ground trajectory to
`waypoint_tracker` and steers the robot along it until every waypoint has passed
under the robot (the trajectory is consumed). It's the low-level controller for a
VLM-planned path: `trajectory_planner`'s `markers` feed straight into the goal.

### Usage

```bash
ros2 run visual_servoing pursuit_servo
```

Send a goal (requires `waypoint_tracker` running; waypoints are normalized image
space, `x`/`y ∈ [-1, 1]`, center 0, **nearest first**):

```bash
ros2 action send_goal /pursuit_servo_node/follow_trajectory \
  hint_interfaces/action/FollowTrajectory \
  "{waypoints: [{x: 0.0, y: 0.8, z: 0.0}, {x: 0.05, y: 0.4, z: 0.0}, {x: 0.1, y: 0.2, z: 0.0}], stamp: {sec: 0, nanosec: 0}}" \
  --feedback
```

Cancel an active goal:

```bash
ros2 action cancel /pursuit_servo_node/follow_trajectory
```

### Behaviour

On goal receipt, the node calls `waypoint_tracker`'s `set_waypoints` service and
starts a control loop at `control_rate`. On result (success or failure) it calls
`stop_tracking` to clean up the tracker. Same single-goal lifecycle as
`visual_servo`.

This runs in **metric top-down world space**: `waypoint_tracker` publishes every
waypoint's live position in `base_link` (x forward, y left, metres) with a per-point
`in_front` flag, and the follower does plain metric pure pursuit — no image space.

**Reaching (retirement lives here, not the tracker).** The tracker only tracks — it
re-publishes the **full** waypoint set every frame and never retires. This follower
owns *reaching*: it keeps a front index over the (as-sent) stream and advances it past
each waypoint the robot drives over. A waypoint is **reached** when the robot comes
within `reach_radius` **metres** of it. Reaching is sequential (in order) and steers
only by waypoints ahead of the front. When the front reaches the end (every waypoint
reached), the trajectory is **consumed** → success. A waypoint the robot passes wide
of is **not** reached; pure pursuit keeps steering — and will **turn back** to it if
it ends up behind — until the robot actually drives within `reach_radius`.

**Control law (metric pure pursuit)** — each tick it picks a **lookahead** carrot from
the still-unreached tail of the stream: the first **in-front** waypoint at least
`lookahead` metres from the robot (the farthest in-front one if none reach that). If
no unreached waypoint is in front (the path continues behind), it targets the first
unreached one so the robot rotates to face it:

- **Angular**: proportional on the carrot's **bearing** `atan2(y, x)` → `cmd_vel.angular.z` (a behind carrot has bearing near ±π, so the robot spins to face it).
- **Linear**: `cruise_speed` scaled down by `|bearing|` → slows on sharp turns, **turns in place** past 90° off-axis, clamped to `max_linear_vel`.

**Action feedback states** (identical to `visual_servo`):

| State | Tracker state | Robot |
|---|---|---|
| `IDLE` | `UNTRACKED` (pre-init) | Stopped — waiting for the tracker to initialise |
| `RUNNING` | `TRACKING` | Following the trajectory |
| `WAITING` | `OCCLUDED` | Stopped — waiting for the tracker to recover |

**Action result**

| Outcome | Condition |
|---|---|
| `success = true` | Trajectory **consumed** — the follower reached every waypoint (front advanced past the last one) |
| `success = false` | Tracker stayed `UNTRACKED` beyond `init_timeout`, `OCCLUDED` beyond `occlusion_timeout`, tracker reset unexpectedly after tracking, waypoints rejected, or goal cancelled |

Only one goal is accepted at a time; new goals are rejected while one is active.

### Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/follow_trajectory` | `hint_interfaces/FollowTrajectory` | Action server |
| `/waypoint_tracking/points` | `hint_interfaces/VisualWaypoints` | Sub — from `waypoint_tracker` |
| `/waypoint_tracking/state` | `std_msgs/String` (latched) | Sub — from `waypoint_tracker` |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Pub |
| `/waypoint_tracker_node/set_waypoints` | `hint_interfaces/SetWaypoints` | Service client |
| `/waypoint_tracker_node/stop_tracking` | `hint_interfaces/StopTracking` | Service client |

**`follow_trajectory` action fields**

| Field | Type | Notes |
|---|---|---|
| **Goal** `waypoints` | `geometry_msgs/Point[]` | Ordered ground waypoints, normalized `x`/`y ∈ [-1, 1]`, nearest first — the `PlanTrajectory.markers` layout |
| **Goal** `stamp` | `builtin_interfaces/Time` | Frame to initialise tracking on; `{sec: 0, nanosec: 0}` uses the next frame |
| **Result** `success` | `bool` | Trajectory consumed vs failed/cancelled |
| **Result** `message` | `string` | Outcome reason |
| **Feedback** `state` | `string` | `IDLE` / `RUNNING` / `WAITING` |

### Parameters

All parameters are live-adjustable via `ros2 param set`.

| Parameter | Default | Effect |
|---|---|---|
| `k_yaw` | 1.5 | Angular gain (rad/s per **radian** of carrot bearing) |
| `cruise_speed` | 0.05 | m/s forward speed when aligned; scaled down by bearing |
| `lookahead` | 0.3 | **Metres** from the robot at which to pick the carrot waypoint. Larger = smoother/less reactive; smaller = tighter path following |
| `reach_radius` | 0.15 | **Metres**: a waypoint is reached (front advances) once the robot is within this distance of it. Larger = retire earlier / more forgiving; smaller = must drive nearly over it |
| `max_linear_vel` | 0.26 | m/s cap — Waffle Pi rated maximum |
| `max_angular_vel` | 1.82 | rad/s cap |
| `init_timeout` | 5.0 | Seconds to wait for the tracker to reach `TRACKING` before failing |
| `occlusion_timeout` | 5.0 | Seconds in `OCCLUDED` before aborting with failure |
| `control_rate` | 20.0 | Control loop rate in Hz |

---
