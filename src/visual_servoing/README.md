# visual_servoing

Servo controllers that turn a tracker's output into `/cmd_vel`. Two executables
share one action-driven lifecycle (single goal at a time; `IDLE`/`RUNNING`/
`WAITING` feedback; `init_timeout`/`occlusion_timeout` failures; `stop_tracking`
on result):

| Executable | Drives off | Action | Control |
|---|---|---|---|
| `visual_servo` | `lk_tracker` (bounding box) | `ApproachTarget` | IBVS — approach until the target fills the frame |
| `pursuit_servo` | `waypoint_tracker` (ground trajectory) | `FollowTrajectory` | Control-Lyapunov path following — follow the trajectory to the last waypoint |

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

Control-Lyapunov **path-following** waypoint follower — the trajectory-following
sibling of `visual_servo`. Exposes a `FollowTrajectory` action that hands an ordered
ground trajectory to `waypoint_tracker` and steers the robot along it until it reaches
the last waypoint. It's the low-level controller for a VLM-planned path:
`trajectory_planner`'s `markers` feed straight into the goal. (The node/action name
is unchanged for historical reasons; the control law is Control-Lyapunov path
following, not pure pursuit.)

> **Control law** — implements **Proposition 1** of Ebrahimi Toulkani, Abdi,
> Koskelainen & Ghabcheloo, *"Reactive Safe Path Following for Differential Drive
> Mobile Robots Using Control Barrier Functions"* (ICCMA 2022) — see
> `docs/Reactive_Safe_Path_Following_..._Control_Barrier_Functions.pdf`. Only the
> Control-**Lyapunov** path follower is implemented here; the paper's Control-Barrier-
> Function QP (obstacle avoidance) is deliberately left as a future layer that would
> wrap `ω`.

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

This runs in **metric top-down world space**: `waypoint_tracker` publishes the full
waypoint set's live position in `base_link` (x forward, y left, metres) every frame,
and the follower treats the **waypoint polyline as the path Γ** — no image space.

**Control law (Control-Lyapunov path following, Proposition 1).** A **virtual target
Q** rides the path at arc length `s`. With the robot at the `base_link` origin heading
+x, the path's tangent–normal frame at Q gives the errors `x_e` (along-track), `y_e`
(cross-track), `ψ_e` (heading). Each tick:

- `σ = −asin( clamp(k2·y_e / (|y_e|+ε0)) )` — the approach angle that bends the robot toward the path.
- `ṡ = v·cos(ψ_e) + k3·x_e` — advances Q along the path (and pulls along-track error to zero); `s` is integrated at `control_rate`.
- `ω = C_c·ṡ + σ̇ − k1·(ψ_e − σ) − v·y_e·Δ`, with `Δ = (sin ψ_e − sin σ)/(ψ_e − σ)` (→ `cos σ` at `ψ_e = σ`) and `C_c` the path curvature at Q (tangent differenced over `curvature_window`).
- **Linear**: `v` is held **constant** at `cruise_speed` (the paper's `v_ref`) — the convergence guarantee requires non-zero `v`, so this follower does not stop or turn in place; it follows the path as a smooth arc.

This drives the Lyapunov function `V = ½(x_e² + y_e² + (ψ_e − σ)²) → 0`, i.e. the robot
provably converges onto and follows the path. Arrival is declared when the robot is
within `reach_radius` of the **last** waypoint (or the virtual target passes the path
end). The `in_front` flag is not needed by this law — the whole polyline is the path.

> **Note (coarse paths / sharp corners):** the law follows the path at constant speed,
> so a VLM path with a near-cusp (e.g. a 180° reversal) may not be exactly trackable at
> `cruise_speed`; the robot rounds it at its turning radius `cruise_speed/max_angular_vel`.
> Curvature is estimated on the coarse polyline via `curvature_window`.

**Action feedback states** (identical to `visual_servo`):

| State | Tracker state | Robot |
|---|---|---|
| `IDLE` | `UNTRACKED` (pre-init) | Stopped — waiting for the tracker to initialise |
| `RUNNING` | `TRACKING` | Following the trajectory |
| `WAITING` | `OCCLUDED` | Stopped — waiting for the tracker to recover |

**Action result**

| Outcome | Condition |
|---|---|
| `success = true` | **Arrived** — the robot reached the last waypoint (within `reach_radius`, or the virtual target passed the path end) |
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
| `cruise_speed` | 0.05 | m/s — **constant** path speed `v_ref` (the law needs non-zero `v`; the robot does not stop/pivot mid-path) |
| `k1` | 2.0 | Gain on the heading/approach error `(ψ_e − σ)` — larger = snappier heading correction |
| `k2` | 1.0 | Cross-track → approach-angle gain, `0..1`; larger = bends harder toward the path off it |
| `k3` | 1.0 | Along-track gain — pulls the virtual target's `x_e` to zero (how fast `Q` tracks the robot's along-path position) |
| `eps0` | 0.35 | Softening constant in the `σ` law near the path (avoids over-reaction as `y_e → 0`) |
| `curvature_window` | 0.15 | **Metres** — arc-length span over which the tangent is differenced to estimate path curvature `C_c` on the coarse polyline. Larger = smoother/less noisy `C_c`, smaller = more local |
| `reach_radius` | 0.15 | **Metres** — arrival radius at the last waypoint |
| `max_linear_vel` | 0.26 | m/s cap — Waffle Pi rated maximum |
| `max_angular_vel` | 1.82 | rad/s cap (also sets the minimum turning radius `cruise_speed/max_angular_vel`) |
| `init_timeout` | 5.0 | Seconds to wait for the tracker to reach `TRACKING` before failing |
| `occlusion_timeout` | 5.0 | Seconds in `OCCLUDED` before aborting with failure |
| `control_rate` | 20.0 | Control loop rate in Hz (also the `s`-integration step) |

---
