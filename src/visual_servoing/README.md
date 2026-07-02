# visual_servoing

Image-based visual servo controller. Exposes an `ApproachTarget` action that drives the TurtleBot3 toward a visually tracked target until the target fills a configurable fraction of the frame.

## Usage

```bash
colcon build --symlink-install --packages-select hint_interfaces visual_servoing
source install/setup.bash
ros2 run visual_servoing visual_servo
```

Send a goal (requires `visual_tracker` to be running):

```bash
ros2 action send_goal /visual_servoing_node/approach_target \
  hint_interfaces/action/ApproachTarget \
  "{roi: {x_offset: 220, y_offset: 140, width: 200, height: 200, do_rectify: false}, stamp: {sec: 0, nanosec: 0}, setpoint_offset: 0.0}" \
  --feedback
```

`setpoint_offset` (`[-1, 1]`, default `0.0`) biases where the target is kept in-frame instead of dead-center — see "Control law" below.

Cancel an active goal:

```bash
ros2 action cancel /visual_servoing_node/approach_target
```

## Behaviour

On goal receipt, the node calls `visual_tracker`'s `set_target` service and starts a 20 Hz control loop. On result (success or failure) it calls `stop_tracking` to clean up the tracker.

**Control law**

- **Angular**: proportional on the normalised horizontal error `e ∈ [−1, +1]` between the target and a setpoint → `cmd_vel.angular.z`. The setpoint is frame-center by default, offset by the goal's `setpoint_offset ∈ [-1, 1]` — positive biases the setpoint (and thus the target) toward the right of frame, which curves the approach in from the left, and vice versa. The effective offset is scaled by `max_setpoint_offset` (default `0.75`) so a full `±1` request still leaves a margin at the frame edge instead of pinning the bbox center there — at the edge, half the bbox would already be off-screen.
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
| `success = true` | `bbox_area / image_area` reached `stop_area_ratio` |
| `success = false` | Tracker stayed `UNTRACKED` beyond `init_timeout`, or target lost after tracking, or goal cancelled |

Only one goal is accepted at a time; new goals are rejected while one is active.

## Interfaces

| Interface | Type | Direction |
|---|---|---|
| `~/approach_target` | `hint_interfaces/ApproachTarget` | Action server |
| `/tracking/state` | `std_msgs/String` (latched) | Sub — from `visual_tracker` |
| `/tracking/bbox` | `sensor_msgs/RegionOfInterest` | Sub — from `visual_tracker` |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Pub |
| `/lk_tracker_node/set_target` | `hint_interfaces/SetTarget` | Service client |
| `/lk_tracker_node/stop_tracking` | `hint_interfaces/StopTracking` | Service client |

## Parameters

All parameters are live-adjustable via `ros2 param set`.

| Parameter | Default | Effect |
|---|---|---|
| `k_yaw` | 0.05 | Angular gain (rad/s per unit normalised error) |
| `k_lin` | 0.10 | Linear gain (m/s per unit area-ratio error) |
| `max_linear_vel` | 0.26 | m/s cap — Waffle Pi rated maximum |
| `max_angular_vel` | 1.82 | rad/s cap |
| `stop_area_ratio` | 0.75 | Fraction of image area at which the robot stops |
| `max_setpoint_offset` | 0.75 | Scales the goal's `setpoint_offset` — caps how close to the frame edge the setpoint can be pushed |
| `init_timeout` | 5.0 | Seconds to wait for tracker to reach `TRACKING` before failing |
| `control_rate` | 20.0 | Control loop rate in Hz |
| `image_width` | 640 | Camera resolution — used to compute normalised error |
| `image_height` | 480 | Camera resolution — used to compute area ratio |

---
