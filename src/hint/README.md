# hint

Bringup package for HINT — groups the launch file, RViz2 configuration, and teleop settings in one place.

## Contents

| Path | Purpose |
|---|---|
| `launch/bringup.launch.py` | Main bringup: teleop, Cartographer SLAM, visual tracker, visual servo, description detector, visual question, trajectory planner, BT executor, RViz2 |
| `config/teleop.yaml` | `teleop_twist_joy` parameters (axes, scales, enable button) |
| `viz/hint.rviz` | RViz2 layout pre-loaded with the tracking debug view |

## Usage

```bash
colcon build --symlink-install --packages-select hint
source install/setup.bash
ros2 launch hint bringup.launch.py
```

## Teleop configuration

`config/teleop.yaml` is passed to `teleop_twist_joy` at launch. Current mapping (Xbox controller):

| Parameter | Value | Effect |
|---|---|---|
| `axis_linear.x` | 1 | Left stick vertical → forward/back |
| `axis_angular.yaw` | 0 | Left stick horizontal → rotation |
| `scale_linear.x` | 0.26 m/s | Maximum linear speed |
| `scale_angular.yaw` | 0.91 rad/s | Maximum angular speed |
| `axis_linear_sign.x` | -1.0 | Inverts forward direction |
| `require_enable_button` | true | Deadman switch required |
| `enable_button` | 4 | LB button enables turbo (when `require_enable_button: true`) |

Edit the file and rebuild to apply changes:

```bash
colcon build --symlink-install --packages-select hint
```

## RViz2 layout

`viz/hint.rviz` opens with a single **Camera Tracking** display subscribed to `/camera/tracking`, the debug overlay published by `visual_tracker`. Open the file in RViz2, adjust the layout, and save — the updated file will be used on the next launch.

---
