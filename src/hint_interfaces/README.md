# hint_interfaces

Custom ROS2 interface definitions for HINT. This is an `ament_cmake` package that must be built before any Python package that uses these interfaces.

```bash
colcon build --symlink-install --packages-select hint_interfaces
source install/setup.bash
```

## Services

### `SetTarget` — `hint_interfaces/srv/SetTarget`

Triggers (or resets) tracking on a specific region of interest.

**Request**

| Field | Type | Notes |
|---|---|---|
| `roi` | `sensor_msgs/RegionOfInterest` | Region to track |
| `stamp` | `builtin_interfaces/Time` | Source frame timestamp; `{sec: 0, nanosec: 0}` uses the next arriving frame |

**Response**

| Field | Type | Notes |
|---|---|---|
| `accepted` | `bool` | `false` if the ROI was immediately rejected (e.g. too few features) |
| `message` | `string` | Human-readable status |

---

### `StopTracking` — `hint_interfaces/srv/StopTracking`

Stops active tracking and resets the tracker to `UNTRACKED`. Empty request and response.

---

## Actions

### `ApproachTarget` — `hint_interfaces/action/ApproachTarget`

Drives the robot toward a visual target until the stopping condition is met.

**Goal**

| Field | Type | Notes |
|---|---|---|
| `roi` | `sensor_msgs/RegionOfInterest` | Initial tracking region, forwarded to `SetTarget` |
| `stamp` | `builtin_interfaces/Time` | Source frame timestamp; `{sec: 0, nanosec: 0}` uses the next arriving frame |

**Result**

| Field | Type | Notes |
|---|---|---|
| `success` | `bool` | `true` if the robot reached the target |
| `message` | `string` | Human-readable outcome |

**Feedback**

| Field | Value | Meaning |
|---|---|---|
| `state` | `IDLE` | Tracker is `UNTRACKED` — waiting for initialisation |
| `state` | `RUNNING` | Tracker is `TRACKING` — robot is moving |
| `state` | `WAITING` | Tracker is `OCCLUDED` — robot is stopped, waiting for target to reappear |

---
