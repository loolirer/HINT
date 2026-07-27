# hint_interfaces

Custom ROS2 interface definitions for HINT. `ament_cmake` — must be built before any package
that uses these interfaces.

```bash
colcon build --symlink-install --packages-select hint_interfaces
source install/setup.bash
```

## Active interfaces (current Nav2 stack)

| Interface | Shape | Used by |
|---|---|---|
| `action/PlanTrajectory` | goal `{stamp, description}` → result `{success, message, markers, turn_degrees, stamp}` / feedback `{state}` | `hint_vlm/trajectory_generator` — plans ground waypoints **and** an end-of-move turn |
| `action/FollowTrajectory` | goal `{waypoints, stamp}` → result `{success, message}` / feedback `{state}` | `hint_navigation/trajectory_navigator` — grounds the waypoints and drives Nav2's `follow_path` |
| `action/Reason` | goal `{prompt, schema, images}` → result `{success, response}` / feedback `{state}` | `hint_vlm/visual_reasoner` — generic text(+image)-in / JSON-out reasoning |
| `action/MissionAdvance` | goal `{success, observation, mission_path}` → result `{mission_done, mission_failed, description, area, message}` / feedback `{state}` | `hint_narrative/narrative_navigation` — report-and-advance cycle of the mission loop |

### `PlanTrajectory` result fields

| Field | Type | Notes |
|---|---|---|
| `markers` | `geometry_msgs/Point[]` | Ordered waypoints, normalized image space `x`/`y ∈ [-1, 1]` (center 0), nearest-first. **May be empty** for a turn-only (scan / re-orient) move |
| `turn_degrees` | `float64` | Signed in-place rotation after the path, from the path's end heading. **+ = left (CCW)**, **− = right (CW)**, 0 = none |

## Legacy interfaces (still defined, not used by the current stack)

Defined for the pre-Nav2 pipeline (visual trackers / IBVS servoing) that has since been
removed. Kept in the package but currently unwired:

- `msg/VisualWaypoints`, `srv/SetTarget`, `srv/SetWaypoints`, `srv/StopTracking`
- `action/ApproachTarget`, `action/GroundDescription`, `action/VisualQuestion`

The `GroundDescription`/`VisualQuestion`/`ApproachTarget` actions (plus the `msg`/`srv`
tracking interfaces) are retained for reference but no longer have live nodes in `hint_vlm`.

> The `.action` / `.msg` / `.srv` files under `action/`, `msg/`, `srv/` are the source of
> truth for exact field definitions.
