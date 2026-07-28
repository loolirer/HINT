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
| `action/PlanVisualPath` | goal `{description, images}` → result `{message, markers, turn_degrees, stamp}` / feedback `{state}` | `hint_vlm/path_planner` — plans ground waypoints **and** an end-of-move turn over the goal's frame buffer |
| `action/FollowVisualPath` | goal `{waypoints, stamp}` → result `{message}` / feedback `{state}` | `hint_navigation/path_projector` — grounds the waypoints into an `odom` path and drives Nav2's `follow_path` |
| `action/Reason` | goal `{prompt, schema, images}` → result `{response, stamp}` / feedback `{state}` | `hint_vlm/visual_reasoner` — generic text(+image)-in / JSON-out reasoning |
| `action/MissionAdvance` | goal `{success, observation, mission_path, first}` → result `{mission_done, mission_failed, description, area, message, images}` / feedback `{state}` | `hint_narrative/narrative_navigation` — report-and-advance cycle of the mission loop |

> **Result success convention.** `PlanVisualPath`, `FollowVisualPath`, and `Reason` carry **no `success` bool** — success/failure is the action's terminal status (SUCCEEDED vs ABORTED), with the reason in `message`/`response`. `MissionAdvance` instead reports completion via `mission_done`/`mission_failed`.
>
> **Shared image buffer.** `hint_narrative` owns the one camera-frame buffer and passes it out on `MissionAdvance.images`; the BT forwards it to `PlanVisualPath.images`, and the director gets the same frames on `Reason.images` — so both VLM calls reason over identical frames. `PlanVisualPath`/`Reason` report the current-view frame's `stamp` (`images[-1]`) so the follow can ground the path.

### `PlanVisualPath` result fields

| Field | Type | Notes |
|---|---|---|
| `markers` | `geometry_msgs/Point[]` | Ordered waypoints, normalized image space `x`/`y ∈ [-1, 1]` (center 0), nearest-first. **May be empty** for a turn-only (scan / re-orient) move |
| `turn_degrees` | `float64` | Signed in-place rotation after the path, from the path's end heading. **+ = left (CCW)**, **− = right (CW)**, 0 = none |

## Removed interfaces (historical note)

The package once carried the pre-Nav2 pipeline's interfaces (visual trackers / IBVS
servoing): `msg/VisualWaypoints`, `srv/SetTarget` / `SetWaypoints` / `StopTracking`, and
`action/ApproachTarget` / `GroundDescription` / `VisualQuestion`. They were **deleted** when
the stack moved to Nav2 — there are no `msg/` or `srv/` directories anymore, and only the four
actions above remain. (Recover any from git history if ever needed.)

> The `.action` files under `action/` are the source of truth for exact field definitions;
> this README summarizes them.
