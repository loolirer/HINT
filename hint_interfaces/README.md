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
| `action/FollowVisualPath` | goal `{waypoints, stamp}` → result `{message}` / feedback `{state}` | `hint_navigation/path_projector` — grounds the waypoints into an `odom` path and drives Nav2's `follow_path` |
| `action/VisualReason` | goal `{prompt, schema, images}` → result `{response, stamp}` / feedback `{state}` | `hint_vlm/visual_reasoner` — generic text(+image)-in / JSON-out reasoning. The **only** VLM action; launched twice (the narrative director **and** the path planner), both driven by `hint_narrative` |
| `action/MissionAdvance` | goal `{success, mission_path, first}` → result `{mission_done, mission_failed, message, waypoints, turn_degrees, stamp}` / feedback `{state}` | `hint_narrative/narrative_navigation` — one cognition cycle of the mission loop: recompiles the narrative **and** plans the path (both via `VisualReason` calls), returning the next move as a trajectory. `mission_done` / `mission_failed` are VLM-declared by the director |

> **Result success convention.** `FollowVisualPath` and `VisualReason` carry **no `success` bool** — success/failure is the action's terminal status (SUCCEEDED vs ABORTED), with the reason in `message`/`response`. `MissionAdvance` instead reports completion via `mission_done`/`mission_failed`.
>
> **Shared image buffer.** `hint_narrative` owns the one camera-frame buffer and, each `MissionAdvance` cycle, attaches it to **both** `VisualReason` calls it makes internally — the director and the planner — so both reason over identical frames. The buffer no longer crosses the BT boundary; `MissionAdvance` returns the resulting trajectory (`waypoints`/`turn_degrees`/`stamp`) directly. `VisualReason` reports the current-view frame's `stamp` (`images[-1]`), which `hint_narrative` uses to ground the follow.

### `MissionAdvance` trajectory fields

The planner is a `VisualReason` call whose JSON reply `hint_narrative` parses into these `MissionAdvance` result fields (the pre-collapse `PlanVisualPath` result shape):

| Field | Type | Notes |
|---|---|---|
| `waypoints` | `geometry_msgs/Point[]` | Ordered waypoints, normalized image space `x`/`y ∈ [-1, 1]` (center 0), nearest-first. **May be empty** for a turn-only (scan / re-orient) move |
| `turn_degrees` | `float64` | Signed in-place rotation after the path, from the path's end heading. **+ = left (CCW)**, **− = right (CW)**, 0 = none |

## Removed interfaces (historical note)

The package once carried the pre-Nav2 pipeline's interfaces (visual trackers / IBVS
servoing): `msg/VisualWaypoints`, `srv/SetTarget` / `SetWaypoints` / `StopTracking`, and
`action/ApproachTarget` / `GroundDescription` / `VisualQuestion`. They were **deleted** when
the stack moved to Nav2 — there are no `msg/` or `srv/` directories anymore, and only the three
actions above remain. (Recover any from git history if ever needed.)

> The `.action` files under `action/` are the source of truth for exact field definitions;
> this README summarizes them.
