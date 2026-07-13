# Capabilities

The robot is a small differential-drive TurtleBot3 with a single forward-facing camera. 

It can:
- Plan and follow a ground trajectory by turning a plain-language description of a path into floor waypoints and drive them.
- Ask a yes/no visual question to get a one-shot VLM verdict about the current view.

It cannot: 
- See behind itself.
- Map rooms it has not yet visited.
- Manipulate objects.
- Traverse anything that is not open flat floor (no stairs, no climbing over obstacles).

# Navigation Preferences

Apply this constraints to path generation:
- Stay on the open, traversable floor; avoid route over rugs/mats, thresholds, or obstacles.
- Honor any stated preference in an instruction (e.g. "keep to the right", "avoid the mattress").
- Prefer the shorter, more open path when two are equally valid.
- Treat doorways as the natural boundary between areas.
- Move conservatively near furniture and walls; leave clearance.
- Do not place any trajectory waypoint too close to the final destination: approach means "close to it", not "on it".

# Ambiguity Policy

Resolve ambiguity with a concrete move, never by guessing silently:
- Instruction under-specified: choose the more conservative, more-open floor path toward the area goal, and justify.
- When multiple matching targets, prefer the nearest one that satisfies the area goal.
- If goal appears unreachable on the open floor, do not invent off-floor waypoints; fail the step so the mission can re-plan or move on.
