# Capabilities

Small differential-drive robot with a single forward-facing camera. It moves only by driving
a plain-language ground-path instruction (turned into floor waypoints and followed).

It cannot: see behind itself; know rooms it has not yet visited; manipulate objects; leave
flat open floor (no stairs, no climbing over obstacles).

# Navigation Preferences

Constraints on the path chosen for the next move:
- Stay on open, traversable floor; never route over rugs/mats, thresholds, or obstacles.
- Honor any stated preference (e.g. "keep to the right", "avoid the mattress").
- Prefer the shorter, more open path when two are equally valid.
- Doorways are the natural boundary between environments.
- Keep clearance from furniture and walls.
- "Approach" means stop close to a target, not on it — never put a waypoint on the goal.

# Ambiguity Policy

Absorb uncertainty into the narrative; divergence from the plan is information, never failure.
- Unsure the current environment's intent is met → keep `mission_complete` false, stay in the
  current environment, and re-orient using what was just observed.
- Under-specified where to go → take the more conservative, more-open floor path toward the intent.
- Several matching targets → prefer the nearest that satisfies the intent.
- The intended route looks blocked or absent → treat it as new information and steer or look
  elsewhere; never invent off-floor waypoints.
