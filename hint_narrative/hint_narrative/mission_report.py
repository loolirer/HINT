"""Compile a mission's minimal MCAP rosbag into one annotated trajectory figure + stats.

Consumes the ``mission.bag`` that ``narrative_navigation`` records per run (see
``_BAG_TOPICS`` there) and writes, **into the mission's own directory** (the bag's parent):
``mission_report.png`` (the plot) and ``mission_stats.json`` (the numbers), also printed.

What the figure shows (all in the reference frame — ``map`` if the bag has one, else ``odom``):
- the **actual** continuous path the robot drove (``<ref>->base_link`` from ``/tf``, or ``/odom``);
- the VLM's **raw** paths (full intent) and the **truncated** paths (what was actually followed),
  from ``trajectory_navigator``'s ``~/path_raw`` / ``~/path``;
- the robot **footprint** (a circle of ``robot_radius``) at each stopped position — a spatial
  cluster of VLM calls — with a centre marker (green at the first stop, red at the last, robot
  colour otherwise) and a label giving the **total** number of VLM calls made there (both the
  director ``advance`` and the executor ``plan_trajectory``);
- heading lines drawn beneath the markers/circle but above the trajectories: an **arrival**
  line in the robot's colour, plus, when the robot spun in place, a distinct-colour **post-spin**
  line showing where it ended up facing;
- a stats box: mission duration, time spent processing the VLM, time spent moving (the number
  of spins is counted there; angle magnitudes are not drawn on the map).

The VLM-vs-movement split is derived from the actions' ``_action/status`` topics: the
``plan_trajectory`` + ``advance`` windows are VLM-thinking (robot stationary), the
``follow_trajectory`` + ``spin`` windows are movement. No runtime node is modified.

Usage:
    ros2 run hint_narrative mission_report /path/to/missions/<name>/mission.bag
"""

import argparse
import json
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless — we only save a PNG
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.lines import Line2D

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

# action_msgs/msg/GoalStatus constants
_STATUS_ACTIVE = (1, 2)          # ACCEPTED, EXECUTING
_STATUS_TERMINAL = (4, 5, 6)     # SUCCEEDED, CANCELED, ABORTED

# The four action-status topics, split by what the robot is doing during each window.
_VLM_STATUS = (
    "/trajectory_generator/plan_trajectory/_action/status",   # executor VLM
    "/narrative_navigation/advance/_action/status",           # director VLM (wraps reason)
)
_MOVE_STATUS = (
    "/trajectory_navigator_node/follow_trajectory/_action/status",
    "/spin/_action/status",
)
_PLAN_STATUS = "/trajectory_generator/plan_trajectory/_action/status"  # footprint anchors
_SPIN_STATUS = "/spin/_action/status"                                  # turn markers

_PATH_TOPIC = "/trajectory_navigator_node/path"
_PATH_RAW_TOPIC = "/trajectory_navigator_node/path_raw"

_ROBOT_RADIUS = 0.20   # local_costmap robot_radius (footprint circle)

# --- Tunable drawing parameters ---
ROBOT_MARKER_SIZE = 90        # the marker inside each robot footprint
ROBOT_LINE_WIDTH = 2.6        # footprint circle border + heading-line width
ROBOT_LINE_COLOR = "black"    # footprint circle outline (always)
# The robot's own colour, used for the centre marker (and the arrival heading line) at every
# footprint that is neither the first nor the last stopped position.
ROBOT_MARKER_COLOR = "black"
# Distinct, parameterized colour for an in-place spin's heading line — a spin the robot made
# without translating; the line shows the heading AFTER the spin.
IN_PLACE_TURN_COLOR = "#c02060"
ROBOT_FILL = False            # fill the footprint circle? (False = hollow outline)
ROBOT_FILL_COLOR = "#8a8a8a"  # footprint circle fill — grey (used when ROBOT_FILL)

# z-order layering: trajectories < heading lines < footprint circle < centre marker < label.
Z_TRAJ = 3.0            # VLM raw / truncated polylines
Z_ACTUAL = 3.5          # actual driven path
Z_HEADING = 3.7         # arrival heading line (above trajectories, below the circle)
Z_SPIN_HEADING = 3.8    # in-place post-spin heading line
Z_CIRCLE = 4.0          # footprint circle
Z_MARKER = 6.0          # centre marker (always on top of the circle)
Z_LABEL = 10.0          # VLM-call-count label

# Muted palette (matches visual_debug's spirit).
_C_ACTUAL = "#1f4fd8"       # blue — the path actually driven
_C_RAW = "#e0a020"          # amber — full VLM intent
_C_CLIP = "#2aa8a8"         # teal — truncated/followed
_C_START = "green"          # start: centre marker of the first stopped position
_C_END = "red"              # end: centre marker of the last stopped position


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _t_seconds(t_nanos):
    return t_nanos * 1e-9


# ----------------------------------------------------------------------
# Bag reading


def _open(bag_path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="mcap"),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_map


def _compose_up(child, transforms, ref):
    """Pose (x, y, yaw) of ``child``'s origin expressed in ``ref``, walking parent links
    (``transforms``: child_frame -> (parent, tx, ty, yaw)). None if the chain breaks."""
    x, y, yaw = 0.0, 0.0, 0.0
    frame = child
    seen = set()
    while frame != ref:
        if frame in seen or frame not in transforms:
            return None
        seen.add(frame)
        parent, tx, ty, pyaw = transforms[frame]
        c, s = math.cos(pyaw), math.sin(pyaw)
        x, y = c * x - s * y + tx, s * x + c * y + ty
        yaw = yaw + pyaw
        frame = parent
    return x, y, wrap(yaw)


def read_bag(bag_path):
    """Single pass over the bag. Returns a dict of everything the plot needs."""
    reader, type_map = _open(bag_path)
    msg_types = {}

    def mtype(topic):
        if topic not in msg_types:
            msg_types[topic] = get_message(type_map[topic])
        return msg_types[topic]

    has_map = "/map" in type_map and any(
        "OccupancyGrid" in type_map.get(t, "") for t in ("/map",))
    transforms = {}                 # child -> (parent, tx, ty, yaw)
    parents = set()                 # every parent frame seen on /tf(/static)
    tf_samples = []                 # (t, x, y, yaw) base_link in ref (filled after ref known)
    tf_raw = []                     # (t, [(parent, child, tx, ty, yaw), ...]) to replay
    odom_samples = []               # (t, x, y, yaw) fallback
    paths = {_PATH_TOPIC: [], _PATH_RAW_TOPIC: []}   # deduped polylines
    seen_paths = {_PATH_TOPIC: set(), _PATH_RAW_TOPIC: set()}
    status = {}                     # topic -> {goal_id: [t0, t1]}
    occ = None
    t_min, t_max = None, None

    while reader.has_next():
        topic, data, t = reader.read_next()
        t_min = t if t_min is None else min(t_min, t)
        t_max = t if t_max is None else max(t_max, t)

        if topic in ("/tf", "/tf_static"):
            tf = deserialize_message(data, mtype(topic))
            step = []
            for tr in tf.transforms:
                p = tr.header.frame_id.lstrip("/")
                ch = tr.child_frame_id.lstrip("/")
                yaw = yaw_from_quat(tr.transform.rotation)
                trans = (p, tr.transform.translation.x, tr.transform.translation.y, yaw)
                transforms[ch] = trans
                parents.add(p)
                step.append((p, ch, *trans[1:]))
            tf_raw.append((t, step))

        elif topic == "/odom":
            od = deserialize_message(data, mtype(topic))
            p = od.pose.pose.position
            odom_samples.append((t, p.x, p.y, yaw_from_quat(od.pose.pose.orientation)))

        elif topic in paths:
            pm = deserialize_message(data, mtype(topic))
            poly = tuple((round(ps.pose.position.x, 3), round(ps.pose.position.y, 3))
                         for ps in pm.poses)
            if poly and poly not in seen_paths[topic]:
                seen_paths[topic].add(poly)
                paths[topic].append([(x, y) for x, y in poly])

        elif topic.endswith("/_action/status"):
            arr = deserialize_message(data, mtype(topic))
            win = status.setdefault(topic, {})
            for gs in arr.status_list:
                gid = bytes(gs.goal_info.goal_id.uuid)
                slot = win.setdefault(gid, [None, None])
                if gs.status in _STATUS_ACTIVE and slot[0] is None:
                    slot[0] = t
                elif gs.status in _STATUS_TERMINAL and slot[1] is None:
                    # First terminal only: a finished goal lingers in the status_list
                    # across later messages, so taking the last would stretch the window
                    # well past the real end (VLM time could exceed the mission duration).
                    slot[1] = t

        elif topic == "/map":
            occ = deserialize_message(data, mtype(topic))  # keep latest

    # Reference frame: map if a map frame was published (or /map exists), else odom.
    ref = "map" if ("map" in parents or has_map) else "odom"

    # Replay /tf to build the actual path in the reference frame.
    running = {}
    for t, step in tf_raw:
        for p, ch, tx, ty, yaw in step:
            running[ch] = (p, tx, ty, yaw)
        pose = _compose_up("base_link", running, ref)
        if pose is not None:
            tf_samples.append((t, *pose))

    return {
        "ref": ref,
        "occ": occ,
        "actual": tf_samples if tf_samples else odom_samples,
        "paths": paths[_PATH_TOPIC],
        "paths_raw": paths[_PATH_RAW_TOPIC],
        "status": status,
        "span": (t_min, t_max),
    }


# ----------------------------------------------------------------------
# Derived quantities


def windows(status, topic):
    """List of (t0, t1) second-windows for completed goals on ``topic``."""
    out = []
    for t0, t1 in status.get(topic, {}).values():
        if t0 is not None and t1 is not None and t1 >= t0:
            out.append((_t_seconds(t0), _t_seconds(t1)))
    return sorted(out)


def _union_windows(status, topics):
    """Total wall-clock time covered by the goal windows on ``topics`` — the *union*,
    so overlapping windows (across goals or topics) count once, not twice. Keeps the
    VLM / movement totals bounded by the mission duration."""
    ivals = sorted(w for topic in topics for w in windows(status, topic))
    total, cur_lo, cur_hi = 0.0, None, None
    for lo, hi in ivals:
        if cur_hi is None or lo > cur_hi:
            if cur_hi is not None:
                total += cur_hi - cur_lo
            cur_lo, cur_hi = lo, hi
        else:
            cur_hi = max(cur_hi, hi)
    if cur_hi is not None:
        total += cur_hi - cur_lo
    return float(total)


def pose_at(actual, t_sec):
    """Nearest actual-path sample (x, y, yaw) to ``t_sec`` (seconds), or None."""
    if not actual:
        return None
    arr = min(actual, key=lambda s: abs(_t_seconds(s[0]) - t_sec))
    return arr[1], arr[2], arr[3]


# ----------------------------------------------------------------------
# Drawing


def draw_map(occ, ax):
    res = occ.info.resolution
    w, h = occ.info.width, occ.info.height
    ox, oy = occ.info.origin.position.x, occ.info.origin.position.y
    grid = np.array(occ.data, dtype=np.int16).reshape((h, w))
    disp = np.full((h, w), 200, dtype=np.uint8)          # unknown -> mid gray
    known = grid >= 0
    disp[known] = (255 - grid[known] * 2.55).astype(np.uint8)
    ax.imshow(disp, cmap="gray", origin="lower", zorder=1,
              extent=[ox, ox + w * res, oy, oy + h * res])


def _heading_line(x, y, yaw, ax, color, zorder):
    """A plain heading line from the footprint centre out to its radius (no arrowhead),
    the same width as the circle border."""
    ax.plot([x, x + _ROBOT_RADIUS * math.cos(yaw)],
            [y, y + _ROBOT_RADIUS * math.sin(yaw)],
            color=color, lw=ROBOT_LINE_WIDTH, alpha=0.9, zorder=zorder,
            solid_capstyle="round")


def _number_label(x, y, text, ax):
    """A VLM-call-count number: black text on a white, black-edged circle, at (x, y)."""
    ax.text(x, y, text, fontsize=11, color="black", ha="center", va="center",
            zorder=Z_LABEL, fontweight="bold",
            bbox=dict(boxstyle="circle,pad=0.35", fc="white", ec="black",
                      lw=1.4, alpha=0.95))


def draw_footprint(x, y, ax, marker_color=ROBOT_MARKER_COLOR, label=None):
    """Robot footprint circle at (x, y): a (grey-fill, optional) black-outlined circle with a
    centre marker in ``marker_color`` (green = first stop, red = last, robot colour otherwise —
    so the robot always has a marker inside it) and an optional VLM-call-count label above it.
    Heading lines are drawn separately (below the circle), not here."""
    ax.add_patch(Circle((x, y), _ROBOT_RADIUS,
                        facecolor=ROBOT_FILL_COLOR if ROBOT_FILL else "none",
                        edgecolor=ROBOT_LINE_COLOR, lw=ROBOT_LINE_WIDTH,
                        alpha=0.9, zorder=Z_CIRCLE))
    ax.scatter(x, y, color=marker_color, s=ROBOT_MARKER_SIZE, marker="o", zorder=Z_MARKER)
    if label is not None:
        _number_label(x, y + _ROBOT_RADIUS + 0.10, label, ax)


def draw_polylines(polys, ax, color, label, lw, alpha):
    first = True
    for poly in polys:
        if len(poly) < 2:
            continue
        a = np.array(poly)
        ax.plot(a[:, 0], a[:, 1], color=color, lw=lw, alpha=alpha, zorder=Z_TRAJ,
                label=label if first else None)
        first = False


# ----------------------------------------------------------------------
# Report


def build_report(bag_path):
    if not rclpy.ok():
        rclpy.init()

    data = read_bag(bag_path)
    ref = data["ref"]
    actual = data["actual"]
    status = data["status"]

    t_min, t_max = data["span"]
    duration = _t_seconds(t_max - t_min) if t_min is not None else 0.0
    vlm_time = _union_windows(status, _VLM_STATUS)
    move_time = _union_windows(status, _MOVE_STATUS)
    plan_wins = windows(status, _PLAN_STATUS)
    n_plans = len(plan_wins)
    turn_wins = windows(status, _SPIN_STATUS)
    # Total VLM calls = executor (plan_trajectory) + director (advance) windows.
    n_vlm = sum(len(windows(status, t)) for t in _VLM_STATUS)

    stats = {
        "reference_frame": ref,
        "mission_duration_s": round(duration, 2),
        "vlm_processing_s": round(vlm_time, 2),
        "movement_s": round(move_time, 2),
        "vlm_calls": n_vlm,
        "plan_calls": n_plans,
        "turns": len(turn_wins),
    }

    fig, ax = plt.subplots(figsize=(11, 11))
    if data["occ"] is not None:
        draw_map(data["occ"], ax)

    # VLM paths: raw (intent) under, truncated (followed) over — semi-transparent so
    # overlapping paths remain legible.
    draw_polylines(data["paths_raw"], ax, _C_RAW, "VLM raw path", 1.6, 0.6)
    draw_polylines(data["paths"], ax, _C_CLIP, "Truncated path", 2.2, 0.75)

    # Actual driven path (continuous) — slight alpha so it doesn't fully mask the VLM paths.
    if actual:
        a = np.array([(x, y) for _, x, y, _ in actual])
        ax.plot(a[:, 0], a[:, 1], color=_C_ACTUAL, lw=2.4, alpha=0.8, zorder=Z_ACTUAL,
                label="Actual path")

    # Stopped positions: cluster the VLM calls (executor `plan_trajectory` + director `advance`)
    # in time order by spatial proximity. Each cluster is one place the robot stopped; its
    # number is the TOTAL count of VLM calls made there — both the visual reasoner (advance) and
    # trajectory generation (plan). Consecutive calls within IN_PLACE_EPS are the same place, so
    # in-place scans/turns accumulate onto one footprint instead of stacking overlapping circles.
    IN_PLACE_EPS = 0.15   # metres; VLM calls within this of the cluster are the "same place"
    vlm_calls = sorted(w for topic in _VLM_STATUS for w in windows(status, topic))
    clusters = []   # each: {"x", "y", "yaw" (first-call heading), "count", "arrive_yaw", "spin_yaw"}
    for (t0, _t1) in vlm_calls:
        p = pose_at(actual, t0)
        if p is None:
            continue
        x, y, yaw = p
        if clusters and math.hypot(x - clusters[-1]["x"], y - clusters[-1]["y"]) < IN_PLACE_EPS:
            clusters[-1]["count"] += 1
        else:
            clusters.append({"x": x, "y": y, "yaw": yaw, "count": 1,
                             "arrive_yaw": None, "spin_yaw": None})

    # Attach spins to their cluster (by end pose): a spin never translates, so its start pose is
    # the ARRIVAL heading (before the end-of-move turn) and its end pose is the POST-SPIN heading.
    # First spin at a cluster sets the arrival heading; the last one wins the post-spin heading.
    for (s0, s1) in sorted(turn_wins):
        pe = pose_at(actual, s1)
        if pe is None:
            continue
        ex, ey, eyaw = pe
        near = min(clusters, key=lambda c: math.hypot(ex - c["x"], ey - c["y"]), default=None)
        if near is None or math.hypot(ex - near["x"], ey - near["y"]) >= IN_PLACE_EPS:
            continue
        near["spin_yaw"] = eyaw
        if near["arrive_yaw"] is None:
            ps = pose_at(actual, s0)
            near["arrive_yaw"] = ps[2] if ps is not None else None

    # Fallback: no VLM calls in the bag but a path exists — still mark start/end footprints.
    if not clusters and actual:
        clusters = [{"x": actual[0][1], "y": actual[0][2], "yaw": actual[0][3], "count": 0,
                     "arrive_yaw": actual[0][3], "spin_yaw": None},
                    {"x": actual[-1][1], "y": actual[-1][2], "yaw": actual[-1][3], "count": 0,
                     "arrive_yaw": actual[-1][3], "spin_yaw": None}]

    for i, c in enumerate(clusters):
        # Centre-marker colour: green at the first stop, red at the last, robot colour otherwise.
        marker_color = (_C_START if i == 0 else
                        _C_END if i == len(clusters) - 1 else ROBOT_MARKER_COLOR)
        arrive_yaw = c["arrive_yaw"] if c["arrive_yaw"] is not None else c["yaw"]
        # Arrival heading line — always the robot colour (only the centre marker is green/red),
        # below the circle/marker.
        _heading_line(c["x"], c["y"], arrive_yaw, ax, ROBOT_MARKER_COLOR, Z_HEADING)
        # In-place spin heading line — its own colour, showing the heading AFTER the spin.
        if c["spin_yaw"] is not None and abs(wrap(c["spin_yaw"] - arrive_yaw)) > math.radians(5):
            _heading_line(c["x"], c["y"], c["spin_yaw"], ax, IN_PLACE_TURN_COLOR, Z_SPIN_HEADING)
        draw_footprint(c["x"], c["y"], ax, marker_color, label=str(c["count"]))

    box = (f"Duration: {stats['mission_duration_s']:.1f} s\n"
           f"VLM processing: {stats['vlm_processing_s']:.1f} s\n"
           f"Movement: {stats['movement_s']:.1f} s\n"
           f"VLM calls: {n_vlm}")
    ax.text(0.02, 0.98, box, transform=ax.transAxes, fontsize=10, va="top", ha="left",
            family="monospace", zorder=9,
            bbox=dict(boxstyle="round", fc="white", ec="0.5", alpha=0.85))

    ax.set_xlabel(f"X [m] ({ref})")
    ax.set_ylabel(f"Y [m] ({ref})")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.5)
    # Start / End are footprint markers, not standalone artists — add legend proxies for them.
    handles, _ = ax.get_legend_handles_labels()
    handles += [
        Line2D([0], [0], marker="o", color="w", markerfacecolor=_C_START,
               markersize=10, label="Start"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor=_C_END,
               markersize=10, label="End"),
    ]
    ax.legend(handles=handles, loc="upper right")
    plt.tight_layout()

    mission_dir = os.path.dirname(os.path.abspath(bag_path.rstrip("/")))
    png = os.path.join(mission_dir, "mission_report.png")
    js = os.path.join(mission_dir, "mission_stats.json")
    fig.savefig(png, dpi=150)
    plt.close(fig)
    with open(js, "w") as f:
        json.dump(stats, f, indent=2)

    print(json.dumps(stats, indent=2))
    print(f"\nSaved {png}\nSaved {js}")
    return stats


def main():
    parser = argparse.ArgumentParser(
        description="Compile a mission's MCAP rosbag into a trajectory report + stats.")
    parser.add_argument("bag_path", help="Path to the mission.bag directory")
    args = parser.parse_args()
    if not os.path.isdir(args.bag_path):
        parser.error(f"Not a bag directory: {args.bag_path}")
    build_report(args.bag_path)


if __name__ == "__main__":
    main()
