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
_STATUS_ACTIVE = (1, 2)  # ACCEPTED, EXECUTING
_STATUS_TERMINAL = (4, 5, 6)  # SUCCEEDED, CANCELED, ABORTED

_VLM_STATUS = (
    "/visual_reasoner/visual_reason/_action/status",  # director VLM call (compile)
    "/path_planner/visual_reason/_action/status",  # executor VLM call (plan)
)

_ADVANCE_STATUS = "/narrative_navigation/mission_advance/_action/status"
_MOVE_STATUS = (
    "/path_projector_node/follow_visual_path/_action/status",
    "/spin/_action/status",
)
_PLAN_STATUS = "/path_planner/visual_reason/_action/status"  # footprint anchors
_SPIN_STATUS = "/spin/_action/status"  # turn markers

_PATH_TOPIC = "/path_projector_node/path"

_ROBOT_RADIUS = 0.20  # local_costmap robot_radius (footprint circle)

PATH_ALPHA = 0.70  # Opacity of ALL displayed paths (actual + truncated), shared
ROBOT_MARKER_SIZE = 90  # The marker inside each robot footprint
ROBOT_LINE_WIDTH = 2.6  # Footprint circle border + heading-line width
ROBOT_LINE_COLOR = "black"  # Footprint circle outline
ROBOT_MARKER_COLOR = "black"
IN_PLACE_TURN_COLOR = "#c02060"
ROBOT_FILL = False  # fill the footprint circle? (False = hollow outline)
ROBOT_FILL_COLOR = "#8a8a8a"  # footprint circle fill — grey (used when ROBOT_FILL)
SHOW_ROBOT = False  # draw the robot glyph (footprint circle + arrival/spin heading lines) at each
#                    stop; False keeps only the numbered sequence labels

Z_REFERENCE = 3.4  # reference Nav2 GoToGoal path — below the actual path
Z_ACTUAL = 3.5  # actual driven path
Z_TRAJ = 3.6  # VLM (followed) path — dashed, above the actual path
Z_HEADING = 3.7  # arrival heading line (above paths, below the circle)
Z_SPIN_HEADING = 3.8  # in-place post-spin heading line
Z_CIRCLE = 4.0  # footprint circle
Z_MARKER = 6.0  # centre marker (always on top of the circle)
Z_LABEL = 10.0  # stop-sequence-index label

_C_ACTUAL = "#1f4fd8"  # blue — the path actually driven
_C_REFERENCE = "#0a8f4f"  # green — the Nav2 GoToGoal reference (target) trajectory
_C_CLIP = "#bd5b00"  # amber — vlm path
_C_START = "green"  # start: centre marker of the first stopped position
_C_END = "red"  # end: centre marker of the last stopped position


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _t_seconds(t_nanos):
    return t_nanos * 1e-9


def _open(bag_path):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="mcap"),
        ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_map


def _compose_up(child, transforms, ref):
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


def _point_up(px, py, frame, transforms, ref):
    x, y = px, py
    seen = set()
    while frame != ref:
        if frame in seen or frame not in transforms:
            return None
        seen.add(frame)
        parent, tx, ty, pyaw = transforms[frame]
        c, s = math.cos(pyaw), math.sin(pyaw)
        x, y = c * x - s * y + tx, s * x + c * y + ty
        frame = parent
    return x, y


def read_bag(bag_path):
    reader, type_map = _open(bag_path)
    msg_types = {}

    def mtype(topic):
        if topic not in msg_types:
            msg_types[topic] = get_message(type_map[topic])
        return msg_types[topic]

    has_map = "/map" in type_map and any(
        "OccupancyGrid" in type_map.get(t, "") for t in ("/map",)
    )
    transforms = {}  # child -> (parent, tx, ty, yaw)
    parents = set()  # every parent frame seen on /tf(/static)
    tf_samples = []  # (t, x, y, yaw) base_link in ref (filled after ref known)
    tf_raw = []  # (t, [(parent, child, tx, ty, yaw), ...]) to replay
    odom_samples = []  # (t, x, y, yaw) fallback
    paths = {_PATH_TOPIC: []}  # deduped polylines
    seen_paths = {_PATH_TOPIC: set()}
    status = {}  # topic -> {goal_id: [t0, t1, terminal_status]}
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
            poly = tuple(
                (round(ps.pose.position.x, 3), round(ps.pose.position.y, 3))
                for ps in pm.poses
            )
            if poly and poly not in seen_paths[topic]:
                seen_paths[topic].add(poly)
                paths[topic].append(
                    {
                        "frame": pm.header.frame_id.lstrip("/"),
                        "poly": [(x, y) for x, y in poly],
                        "tf": dict(transforms),
                    }
                )

        elif topic.endswith("/_action/status"):
            arr = deserialize_message(data, mtype(topic))
            win = status.setdefault(topic, {})
            for gs in arr.status_list:
                gid = bytes(gs.goal_info.goal_id.uuid)
                slot = win.setdefault(gid, [None, None, None])
                if gs.status in _STATUS_ACTIVE and slot[0] is None:
                    slot[0] = t
                elif gs.status in _STATUS_TERMINAL and slot[1] is None:
                    slot[1] = t
                    slot[2] = gs.status  # SUCCEEDED (4) vs CANCELED/ABORTED (5/6)

        elif topic == "/map":
            occ = deserialize_message(data, mtype(topic))

    ref = "map" if ("map" in parents or has_map) else "odom"

    def resolve_paths(entries):
        out = []
        for e in entries:
            pts = []
            for x, y in e["poly"]:
                q = _point_up(x, y, e["frame"], e["tf"], ref)
                if q is None:
                    pts = list(e["poly"])
                    break
                pts.append(q)
            out.append(pts)
        return out

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
        "paths": resolve_paths(paths[_PATH_TOPIC]),
        "status": status,
        "span": (t_min, t_max),
    }


def windows(status, topic):
    out = []
    for slot in status.get(topic, {}).values():
        t0, t1 = slot[0], slot[1]
        if t0 is not None and t1 is not None and t1 >= t0:
            out.append((_t_seconds(t0), _t_seconds(t1)))
    return sorted(out)


def hit_rate(status, topics):
    ok = total = 0
    for topic in topics:
        for slot in status.get(topic, {}).values():
            st = slot[2] if len(slot) > 2 else None
            if st in (4, 5, 6):
                total += 1
                if st == 4:
                    ok += 1
    return ok, total


def _union_windows(status, topics):
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
    if not actual:
        return None
    arr = min(actual, key=lambda s: abs(_t_seconds(s[0]) - t_sec))
    return arr[1], arr[2], arr[3]


def draw_map(occ, ax):
    res = occ.info.resolution
    w, h = occ.info.width, occ.info.height
    ox, oy = occ.info.origin.position.x, occ.info.origin.position.y
    grid = np.array(occ.data, dtype=np.int16).reshape((h, w))
    disp = np.full((h, w), 200, dtype=np.uint8)  # unknown -> mid gray
    known = grid >= 0
    disp[known] = (255 - grid[known] * 2.55).astype(np.uint8)
    ax.imshow(
        disp,
        cmap="gray",
        origin="lower",
        zorder=1,
        extent=[ox, ox + w * res, oy, oy + h * res],
    )


def _heading_line(x, y, yaw, ax, color, zorder):
    ax.plot(
        [x, x + _ROBOT_RADIUS * math.cos(yaw)],
        [y, y + _ROBOT_RADIUS * math.sin(yaw)],
        color=color,
        lw=ROBOT_LINE_WIDTH,
        alpha=1.0,
        zorder=zorder,
        solid_capstyle="round",
    )


def _number_label(x, y, text, ax):
    ax.text(
        x,
        y,
        text,
        fontsize=11,
        color="black",
        ha="center",
        va="center",
        zorder=Z_LABEL,
        fontweight="bold",
        bbox=dict(
            boxstyle="circle,pad=0.35", fc="white", ec="black", lw=1.4, alpha=0.95
        ),
    )


def draw_footprint(x, y, ax, marker_color=ROBOT_MARKER_COLOR, label=None, marker=True):
    if SHOW_ROBOT:
        ax.add_patch(
            Circle(
                (x, y),
                _ROBOT_RADIUS,
                facecolor=ROBOT_FILL_COLOR if ROBOT_FILL else "none",
                edgecolor=ROBOT_LINE_COLOR,
                lw=ROBOT_LINE_WIDTH,
                alpha=1.0,
                zorder=Z_CIRCLE,
            )
        )
    if marker:
        ax.scatter(
            x, y, color=marker_color, s=ROBOT_MARKER_SIZE, marker="o", zorder=Z_MARKER
        )
    if label is not None:
        label_y = y + _ROBOT_RADIUS + 0.10 if (SHOW_ROBOT or marker) else y
        _number_label(x, label_y, label, ax)


def draw_polylines(polys, ax, color, label, lw, alpha, linestyle="-"):
    first = True
    for poly in polys:
        if len(poly) < 2:
            continue
        a = np.array(poly)
        ax.plot(
            a[:, 0],
            a[:, 1],
            color=color,
            lw=lw,
            alpha=alpha,
            zorder=Z_TRAJ,
            linestyle=linestyle,
            label=label if first else None,
        )
        first = False


def build_report(bag_path, reference_path=None):
    if not rclpy.ok():
        rclpy.init()

    data = read_bag(bag_path)
    ref = data["ref"]
    actual = data["actual"]
    status = data["status"]

    # The reference (Nav2 GoToGoal) run localizes AMCL on the SAME saved map as the HINT run,
    # so its trajectory shares this report's map frame and overlays directly.
    reference = read_bag(reference_path)["actual"] if reference_path else None

    t_min, t_max = data["span"]
    duration = _t_seconds(t_max - t_min) if t_min is not None else 0.0
    vlm_time = _union_windows(status, (_ADVANCE_STATUS,))
    move_time = _union_windows(status, _MOVE_STATUS)
    plan_wins = windows(status, _PLAN_STATUS)
    n_plans = len(plan_wins)
    turn_wins = windows(status, _SPIN_STATUS)
    vlm_ok, vlm_total = hit_rate(status, _VLM_STATUS)

    stats = {
        "reference_frame": ref,
        "mission_duration_s": round(duration, 2),
        "vlm_processing_s": round(vlm_time, 2),
        "movement_s": round(move_time, 2),
        "vlm_successful_calls": vlm_ok,
        "vlm_total_calls": vlm_total,
        "plan_calls": n_plans,
        "turns": len(turn_wins),
    }

    fig, ax = plt.subplots(figsize=(11, 11))
    if data["occ"] is not None:
        draw_map(data["occ"], ax)

    draw_polylines(
        data["paths"], ax, _C_CLIP, "VLM path", 2.2, PATH_ALPHA, linestyle="--"
    )

    if reference:
        r = np.array([(x, y) for _, x, y, _ in reference])
        ax.plot(
            r[:, 0],
            r[:, 1],
            color=_C_REFERENCE,
            lw=2.4,
            alpha=PATH_ALPHA,
            zorder=Z_REFERENCE,
            label="Reference (Nav2)",
        )

    if actual:
        a = np.array([(x, y) for _, x, y, _ in actual])
        ax.plot(
            a[:, 0],
            a[:, 1],
            color=_C_ACTUAL,
            lw=2.4,
            alpha=PATH_ALPHA,
            zorder=Z_ACTUAL,
            label="Actual path",
        )

    IN_PLACE_EPS = (
        _ROBOT_RADIUS / 2
    )  # VLM calls within this of the cluster are the "same place"
    vlm_calls = sorted(w for topic in _VLM_STATUS for w in windows(status, topic))
    clusters = (
        []
    )  # each: {"x", "y", "yaw" (first-call heading), "arrive_yaw", "spin_yaw"}
    for t0, _t1 in vlm_calls:
        p = pose_at(actual, t0)
        if p is None:
            continue
        x, y, yaw = p
        if (
            clusters
            and math.hypot(x - clusters[-1]["x"], y - clusters[-1]["y"]) < IN_PLACE_EPS
        ):
            continue  # same stop — no new footprint, no sequence advance
        clusters.append(
            {"x": x, "y": y, "yaw": yaw, "arrive_yaw": None, "spin_yaw": None}
        )

    for s0, s1 in sorted(turn_wins):
        pe = pose_at(actual, s1)
        if pe is None:
            continue
        ex, ey, eyaw = pe
        near = min(
            clusters, key=lambda c: math.hypot(ex - c["x"], ey - c["y"]), default=None
        )
        if near is None or math.hypot(ex - near["x"], ey - near["y"]) >= IN_PLACE_EPS:
            continue
        near["spin_yaw"] = eyaw
        if near["arrive_yaw"] is None:
            ps = pose_at(actual, s0)
            near["arrive_yaw"] = ps[2] if ps is not None else None

    if not clusters and actual:
        clusters = [
            {
                "x": actual[0][1],
                "y": actual[0][2],
                "yaw": actual[0][3],
                "arrive_yaw": actual[0][3],
                "spin_yaw": None,
            },
            {
                "x": actual[-1][1],
                "y": actual[-1][2],
                "yaw": actual[-1][3],
                "arrive_yaw": actual[-1][3],
                "spin_yaw": None,
            },
        ]

    for i, c in enumerate(clusters):
        is_endpoint = i == 0 or i == len(clusters) - 1
        marker_color = (
            _C_START
            if i == 0
            else _C_END if i == len(clusters) - 1 else ROBOT_MARKER_COLOR
        )
        arrive_yaw = c["arrive_yaw"] if c["arrive_yaw"] is not None else c["yaw"]
        if SHOW_ROBOT:
            _heading_line(c["x"], c["y"], arrive_yaw, ax, ROBOT_MARKER_COLOR, Z_HEADING)

            if c["spin_yaw"] is not None and abs(
                wrap(c["spin_yaw"] - arrive_yaw)
            ) > math.radians(5):
                _heading_line(
                    c["x"], c["y"], c["spin_yaw"], ax, IN_PLACE_TURN_COLOR, Z_SPIN_HEADING
                )
        draw_footprint(
            c["x"], c["y"], ax, marker_color, label=str(i),
            marker=SHOW_ROBOT or is_endpoint,
        )

    box = (
        f"Duration: {stats['mission_duration_s']:.1f} s\n"
        f"VLM processing: {stats['vlm_processing_s']:.1f} s\n"
        f"Movement: {stats['movement_s']:.1f} s\n"
        f"VLM Hit Rate: {vlm_ok}/{vlm_total}"
    )
    ax.text(
        0.02,
        0.98,
        box,
        transform=ax.transAxes,
        fontsize=10,
        va="top",
        ha="left",
        family="monospace",
        zorder=9,
        bbox=dict(boxstyle="round", fc="white", ec="0.5", alpha=0.85),
    )

    ax.set_xlabel(f"X [m] ({ref})")
    ax.set_ylabel(f"Y [m] ({ref})")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.5)

    handles, _ = ax.get_legend_handles_labels()
    handles += [
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor=_C_START,
            markersize=10,
            label="Start",
        ),
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor=_C_END,
            markersize=10,
            label="End",
        ),
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
        description="Compile a mission's MCAP rosbag into a path report + stats."
    )
    parser.add_argument("bag_path", help="Path to the mission.bag directory")
    parser.add_argument(
        "--reference",
        default=None,
        help="Optional reference Nav2 GoToGoal bag to overlay as the target trajectory "
             "(sibling of the map, e.g. hint_navigation/maps/<region>/reference.bag)",
    )
    args = parser.parse_args()
    if not os.path.isdir(args.bag_path):
        parser.error(f"Not a bag directory: {args.bag_path}")
    if args.reference is not None and not os.path.isdir(args.reference):
        parser.error(f"Not a bag directory: {args.reference}")
    build_report(args.bag_path, args.reference)


if __name__ == "__main__":
    main()
