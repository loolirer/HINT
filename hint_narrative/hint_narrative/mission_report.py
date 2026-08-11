import argparse
import json
import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")  # headless — we only save a PNG
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

# action_msgs/msg/GoalStatus constants
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
_PLAN_STATUS = "/path_planner/visual_reason/_action/status"  # one window per planned move
_SPIN_STATUS = "/spin/_action/status"  # the BT sends one per cycle, 0 deg included

_PATH_TOPIC = "/path_projector_node/path"

PATH_ALPHA = 0.70  # Opacity of ALL displayed paths, shared
ENDPOINT_MARKER_SIZE = 90  # start/end dot
ENDPOINT_MARKER = "o"  # start/end marker on the driven path
REFERENCE_ENDPOINT_MARKER = "s"  # start/end on the reference path — a square tells the two apart

LENGTH_STEP = 0.05  # m — hops shorter than this don't extend the measured length, so the pose
#                     jitter every AMCL correction injects can't accumulate into phantom distance
MOVING_WINDOW = 0.25  # s — window a trajectory's speed is measured over when deciding "moving"
MOVING_SPEED_EPS = 0.02  # m/s — slower than this over MOVING_WINDOW counts as stopped
MOVING_YAW_RATE_EPS = 0.05  # rad/s — an in-place turn is still movement

VIEW_PADDING = 1.0  # m of clear margin kept around the outermost trajectory point — the knob that
#                     crops a map far larger than the run down to the run itself
VIEW_MAX_ASPECT = 2.0  # widest the crop may run (width:height); a slimmer one grows its short side
FIG_WIDTH = 11.0  # in — figure width; the height follows the cropped view, so the PNG is landscape

Z_REFERENCE = 3.4  # reference Nav2 GoToGoal path — below the actual path
Z_ACTUAL = 3.5  # actual driven path
Z_TRAJ = 3.6  # VLM (planned) path — dashed, above the actual path
Z_MARKER = 6.0  # start/end markers, on top of every path

_C_ACTUAL = "#1f4fd8"  # blue — the path actually driven
_C_REFERENCE = "#0a8f4f"  # green — the Nav2 GoToGoal reference (target) trajectory
_C_CLIP = "#bd5b00"  # amber — vlm path
_C_START = "green"  # start: first point of a trajectory
_C_END = "red"  # end: last point of a trajectory


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
                if slot[0] is None:
                    # First sight, whatever the status: a goal that accepts and finishes between
                    # two publications is never observed active, and keying the window's start on
                    # an active status would drop it from every duration and count.
                    slot[0] = t
                if gs.status in _STATUS_TERMINAL and slot[1] is None:
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


def _xy(samples):
    return [(x, y) for _, x, y, _ in samples]


def path_length(points, step=0.0):
    """Arc length of an (x, y) polyline; `step` drops sub-`step` hops before they accumulate."""
    total, anchor = 0.0, None
    for p in points:
        if anchor is None:
            anchor = p
            continue
        d = math.hypot(p[0] - anchor[0], p[1] - anchor[1])
        if d >= step:
            total += d
            anchor = p
    return total


def moving_time(samples):
    """Seconds a trajectory was actually in motion — derived from the poses themselves, since a
    reference bag carries no action topics to take windows from."""
    total, anchor = 0.0, None
    for s in samples:
        t = _t_seconds(s[0])
        if anchor is None:
            anchor = (t, s[1], s[2], s[3])
            continue
        dt = t - anchor[0]
        if dt < MOVING_WINDOW:
            continue
        moved = math.hypot(s[1] - anchor[1], s[2] - anchor[2])
        turned = abs(wrap(s[3] - anchor[3]))
        if moved / dt >= MOVING_SPEED_EPS or turned / dt >= MOVING_YAW_RATE_EPS:
            total += dt
        anchor = (t, s[1], s[2], s[3])
    return total


def draw_map(occ, ax, flip=False):
    res = occ.info.resolution
    w, h = occ.info.width, occ.info.height
    ox, oy = occ.info.origin.position.x, occ.info.origin.position.y
    grid = np.array(occ.data, dtype=np.int16).reshape((h, w))
    disp = np.full((h, w), 200, dtype=np.uint8)  # unknown -> mid gray
    known = grid >= 0
    disp[known] = (255 - grid[known] * 2.55).astype(np.uint8)
    extent = [ox, ox + w * res, oy, oy + h * res]
    if flip:
        disp = np.rot90(disp)
        extent = [extent[2], extent[3], -extent[1], -extent[0]]
    ax.imshow(disp, cmap="gray", origin="lower", zorder=1, extent=extent)


def draw_endpoints(samples, ax, marker):
    for sample, color in ((samples[0], _C_START), (samples[-1], _C_END)):
        ax.scatter(
            sample[1],
            sample[2],
            color=color,
            s=ENDPOINT_MARKER_SIZE,
            marker=marker,
            zorder=Z_MARKER,
        )


def _endpoint_handle(marker, color, label):
    return Line2D(
        [0], [0], marker=marker, color="w", markerfacecolor=color, markersize=10, label=label
    )


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


def _flip_point(x, y):
    return y, -x


def _flip_samples(samples):
    return [(t, y, -x, wrap(yaw - math.pi / 2)) for t, x, y, yaw in samples]


def _flip_polys(polys):
    return [[_flip_point(x, y) for x, y in poly] for poly in polys]


def _trajectory_points(actual, reference, polys):
    pts = [(x, y) for _, x, y, _ in actual]
    pts += [(x, y) for _, x, y, _ in (reference or ())]
    for poly in polys:
        pts.extend(poly)
    return pts


def _is_portrait(points):
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    return (max(ys) - min(ys)) > (max(xs) - min(xs))


def _view_box(points):
    """Crop window: VIEW_PADDING around the outermost points, never slimmer than VIEW_MAX_ASPECT."""
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    x0, x1 = min(xs) - VIEW_PADDING, max(xs) + VIEW_PADDING
    y0, y1 = min(ys) - VIEW_PADDING, max(ys) + VIEW_PADDING
    if x1 - x0 < 1.0:  # a run that never left one spot still needs a window to draw in
        cx = (x0 + x1) / 2.0
        x0, x1 = cx - 0.5, cx + 0.5
    grow = ((x1 - x0) / VIEW_MAX_ASPECT - (y1 - y0)) / 2.0
    if grow > 0.0:
        y0, y1 = y0 - grow, y1 + grow
    return x0, x1, y0, y1


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
    paths = data["paths"]

    points = _trajectory_points(actual, reference, paths)
    flip = bool(points) and _is_portrait(points)
    if flip:
        actual = _flip_samples(actual)
        reference = _flip_samples(reference) if reference else reference
        paths = _flip_polys(paths)
        points = [_flip_point(x, y) for x, y in points]
    view = _view_box(points) if points else None

    t_min, t_max = data["span"]
    duration = _t_seconds(t_max - t_min) if t_min is not None else 0.0
    vlm_time = _union_windows(status, (_ADVANCE_STATUS,))
    move_time = _union_windows(status, _MOVE_STATUS)
    plan_wins = windows(status, _PLAN_STATUS)
    n_plans = len(plan_wins)
    spin_goals = len(windows(status, _SPIN_STATUS))
    vlm_ok, vlm_total = hit_rate(status, _VLM_STATUS)

    stats = {
        "reference_frame": ref,
        "mission_duration_s": round(duration, 2),
        "vlm_processing_s": round(vlm_time, 2),
        "movement_s": round(move_time, 2),
        "geometric_movement_s": round(moving_time(reference), 2) if reference else None,
        "semantic_path_length_m": round(path_length(_xy(actual), LENGTH_STEP), 2),
        "geometric_path_length_m": (
            round(path_length(_xy(reference), LENGTH_STEP), 2) if reference else None
        ),
        "vlm_path_length_m": round(sum(path_length(p) for p in paths), 2),
        "vlm_successful_calls": vlm_ok,
        "vlm_total_calls": vlm_total,
        "plan_calls": n_plans,
        "spin_goals": spin_goals,  # Spin actions sent, not turns actually made
    }

    fig_height = FIG_WIDTH * (view[3] - view[2]) / (view[1] - view[0]) if view else FIG_WIDTH
    fig, ax = plt.subplots(figsize=(FIG_WIDTH, fig_height))
    if data["occ"] is not None:
        draw_map(data["occ"], ax, flip)

    draw_polylines(paths, ax, _C_CLIP, "Caminho (VLM)", 2.2, PATH_ALPHA, linestyle="--")

    if reference:
        r = np.array([(x, y) for _, x, y, _ in reference])
        ax.plot(
            r[:, 0],
            r[:, 1],
            color=_C_REFERENCE,
            lw=2.4,
            alpha=PATH_ALPHA,
            zorder=Z_REFERENCE,
            linestyle=":",
            label="Caminho (Nav2)",
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
            label="Caminho (Semântico)",
        )

    if reference:
        draw_endpoints(reference, ax, REFERENCE_ENDPOINT_MARKER)
    if actual:
        draw_endpoints(actual, ax, ENDPOINT_MARKER)

    lines = [
        f"Tempo de Cognição (Semântico): {stats['vlm_processing_s']:.1f} s",
        f"Tempo de Movimento (Semântico): {stats['movement_s']:.1f} s",
    ]
    if reference:
        lines.append(
            f"Tempo de Movimento (Geométrico): {stats['geometric_movement_s']:.1f} s"
        )
    lines.append(f"Comprimento (Semântico): {stats['semantic_path_length_m']:.2f} m")
    if reference:
        lines.append(
            f"Comprimento (Geométrico): {stats['geometric_path_length_m']:.2f} m"
        )
    box = "\n".join(lines)
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

    # A flipped view is the frame rotated -90°, so the screen axes are the frame's (Y, -X).
    ax.set_xlabel(f"{'Y' if flip else 'X'} [m] ({ref})")
    ax.set_ylabel(f"{'-X' if flip else 'Y'} [m] ({ref})")
    ax.set_aspect("equal", adjustable="box")
    if view:
        ax.set_xlim(view[0], view[1])
        ax.set_ylim(view[2], view[3])
    ax.grid(True, linestyle="--", alpha=0.5)

    handles, _ = ax.get_legend_handles_labels()
    handles += [
        _endpoint_handle(ENDPOINT_MARKER, _C_START, "Início (Semântico)"),
        _endpoint_handle(ENDPOINT_MARKER, _C_END, "Fim (Semântico)"),
    ]
    if reference:
        handles += [
            _endpoint_handle(REFERENCE_ENDPOINT_MARKER, _C_START, "Início (Geométrico)"),
            _endpoint_handle(REFERENCE_ENDPOINT_MARKER, _C_END, "Fim (Geométrico)"),
        ]
    ax.legend(handles=handles, loc="upper right")
    plt.tight_layout()

    mission_dir = os.path.dirname(os.path.abspath(bag_path.rstrip("/")))
    png = os.path.join(mission_dir, "mission_report.png")
    js = os.path.join(mission_dir, "mission_stats.json")
    fig.savefig(png, dpi=150, bbox_inches="tight")
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
