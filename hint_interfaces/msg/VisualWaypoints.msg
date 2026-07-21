# Continuous stream of tracked ground waypoints. The point layout depends on the
# tracker publishing it:
#
#   odom_waypoint_tracker (default): `points` are 2D top-down WORLD coordinates in
#   metric base_link (point.x = forward, point.y = left, in metres; z = 0), ordered
#   as the planner/VLM sent them (index 0 first). `in_front` flags whether each
#   waypoint is ahead of the camera (true) or behind it (false). ALL waypoints are
#   published every frame, so a behind waypoint (in_front=false) re-appears (flips
#   true) once the robot turns back to face it. `tracked` carries image-visibility
#   (in front AND inside the camera frame) for reference.
#
#   waypoint_tracker (DIS, legacy image-space backend): `points` are normalized
#   image coordinates (x, y in [-1, 1], center 0); `tracked` = measured vs coasting;
#   `in_front` is left empty.
std_msgs/Header header
geometry_msgs/Point[] points
# Per-waypoint image validity, parallel to points: DIS backend -> measured (true) vs
# coasting (false); odom backend -> visible in the camera frame (in front AND in
# bounds).
bool[] tracked
# Per-waypoint front/back flag, parallel to points (odom backend): true = in front of
# the camera, false = behind (the robot has passed it). Empty for the DIS backend.
bool[] in_front
