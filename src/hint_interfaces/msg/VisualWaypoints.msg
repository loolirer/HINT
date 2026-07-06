# Continuous stream of tracked ground waypoints in the image, mirroring the
# SetWaypoints / PlanTrajectory.markers layout: geometry_msgs/Point[] in
# normalized image space (point.x, point.y in [-1, 1], center = 0; z unused),
# ordered nearest -> farthest.
std_msgs/Header header
geometry_msgs/Point[] points
# Per-waypoint validity, parallel to points: true = measured this frame,
# false = coasting on odometry.
bool[] tracked
