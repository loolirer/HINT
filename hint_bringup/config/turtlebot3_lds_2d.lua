-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- HINT-owned, VISUALIZATION-ONLY cartographer config. A copy of turtlebot3's
-- turtlebot3_lds_2d.lua, lightened for a viz map that does NOT feed navigation.
-- Point cartographer here with `cartographer_config_dir` + `configuration_basename`
-- (see hint_bringup/launch/cartographer.launch.py) so the upstream submodule stays
-- untouched. Cartographer is not tunable via ROS params — this Lua file IS its
-- parameter interface; the `include`s below resolve to cartographer_ros's own
-- built-in config dir.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 1.0,   -- viz-only: 1 Hz submaps instead of ~3 Hz
  pose_publish_period_sec = 20e-3,   -- viz-only: 50 Hz map->odom TF is plenty (was 200 Hz)
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- Drop the real-time correlative scan matcher: it brute-force searches a window
-- on every scan (the single biggest per-scan CPU cost). Odometry seeds the motion
-- and the Ceres matcher still refines, so for a viz map this is a big, safe saving.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
-- Insert a node only after real motion, so near-duplicate scans don't pile work on
-- the graph. Was 0.1 deg (almost every scan) -> 1 deg / 10 cm / 0.5 s.
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- Viz-only + not in the nav pipeline: disable the pose-graph optimizer (loop
-- closure + global bundle adjustment) — the heaviest background CPU consumer.
-- Cartographer then runs as pure local SLAM. Trade-off: no loop closure, so the
-- map can drift a little when you revisit a place — fine for a small-area viz map.
-- (Want occasional loop closure instead? Use a high value like 320, not 0.)
POSE_GRAPH.optimize_every_n_nodes = 0

return options
