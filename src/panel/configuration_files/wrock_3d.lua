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

include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  sensor_bridge = {
    horizontal_laser_min_range = 0.,
    horizontal_laser_max_range = 30.,
    horizontal_laser_missing_echo_ray_length = 5.,
    constant_odometry_translational_variance = 0.,
    constant_odometry_rotational_variance = 0.,
  },
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry_data = false,
  use_horizontal_laser = false,
  use_horizontal_multi_echo_laser = false,
  num_lasers_3d = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
}


MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 30

MAX_3D_LASER_RANGE = 3.5
TRAJECTORY_BUILDER_3D.laser_min_range = 0.1
TRAJECTORY_BUILDER_3D.laser_max_range = MAX_3D_LASER_RANGE
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = MAX_3D_LASER_RANGE
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 500
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = MAX_3D_LASER_RANGE
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 250
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = MAX_3D_LASER_RANGE

TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.25
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.035
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.2

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_cost_functor_weight_0 = 10.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_cost_functor_weight_1 = 15.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.previous_pose_translation_delta_cost_functor_weight = 4.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.initial_pose_estimate_rotation_delta_cost_functor_weight = 1e3
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.covariance_scale = 1e-3
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = true

SPARSE_POSE_GRAPH.constraint_builder.sampling_ratio = 0.2
SPARSE_POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
SPARSE_POSE_GRAPH.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter
SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.48
SPARSE_POSE_GRAPH.constraint_builder.log_matches = true
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 2.

SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 1e1
SPARSE_POSE_GRAPH.optimization_problem.acceleration_scale = 1e-1
SPARSE_POSE_GRAPH.optimization_problem.rotation_scale = 1e3

return options
