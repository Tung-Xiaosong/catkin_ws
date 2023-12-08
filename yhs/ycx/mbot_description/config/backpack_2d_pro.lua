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
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map", --地图坐标系名称
  tracking_frame = "base_footprint",--车体坐标系名称base_link
  published_frame = "odom",--base_footprint--车体坐标系名称
  odom_frame = "odom",--里程计坐标系名称
  provide_odom_frame = false,--false--是否发布里程计坐标
  publish_frame_projected_to_2d = true,--是否无滚动、俯仰或z偏移 false
  use_pose_extrapolator = true,--是否使用里程数据  false
  use_odometry = true,--false
  use_nav_sat = false,--是否使用gps
  use_landmarks = false,--是否使用路标
  num_laser_scans = 1,--要订阅的激光扫描主题数。订阅
  num_multi_echo_laser_scans = 0,--要订阅的多回波激光扫描主题数。订阅
  num_subdivisions_per_laser_scan = 1,--将每个接收到的（多回波）激光扫描分割成的点云数量。
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,--发布子图姿势的时间间隔（以秒为单位），例如 0.3 秒。
  pose_publish_period_sec = 5e-3,--发布姿势的间隔（以秒为单位），例如 5e-3 用于 200 Hz 的频率
  trajectory_publish_period_sec = 10e-3,--发布轨迹标记的时间间隔（以秒为单位），例如 30e-3 表示 30 毫秒
  rangefinder_sampling_ratio = 1.,--范围查找器消息的固定比率采样。
  odometry_sampling_ratio = 1.,--里程计消息的固定比率采样。
  fixed_frame_pose_sampling_ratio = 1.,--固定帧消息的固定比率采样。
  imu_sampling_ratio = 1.,--IMU 消息的固定比率采样。
  landmarks_sampling_ratio = 1.,--地标消息的固定比率采样。
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90   --35--改大一点,图上的坐标点少一点
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 --增大体素滤波的大小,减小需要处理的点云数.!!!!!!!!!!!!!!!!!!!!!--
TRAJECTORY_BUILDER_2D.min_range = 0.1  --激光的最近有效距离
TRAJECTORY_BUILDER_2D.max_range = 20.0--激光最远的有效距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.  --无效激光数据设置距离为该数值
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1--积累几帧激光数据作为一个标准单位scan
TRAJECTORY_BUILDER_2D.use_imu_data = false--是否使用imu数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
POSE_GRAPH.optimize_every_n_nodes = 5 --改小

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 2e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 2e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 3e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1.
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1.
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e3

return options
