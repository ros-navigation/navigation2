// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "gtest/gtest.h"

using nav2_util::geometry_utils::euclidean_distance;
using nav2_util::geometry_utils::calculate_path_length;

TEST(GeometryUtils, euclidean_distance_point_3d)
{
  geometry_msgs::msg::Point point1;
  point1.x = 3.0;
  point1.y = 2.0;
  point1.z = 1.0;

  geometry_msgs::msg::Point point2;
  point2.x = 1.0;
  point2.y = 2.0;
  point2.z = 3.0;

  ASSERT_NEAR(euclidean_distance(point1, point2, true), 2.82843, 1e-5);
}

TEST(GeometryUtils, euclidean_distance_point_2d)
{
  geometry_msgs::msg::Point point1;
  point1.x = 3.0;
  point1.y = 2.0;
  point1.z = 1.0;

  geometry_msgs::msg::Point point2;
  point2.x = 1.0;
  point2.y = 2.0;
  point2.z = 3.0;

  ASSERT_NEAR(euclidean_distance(point1, point2), 2.0, 1e-5);
}

TEST(GeometryUtils, euclidean_distance_pose_3d)
{
  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 7.0;
  pose1.position.y = 4.0;
  pose1.position.z = 3.0;

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 17.0;
  pose2.position.y = 6.0;
  pose2.position.z = 2.0;

  ASSERT_NEAR(euclidean_distance(pose1, pose2, true), 10.24695, 1e-5);
}

TEST(GeometryUtils, euclidean_distance_pose_2d)
{
  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 7.0;
  pose1.position.y = 4.0;
  pose1.position.z = 3.0;

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 17.0;
  pose2.position.y = 6.0;
  pose2.position.z = 2.0;

  ASSERT_NEAR(euclidean_distance(pose1, pose2), 10.19804, 1e-5);
}

TEST(GeometryUtils, calculate_path_length)
{
  nav_msgs::msg::Path straight_line_path;
  size_t nb_path_points = 10;
  float distance_between_poses = 2.0;
  float current_x_loc = 0.0;

  for (size_t i = 0; i < nb_path_points; ++i) {
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.pose.position.x = current_x_loc;

    straight_line_path.poses.push_back(pose_stamped_msg);

    current_x_loc += distance_between_poses;
  }

  ASSERT_NEAR(
    calculate_path_length(straight_line_path),
    (nb_path_points - 1) * distance_between_poses, 1e-5);

  ASSERT_NEAR(
    calculate_path_length(straight_line_path, straight_line_path.poses.size()),
    0.0, 1e-5);

  nav_msgs::msg::Path circle_path;
  float polar_distance = 2.0;
  uint32_t current_polar_angle_deg = 0;
  constexpr float pi = 3.14159265358979;

  while (current_polar_angle_deg != 360) {
    float x_loc = polar_distance * std::cos(current_polar_angle_deg * (pi / 180.0));
    float y_loc = polar_distance * std::sin(current_polar_angle_deg * (pi / 180.0));

    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.pose.position.x = x_loc;
    pose_stamped_msg.pose.position.y = y_loc;

    circle_path.poses.push_back(pose_stamped_msg);

    current_polar_angle_deg += 1;
  }

  ASSERT_NEAR(
    calculate_path_length(circle_path),
    2 * pi * polar_distance, 1e-1);
}
