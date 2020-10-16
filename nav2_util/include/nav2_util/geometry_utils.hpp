// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__GEOMETRY_UTILS_HPP_
#define NAV2_UTIL__GEOMETRY_UTILS_HPP_

#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace nav2_util
{
namespace geometry_utils
{

inline geometry_msgs::msg::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return tf2::toMsg(q);
}

inline double euclidean_distance(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;
  double dz = pos1.z - pos2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double euclidean_distance(
  const geometry_msgs::msg::Pose & pos1,
  const geometry_msgs::msg::Pose & pos2)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;
  double dz = pos1.position.z - pos2.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline double euclidean_distance(
  const geometry_msgs::msg::PoseStamped & pos1,
  const geometry_msgs::msg::PoseStamped & pos2)
{
  return euclidean_distance(pos1.pose, pos2.pose);
}

}  // namespace geometry_utils
}  // namespace nav2_util

#endif  // NAV2_UTIL__GEOMETRY_UTILS_HPP_
