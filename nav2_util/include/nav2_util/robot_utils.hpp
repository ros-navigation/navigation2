// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Steven Macenski
// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_UTIL__ROBOT_UTILS_HPP_
#define NAV2_UTIL__ROBOT_UTILS_HPP_

#include <string>
#include <memory>
#include <algorithm>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  tf2_ros::Buffer & tf_buffer, const std::string global_frame = "map",
  const std::string robot_frame = "base_link", const double transform_timeout = 0.1);

}  // end namespace nav2_util

#endif  // NAV2_UTIL__ROBOT_UTILS_HPP_
