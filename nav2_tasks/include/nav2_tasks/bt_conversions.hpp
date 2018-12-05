// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_TASKS__BT_CONVERSIONS_HPP_
#define NAV2_TASKS__BT_CONVERSIONS_HPP_

#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard/blackboard.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/msg/path.hpp"
#include "nav2_msgs/msg/path_end_points.hpp"

namespace BT
{

// The following four conversion functions are required to be defined by the BT library,
// but are not actually called. TODO(mjeronimo): See if we can avoid these.

template<>
inline rclcpp::Node::SharedPtr convertFromString(const StringView & /*key*/)
{
  return nullptr;
}

template<>
inline std::chrono::milliseconds convertFromString(const StringView & /*key*/)
{
  return std::chrono::milliseconds(0);
}

template<>
inline nav2_msgs::msg::Path::SharedPtr convertFromString(const StringView & /*key*/)
{
  return nullptr;
}

template<>
inline nav2_msgs::msg::PathEndPoints::SharedPtr convertFromString(const StringView & /*key*/)
{
  return nullptr;
}

// These are needed to be able to set parameters for these types in the BT XML

template<>
inline geometry_msgs::msg::Point convertFromString(const StringView & key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3) {
    throw std::runtime_error("invalid number of fields for point attribute)");
  } else {
    geometry_msgs::msg::Point position;
    position.x = BT::convertFromString<double>(parts[0]);
    position.y = BT::convertFromString<double>(parts[1]);
    position.z = BT::convertFromString<double>(parts[2]);
    return position;
  }
}

template<>
inline geometry_msgs::msg::Quaternion convertFromString(const StringView & key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 4) {
    throw std::runtime_error("invalid number of fields for orientation attribute)");
  } else {
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = BT::convertFromString<double>(parts[0]);
    orientation.y = BT::convertFromString<double>(parts[1]);
    orientation.z = BT::convertFromString<double>(parts[2]);
    orientation.w = BT::convertFromString<double>(parts[3]);
    return orientation;
  }
}

}  // namespace BT

#endif  // NAV2_TASKS__BT_CONVERSIONS_HPP_
