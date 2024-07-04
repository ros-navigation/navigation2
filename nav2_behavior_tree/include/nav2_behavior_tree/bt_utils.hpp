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

#ifndef NAV2_BEHAVIOR_TREE__BT_UTILS_HPP_
#define NAV2_BEHAVIOR_TREE__BT_UTILS_HPP_

#include <string>
#include <set>

#include "rclcpp/time.hpp"
#include "rclcpp/node.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace BT
{

// The follow templates are required when using these types as parameters
// in our BT XML files. They parse the strings in the XML into their corresponding
// data type.

/**
 * @brief Parse XML string to geometry_msgs::msg::Point
 * @param key XML string
 * @return geometry_msgs::msg::Point
 */
template<>
inline geometry_msgs::msg::Point convertFromString(const StringView key)
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

/**
 * @brief Parse XML string to geometry_msgs::msg::Quaternion
 * @param key XML string
 * @return geometry_msgs::msg::Quaternion
 */
template<>
inline geometry_msgs::msg::Quaternion convertFromString(const StringView key)
{
  // four real numbers separated by semicolons
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

/**
 * @brief Parse XML string to geometry_msgs::msg::PoseStamped
 * @param key XML string
 * @return geometry_msgs::msg::PoseStamped
 */
template<>
inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key)
{
  // 7 real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 9) {
    throw std::runtime_error("invalid number of fields for PoseStamped attribute)");
  } else {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(BT::convertFromString<int64_t>(parts[0]));
    pose_stamped.header.frame_id = BT::convertFromString<std::string>(parts[1]);
    pose_stamped.pose.position.x = BT::convertFromString<double>(parts[2]);
    pose_stamped.pose.position.y = BT::convertFromString<double>(parts[3]);
    pose_stamped.pose.position.z = BT::convertFromString<double>(parts[4]);
    pose_stamped.pose.orientation.x = BT::convertFromString<double>(parts[5]);
    pose_stamped.pose.orientation.y = BT::convertFromString<double>(parts[6]);
    pose_stamped.pose.orientation.z = BT::convertFromString<double>(parts[7]);
    pose_stamped.pose.orientation.w = BT::convertFromString<double>(parts[8]);
    return pose_stamped;
  }
}

/**
 * @brief Parse XML string to std::chrono::milliseconds
 * @param key XML string
 * @return std::chrono::milliseconds
 */
template<>
inline std::chrono::milliseconds convertFromString<std::chrono::milliseconds>(const StringView key)
{
  return std::chrono::milliseconds(std::stoul(key.data()));
}

/**
 * @brief Parse XML string to std::set<int>
 * @param key XML string
 * @return std::set<int>
 */
template<>
inline std::set<int> convertFromString(StringView key)
{
  // Real numbers separated by semicolons
  auto parts = splitString(key, ';');

  std::set<int> set;
  for (const auto part : parts) {
    set.insert(convertFromString<int>(part));
  }
  return set;
}

/**
 * @brief Return parameter value from behavior tree node or ros2 parameter file
 * @param node rclcpp::Node::SharedPtr
 * @param param_name std::string
 * @param behavior_tree_node T2
 * @return <T1>
 */
template<typename T1, typename T2>
T1 deconflictPortAndParamFrame(
  rclcpp::Node::SharedPtr node,
  std::string param_name,
  T2 * behavior_tree_node)
{
  T1 param_value;
  if (!behavior_tree_node->getInput(param_name, param_value)) {
    RCLCPP_DEBUG(
      node->get_logger(),
      "Parameter '%s' not provided by behavior tree xml file, "
      "using parameter from ros2 parameter file",
      param_name.c_str());
    node->get_parameter(param_name, param_value);
    return param_value;
  } else {
    RCLCPP_DEBUG(
      node->get_logger(),
      "Parameter '%s' provided by behavior tree xml file",
      param_name.c_str());
    return param_value;
  }
}

}  // namespace BT

#endif  // NAV2_BEHAVIOR_TREE__BT_UTILS_HPP_
