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

#ifndef NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
#define NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_

#include <string>

#include "rclcpp/time.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace BT
{
/// @brief  Custom type
struct Vector4D
{
    double x{0.0}, y{0.0}, z{0.0}, w{0.0};
};

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
 * @brief Parse XML string to custom Vector4D
 * by comma-separated-values.
 * FORMAT: X,Y,Z,W;X,Y,Z,W; ...
 * @param key XML string
 * @return Vector4D
 */
template <> inline
Vector4D convertFromString(StringView key)
{
    // three real numbers separated by colons
    auto parts = BT::splitString(key, ',');
    if (parts.size() != 4)
    {
      throw BT::RuntimeError("invalid input)");
    }
    else
    {
      Vector4D output;
      output.x = convertFromString<double>(parts[0]);
      output.y = convertFromString<double>(parts[1]);
      output.z = convertFromString<double>(parts[2]);
      output.w = convertFromString<double>(parts[3]);
      return output; 
    }
}

/**
 * @brief Parse XML string to custom std::vector of Vector4D
 * by comma-separated-values.
 * FORMAT: X,Y,Z,W;X,Y,Z,W; ...
 * @param key XML string
 * @return std::vector<Vector4D>
 */
template <> inline
std::vector<Vector4D> convertFromString(StringView key)
{
    std::vector<Vector4D> output_vector;
    auto parts_vector = BT::splitString(key, ';');

    for (auto& part_vector : parts_vector)
    {
      Vector4D output = convertFromString<Vector4D>(part_vector);
      output_vector.push_back(output);
    }
    return output_vector; 

}

/**
 * @brief Parse XML string to custom std::vector of string
 * by colon-separated-values.
 * FORMAT: str;str; ...
 * @param key XML string
 * @return std::vector<std::string>
 */
template <> inline
std::vector<std::string> convertFromString(StringView key)
{
    std::vector<std::string> output_vector;
    auto parts_vector = BT::splitString(key, ';');

    for (auto& part_vector : parts_vector)
    {
      std::string output = convertFromString<std::string>(part_vector);
      output_vector.push_back(output);
    }
    return output_vector; 

}

}  // namespace BT

#endif  // NAV2_BEHAVIOR_TREE__BT_CONVERSIONS_HPP_
