// Copyright (c) 2024 Open Navigation LLC
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

#ifndef OPENNAV_DOCKING__UTILS_HPP_
#define OPENNAV_DOCKING__UTILS_HPP_

#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "angles/angles.h"
#include "opennav_docking/types.hpp"
#include "opennav_docking_core/charging_dock.hpp"
#include "tf2/utils.hpp"

namespace utils
{

using rclcpp::ParameterType::PARAMETER_STRING;
using rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
using rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
using nav2_util::geometry_utils::orientationAroundZAxis;
using opennav_docking_core::DockDirection;

/**
* @brief Parse a yaml file to obtain docks
* @param yaml_file A yaml file path with docks
* @param node Node to use for logging
* @param dock_db Database to populate
*/
inline bool parseDockFile(
  const std::string & yaml_filepath,
  const nav2::LifecycleNode::SharedPtr & node,
  DockMap & dock_db)
{
  YAML::Node yaml_file;
  try {
    yaml_file = YAML::LoadFile(yaml_filepath);
  } catch (...) {
    return false;
  }

  if (!yaml_file["docks"]) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Dock database (%s) does not contain 'docks'.", yaml_filepath.c_str());
    return false;
  }

  auto yaml_docks = yaml_file["docks"];
  Dock curr_dock;
  for (const auto & yaml_dock : yaml_docks) {
    std::string dock_name = yaml_dock.first.as<std::string>();
    const YAML::Node & dock_attribs = yaml_dock.second;

    curr_dock.frame = "map";
    if (dock_attribs["frame"]) {
      curr_dock.frame = dock_attribs["frame"].as<std::string>();
    }

    if (!dock_attribs["type"]) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Dock database (%s) entries do not contain 'type'.", yaml_filepath.c_str());
      return false;
    }
    curr_dock.type = dock_attribs["type"].as<std::string>();

    if (!dock_attribs["pose"]) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Dock database (%s) entries do not contain 'pose'.", yaml_filepath.c_str());
      return false;
    }
    std::vector<double> pose_arr = dock_attribs["pose"].as<std::vector<double>>();
    if (pose_arr.size() != 3u) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Dock database (%s) entries do not contain pose of size 3.", yaml_filepath.c_str());
      return false;
    }
    curr_dock.pose.position.x = pose_arr[0];
    curr_dock.pose.position.y = pose_arr[1];
    curr_dock.pose.orientation = orientationAroundZAxis(pose_arr[2]);

    if (dock_attribs["id"]) {
      curr_dock.id = dock_attribs["id"].as<std::string>();
    }

    // Insert into dock instance database
    dock_db.emplace(dock_name, curr_dock);
  }

  return true;
}

/**
* @brief Parse a parameter file to obtain docks
* @param docks_param Parameter dock namespaces to parse
* @param node Node to use for logging & getting parameters
* @param dock_db Database to populate
*/
inline bool parseDockParams(
  const std::vector<std::string> & docks_param,
  const nav2::LifecycleNode::SharedPtr & node,
  DockMap & dock_db)
{
  Dock curr_dock;
  std::vector<double> pose_arr;
  for (const auto & dock_name : docks_param) {
    curr_dock.frame = node->declare_or_get_parameter(dock_name + ".frame", std::string("map"));

    try {
      curr_dock.type = node->declare_or_get_parameter<std::string>(dock_name + ".type");
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Dock %s has no dock 'type'.", dock_name.c_str());
      return false;
    }

    pose_arr.clear();
    try {
      pose_arr = node->declare_or_get_parameter<std::vector<double>>(dock_name + ".pose");
      if (pose_arr.size() != 3u) {
        throw std::runtime_error("Dock pose is incorrect size!");
      }
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Dock %s has no valid 'pose'.", dock_name.c_str());
      return false;
    }

    curr_dock.pose.position.x = pose_arr[0];
    curr_dock.pose.position.y = pose_arr[1];
    curr_dock.pose.orientation = orientationAroundZAxis(pose_arr[2]);

    curr_dock.id = node->declare_or_get_parameter(dock_name + ".id", std::string(""));

    // Insert into dock instance database
    dock_db.emplace(dock_name, curr_dock);
  }
  return true;
}

/**
* @brief Convert dock into a pose stamped message
* @param dock Dock to convert
* @param t Time for stamping
* @return PoseStamped of dock pose at time t
*/
inline geometry_msgs::msg::PoseStamped getDockPoseStamped(
  const Dock * dock, const rclcpp::Time & t)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose = dock->pose;
  pose.header.frame_id = dock->frame;
  pose.header.stamp = t;
  return pose;
}

inline double l2Norm(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  double angle_a = tf2::getYaw(a.orientation);
  double angle_b = tf2::getYaw(b.orientation);
  double delta_angle = angles::shortest_angular_distance(angle_a, angle_b);
  return sqrt(
    (a.position.x - b.position.x) * (a.position.x - b.position.x) +
    (a.position.y - b.position.y) * (a.position.y - b.position.y) +
    delta_angle * delta_angle);
}

inline DockDirection getDockDirectionFromString(const std::string & direction)
{
  auto upper_direction = direction;
  std::transform(
    upper_direction.begin(), upper_direction.end(), upper_direction.begin(), ::toupper);

  if (upper_direction == "FORWARD") {
    return DockDirection::FORWARD;
  } else if (upper_direction == "BACKWARD") {
    return DockDirection::BACKWARD;
  } else {
    return DockDirection::UNKNOWN;
  }
}

}  // namespace utils

#endif  // OPENNAV_DOCKING__UTILS_HPP_
