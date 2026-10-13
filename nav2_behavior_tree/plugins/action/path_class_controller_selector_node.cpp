// Copyright (c) 2026 Origin
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

#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_behavior_tree/plugins/action/path_class_controller_selector_node.hpp"

namespace nav2_behavior_tree
{

using nav2_util::geometry_utils::euclidean_distance;
using nav2_util::geometry_utils::first_after_integrated_distance;

PathClassControllerSelector::PathClassControllerSelector(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf),
  transform_tolerance_(0.1),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  lookahead_arc_dist_(1.0),
  max_robot_pose_search_dist_(1.0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("lookahead_arc_dist", lookahead_arc_dist_);
  getInput("max_robot_pose_search_dist", max_robot_pose_search_dist_);
  getInput("default_controller", default_controller_);

  std::string map_str;
  getInput("class_controller_map", map_str);
  parseClassControllerMap(map_str);
}

void PathClassControllerSelector::parseClassControllerMap(const std::string & map_str)
{
  std::stringstream pairs(map_str);
  std::string pair;
  while (std::getline(pairs, pair, ';')) {
    if (pair.empty()) {
      continue;
    }
    const auto sep = pair.find(':');
    if (sep == std::string::npos || sep == 0 || sep == pair.size() - 1) {
      throw std::runtime_error(
              "PathClassControllerSelector: invalid class_controller_map entry '" + pair + "'");
    }
    try {
      const auto class_type = static_cast<uint16_t>(std::stoi(pair.substr(0, sep)));
      class_to_controller_[class_type] = pair.substr(sep + 1);
    } catch (const std::logic_error &) {
      throw std::runtime_error(
              "PathClassControllerSelector: invalid class in class_controller_map entry '" +
              pair + "'");
    }
  }
}

BT::NodeStatus PathClassControllerSelector::tick()
{
  nav_msgs::msg::Path path;
  std::vector<nav2_msgs::msg::ClassifiedPose> classified_path;
  getInput("path", path);
  getInput("classified_path", classified_path);

  if (path.poses.empty()) {
    if (default_controller_.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("selected_controller", default_controller_);
    return BT::NodeStatus::SUCCESS;
  }

  // The path is replanned before it is classified, so they can briefly differ
  if (classified_path.size() != path.poses.size()) {
    return keepSelectedController();
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN(node_->get_logger(), "PathClassControllerSelector: failed to get robot pose");
    return keepSelectedController();
  }

  if (path != last_path_) {
    closest_idx_ = 0;
    last_path_ = path;
  }
  closest_idx_ = findClosestPoseIdx(path, robot_pose, closest_idx_);

  const uint16_t lookahead_class =
    classified_path[findLookaheadPoseIdx(path, closest_idx_)].class_type;
  const auto it = class_to_controller_.find(lookahead_class);
  const std::string & controller =
    it != class_to_controller_.end() ? it->second : default_controller_;

  if (controller.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "PathClassControllerSelector: class %u is not mapped and no default controller is set",
      lookahead_class);
    return BT::NodeStatus::FAILURE;
  }

  if (controller != selected_controller_) {
    RCLCPP_INFO(
      node_->get_logger(), "PathClassControllerSelector: selecting controller %s for class %u",
      controller.c_str(), lookahead_class);
    selected_controller_ = controller;
  }

  setOutput("selected_controller", selected_controller_);
  return BT::NodeStatus::SUCCESS;
}

size_t PathClassControllerSelector::findClosestPoseIdx(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  size_t start_idx) const
{
  size_t closest_idx = start_idx;
  double closest_dist = euclidean_distance(robot_pose, path.poses[start_idx]);
  double arc_dist = 0.0;
  for (size_t i = start_idx + 1; i < path.poses.size(); ++i) {
    arc_dist += euclidean_distance(path.poses[i - 1], path.poses[i]);
    if (arc_dist > max_robot_pose_search_dist_) {
      break;
    }
    const double dist = euclidean_distance(robot_pose, path.poses[i]);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

size_t PathClassControllerSelector::findLookaheadPoseIdx(
  const nav_msgs::msg::Path & path, size_t closest_idx) const
{
  auto lookahead = first_after_integrated_distance(
    path.poses.begin() + closest_idx, path.poses.end(), lookahead_arc_dist_);
  if (lookahead == path.poses.end()) {
    return path.poses.size() - 1;
  }
  return lookahead - path.poses.begin();
}

BT::NodeStatus PathClassControllerSelector::keepSelectedController()
{
  const std::string & controller =
    selected_controller_.empty() ? default_controller_ : selected_controller_;
  if (controller.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("selected_controller", controller);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathClassControllerSelector>(
    "PathClassControllerSelector");
}
