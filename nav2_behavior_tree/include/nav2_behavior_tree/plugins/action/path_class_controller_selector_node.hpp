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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_CLASS_CONTROLLER_SELECTOR_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_CLASS_CONTROLLER_SELECTOR_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/classified_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief Selects the controller mapped to the class of the pose a lookahead
 * arc length ahead of the robot on the path
 */
class PathClassControllerSelector : public BT::SyncActionNode
{
public:
  PathClassControllerSelector(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path being followed"),
      BT::InputPort<std::vector<nav2_msgs::msg::ClassifiedPose>>(
        "classified_path", "Classified poses of the path"),
      BT::InputPort<std::string>(
        "class_controller_map", "'class:controller' pairs separated by ';'"),
      BT::InputPort<std::string>(
        "default_controller", "Controller used when no class is mapped"),
      BT::InputPort<double>(
        "lookahead_arc_dist", 1.0, "Arc length ahead of the robot to check"),
      BT::InputPort<double>(
        "max_robot_pose_search_dist", 1.0, "Arc length to search for the closest pose"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::OutputPort<std::string>("selected_controller", "Selected controller")
    };
  }

private:
  BT::NodeStatus tick() override;

  void parseClassControllerMap(const std::string & map_str);

  size_t findClosestPoseIdx(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    size_t start_idx) const;

  size_t findLookaheadPoseIdx(const nav_msgs::msg::Path & path, size_t closest_idx) const;

  BT::NodeStatus keepSelectedController();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;

  double lookahead_arc_dist_;
  double max_robot_pose_search_dist_;
  std::string default_controller_;
  std::unordered_map<uint16_t, std::string> class_to_controller_;

  nav_msgs::msg::Path last_path_;
  size_t closest_idx_{0};
  std::string selected_controller_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PATH_CLASS_CONTROLLER_SELECTOR_NODE_HPP_
