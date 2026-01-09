// Copyright (c) 2026 Jakub Chudziński
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_NEARBY_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_NEARBY_CONDITION_HPP_

#include <string>
#include <limits>
#include <memory>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when
 * remaining length of the current planned path
 * is less than proximity_threshold and FAILURE otherwise.
 *
 * Use max_robot_pose_search_dist to limit search distance when path updates regularly
 * to reduce computational cost. Disable (set negative) for full path search.
 */
class IsGoalNearbyCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsGoalNearbyCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsGoalNearbyCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsGoalNearbyCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned path"),
      BT::InputPort<double>(
        "proximity_threshold", 1.0,
        "Proximity length (m) of the remaining path considered as a nearby"),
      BT::InputPort<double>(
        "max_robot_pose_search_dist", -1.0,
        "Maximum forward integrated distance along the path "
        "(starting from the last detected pose) to bound the search for the closest pose "
        "to the robot. When set to negative value (default), whole path is searched every time"),
      BT::InputPort<std::string>("global_frame", "map", "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot base frame"),
    };
  }

private:
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Path path_;
  std::vector<geometry_msgs::msg::PoseStamped>::iterator closest_pose_detection_begin_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_NEARBY_CONDITION_HPP_
