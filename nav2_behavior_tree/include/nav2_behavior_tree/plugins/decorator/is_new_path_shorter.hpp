// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2022 Pradheep Padmanabhan - Neobotix GmbH
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__IS_NEW_PATH_SHORTER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__IS_NEW_PATH_SHORTER_HPP_

#include <string>
#include <memory>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorators/retry_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child everytime when the length of
 * the new path is smaller than the old one by the length given by the user.
 */
class IsNewPathShorter : public BT::RetryNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsNewPathShorter
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsNewPathShorter(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to follow"),
      BT::InputPort<double>(
        "allowed_goal_distance_proximity", 1.0,
        "Distance proximity of the robot to the goal")
    };
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;
  bool child_status;

private:
  nav_msgs::msg::Path new_path;
  nav_msgs::msg::Path old_path;
  double allowed_goal_distance_proximity = std::numeric_limits<double>::max();
  rclcpp::Node::SharedPtr node_;
  bool first_time_ = false;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__IS_NEW_PATH_SHORTER_HPP_
