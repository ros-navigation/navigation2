// Copyright (c) 2024 Angsa Robotics
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_IN_COLLISION_GOALS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_IN_COLLISION_GOALS_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/action_node.h"
#include "nav2_msgs/srv/get_cost.hpp"

namespace nav2_behavior_tree
{

class RemoveInCollisionGoals : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;

  RemoveInCollisionGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Original goals to remove from"),
      BT::InputPort<std::string>("costmap",
          "Which costmap to use for checking collision. Choices: local, global, both"),
      BT::InputPort<std::string>("cost_threshold",
          "Cost threshold for considering a goal in collision"),
      BT::OutputPort<Goals>("output_goals", "Goals with in-collision goals removed"),
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  bool initialized_;
  std::string which_costmap_;
  double cost_threshold_;
  rclcpp::Client<nav2_msgs::srv::GetCost>::SharedPtr get_global_cost_client_;
  rclcpp::Client<nav2_msgs::srv::GetCost>::SharedPtr get_local_cost_client_;
  std::chrono::milliseconds server_timeout_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_IN_COLLISION_GOALS_ACTION_HPP_
