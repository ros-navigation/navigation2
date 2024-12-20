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
#include "nav2_msgs/msg/pose_stamped_array.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/get_costs.hpp"

namespace nav2_behavior_tree
{

class RemoveInCollisionGoals : public BtServiceNode<nav2_msgs::srv::GetCosts>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RemoveInCollisionGoals
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  RemoveInCollisionGoals(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  BT::NodeStatus on_completion(std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response)
  override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav2_msgs::msg::PoseStampedArray>("input_goals",
          "Original goals to remove from"),
        BT::InputPort<double>(
          "cost_threshold", 254.0,
          "Cost threshold for considering a goal in collision"),
        BT::InputPort<bool>("use_footprint", true, "Whether to use footprint cost"),
        BT::InputPort<bool>(
          "consider_unknown_as_obstacle", false,
          "Whether to consider unknown cost as obstacle"),
        BT::OutputPort<nav2_msgs::msg::PoseStampedArray>("output_goals",
          "Goals with in-collision goals removed"),
      });
  }

private:
  bool use_footprint_;
  bool consider_unknown_as_obstacle_;
  double cost_threshold_;
  nav2_msgs::msg::PoseStampedArray input_goals_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_IN_COLLISION_GOALS_ACTION_HPP_
