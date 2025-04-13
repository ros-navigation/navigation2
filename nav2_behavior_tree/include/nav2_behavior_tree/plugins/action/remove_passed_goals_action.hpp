// Copyright (c) 2021 Samsung Research America
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"
#include "nav_msgs/msg/goals.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav2_msgs/msg/waypoint_status.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"


namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase that removes goals that the robot passed near to
 * @note This is an Asynchronous node. It will re-initialize when halted.
 */
class RemovePassedGoals : public BT::ActionNodeBase
{
public:
  RemovePassedGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  static BT::PortsList providedPorts()
  {
    // Register JSON definitions for the types used in the ports
    BT::RegisterJsonDefinition<nav_msgs::msg::Goals>();
    BT::RegisterJsonDefinition<nav2_msgs::msg::WaypointStatus>();
    BT::RegisterJsonDefinition<std::vector<nav2_msgs::msg::WaypointStatus>>();

    return {
      BT::InputPort<nav_msgs::msg::Goals>("input_goals",
          "Original goals to remove viapoints from"),
      BT::OutputPort<nav_msgs::msg::Goals>("output_goals",
          "Goals with passed viapoints removed"),
      BT::InputPort<double>("radius", 0.5, "radius to goal for it to be considered for removal"),
      BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
      BT::InputPort<std::vector<nav2_msgs::msg::WaypointStatus>>("input_waypoint_statuses",
          "Original waypoint_statuses to mark waypoint status from"),
      BT::OutputPort<std::vector<nav2_msgs::msg::WaypointStatus>>("output_waypoint_statuses",
          "Waypoint_statuses with passed waypoints marked")
    };
  }

private:
  void halt() override {}
  BT::NodeStatus tick() override;

  double viapoint_achieved_radius_;
  double transform_tolerance_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string robot_base_frame_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
