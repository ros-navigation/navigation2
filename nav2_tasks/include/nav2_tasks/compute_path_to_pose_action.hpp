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

#ifndef NAV2_TASKS__COMPUTE_PATH_TO_POSE_ACTION_HPP_
#define NAV2_TASKS__COMPUTE_PATH_TO_POSE_ACTION_HPP_

#include <string>
#include <memory>
#include "nav2_tasks/bt_conversions.hpp"
#include "nav2_tasks/bt_action_node.hpp"
#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace nav2_tasks
{

class ComputePathToPoseAction
  : public BtActionNode<ComputePathToPoseCommand, ComputePathToPoseResult>
{
public:
  ComputePathToPoseAction(const std::string & action_name, const BT::NodeParameters & params)
  : BtActionNode<ComputePathToPoseCommand, ComputePathToPoseResult>(action_name, params)
  {
    // Get the starting pose information from the XML attributes

    geometry_msgs::msg::Point start_position;
    bool have_start_position =
      getParam<geometry_msgs::msg::Point>("start_position", start_position);

    geometry_msgs::msg::Quaternion start_orientation;
    bool have_start_orientation =
      getParam<geometry_msgs::msg::Quaternion>("start_orientation", start_orientation);

    if (!have_start_position || !have_start_orientation) {
      RCLCPP_ERROR(node_->get_logger(),
        "ComputePathToPoseAction: starting position or orientation not provided");
    }

    // Get the ending pose information from the XML attributes

    geometry_msgs::msg::Point goal_position;
    bool have_goal_position = getParam<geometry_msgs::msg::Point>("goal_position", goal_position);

    geometry_msgs::msg::Quaternion goal_orientation;
    bool have_goal_orientation =
      getParam<geometry_msgs::msg::Quaternion>("goal_orientation", goal_orientation);

    if (!have_goal_position || !have_goal_orientation) {
      RCLCPP_ERROR(node_->get_logger(),
        "ComputePathToPoseAction: goal position or orientation not provided");
    }

    // Create the command message for this task
    command_ = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
    command_->start.position = start_position;
    command_->start.orientation = start_orientation;
    command_->goal.position = goal_position;
    command_->goal.orientation = goal_orientation;

    // Create the result message
    result_ = std::make_shared<nav2_tasks::ComputePathToPoseResult>();
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static const BT::NodeParameters & requiredNodeParameters()
  {
    static BT::NodeParameters params = {
      {"start_position", "0;0;0"}, {"start_orientation", "0;0;0;0"},
      {"goal_position", "0;0;0"}, {"goal_orientation", "0;0;0;0"}};
    return params;
  }
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__COMPUTE_PATH_TO_POSE_ACTION_HPP_
