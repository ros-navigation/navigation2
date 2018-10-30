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

#ifndef NAV2_TASKS__NAVIGATE_TO_POSE_ACTION_HPP_
#define NAV2_TASKS__NAVIGATE_TO_POSE_ACTION_HPP_

#include <string>
#include <memory>
#include "nav2_tasks/bt_conversions.hpp"
#include "nav2_tasks/bt_action_node.hpp"
#include "nav2_tasks/navigate_to_pose_task.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace nav2_tasks
{

class NavigateToPoseAction : public BtActionNode<NavigateToPoseCommand, NavigateToPoseResult>
{
public:
  NavigateToPoseAction(const std::string & action_name, const BT::NodeParameters & params)
  : BtActionNode<NavigateToPoseCommand, NavigateToPoseResult>(action_name, params)
  {
    // Use the position and orientation fields from the XML attributes
    geometry_msgs::msg::Point position;
    bool have_position = getParam<geometry_msgs::msg::Point>("position", position);

    geometry_msgs::msg::Quaternion orientation;
    bool have_orientation = getParam<geometry_msgs::msg::Quaternion>("orientation", orientation);

    if (!have_position || !have_orientation) {
      RCLCPP_ERROR(node_->get_logger(),
        "NavigateToPoseAction: position or orientation not provided");
    }

    // Create the command message for this task
    command_ = std::make_shared<nav2_tasks::NavigateToPoseCommand>();
    command_->pose.position = position;
    command_->pose.orientation = orientation;

    // Create the result message
    result_ = std::make_shared<nav2_tasks::NavigateToPoseResult>();
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static const BT::NodeParameters & requiredNodeParameters()
  {
    static BT::NodeParameters params = {{"position", "0;0;0"}, {"orientation", "0;0;0;0"}};
    return params;
  }
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__NAVIGATE_TO_POSE_ACTION_HPP_
