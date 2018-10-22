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
    // Create the input and output messages
    command_ = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
    result_ = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

    // Use the position and orientation fields from the BT node parameter
    geometry_msgs::msg::Point position;
    bool position_passed = getParam<geometry_msgs::msg::Point>("position", position);

    geometry_msgs::msg::Quaternion orientation;
    bool orientation_passed = getParam<geometry_msgs::msg::Quaternion>("orientation", orientation);

    if (!position_passed || !orientation_passed) {
      RCLCPP_ERROR(node_->get_logger(),
        "ComputePathToPoseAction: position or orientation not provided");
    }

    // TODO(mjeronimo): should push the starting pose down to the global/local planners
    command_->start.position.x = 0;
    command_->start.position.y = 0;
    command_->start.position.z = 0;
    command_->start.orientation.x = 0;
    command_->start.orientation.y = 0;
    command_->start.orientation.z = 0;
    command_->start.orientation.w = 0;

    command_->goal.position = position;
    command_->goal.orientation = orientation;
    command_->tolerance = 2.0;  // TODO(mjeronimo): should also be a parameter
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static const BT::NodeParameters & requiredNodeParameters()
  {
    static BT::NodeParameters params = {{"position", "0;0;0"}, {"orientation", "0;0;0;0"}};
    return params;
  }
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__COMPUTE_PATH_TO_POSE_ACTION_HPP_
