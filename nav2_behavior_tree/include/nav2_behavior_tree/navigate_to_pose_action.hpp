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

#ifndef NAV2_BEHAVIOR_TREE__NAVIGATE_TO_POSE_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__NAVIGATE_TO_POSE_ACTION_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

class NavigateToPoseAction : public BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToPoseAction(const std::string & action_name, const BT::NodeParameters & params)
  : BtActionNode<nav2_msgs::action::NavigateToPose>(action_name, params)
  {
  }

  void on_tick() override
  {
    // Use the position and orientation fields from the XML attributes to initialize the goal
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Quaternion orientation;

    bool have_position = getParam<geometry_msgs::msg::Point>("position", position);
    bool have_orientation = getParam<geometry_msgs::msg::Quaternion>("orientation", orientation);

    if (!have_position || !have_orientation) {
      RCLCPP_ERROR(node_->get_logger(),
        "NavigateToPoseAction: position or orientation not provided");
    }

    goal_.pose.pose.position = position;
    goal_.pose.pose.orientation = orientation;
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static const BT::NodeParameters & requiredNodeParameters()
  {
    static BT::NodeParameters params = {{"position", "0;0;0"}, {"orientation", "0;0;0;0"}};
    return params;
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__NAVIGATE_TO_POSE_ACTION_HPP_
