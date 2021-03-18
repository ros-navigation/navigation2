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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_ACTION_HPP_

#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

class NavigateThroughPosesAction : public BtActionNode<nav2_msgs::action::NavigateThroughPoses>
{
public:
  NavigateThroughPosesAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goals", "Destinations to plan through"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_THROUGH_POSES_ACTION_HPP_
