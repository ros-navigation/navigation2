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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_

#include <string>

#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/path.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace nav2_behavior_tree
{

class ComputePathThroughPosesAction : public BtActionNode<nav2_msgs::action::ComputePathThroughPoses>
{
public:
  ComputePathThroughPosesAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathThroughPoses node"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goals", "Destinations to plan through"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start", "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>("planner_id", ""),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__COMPUTE_PATH_THROUGH_POSES_ACTION_HPP_
