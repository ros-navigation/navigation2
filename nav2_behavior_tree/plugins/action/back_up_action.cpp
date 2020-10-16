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

#ifndef NAV2_BEHAVIOR_TREE__BACK_UP_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__BACK_UP_ACTION_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/back_up.hpp"

namespace nav2_behavior_tree
{

class BackUpAction : public BtActionNode<nav2_msgs::action::BackUp>
{
public:
  BackUpAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::BackUp>(xml_tag_name, action_name, conf)
  {
    double dist;
    getInput("backup_dist", dist);
    double speed;
    getInput("backup_speed", speed);

    // silently fix, vector direction determined by distance sign
    if (speed < 0.0) {
      speed *= -1.0;
    }

    // Populate the input message
    goal_.target.x = dist;
    goal_.target.y = 0.0;
    goal_.target.z = 0.0;
    goal_.speed = speed;
  }

  void on_tick() override
  {
    increment_recovery_count();
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("backup_dist", -0.15, "Distance to backup"),
        BT::InputPort<double>("backup_speed", 0.025, "Speed at which to backup")
      });
  }
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::BackUpAction>(
        name, "back_up", config);
    };

  factory.registerBuilder<nav2_behavior_tree::BackUpAction>("BackUp", builder);
}

#endif  // NAV2_BEHAVIOR_TREE__BACK_UP_ACTION_HPP_
