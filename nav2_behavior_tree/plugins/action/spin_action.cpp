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

#ifndef NAV2_BEHAVIOR_TREE__SPIN_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__SPIN_ACTION_HPP_

#include <string>
#include <memory>
#include <cmath>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace nav2_behavior_tree
{

class SpinAction : public BtActionNode<nav2_msgs::action::Spin>
{
public:
  SpinAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<nav2_msgs::action::Spin>(xml_tag_name, action_name, conf)
  {
    double dist;
    getInput("spin_dist", dist);
    goal_.target_yaw = dist;
  }

  void on_tick() override
  {
    increment_recovery_count();
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("spin_dist", 1.57, "Spin distance")
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
      return std::make_unique<nav2_behavior_tree::SpinAction>(name, "spin", config);
    };

  factory.registerBuilder<nav2_behavior_tree::SpinAction>("Spin", builder);
}

#endif  // NAV2_BEHAVIOR_TREE__SPIN_ACTION_HPP_
