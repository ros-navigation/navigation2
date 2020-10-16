// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

class TimeExpiredCondition : public BT::ConditionNode
{
public:
  TimeExpiredCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  TimeExpiredCondition() = delete;

  BT::NodeStatus tick() override;

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("seconds", 1.0, "Seconds")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_;
  double period_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TIME_EXPIRED_CONDITION_HPP_
