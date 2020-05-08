// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#ifndef NAV2_BEHAVIOR_TREE__TIME_EXPIRED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__TIME_EXPIRED_CONDITION_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

class TimeExpiredCondition : public BT::ConditionNode
{
public:
  TimeExpiredCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
    first_time_(true)
  {
    double hz = 1.0;
    getInput("hz", hz);
    period_ = 1.0 / hz;
  }

  TimeExpiredCondition() = delete;

  BT::NodeStatus tick() override
  {
    if (first_time_) {
      start_ = std::chrono::high_resolution_clock::now();
      first_time_ = false;
      return BT::NodeStatus::SUCCESS;
    }

    // Determine how long its been since we've started this iteration
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = now - start_;

    // Now, get that in seconds
    typedef std::chrono::duration<float> float_seconds;
    auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

    if (seconds.count() < period_) {
      return BT::NodeStatus::FAILURE;
    }

    start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
    return BT::NodeStatus::SUCCESS;
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hz", 10.0, "Rate")
    };
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
  bool first_time_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TimeExpiredCondition>("TimeExpired");
}

#endif  // NAV2_BEHAVIOR_TREE__TIME_EXPIRED_CONDITION_HPP_
