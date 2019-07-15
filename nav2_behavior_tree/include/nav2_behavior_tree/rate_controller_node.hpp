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

#ifndef NAV2_BEHAVIOR_TREE__RATE_CONTROLLER_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__RATE_CONTROLLER_NODE_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace nav2_behavior_tree
{

class RateController : public BT::DecoratorNode
{
public:
  RateController(const std::string & name, const BT::NodeParameters & params)
  : BT::DecoratorNode(name, params)
  {
    double hz = 1.0;
    getParam<double>("hz", hz);
    period_ = 1.0 / hz;
  }

  // Any BT node that accepts parameters must provide a requiredNodeParameters method
  static const BT::NodeParameters & requiredNodeParameters()
  {
    static BT::NodeParameters params = {{"hz", "10"}};
    return params;
  }

private:
  BT::NodeStatus tick() override;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
};

static bool first_time{false};

inline BT::NodeStatus RateController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting point since we're starting a new iteration of
    // the rate controller (moving from IDLE to RUNNING)
    start_ = std::chrono::high_resolution_clock::now();
    first_time = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // Determine how long its been since we've started this iteration
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  // Now, get that in seconds
  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  // If we've exceed the specified period, execute the child node
  if (first_time || seconds.count() >= period_) {
    first_time = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        child_node_->setStatus(BT::NodeStatus::IDLE);
        start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        child_node_->setStatus(BT::NodeStatus::IDLE);
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__RATE_CONTROLLER_NODE_HPP_
