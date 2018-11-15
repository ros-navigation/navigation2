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
  
#ifndef NAV2_TASKS__RATE_CONTROLLER_NODE_HPP_
#define NAV2_TASKS__RATE_CONTROLLER_NODE_HPP_

#include <chrono>
#include "behavior_tree_core/decorator_node.h"

namespace nav2_tasks
{

class RateController : public BT::DecoratorNode
{
public:
  RateController(const std::string& name)
  : BT::DecoratorNode(name, BT::NodeParameters())
  {
  }

private:
  virtual BT::NodeStatus tick() override;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

inline BT::NodeStatus RateController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    setStatus(BT::NodeStatus::RUNNING);
    start_ = std::chrono::high_resolution_clock::now();
    return status();
  }

  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_);

  if (elapsed.count() >= 2) {
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state)
    {
      case BT::NodeStatus::SUCCESS:
        child_node_->setStatus(BT::NodeStatus::IDLE);
        printf("RateController: child has returned SUCCESS\n");

        // Reset the timer
        start_ = std::chrono::high_resolution_clock::now();
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::FAILURE:
      default:
        printf("RateController: child has failed\n");
        child_node_->setStatus(BT::NodeStatus::IDLE);
        return BT::NodeStatus::RUNNING;
    }
  }

  return status();
}

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__RATE_CONTROLLER_NODE_HPP_
