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

#ifndef NAV2_TASKS__BT_ACTION_NODE_HPP_
#define NAV2_TASKS__BT_ACTION_NODE_HPP_

#include <string>
#include <chrono>
#include "BTpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "nav2_tasks/task_client.hpp"

namespace nav2_tasks
{

template<class CommandMsg, class ResultMsg>
class BtActionNode : public BT::ActionNode
{
public:
  BtActionNode(
    rclcpp::Node * node, const std::string & actionName,
    typename CommandMsg::SharedPtr command,
    typename ResultMsg::SharedPtr result)
  : BT::ActionNode(actionName),
    node_(node),
    taskClient_(node),
    command_(command),
    result_(result)
  {
    if (!taskClient_.waitForServer(nav2_tasks::defaultServerTimeout)) {
      throw std::runtime_error("BtActionNode: server not running");
	  }
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  BT::ReturnStatus Tick()
  {
    taskClient_.sendCommand(command_);

    // Loop until the subtasks are completed
    while (get_status() != BT::HALTED) {
      // Check if the planning task has completed
      nav2_tasks::TaskStatus status =
        taskClient_.waitForResult(result_, std::chrono::milliseconds(100));

      switch (status) {
        case nav2_tasks::TaskStatus::SUCCEEDED:
          // RCLCPP_INFO(node_->get_logger(), "BtActionNode::executeAsync: task completed");
          return BT::SUCCESS;

        case nav2_tasks::TaskStatus::FAILED:
          return BT::FAILURE;

        case nav2_tasks::TaskStatus::RUNNING:
          break;

        default:
          throw std::logic_error("BtActionNode::Tick: invalid status value");
      }
    }

    return BT::HALTED;
  }

  void Halt()
  {
#if 0
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task
    if (cancelRequested()) {
      RCLCPP_INFO(get_logger(), "SimpleNavigator::executeAsync: task has been canceled");
      planner_->cancel();
      setCanceled();
      return TaskStatus::CANCELED;
    }
#endif
  }

private:
  rclcpp::Node * node_;

  nav2_tasks::TaskClient<CommandMsg, ResultMsg> taskClient_;

  typename CommandMsg::SharedPtr command_;
  typename ResultMsg::SharedPtr result_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__BT_ACTION_NODE_HPP_
