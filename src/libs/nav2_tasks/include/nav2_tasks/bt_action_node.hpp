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
#include <condition_variable>
#include <mutex>
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

    CommandMsg foo = *command_;

    // Loop until the task has completed
    while (get_status() != BT::HALTED) {
      nav2_tasks::TaskStatus status =
        taskClient_.waitForResult(result_, std::chrono::milliseconds(100));

      switch (status) {
        case nav2_tasks::TaskStatus::SUCCEEDED:
          printf("TaskClient: Tick: received SUCCEEDED message\n");
          return BT::SUCCESS;

        case nav2_tasks::TaskStatus::FAILED:
          printf("TaskClient: Tick: received FAILED message\n");
          return BT::FAILURE;

        case nav2_tasks::TaskStatus::CANCELED:
          printf("TaskClient: Tick: received CANCELED message\n");
          cvCancel_.notify_one();
          return BT::HALTED;

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
    printf("TaskClient: Halt\n");
    taskClient_.cancel();

    std::unique_lock<std::mutex> lock(cancelMutex_);

    printf("TaskClient: waiting for CANCELED message\n");
    cvCancel_.wait(lock);
    printf("TaskClient: after waiting for CANCELED message\n");
  }

private:
  nav2_tasks::TaskClient<CommandMsg, ResultMsg> taskClient_;

  typename CommandMsg::SharedPtr command_;
  typename ResultMsg::SharedPtr result_;

  std::mutex cancelMutex_;
  std::condition_variable cvCancel_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__BT_ACTION_NODE_HPP_
