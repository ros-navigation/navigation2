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
#include "behavior_tree_core/action_node.h"
#include "behavior_tree_core/bt_factory.h"
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
    rclcpp::Node::SharedPtr node, 
    const std::string & action_name,
    typename CommandMsg::SharedPtr command,
    typename ResultMsg::SharedPtr result,
    std::chrono::milliseconds tick_timeout = std::chrono::milliseconds(100))

  : BT::ActionNode(action_name),
    task_client_(node),
    command_(command),
    result_(result),
    tick_timeout_(tick_timeout)
  {
    // Retrieve the parameter using getParam()
    //Pose2D goal; 
    //bool goal_passed = getParam<Pose2D>("goal", goal);

    //rclcpp::Node::SharedPtr mynode;
    //bool hasNode = blackboard()->template get<rclcpp::Node::SharedPtr>("node", mynode);

    rclcpp::Node::SharedPtr mynode = blackboard()->template get<rclcpp::Node::SharedPtr>("node");
    printf("BtActionNode: node: %p", (void *) mynode.get());

    if (!task_client_->waitForServer(nav2_tasks::defaultServerTimeout)) {
      throw std::runtime_error("BtActionNode: server not running");
    }
  }

  BtActionNode(const std::string & action_name)
  : BT::ActionNode(action_name), task_client_(nullptr)
  {
    printf("bt_action_node: constructor\n");
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
    printf("bt_action_node: destructor\n");
  }

  BT::NodeStatus tick() // override
  {
    printf("bt_action_node: tick\n");

    if (task_client_ == nullptr) {
      rclcpp::Node::SharedPtr mynode = blackboard()->template get<rclcpp::Node::SharedPtr>("node");
      printf("BtActionNode: node: %p", (void *) mynode.get());

      task_client_ = std::make_unique<nav2_tasks::TaskClient<CommandMsg, ResultMsg>>(mynode);
      tick_timeout_ = blackboard()->template get<std::chrono::milliseconds>("tick_timeout");

      command_ = blackboard()->template get<CommandMsgPtr>("command");
      result_ = blackboard()->template get<ResultMsgPtr>("result");
    }

    task_client_->sendCommand(command_);

    printf("bt_action_node: tick: after send command\n");

    // Loop until the task has completed
    while (!isHalted()) {
      nav2_tasks::TaskStatus status = task_client_->waitForResult(result_, tick_timeout_);

      switch (status) {
        case nav2_tasks::TaskStatus::SUCCEEDED:
          return BT::NodeStatus::SUCCESS;

        case nav2_tasks::TaskStatus::FAILED:
          return BT::NodeStatus::FAILURE;

        case nav2_tasks::TaskStatus::CANCELED:
          cvCancel_.notify_one();
          return BT::NodeStatus::IDLE;

        case nav2_tasks::TaskStatus::RUNNING:
          break;

        default:
          throw std::logic_error("BtActionNode::Tick: invalid status value");
      }
    }

    return BT::NodeStatus::IDLE;
  }

  void halt() override
  {
    // Send a cancel message to the task server
    task_client_->cancel();

    // Then wait for the response before continuing
    std::unique_lock<std::mutex> lock(cancelMutex_);
    cvCancel_.wait(lock);
  }

private:
  typename std::unique_ptr<nav2_tasks::TaskClient<CommandMsg, ResultMsg>> task_client_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds tick_timeout_;

  typename CommandMsg::SharedPtr command_;
  typename ResultMsg::SharedPtr result_;

  typedef typename CommandMsg::SharedPtr CommandMsgPtr;
  typedef typename ResultMsg::SharedPtr ResultMsgPtr;

  // Allow for signaling receipt of the cancel message
  std::mutex cancelMutex_;
  std::condition_variable cvCancel_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__BT_ACTION_NODE_HPP_
