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
#include <memory>
#include <condition_variable>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_tasks/task_client.hpp"
#include "behavior_tree_core/action_node.h"
#include "behavior_tree_core/bt_factory.h"

namespace nav2_tasks
{

template<class CommandMsg, class ResultMsg>
class BtActionNode : public BT::ActionNode
{
public:
  explicit BtActionNode(const std::string & action_name)
  : BT::ActionNode(action_name), task_client_(nullptr)
  {
  }

  BtActionNode(const std::string & action_name, const BT::NodeParameters & params)
  : BT::ActionNode(action_name, params), task_client_(nullptr)
  {
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  BT::NodeStatus tick() override
  {
    if (task_client_ == nullptr) {
      // Get the required items from the blackboard
      node_ = blackboard()->template get<rclcpp::Node::SharedPtr>("node");
      node_loop_timeout_ = blackboard()->template get<std::chrono::milliseconds>("node_loop_timeout");

      // Now that we have the ROS node to use, create the task client for this action
      task_client_ = std::make_unique<nav2_tasks::TaskClient<CommandMsg, ResultMsg>>(node_);
    }

    task_client_->sendCommand(command_);

    // Loop until the task has completed
    while (!isHalted()) {
      nav2_tasks::TaskStatus status = task_client_->waitForResult(result_, node_loop_timeout_);

      switch (status) {
        case nav2_tasks::TaskStatus::SUCCEEDED:
          return BT::NodeStatus::SUCCESS;

        case nav2_tasks::TaskStatus::FAILED:
          return BT::NodeStatus::FAILURE;

        case nav2_tasks::TaskStatus::CANCELED:
          cv_cancel_.notify_one();
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
    std::unique_lock<std::mutex> lock(cancel_mutex_);
    cv_cancel_.wait(lock);
  }

protected:
  typename std::unique_ptr<nav2_tasks::TaskClient<CommandMsg, ResultMsg>> task_client_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds node_loop_timeout_;

  typename CommandMsg::SharedPtr command_;
  typename ResultMsg::SharedPtr result_;

  typedef typename CommandMsg::SharedPtr CommandMsgPtr;
  typedef typename ResultMsg::SharedPtr ResultMsgPtr;

  // Allow for signaling receipt of the cancel message
  std::mutex cancel_mutex_;
  std::condition_variable cv_cancel_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__BT_ACTION_NODE_HPP_
