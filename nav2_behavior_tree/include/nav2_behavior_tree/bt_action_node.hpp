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

#ifndef NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_behavior_tree
{

template<class ActionT>
class BtActionNode : public BT::CoroActionNode
{
public:
  explicit BtActionNode(const std::string & action_name)
  : BT::CoroActionNode(action_name), action_name_(action_name)
  {
  }

  BtActionNode(const std::string & action_name, const BT::NodeParameters & params)
  : BT::CoroActionNode(action_name, params), action_name_(action_name)
  {
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  // This is a callback from the BT library invoked after the node is created and after the
  // blackboard has been set for the node by the library. It is the first opportunity for
  // the node to access the blackboard. Derived classes do not override this method,
  // but override on_init instead.
  void onInit() final
  {
    node_ = blackboard()->template get<rclcpp::Node::SharedPtr>("node");

    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    // Get the required items from the blackboard
    node_loop_timeout_ =
      blackboard()->template get<std::chrono::milliseconds>("node_loop_timeout");

    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name_);

    // Make sure the server is actually there before continuing
    RCLCPP_INFO(node_->get_logger(), "Waiting for \"%s\" action server", action_name_.c_str());
    action_client_->wait_for_action_server();

    // Give the derive class a chance to do any initialization
    on_init();
    RCLCPP_INFO(node_->get_logger(), "\"%s\" BtActionNode initialized", action_name_.c_str());
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_init, on_tick, on_loop_timeout, and on_success

  // Perform any local initialization such as getting values from the blackboard
  virtual void on_init()
  {
  }

  // Could do dynamic checks, such as getting updates to values on the blackboard
  virtual void on_tick()
  {
  }

  // There can be many loop iterations per tick. Any opportunity to do something after
  // a timeout waiting for a result that hasn't been received yet
  virtual void on_loop_timeout()
  {
  }

  // Called upon successful completion of the action. A derived class can override this
  // method to put a value on the blackboard, for example
  virtual void on_success()
  {
  }

  // The main override required by a BT action
  BT::NodeStatus tick() override
  {
    on_tick();

    // Enable result awareness by providing an empty lambda function
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback = [](auto) {};

new_goal_received:
    auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);
    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("send_goal failed");
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      throw std::runtime_error("Goal was rejected by the action server");
    }

    auto future_result = goal_handle_->async_result();
    rclcpp::executor::FutureReturnCode rc;
    do {
      rc = rclcpp::spin_until_future_complete(node_, future_result, node_loop_timeout_);
      if (rc == rclcpp::executor::FutureReturnCode::TIMEOUT) {
        on_loop_timeout();

        // We can handle a new goal if we're still executing
        auto status = goal_handle_->get_status();
        if (goal_updated_ && (status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)) {
          goal_updated_ = false;
          goto new_goal_received;
        }

        // Yield to any other CoroActionNodes (coroutines)
        setStatusRunningAndYield();
      }
    } while (rc != rclcpp::executor::FutureReturnCode::SUCCESS);

    result_ = future_result.get();
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        on_success();
        setStatus(BT::NodeStatus::IDLE);
        return BT::NodeStatus::SUCCESS;

      case rclcpp_action::ResultCode::ABORTED:
        setStatus(BT::NodeStatus::IDLE);
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        setStatus(BT::NodeStatus::IDLE);
        return BT::NodeStatus::SUCCESS;

      default:
        throw std::logic_error("BtActionNode::Tick: invalid status value");
    }
  }

  // The other (optional) override required by a BT action. In this case, we
  // make sure to cancel the ROS2 action if it is still running.
  void halt() override
  {
    if (should_cancel_goal()) {
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      if (rclcpp::spin_until_future_complete(node_, future_cancel) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(),
          "Failed to cancel action server for %s", action_name_.c_str());
      }
    }

    setStatus(BT::NodeStatus::IDLE);
    CoroActionNode::halt();
  }

protected:
  bool should_cancel_goal()
  {
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    rclcpp::spin_some(node_);
    auto status = goal_handle_->get_status();

    // Check if the goal is still executing
    if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
    {
      return true;
    }

    return false;
  }

  const std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds node_loop_timeout_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
