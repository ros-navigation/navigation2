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

#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"

namespace nav2_behavior_tree
{

template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  BtActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

    // Get the required items from the blackboard
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }
    createActionClient(action_name_);

    // Give the derive class a chance to do any initialization
    RCLCPP_INFO(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  // Create instance of an action server
  void createActionClient(const std::string & action_name)
  {
    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

    // Make sure the server is actually there before continuing
    RCLCPP_INFO(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
    action_client_->wait_for_action_server();
  }

  // Any subclass of BtActionNode that accepts parameters must provide a providedPorts method
  // and call providedBasicPorts in it.
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  // Could do dynamic checks, such as getting updates to values on the blackboard
  virtual void on_tick()
  {
  }

  // There can be many loop iterations per tick. Any opportunity to do something after
  // a timeout waiting for a result that hasn't been received yet
  virtual void on_wait_for_result()
  {
  }

  // Called upon successful completion of the action. A derived class can override this
  // method to put a value on the blackboard, for example.
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  // Called when a the action is aborted. By default, the node will return FAILURE.
  // The user may override it to return another value, instead.
  virtual BT::NodeStatus on_aborted()
  {
    return BT::NodeStatus::FAILURE;
  }

  // Called when a the action is cancelled. By default, the node will return SUCCESS.
  // The user may override it to return another value, instead.
  virtual BT::NodeStatus on_cancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  // The main override required by a BT action
  BT::NodeStatus tick() override
  {
    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      // user defined callback
      on_tick();

      on_new_goal_received();
    }

    // The following code corresponds to the "RUNNING" loop
    if (rclcpp::ok() && !goal_result_available_) {
      // user defined callback. May modify the value of "goal_updated_"
      on_wait_for_result();

      auto goal_status = goal_handle_->get_status();
      if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
      {
        goal_updated_ = false;
        on_new_goal_received();
      }

      rclcpp::spin_some(node_);

      // check if, after invoking spin_some(), we finally received the result
      if (!goal_result_available_) {
        // Yield this Action, returning RUNNING
        return BT::NodeStatus::RUNNING;
      }
    }

    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return on_success();

      case rclcpp_action::ResultCode::ABORTED:
        return on_aborted();

      case rclcpp_action::ResultCode::CANCELED:
        return on_cancelled();

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
      if (rclcpp::spin_until_future_complete(node_, future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "Failed to cancel action server for %s", action_name_.c_str());
      }
    }

    setStatus(BT::NodeStatus::IDLE);
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
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }


  void on_new_goal_received()
  {
    goal_result_available_ = false;
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
        // TODO(#1652): a work around until rcl_action interface is updated
        // if goal ids are not matched, the older goal call this callback so ignore the result
        // if matched, it must be processed (including aborted)
        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          goal_result_available_ = true;
          result_ = result;
        }
      };

    auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);

    if (rclcpp::spin_until_future_complete(node_, future_goal_handle, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("send_goal failed");
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      throw std::runtime_error("Goal was rejected by the action server");
    }
  }

  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;

  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
