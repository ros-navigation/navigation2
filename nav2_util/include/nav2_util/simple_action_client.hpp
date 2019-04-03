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

#ifndef NAV2_UTIL__SIMPLE_ACTION_CLIENT_HPP_
#define NAV2_UTIL__SIMPLE_ACTION_CLIENT_HPP_

#include <chrono>
#include <stdexcept>
#include <string>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status.hpp"

namespace nav2_util
{

typedef enum
{
  SUCCEEDED,
  FAILED,
  RUNNING,
  CANCELED
} ActionStatus;

template<typename ActionT>
class SimpleActionClient
{
public:
  explicit SimpleActionClient(rclcpp::Node::SharedPtr node, const std::string & action_name)
  : node_(node)
  {
    action_client_ = rclcpp_action::create_client<ActionT>(node, action_name);
  }

  bool wait_for_server(std::chrono::milliseconds timeout = std::chrono::milliseconds::max())
  {
    return action_client_->wait_for_action_server(timeout);
  }

  ActionStatus invoke(
    const typename ActionT::Goal & goal,
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result,
    typename rclcpp_action::ClientGoalHandle<ActionT>::FeedbackCallback callback = nullptr)
  {
    // Send the goal

    auto future_goal_handle = action_client_->async_send_goal(goal, callback);

    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("send_goal failed");
    }

    auto goal_handle = future_goal_handle.get();

    if (!goal_handle) {
      throw std::runtime_error("Goal was rejected by the server");
    }

    // Wait for the result

    auto future_result = action_client_->async_get_result(goal_handle);

    if (rclcpp::spin_until_future_complete(node_, future_result) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("spin_until_future_complete failed");
    }

    result = future_result.get();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return SUCCEEDED;

      case rclcpp_action::ResultCode::ABORTED:
        return FAILED;

      case rclcpp_action::ResultCode::CANCELED:
        return CANCELED;

      default:
        throw std::runtime_error("Unknown result code");
    }
  }

  void send_goal(
    const typename ActionT::Goal & goal,
    typename rclcpp_action::ClientGoalHandle<ActionT>::FeedbackCallback callback = nullptr)
  {
    auto future_goal_handle = action_client_->async_send_goal(goal, callback);

    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("send_goal failed");
    }

    goal_handle_ = future_goal_handle.get();

    if (!goal_handle_) {
      throw std::runtime_error("Goal was rejected by the server");
    }
  }

  bool cancel()
  {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    return rclcpp::spin_until_future_complete(node_, future_cancel) ==
           rclcpp::executor::FutureReturnCode::SUCCESS;
  }

  ActionStatus
  wait_for_result(std::chrono::milliseconds timeout = std::chrono::milliseconds::max())
  {
    auto future_result = action_client_->async_get_result(goal_handle_);
    auto wait_result = rclcpp::spin_until_future_complete(node_, future_result, timeout);

    if (wait_result == rclcpp::executor::FutureReturnCode::TIMEOUT) {
      return RUNNING;
    } else if (wait_result != rclcpp::executor::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("get result failed");
    }

    result_ = future_result.get();

    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return SUCCEEDED;

      case rclcpp_action::ResultCode::ABORTED:
        return FAILED;

      case rclcpp_action::ResultCode::CANCELED:
        return CANCELED;

      default:
        throw std::runtime_error("Unknown result code");
    }
  }

  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult
  get_result()
  {
    return result_;
  }

protected:
  // The SimpleActionClient isn't itself a node, so it receives one to use
  rclcpp::Node::SharedPtr node_;

  // The action client to invoke
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;

  // The SimpleActionClient supports a single goal at a time
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;

  // The result of the action
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;
};

}  // namespace nav2_util

#endif   // NAV2_UTIL__SIMPLE_ACTION_CLIENT_HPP_
