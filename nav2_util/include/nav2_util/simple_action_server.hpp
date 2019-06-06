// Copyright (c) 2019 Intel Corporation
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

#ifndef NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_
#define NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_util
{

template<typename ActionT>
class SimpleActionServer
{
public:
  typedef std::function<
      void (const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)> ExecuteCallback;

  explicit SimpleActionServer(
    rclcpp::Node::SharedPtr node,
    const std::string & action_name,
    ExecuteCallback execute_callback)
  : node_(node), execute_callback_(execute_callback)
  {
    auto handle_goal =
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)
      {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

    auto handle_cancel = [](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
      {
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto handle_accepted = [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
      {
        std::lock_guard<std::mutex> lock(update_mutex_);

        // If we're currently working on a task, set a flag so that the
        // action server can grab the pre-empting request in its loop
        if (current_handle_ != nullptr && current_handle_->is_active()) {
          preempt_requested_ = true;
          new_handle_ = handle;
        } else {
          // Otherwise, safe to start a new task
          current_handle_ = handle;
          std::thread{execute_callback_, handle}.detach();
        }
      };

    action_server_ = rclcpp_action::create_server<ActionT>(
      node_,
      action_name,
      handle_goal,
      handle_cancel,
      handle_accepted);
  }

  bool preempt_requested()
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    return preempt_requested_;
  }

  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>
  get_updated_goal_handle()
  {
    std::lock_guard<std::mutex> lock(update_mutex_);

    current_handle_->abort(std::make_shared<typename ActionT::Result>());
    current_handle_ = new_handle_;
    new_handle_.reset();
    preempt_requested_ = false;

    return current_handle_;
  }

protected:
  // The SimpleActionServer isn't itself a node, so needs to know which one to use
  rclcpp::Node::SharedPtr node_;

  ExecuteCallback execute_callback_;

  std::mutex update_mutex_;
  bool preempt_requested_{false};
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> new_handle_;

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
};

}  // namespace nav2_util

#endif   // NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_
