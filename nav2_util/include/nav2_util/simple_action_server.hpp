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

#ifndef NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_
#define NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_

#include <memory>
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
        // If we're currently working on a task, set a flag so that the
        // action server can grab the pre-empting request in its loop
        if (received_handle_ != nullptr && received_handle_->is_active()) {
          update_requested_ = true;
          received_handle_ = handle;
        } else {
          // Otherwise, safe to start a new task
          received_handle_ = handle;
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

  bool update_requested()
  {
    return update_requested_;
  }

  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>
  get_updated_goal_handle()
  {
    update_requested_ = false;
    return received_handle_;
  }

protected:
  // The SimpleActionServer isn't itself a node, so needs to know which one to use
  rclcpp::Node::SharedPtr node_;

  ExecuteCallback execute_callback_;

  bool update_requested_{false};

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> received_handle_;
};

}  // namespace nav2_util

#endif   // NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_
