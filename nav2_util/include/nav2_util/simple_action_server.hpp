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
  typedef std::function<void ()> ExecuteCallback;

  explicit SimpleActionServer(
    rclcpp::Node::SharedPtr node,
    const std::string & action_name,
    ExecuteCallback execute_callback,
    bool autostart = true)
  : node_(node), action_name_(action_name), execute_callback_(execute_callback)
  {
    if (autostart) {
      server_active_ = true;
    }

    auto handle_goal =
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)
      {
        std::lock_guard<std::mutex> lock(update_mutex_);

        if (!server_active_) {
          return rclcpp_action::GoalResponse::REJECT;
        }

        debug_msg("Received request for goal acceptance");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      };

    auto handle_cancel =
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
      {
        debug_msg("Received request for goal cancellation");
        return rclcpp_action::CancelResponse::ACCEPT;
      };

    auto handle_accepted =
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
      {
        std::lock_guard<std::mutex> lock(update_mutex_);
        debug_msg("Receiving a new goal");

        if (is_active(current_handle_)) {
          debug_msg("An older goal is active, moving the new goal to a pending slot.");

          if (is_active(pending_handle_)) {
            debug_msg("The pending slot is occupied."
              " The pending goal will be canceled and replaced.");

            pending_handle_->canceled(empty_result());
            pending_handle_.reset();
          }

          debug_msg("Setting flag so the action server can grab the preempt request.");
          preempt_requested_ = true;
          pending_handle_ = handle;
        } else {
          if (is_active(pending_handle_)) {
            // Shouldn't reach a state with a pending goal but no current one.
            error_msg("Forgot to handle a preemption. Cancelling the pending goal.");

            pending_handle_->canceled(empty_result());
            pending_handle_.reset();
          }

          debug_msg("Starting a thread to process the goals");

          current_handle_ = handle;
          std::thread{execute_callback_}.detach();
        }
      };

    action_server_ = rclcpp_action::create_server<ActionT>(
      node_,
      action_name_,
      handle_goal,
      handle_cancel,
      handle_accepted);
  }

  void activate()
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    server_active_ = true;
  }

  void deactivate()
  {
    std::lock_guard<std::mutex> lock_goal_handle(update_mutex_);

    server_active_ = false;

    // TODO(orduno) Replace with `abort_all()` once #849 is merged
    if (current_handle_ != nullptr && current_handle_->is_active()) {
      RCLCPP_WARN(node_->get_logger(), "Taking action server to deactive state "
        " with an active goal. Cancelling the current goal.");
      current_handle_->abort(std::make_shared<typename ActionT::Result>());
      current_handle_.reset();
    }

    if (new_handle_ != nullptr && new_handle_->is_active()) {
      RCLCPP_WARN(node_->get_logger(), "Taking action server to deactive state "
        " with a pending preemption. Cancelling the preemptive goal.");
      new_handle_->abort(std::make_shared<typename ActionT::Result>());
      new_handle_.reset();
    }
  }

  bool serverIsActive()
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    return server_active_;
  }

  bool preempt_requested() const
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    return preempt_requested_;
  }

  const std::shared_ptr<const typename ActionT::Goal> accept_pending_goal()
  {
    std::lock_guard<std::mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      error_msg("Attempting to get pending goal when not available");
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    if (is_active(current_handle_) && current_handle_ != pending_handle_) {
      debug_msg("Cancelling the previous goal");
      current_handle_->abort(empty_result());
    }

    current_handle_ = pending_handle_;
    pending_handle_.reset();
    preempt_requested_ = false;

    debug_msg("Preempted goal");

    return current_handle_->get_goal();
  }

  const std::shared_ptr<const typename ActionT::Goal> get_current_goal() const
  {
    std::lock_guard<std::mutex> lock(update_mutex_);

    if (!is_active(current_handle_)) {
      error_msg("A goal is not available or has reached a final state");
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    return current_handle_->get_goal();
  }

  bool is_cancelling_current_goal() const
  {
    std::lock_guard<std::mutex> lock(update_mutex_);

    if (current_handle_ == nullptr) {
      error_msg("Current goal is not available");
      return false;
    }

    return current_handle_->is_canceling();
  }

  void cancel_all()
  {
    std::lock_guard<std::mutex> lock(update_mutex_);

    if (current_handle_ != nullptr) {
      debug_msg("Cancelling the current goal.");
      current_handle_->canceled(empty_result());
      current_handle_.reset();
    }

    if (pending_handle_ != nullptr) {
      warn_msg("Cancelling a pending goal. Should check for pre-empt requests.");
      pending_handle_->canceled(empty_result());
      pending_handle_.reset();
    }
  }

  void abort_all()
  {
    std::lock_guard<std::mutex> lock(update_mutex_);

    if (current_handle_ != nullptr) {
      debug_msg("Aborting the current goal.");
      current_handle_->abort(empty_result());
      current_handle_.reset();
    }

    if (pending_handle_ != nullptr) {
      warn_msg("Aborting a pending goal. Should check for pre-empt requests.");
      pending_handle_->abort(empty_result());
      pending_handle_.reset();
    }
  }

  void succeeded_current(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::mutex> lock(update_mutex_);

    if (is_active(current_handle_)) {
      debug_msg("Setting succeed on current goal.");
      current_handle_->succeed(result);
      current_handle_.reset();
    }

    // TODO(orduno) Cancelling any pending goal. Get consensus on policy.
    if (is_active(pending_handle_)) {
      warn_msg("A preemption request was available before succeeding on the current goal.");
      pending_handle_->canceled(empty_result());
      pending_handle_.reset();
    }
  }

protected:
  // The SimpleActionServer isn't itself a node, so needs to know which one to use
  rclcpp::Node::SharedPtr node_;
  std::string action_name_;

  ExecuteCallback execute_callback_;

  mutable std::mutex update_mutex_;
  bool server_active_{false};
  bool preempt_requested_{false};
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> pending_handle_;

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;

  constexpr auto empty_result() const
  {
    return std::make_shared<typename ActionT::Result>();
  }

  constexpr bool is_active(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle) const
  {
    return handle != nullptr && handle->is_active();
  }

  void debug_msg(const std::string & msg) const
  {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  void error_msg(const std::string & msg) const
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  void warn_msg(const std::string & msg) const
  {
    RCLCPP_WARN(node_->get_logger(), "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }
};

}  // namespace nav2_util

#endif   // NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_
