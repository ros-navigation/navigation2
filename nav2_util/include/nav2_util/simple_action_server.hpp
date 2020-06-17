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
#include <thread>
#include <future>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_util
{

template<typename ActionT, typename nodeT = rclcpp::Node>
class SimpleActionServer
{
public:
  typedef std::function<void ()> ExecuteCallback;

  explicit SimpleActionServer(
    typename nodeT::SharedPtr node,
    const std::string & action_name,
    ExecuteCallback execute_callback,
    bool autostart = true,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500))
  : SimpleActionServer(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      action_name, execute_callback, autostart, server_timeout)
  {}

  explicit SimpleActionServer(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    const std::string & action_name,
    ExecuteCallback execute_callback,
    bool autostart = true,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500))
  : node_base_interface_(node_base_interface),
    node_clock_interface_(node_clock_interface),
    node_logging_interface_(node_logging_interface),
    node_waitables_interface_(node_waitables_interface),
    action_name_(action_name),
    execute_callback_(execute_callback),
    server_timeout_(server_timeout)
  {
    if (autostart) {
      server_active_ = true;
    }

    using namespace std::placeholders;  // NOLINT

    action_server_ = rclcpp_action::create_server<ActionT>(
      node_base_interface_,
      node_clock_interface_,
      node_logging_interface_,
      node_waitables_interface_,
      action_name_,
      std::bind(&SimpleActionServer::handle_goal, this, _1, _2),
      std::bind(&SimpleActionServer::handle_cancel, this, _1),
      std::bind(&SimpleActionServer::handle_accepted, this, _1));
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const typename ActionT::Goal>/*goal*/)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!server_active_) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    debug_msg("Received request for goal acceptance");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>/*handle*/)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    debug_msg("Received request for goal cancellation");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    debug_msg("Receiving a new goal");

    if (is_active(current_handle_) || is_running()) {
      debug_msg("An older goal is active, moving the new goal to a pending slot.");

      if (is_active(pending_handle_)) {
        debug_msg(
          "The pending slot is occupied."
          " The previous pending goal will be terminated and replaced.");
        terminate(pending_handle_);
      }
      pending_handle_ = handle;
      preempt_requested_ = true;
    } else {
      if (is_active(pending_handle_)) {
        // Shouldn't reach a state with a pending goal but no current one.
        error_msg("Forgot to handle a preemption. Terminating the pending goal.");
        terminate(pending_handle_);
        preempt_requested_ = false;
      }

      current_handle_ = handle;

      // Return quickly to avoid blocking the executor, so spin up a new thread
      debug_msg("Executing goal asynchronously.");
      execution_future_ = std::async(std::launch::async, [this]() {work();});
    }
  }

  void work()
  {
    while (rclcpp::ok() && !stop_execution_ && is_active(current_handle_)) {
      debug_msg("Executing the goal...");
      try {
        execute_callback_();
      } catch (std::exception & ex) {
        RCLCPP_ERROR(
          node_logging_interface_->get_logger(),
          "Action server failed while executing action callback: \"%s\"", ex.what());
        terminate_all();
        return;
      }

      debug_msg("Blocking processing of new goal handles.");
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);

      if (stop_execution_) {
        warn_msg("Stopping the thread per request.");
        terminate_all();
        break;
      }

      if (is_active(current_handle_)) {
        warn_msg("Current goal was not completed successfully.");
        terminate(current_handle_);
      }

      if (is_active(pending_handle_)) {
        debug_msg("Executing a pending handle on the existing thread.");
        accept_pending_goal();
      } else {
        debug_msg("Done processing available goals.");
        break;
      }
    }
    debug_msg("Worker thread done.");
  }

  void activate()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    server_active_ = true;
    stop_execution_ = false;
  }

  void deactivate()
  {
    debug_msg("Deactivating...");

    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      server_active_ = false;
      stop_execution_ = true;
    }

    if (!execution_future_.valid()) {
      return;
    }

    if (is_running()) {
      warn_msg(
        "Requested to deactivate server but goal is still executing."
        " Should check if action server is running before deactivating.");
    }

    using namespace std::chrono;  //NOLINT
    auto start_time = steady_clock::now();
    while (execution_future_.wait_for(milliseconds(100)) != std::future_status::ready) {
      info_msg("Waiting for async process to finish.");
      if (steady_clock::now() - start_time >= server_timeout_) {
        terminate_all();
        throw std::runtime_error("Action callback is still running and missed deadline to stop");
      }
    }

    debug_msg("Deactivation completed.");
  }

  bool is_running()
  {
    return execution_future_.valid() &&
           (execution_future_.wait_for(std::chrono::milliseconds(0)) ==
           std::future_status::timeout);
  }

  bool is_server_active()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return server_active_;
  }

  bool is_preempt_requested() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return preempt_requested_;
  }

  const std::shared_ptr<const typename ActionT::Goal> accept_pending_goal()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

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
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!is_active(current_handle_)) {
      error_msg("A goal is not available or has reached a final state");
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    return current_handle_->get_goal();
  }

  bool is_cancel_requested() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    // A cancel request is assumed if either handle is canceled by the client.

    if (current_handle_ == nullptr) {
      error_msg("Checking for cancel but current goal is not available");
      return false;
    }

    if (pending_handle_ != nullptr) {
      return pending_handle_->is_canceling();
    }

    return current_handle_->is_canceling();
  }

  void terminate_all(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
    terminate(pending_handle_, result);
    preempt_requested_ = false;
  }

  void terminate_current(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
  }

  void succeeded_current(
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (is_active(current_handle_)) {
      debug_msg("Setting succeed on current goal.");
      current_handle_->succeed(result);
      current_handle_.reset();
    }
  }

  void publish_feedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    if (!is_active(current_handle_)) {
      error_msg("Trying to publish feedback when the current goal handle is not active");
      return;
    }

    current_handle_->publish_feedback(feedback);
  }

protected:
  // The SimpleActionServer isn't itself a node, so it needs interfaces to one
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;
  std::string action_name_;

  ExecuteCallback execute_callback_;
  std::future<void> execution_future_;
  bool stop_execution_{false};

  mutable std::recursive_mutex update_mutex_;
  bool server_active_{false};
  bool preempt_requested_{false};
  std::chrono::milliseconds server_timeout_;

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

  void terminate(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle,
    typename std::shared_ptr<typename ActionT::Result> result =
    std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (is_active(handle)) {
      if (handle->is_canceling()) {
        warn_msg("Client requested to cancel the goal. Cancelling.");
        handle->canceled(result);
      } else {
        warn_msg("Aborting handle.");
        handle->abort(result);
      }
      handle.reset();
    }
  }

  void info_msg(const std::string & msg) const
  {
    RCLCPP_INFO(
      node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  void debug_msg(const std::string & msg) const
  {
    RCLCPP_DEBUG(
      node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  void error_msg(const std::string & msg) const
  {
    RCLCPP_ERROR(
      node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }

  void warn_msg(const std::string & msg) const
  {
    RCLCPP_WARN(
      node_logging_interface_->get_logger(),
      "[%s] [ActionServer] %s", action_name_.c_str(), msg.c_str());
  }
};

}  // namespace nav2_util

#endif   // NAV2_UTIL__SIMPLE_ACTION_SERVER_HPP_
