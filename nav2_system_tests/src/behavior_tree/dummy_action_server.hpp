// Copyright (c) 2020 Sarthak Mittal
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
// limitations under the License. Reserved.

#ifndef BEHAVIOR_TREE__DUMMY_ACTION_SERVER_HPP_
#define BEHAVIOR_TREE__DUMMY_ACTION_SERVER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_system_tests
{

using namespace std::chrono_literals;  // NOLINT
using namespace std::chrono;  // NOLINT
using namespace std::placeholders;  // NOLINT

using Range = std::pair<unsigned int, unsigned int>;
using Ranges = std::vector<Range>;

template<class ActionT>
class DummyActionServer
{
public:
  explicit DummyActionServer(
    const rclcpp::Node::SharedPtr & node,
    std::string action_name)
  : action_name_(action_name),
    goal_count_(0),
    shutting_down_(false),
    result_(std::make_shared<typename ActionT::Result>())
  {
    action_server_ = rclcpp_action::create_server<ActionT>(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      action_name,
      std::bind(&DummyActionServer::handle_goal, this, _1, _2),
      std::bind(&DummyActionServer::handle_cancel, this, _1),
      std::bind(&DummyActionServer::handle_accepted, this, _1));
  }

  virtual ~DummyActionServer()
  {
    shutdown();
  }

  void shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutting_down_ = true;
    }
    for (auto & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    workers_.clear();
  }

  void setFailureRanges(const Ranges & failureRanges)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    failure_ranges_ = failureRanges;
  }

  void setRunningRanges(const Ranges & runningRanges)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ranges_ = runningRanges;
  }

  void reset()
  {
    // Wait for all existing workers to finish before resetting state
    // to prevent race conditions between tests
    for (auto & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
    workers_.clear();

    std::lock_guard<std::mutex> lock(mutex_);
    failure_ranges_.clear();
    running_ranges_.clear();
    goal_count_ = 0;
    updateResultForSuccess(result_);
  }

  int getGoalCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goal_count_;
  }

  std::shared_ptr<typename ActionT::Result> getResult()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return result_;
  }

protected:
  virtual void updateResultForFailure(
    std::shared_ptr<typename ActionT::Result> & result)
  {
    result->error_code = ActionT::Result::UNKNOWN;
    result->error_msg = "Unknown Failure";
  }

  virtual void updateResultForSuccess(
    std::shared_ptr<typename ActionT::Result> & result)
  {
    result->error_code = ActionT::Result::NONE;
    result->error_msg = "";
  }

  virtual rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const typename ActionT::Goal>/*goal*/)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  virtual rclcpp_action::CancelResponse handle_cancel(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    unsigned int my_goal_index;
    bool should_fail = false;
    bool should_run = false;

    // 1. Capture immutable index and read config under lock
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (shutting_down_) {
        return;
      }
      my_goal_index = ++goal_count_;

      for (const auto & range : running_ranges_) {
        if (my_goal_index >= range.first && my_goal_index <= range.second) {
          should_run = true;
          break;
        }
      }

      for (const auto & range : failure_ranges_) {
        if (my_goal_index >= range.first && my_goal_index <= range.second) {
          should_fail = true;
          break;
        }
      }
    }

    // 2. Simulate long running action, checking for cancellation periodically
    if (should_run) {
      auto start_time = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start_time < 1s) {
        if (shutting_down_) {
          return;
        }
        if (goal_handle->is_canceling()) {
          auto cancel_result = std::make_shared<typename ActionT::Result>();
          goal_handle->canceled(cancel_result);
          return;
        }
        std::this_thread::sleep_for(50ms);
      }
    }

    // 3. Update shared result and finish goal
    auto final_result = std::make_shared<typename ActionT::Result>();
    if (should_fail) {
      updateResultForFailure(final_result);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        updateResultForFailure(result_);
      }
      goal_handle->abort(final_result);
    } else {
      updateResultForSuccess(final_result);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        updateResultForSuccess(result_);
      }
      goal_handle->succeed(final_result);
    }
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutting_down_) {
      return;
    }
    // Track the thread instead of detaching to prevent use-after-free
    workers_.emplace_back(&DummyActionServer::execute, this, goal_handle);
  }

protected:
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  std::string action_name_;

  // contains pairs of indices which define a range for which the
  // requested action goal will return running for 1s or be aborted.
  // for all other indices, the action server will return success.
  Ranges failure_ranges_;
  Ranges running_ranges_;

  unsigned int goal_count_;
  bool shutting_down_;
  std::shared_ptr<typename ActionT::Result> result_;

  mutable std::mutex mutex_;
  std::vector<std::thread> workers_;
};

}  // namespace nav2_system_tests

#endif  // BEHAVIOR_TREE__DUMMY_ACTION_SERVER_HPP_
