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
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__UTILS__TEST_ACTION_SERVER_HPP_
#define NAV2_BEHAVIOR_TREE__UTILS__TEST_ACTION_SERVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

template<class ActionT>
class TestActionServer : public rclcpp::Node
{
public:
  explicit TestActionServer(
    std::string action_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_action_server", options)
  {
    using namespace std::placeholders;  // NOLINT

    this->action_server_ = rclcpp_action::create_server<ActionT>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      action_name,
      std::bind(&TestActionServer::handle_goal, this, _1, _2),
      std::bind(&TestActionServer::handle_cancel, this, _1),
      std::bind(&TestActionServer::handle_accepted, this, _1));
  }

  virtual ~TestActionServer()
  {
    stop_requested_.store(true);
    joinWorkers();
  }

  std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
  {
    std::lock_guard<std::mutex> lock(current_goal_mutex_);
    return current_goal_;
  }

  void setReturnSuccess(bool return_success)
  {
    return_success_.store(return_success);
  }

  bool getReturnSuccess(void)
  {
    return return_success_.load();
  }

  bool isGoalCancelled()
  {
    return goal_cancelled_.load();
  }

protected:
  bool isStopRequested() const
  {
    return stop_requested_.load();
  }

  virtual rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const typename ActionT::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(current_goal_mutex_);
    current_goal_ = goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  virtual rclcpp_action::CancelResponse handle_cancel(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
  {
    goal_cancelled_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  virtual void execute(
    const typename std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle) = 0;

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    if (stop_requested_.load()) {
      return;
    }

    std::lock_guard<std::mutex> lock(workers_mutex_);
    workers_.emplace_back([this, goal_handle]() {
      execute(goal_handle);
    });
  }

private:
  void joinWorkers()
  {
    while (true) {
      std::vector<std::thread> workers;
      {
        std::lock_guard<std::mutex> lock(workers_mutex_);
        workers.swap(workers_);
      }

      if (workers.empty()) {
        return;
      }

      for (auto & worker : workers) {
        if (worker.joinable()) {
          worker.join();
        }
      }
    }
  }

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  mutable std::mutex current_goal_mutex_;
  std::mutex workers_mutex_;
  std::vector<std::thread> workers_;
  std::atomic_bool stop_requested_{false};
  std::shared_ptr<const typename ActionT::Goal> current_goal_;
  std::atomic_bool return_success_{true};
  std::atomic_bool goal_cancelled_{false};
};

#endif  // NAV2_BEHAVIOR_TREE__UTILS__TEST_ACTION_SERVER_HPP_
