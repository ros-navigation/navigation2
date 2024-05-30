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

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

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
    goal_count_(0)
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

  virtual ~DummyActionServer() = default;
  void setFailureRanges(const Ranges & failureRanges)
  {
    failure_ranges_ = failureRanges;
  }

  void setRunningRanges(const Ranges & runningRanges)
  {
    running_ranges_ = runningRanges;
  }

  void reset()
  {
    failure_ranges_.clear();
    running_ranges_.clear();
    goal_count_ = 0;
  }

  int getGoalCount() const
  {
    return goal_count_;
  }

protected:
  virtual std::shared_ptr<typename ActionT::Result> fillResult()
  {
    return std::make_shared<typename ActionT::Result>();
  }

  virtual void updateResultForFailure(std::shared_ptr<typename ActionT::Result> &)
  {
  }

  virtual void updateResultForSuccess(std::shared_ptr<typename ActionT::Result> &)
  {
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
    goal_count_++;
    auto result = fillResult();

    // if current goal index exists in running range, the thread sleeps for 1 second
    // to simulate a long running action
    for (auto & index : running_ranges_) {
      if (goal_count_ >= index.first && goal_count_ <= index.second) {
        std::this_thread::sleep_for(1s);
        break;
      }
    }

    // if current goal index exists in failure range, the goal will be aborted
    for (auto & index : failure_ranges_) {
      if (goal_count_ >= index.first && goal_count_ <= index.second) {
        updateResultForFailure(result);
        goal_handle->abort(result);
        return;
      }
    }

    // goal succeeds for all other indices
    updateResultForSuccess(result);
    goal_handle->succeed(result);
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    using namespace std::placeholders;  // NOLINT
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DummyActionServer::execute, this, _1), goal_handle}.detach();
  }

protected:
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  std::string action_name_;

  // contains pairs of indices which define a range for which the
  // requested action goal will return running for 1s or be aborted
  // for all other indices, the action server will return success
  Ranges failure_ranges_;
  Ranges running_ranges_;

  unsigned int goal_count_;
};
}  // namespace nav2_system_tests

#endif  // BEHAVIOR_TREE__DUMMY_ACTION_SERVER_HPP_
