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

#ifndef BEHAVIOR_TREE__DUMMY_SERVERS_HPP_
#define BEHAVIOR_TREE__DUMMY_SERVERS_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT
using namespace std::chrono;  // NOLINT
using namespace std::placeholders;  // NOLINT

template<class ServiceT>
class DummyService
{
public:
  explicit DummyService(
    const rclcpp::Node::SharedPtr & node,
    std::string service_name)
  : node_(node),
    service_name_(service_name),
    request_count_(0),
    disabled_(false)
  {
    server_ = node->create_service<ServiceT>(
      service_name,
      std::bind(&DummyService::handle_service, this, _1, _2, _3));
  }

  void disable()
  {
    server_.reset();
    disabled_ = true;
  }

  void enable()
  {
    if (disabled_) {
      server_ = node_->create_service<ServiceT>(
        service_name_,
        std::bind(&DummyService::handle_service, this, _1, _2, _3));
      disabled_ = false;
    }
  }

  void reset()
  {
    enable();
    request_count_ = 0;
  }

  int getRequestCount() const
  {
    return request_count_;
  }

protected:
  virtual void fillResponse(
    const std::shared_ptr<typename ServiceT::Request>/*request*/,
    const std::shared_ptr<typename ServiceT::Response>/*response*/) {}

  void handle_service(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<typename ServiceT::Request> request,
    const std::shared_ptr<typename ServiceT::Response> response)
  {
    request_count_++;
    fillResponse(request, response);
  }

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Service<ServiceT>::SharedPtr server_;
  std::string service_name_;
  int request_count_;
  bool disabled_;
};

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
    this->action_server_ = rclcpp_action::create_server<ActionT>(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      action_name,
      std::bind(&DummyActionServer::handle_goal, this, _1, _2),
      std::bind(&DummyActionServer::handle_cancel, this, _1),
      std::bind(&DummyActionServer::handle_accepted, this, _1));
  }

  void setFailureRanges(const std::vector<std::pair<int, int>> & failureRanges)
  {
    failure_ranges_ = failureRanges;
  }

  void setRunningRanges(const std::vector<std::pair<int, int>> & runningRanges)
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
        goal_handle->abort(result);
        return;
      }
    }

    // goal succeeds for all other indices
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
  std::vector<std::pair<int, int>> failure_ranges_;
  std::vector<std::pair<int, int>> running_ranges_;

  int goal_count_;
};

#endif  // BEHAVIOR_TREE__DUMMY_SERVERS_HPP_
