// Copyright (c) 2020 Vinny Ruia
// Copyright (c) 2020 Sarthak Mittal
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
// limitations under the License. Reserved.

#include <string>
#include <random>
#include <tuple>
#include <memory>
#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "behavior_tree_tester.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT

namespace nav2_system_tests
{

BehaviorTreeTester::BehaviorTreeTester()
: is_active_(false)
{
  node_ = rclcpp::Node::make_shared("behavior_tree_test");

  // This could probably be simplified into template classes
  auto compute_path_to_pose_client_ptr_ =
    rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    "compute_path_to_pose");
  auto follow_path_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    "follow_path");
  auto spin_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    "spin");
  auto wait_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::Wait>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    "wait");
  auto back_up_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::BackUp>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    "back_up");
}

BehaviorTreeTester::~BehaviorTreeTester()
{
  if (is_active_) {
    deactivate();
  }
}

void BehaviorTreeTester::activate()
{
  if (is_active_) {
    throw std::runtime_error("Trying to activate while already activated");
    return;
  }

  std::this_thread::sleep_for(10s);

  // checkForActionClient

  RCLCPP_INFO(this->node_->get_logger(), "All action servers are ready");
  is_active_ = true;
}

void BehaviorTreeTester::deactivate()
{
  if (!is_active_) {
    throw std::runtime_error("Trying to deactivate while already inactive");
  }
  is_active_ = false;
}

bool BehaviorTreeTester::defaultBehaviorTreeTest(
  struct should_action_server_return_success_t test_case)
{
  // This is where we use this structure to tell the BT to return particular status

  if (!is_active_) {
    RCLCPP_ERROR(node_->get_logger(), "Not activated");
    return false;
  }

  // Sleep to let recovery server be ready for serving in multiple runs
  std::this_thread::sleep_for(10s);

  bool result;
  if (test_case.wait) {
    result = true;
  } else {
    result = false;
  }
  return result;
}

}  //  namespace nav2_system_tests
