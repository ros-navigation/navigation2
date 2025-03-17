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

#include <memory>
#include <thread>

#include "server_handler.hpp"

using namespace std::chrono_literals;  // NOLINT
using namespace std::chrono;  // NOLINT

namespace nav2_system_tests
{


ServerHandler::ServerHandler()
: is_active_(false)
{
  node_ = rclcpp::Node::make_shared("behavior_tree_tester");

  clear_local_costmap_server = std::make_unique<DummyService<nav2_msgs::srv::ClearEntireCostmap>>(
    node_, "local_costmap/clear_entirely_local_costmap");
  clear_global_costmap_server = std::make_unique<DummyService<nav2_msgs::srv::ClearEntireCostmap>>(
    node_, "global_costmap/clear_entirely_global_costmap");
  compute_path_to_pose_server = std::make_unique<DummyComputePathToPoseActionServer>(node_);
  follow_path_server = std::make_unique<DummyFollowPathActionServer>(node_);
  spin_server = std::make_unique<DummyActionServer<nav2_msgs::action::Spin>>(
    node_, "spin");
  wait_server = std::make_unique<DummyActionServer<nav2_msgs::action::Wait>>(
    node_, "wait");
  backup_server = std::make_unique<DummyActionServer<nav2_msgs::action::BackUp>>(
    node_, "backup");
  drive_on_heading_server = std::make_unique<DummyActionServer<nav2_msgs::action::DriveOnHeading>>(
    node_, "drive_on_heading");
  ntp_server = std::make_unique<DummyActionServer<nav2_msgs::action::ComputePathThroughPoses>>(
    node_, "compute_path_through_poses");
}

ServerHandler::~ServerHandler()
{
  if (is_active_) {
    deactivate();
  }
}

void ServerHandler::activate()
{
  if (is_active_) {
    throw std::runtime_error("Trying to activate while already activated");
  }

  is_active_ = true;
  server_thread_ =
    std::make_shared<std::thread>(std::bind(&ServerHandler::spinThread, this));

  std::cout << "Server handler is active!" << std::endl;
}

void ServerHandler::deactivate()
{
  if (!is_active_) {
    throw std::runtime_error("Trying to deactivate while already inactive");
  }

  is_active_ = false;
  server_thread_->join();

  std::cout << "Server handler has been deactivated!" << std::endl;
}

void ServerHandler::reset() const
{
  clear_global_costmap_server->reset();
  clear_local_costmap_server->reset();
  compute_path_to_pose_server->reset();
  follow_path_server->reset();
  spin_server->reset();
  wait_server->reset();
  backup_server->reset();
  drive_on_heading_server->reset();
}

void ServerHandler::spinThread()
{
  rclcpp::spin(node_);
}

}  // namespace nav2_system_tests
