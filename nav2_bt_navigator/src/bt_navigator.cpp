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

#include "nav2_bt_navigator/bt_navigator.hpp"

#include <memory>
#include <string>
#include <fstream>
#include <streambuf>

#include "nav2_bt_navigator/navigate_to_pose_behavior_tree.hpp"
#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "nav2_tasks/bt_conversions.hpp"
#include "behaviortree_cpp/blackboard/blackboard_local.h"

using nav2_tasks::TaskStatus;

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: nav2_lifecycle::LifecycleNode("bt_navigator")
{
  RCLCPP_INFO(get_logger(), "Creating");
}

BtNavigator::~BtNavigator()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_lifecycle::CallbackReturn
BtNavigator::onConfigure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "onConfigure");

  task_server_ = std::make_unique<nav2_tasks::NavigateToPoseTaskServer>(shared_from_this());
  task_server_->onConfigure(state);
  task_server_->setExecuteCallback(std::bind(&BtNavigator::navigateToPose, this, std::placeholders::_1));

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::onActivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "onActivate");

  task_server_->onActivate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::onDeactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "onDeactivate");

  task_server_->onDeactivate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::onCleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "onCleanup");

  task_server_->onCleanup(state);
  task_server_.reset();

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

TaskStatus
BtNavigator::navigateToPose(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    command->pose.position.x, command->pose.position.y);

  // Create the blackboard that will be shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create<BT::BlackboardLocal>();

  // Create the path to be returned from ComputePath and sent to the FollowPath task
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  // Set the shared data (commands/results)
  blackboard->set<nav2_tasks::ComputePathToPoseCommand::SharedPtr>("goal", command);
  blackboard->set<nav2_tasks::ComputePathToPoseResult::SharedPtr>("path", path);  // NOLINT
  blackboard->set<bool>("initial_pose_received", task_server_->isInitialPoseReceieved());  // NOLINT

  // Get the filename to use from the parameter
  std::string bt_xml_filename;
  get_parameter_or<std::string>(std::string("bt_xml_filename"), bt_xml_filename,
    std::string("bt_navigator.xml"));

  // Read the input BT XML file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return TaskStatus::FAILED;
  }

  std::string xml_string((std::istreambuf_iterator<char>(xml_file)),
    std::istreambuf_iterator<char>());

  RCLCPP_INFO(get_logger(), "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_INFO(get_logger(), "Behavior Tree XML: %s", xml_string.c_str());

  // Create and run the behavior tree
  NavigateToPoseBehaviorTree bt(shared_from_this());

  TaskStatus result = bt.run(blackboard, xml_string,
      std::bind(&nav2_tasks::NavigateToPoseTaskServer::cancelRequested, task_server_.get()));

  task_server_->setInitialPose(blackboard->get<bool>("initial_pose_received"));

  RCLCPP_INFO(get_logger(), "Completed navigation: result: %d", result);
  return result;
}

}  // namespace nav2_bt_navigator
