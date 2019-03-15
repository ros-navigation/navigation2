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

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>

#include "nav2_bt_navigator/navigate_to_pose_behavior_tree.hpp"
#include "nav2_tasks/bt_conversions.hpp"

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
BtNavigator::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_configure");

  // Create the NavigateToPose task server for this node
  task_server_ = std::make_unique<nav2_tasks::NavigateToPoseTaskServer>(shared_from_this());
  task_server_->on_configure(state);
  task_server_->setExecuteCallback(
    std::bind(&BtNavigator::navigateToPose, this, std::placeholders::_1));

  // Create the path to be returned from ComputePath and sent to the FollowPath task
  path_ = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create<BT::BlackboardLocal>();

  // Put items on the blackboard
  blackboard_->set<nav2_tasks::ComputePathToPoseResult::SharedPtr>("path", path_);  // NOLINT
  blackboard_->set<bool>("initial_pose_received", false);

  // Get the BT filename to use from the node parameter
  std::string bt_xml_filename;
  get_parameter_or<std::string>(std::string("bt_xml_filename"), bt_xml_filename,
    std::string("bt_navigator.xml"));

  // Read the input BT XML file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return nav2_lifecycle::CallbackReturn::FAILURE;
  }

  xml_string_ = std::string(std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());

  RCLCPP_DEBUG(get_logger(), "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_DEBUG(get_logger(), "Behavior Tree XML: %s", xml_string_.c_str());

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_activate");

  task_server_->on_activate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_deactivate");

  task_server_->on_deactivate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_cleanup");

  task_server_->on_cleanup(state);
  task_server_.reset();

  path_.reset();
  blackboard_.reset();
  xml_string_.clear();

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_error");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_shutdown");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

TaskStatus
BtNavigator::navigateToPose(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    command->pose.position.x, command->pose.position.y);

  blackboard_->set<nav2_tasks::ComputePathToPoseCommand::SharedPtr>("goal", command); // NOLINT

  // TODO(mjeronimo): Move creation of BT to on_configure state (#611)

  // Create and run the behavior tree
  std::unique_ptr<NavigateToPoseBehaviorTree> bt_ =
    std::make_unique<NavigateToPoseBehaviorTree>(shared_from_this());

  TaskStatus result = bt_->run(blackboard_, xml_string_,
      std::bind(&nav2_tasks::NavigateToPoseTaskServer::cancelRequested, task_server_.get()));

  RCLCPP_INFO(get_logger(), "Completed navigation: result: %d", result);
  return result;
}

}  // namespace nav2_bt_navigator
