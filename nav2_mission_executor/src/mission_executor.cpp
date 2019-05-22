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

#include "nav2_mission_executor/mission_executor.hpp"

#include <string>
#include <memory>

#include "nav2_mission_executor/execute_mission_behavior_tree.hpp"

namespace nav2_mission_executor
{

MissionExecutor::MissionExecutor()
: nav2_util::LifecycleNode("mission_executor", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");
}

MissionExecutor::~MissionExecutor()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
MissionExecutor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  client_node_ = std::make_shared<rclcpp::Node>("mission_executor_client_node");

  // Create the action server that we implement with our executeMission method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "ExecuteMission",
      std::bind(&MissionExecutor::executeMission, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MissionExecutor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MissionExecutor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MissionExecutor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  client_node_.reset();
  action_server_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MissionExecutor::on_error(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MissionExecutor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
MissionExecutor::executeMission(const std::shared_ptr<GoalHandle> goal_handle)
{
  // Initialize the goal and result
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<nav2_msgs::action::ExecuteMission::Result>();

  // Get a convenient reference to the mission plan string
  const std::string & xml_string = goal->mission_plan.mission_plan;
  RCLCPP_INFO(get_logger(), "Executing mission plan: %s", xml_string.c_str());

  // Create the blackboard that will be shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create<BT::BlackboardLocal>();

  // Set a couple values on the blackboard that all of the nodes require
  blackboard->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard->set<std::chrono::milliseconds>("node_loop_timeout",  // NOLINT
    std::chrono::milliseconds(100));

  // Create the Behavior Tree for this mission
  ExecuteMissionBehaviorTree bt;

  // Run the Behavior Tree
  auto is_canceling = [goal_handle]() -> bool {return goal_handle->is_canceling();};
  nav2_tasks::BtStatus rc = bt.run(blackboard, xml_string, is_canceling);

  // Handle the result
  switch (rc) {
    case nav2_tasks::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Mission succeeded");
      goal_handle->succeed(result);
      return;

    case nav2_tasks::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Mission failed");
      goal_handle->abort(result);
      return;

    case nav2_tasks::BtStatus::CANCELED:
      RCLCPP_INFO(get_logger(), "Mission canceled");
      goal_handle->canceled(result);
      return;

    default:
      throw std::logic_error("Invalid status return from BT");
  }
}

}  // namespace nav2_mission_executor
