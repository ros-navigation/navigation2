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
#include <utility>

#include "nav2_tasks/bt_conversions.hpp"
#include "nav2_tasks/task_status.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: nav2_lifecycle::LifecycleNode("bt_navigator", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");
}

BtNavigator::~BtNavigator()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  auto node = shared_from_this();

  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("bt_navigator_client_node");

  self_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node_, "navigate_to_pose");

  goal_sub_ = rclcpp_node_->create_subscription<geometry_msgs::msg::PoseStamped>("goal",
      std::bind(&BtNavigator::onGoalPoseReceived, this, std::placeholders::_1));

  // Create an action server that we implement with our navigateToPose method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "navigate_to_pose",
      std::bind(&BtNavigator::navigateToPose, this, std::placeholders::_1));

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<NavigateToPoseBehaviorTree>(node);

  // Create the path that will be returned from ComputePath and sent to FollowPath
  goal_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  path_ = std::make_shared<nav2_msgs::msg::Path>();

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create<BT::BlackboardLocal>();

  // Put items on the blackboard
  blackboard_->set<geometry_msgs::msg::PoseStamped::SharedPtr>("goal", goal_);  // NOLINT
  blackboard_->set<nav2_msgs::msg::Path::SharedPtr>("path", path_);  // NOLINT
  blackboard_->set<nav2_lifecycle::LifecycleNode::SharedPtr>("node", node);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("node_loop_timeout", std::chrono::milliseconds(10));  // NOLINT
  blackboard_->set<bool>("initial_pose_received", false);  // NOLINT

  // Get the BT filename to use from the node parameter
  std::string bt_xml_filename;
  get_parameter_or<std::string>(std::string("bt_xml_filename"), bt_xml_filename,
    std::string("bt_navigator.xml"));

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return nav2_lifecycle::CallbackReturn::FAILURE;
  }

  xml_string_ = std::string(std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());

  RCLCPP_DEBUG(get_logger(), "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_DEBUG(get_logger(), "Behavior Tree XML: %s", xml_string_.c_str());

  // Create the Behavior Tree from the XML input (after registering our own node types)
  BT::Tree temp_tree = bt_->buildTreeFromText(xml_string_, blackboard_);

  // Unfortunately, the BT library provides the tree as a struct instead of a pointer. So, we will
  // createa new BT::Tree ourselves and move the data over
  tree_ = std::make_unique<BT::Tree>();
  tree_->root_node = temp_tree.root_node;
  tree_->nodes = std::move(temp_tree.nodes);
  temp_tree.root_node = nullptr;

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  goal_sub_.reset();
  client_node_.reset();
  self_client_.reset();
  action_server_.reset();
  path_.reset();
  xml_string_.clear();
  tree_.reset();
  blackboard_.reset();
  bt_.reset();

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_logger(), "Handling error state");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

void
BtNavigator::navigateToPose(const std::shared_ptr<GoalHandle> goal_handle)
{
  // Initialize the goal and result
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

  // TODO(mjeronimo): handle or reject an attempted pre-emption

  RCLCPP_INFO(get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  // Get the goal pointer off of the blackboard...
  auto bb_goal = blackboard_->get<geometry_msgs::msg::PoseStamped::SharedPtr>("goal");

  // and update it with the incoming command
  *bb_goal = goal->pose;

  // Execute the BT that was previously created in the configure step
  auto is_canceling = [goal_handle]() -> bool {return goal_handle->is_canceling();};
  TaskStatus rc = bt_->run(tree_, is_canceling);

  switch (rc) {
    case TaskStatus::CANCELED:
      // Even though the BT is no longer running, remote actions may still be executing. So,
      // an explicit canceling of all actions allows the task clients to send cancel messages
      // to their corresponding task servers
      bt_->cancelAllActions(tree_->root_node);

      // Reset the BT so that it can be run again in the future
      bt_->resetTree(tree_->root_node);
      goal_handle->set_canceled(result);
      break;

    case TaskStatus::SUCCEEDED:
      goal_handle->set_succeeded(result);
      break;

    case TaskStatus::FAILED:
      goal_handle->set_aborted(result);
      break;

    default:
      throw std::logic_error("Invalid status return from BT");
  }

  RCLCPP_INFO(get_logger(), "Completed navigation: result: %d", rc);
}

void
BtNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  RCLCPP_INFO(get_logger(), "onGoalPoseReceived");

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose = *pose;

  self_client_->async_send_goal(goal);
}

}  // namespace nav2_bt_navigator
