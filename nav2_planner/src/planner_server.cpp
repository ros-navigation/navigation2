// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_planner/planner_server.hpp"

using namespace std::chrono_literals;

namespace nav2_planner
{

PlannerServer::PlannerServer()
: nav2_util::LifecycleNode("nav2_planner", "", true),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"), costmap_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("planner_plugin", "nav2_navfn_planner/NavfnPlanner");

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", nav2_util::add_namespaces(std::string{get_namespace()},
    "global_costmap"));

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<std::thread>(
    [&](rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    {
      // TODO(mjeronimo): Once Brian pushes his change upstream to rlcpp executors, we'll
      costmap_executor_.add_node(node->get_node_base_interface());
      costmap_executor_.spin();
      costmap_executor_.remove_node(node->get_node_base_interface());
    }, costmap_ros_);
}

PlannerServer::~PlannerServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  costmap_executor_.cancel();
  costmap_thread_->join();
  planner_.reset();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_DEBUG(get_logger(), "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  get_parameter("planner_plugin", planner_plugin_name_);
  auto node = shared_from_this();

  try {
    planner_ = gp_loader_.createUniqueInstance(planner_plugin_name_);
    RCLCPP_INFO(get_logger(), "Created global planner plugin %s",
      planner_plugin_name_.c_str());
    planner_->configure(node,
      gp_loader_.getName(planner_plugin_name_), tf_.get(), costmap_ros_.get());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(get_logger(), "Failed to create global planner. Exception: %s",
      ex.what());
    exit(-1);
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // Create the action server that we implement with our navigateToPose method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "ComputePathToPose",
      std::bind(&PlannerServer::computePathToPose, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_->activate();
  costmap_ros_->on_activate(state);
  planner_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  plan_publisher_->on_deactivate();
  costmap_ros_->on_deactivate(state);
  planner_->deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();
  plan_publisher_.reset();
  tf_.reset();
  costmap_ros_->on_cleanup(state);
  costmap_ros_.reset();
  planner_->cleanup();
  planner_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
PlannerServer::computePathToPose()
{
  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<nav2_msgs::action::ComputePathToPose::Result>();

  try {
    if (action_server_ == nullptr) {
      RCLCPP_DEBUG(get_logger(), "Action server unavailable. Stopping.");
      return;
    }

    if (!action_server_->is_server_active()) {
      RCLCPP_DEBUG(get_logger(), "Action server is inactive. Stopping.");
      return;
    }

    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
      action_server_->terminate_goals();
      return;
    }

    geometry_msgs::msg::PoseStamped start;
    if (!nav2_util::getCurrentPose(start, *tf_)) {
      return;
    }

    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
    }

    RCLCPP_DEBUG(get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
      "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
      goal->pose.pose.position.x, goal->pose.pose.position.y);

    result->path = planner_->createPlan(start, goal->pose);

    if (result->path.poses.size() == 0) {
      RCLCPP_WARN(get_logger(), "Planning algorithm %s failed to generate a valid"
        " path to (%.2f, %.2f)", planner_plugin_name_.c_str(),
        goal->pose.pose.position.x, goal->pose.pose.position.y);
      // TODO(orduno): define behavior if a preemption is available
      action_server_->terminate_goals();
      return;
    }

    RCLCPP_DEBUG(get_logger(),
      "Found valid path of size %u to (%.2f, %.2f)",
      result->path.poses.size(), goal->pose.pose.position.x,
      goal->pose.pose.position.y);

    // Publish the plan for visualization purposes
    RCLCPP_DEBUG(get_logger(), "Publishing the valid path");
    publishPlan(result->path);

    action_server_->succeeded_current(result);
    return;
  } catch (std::exception & ex) {
    RCLCPP_WARN(get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
      planner_plugin_name_.c_str(), goal->pose.pose.position.x,
      goal->pose.pose.position.y, ex.what());

    // TODO(orduno): provide information about fail error to parent task,
    //               for example: couldn't get costmap update
    action_server_->terminate_goals();
    return;
  } catch (...) {
    RCLCPP_WARN(get_logger(), "Plan calculation failed, "
      "An unexpected error has occurred. The planner server"
      " may not be able to continue operating correctly.");

    // TODO(orduno): provide information about the failure to the parent task,
    //               for example: couldn't get costmap update
    action_server_->terminate_goals();
    return;
  }
}

void
PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  plan_publisher_->publish(path);
}

}  // namespace nav2_planner
