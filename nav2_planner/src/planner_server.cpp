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
#include <utility>

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
  std::vector<std::string> default_id, default_type;
  default_id.push_back("GridBased");
  default_type.push_back("nav2_navfn_planner/NavfnPlanner");
  declare_parameter("planner_plugin_ids", default_id);
  declare_parameter("planner_plugin_types", default_type);
  declare_parameter("expected_planner_frequency", 20.0);

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap");

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

PlannerServer::~PlannerServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second.reset();
  }
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  get_parameter("planner_plugin_ids", plugin_ids_);
  get_parameter("planner_plugin_types", plugin_types_);
  auto node = shared_from_this();

  if (plugin_ids_.size() != plugin_types_.size()) {
    RCLCPP_FATAL(
      get_logger(),
      "Planner plugin names and types sizes do not match!");
    exit(-1);
  }

  for (size_t i = 0; i != plugin_types_.size(); i++) {
    try {
      nav2_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(plugin_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created global planner plugin %s of type %s",
        plugin_ids_[i].c_str(), plugin_types_[i].c_str());
      planner->configure(node, plugin_ids_[i], tf_, costmap_ros_);
      planners_.insert({plugin_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
      exit(-1);
    }
  }

  for (size_t i = 0; i != plugin_types_.size(); i++) {
    planner_ids_concat_ += plugin_ids_[i] + std::string(" ");
  }

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    max_planner_duration_ = 0.0;

    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value has to be greater"
      " than 0.0 to turn on displaying warning messages", expected_planner_frequency);
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // Create the action server that we implement with our navigateToPose method
  action_server_ = std::make_unique<ActionServer>(
    rclcpp_node_,
    "compute_path_to_pose",
    std::bind(&PlannerServer::computePlan, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_->activate();
  costmap_ros_->on_activate(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  plan_publisher_->on_deactivate();
  costmap_ros_->on_deactivate(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

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

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }
  planners_.clear();

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
PlannerServer::computePlan()
{
  auto start_time = now();

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
      action_server_->terminate_all();
      return;
    }

    geometry_msgs::msg::PoseStamped start;
    if (!costmap_ros_->getRobotPose(start)) {
      RCLCPP_ERROR(this->get_logger(), "Could not get robot pose");
      return;
    }

    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
    }

    RCLCPP_DEBUG(
      get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
      "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
      goal->pose.pose.position.x, goal->pose.pose.position.y);

    if (planners_.find(goal->planner_id) != planners_.end()) {
      result->path = planners_[goal->planner_id]->createPlan(start, goal->pose);
    } else {
      if (planners_.size() == 1 && goal->planner_id.empty()) {
        if (!single_planner_warning_given_) {
          single_planner_warning_given_ = true;
          RCLCPP_WARN(
            get_logger(), "No planners specified in action call. "
            "Server will use only plugin %s in server."
            " This warning will appear once.", planner_ids_concat_.c_str());
        }
        result->path = planners_[planners_.begin()->first]->createPlan(start, goal->pose);
      } else {
        RCLCPP_ERROR(
          get_logger(), "planner %s is not a valid planner. "
          "Planner names are: %s", goal->planner_id.c_str(),
          planner_ids_concat_.c_str());
      }
    }

    if (result->path.poses.size() == 0) {
      RCLCPP_WARN(
        get_logger(), "Planning algorithm %s failed to generate a valid"
        " path to (%.2f, %.2f)", goal->planner_id.c_str(),
        goal->pose.pose.position.x, goal->pose.pose.position.y);
      action_server_->terminate_current();
      return;
    }

    RCLCPP_DEBUG(
      get_logger(),
      "Found valid path of size %u to (%.2f, %.2f)",
      result->path.poses.size(), goal->pose.pose.position.x,
      goal->pose.pose.position.y);

    // Publish the plan for visualization purposes
    RCLCPP_DEBUG(get_logger(), "Publishing the valid path");
    publishPlan(result->path);

    action_server_->succeeded_current(result);

    auto cycle_duration = (now() - start_time).seconds();
    if (max_planner_duration_ && cycle_duration > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration);
    }

    return;
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->pose.pose.position.x,
      goal->pose.pose.position.y, ex.what());
    // TODO(orduno): provide information about fail error to parent task,
    //               for example: couldn't get costmap update
    action_server_->terminate_current();
    return;
  } catch (...) {
    RCLCPP_WARN(
      get_logger(), "Plan calculation failed, "
      "An unexpected error has occurred. The planner server"
      " may not be able to continue operating correctly.");
    // TODO(orduno): provide information about fail error to parent task,
    //               for example: couldn't get costmap update
    action_server_->terminate_current();
    return;
  }
}

void
PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  plan_publisher_->publish(std::move(msg));
}

}  // namespace nav2_planner
