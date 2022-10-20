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
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_planner/planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_planner
{
PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
    : nav2_util::LifecycleNode("planner_server", "", options),
      gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
      default_ids_{"GridBased"},
      default_types_{"nav2_navfn_planner/NavfnPlanner"},
      costmap_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", std::string{get_namespace()}, "global_costmap");

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

PlannerServer::~PlannerServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  planners_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_DEBUG(
      get_logger(), "Costmap size: %d,%d",
      costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = nav2_util::get_plugin_type_param(
          node, planner_ids_[i]);
      nav2_core::GlobalPlanner::Ptr planner =
          gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
          get_logger(), "Created global planner plugin %s of type %s",
          planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->configure(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException &ex) {
      RCLCPP_FATAL(
          get_logger(), "Failed to create global planner. Exception: %s",
          ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++){
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
      get_logger(),
      "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
        get_logger(),
        "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
        " than 0.0 to turn on duration overrrun warning messages",
        expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav2_msgs::msg::PathAndBoundary>("plan", 1);

  // Create the action server that we implement with our navigateToPose method
  action_server_ = std::make_unique<ActionServer>(
      node,
      "compute_path_to_pose",
      std::bind(&PlannerServer::computePlan, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_->activate();
  costmap_ros_->on_activate(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();

  is_path_valid_service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
    "is_path_valid",
    std::bind(
      &PlannerServer::isPathValid, this,
      std::placeholders::_1, std::placeholders::_2));

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PlannerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State &state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  plan_publisher_->on_deactivate();
  costmap_ros_->on_deactivate(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State &state)
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
  costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void PlannerServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool PlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool PlannerServer::getStartPose(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal,
  geometry_msgs::msg::PoseStamped & start)
{
  if (goal->use_start_poses) {
    start = goal->start;
  } else if (!costmap_ros_->getRobotPose(start)) {
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::transformPosesToGlobalFrame(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  geometry_msgs::msg::PoseStamped & curr_start,
  geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
  {
    RCLCPP_WARN(
      get_logger(), "Could not transform the start or goal pose in the costmap frame");
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::validatePath(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  if (path.poses.size() == 0) {
    RCLCPP_WARN(
      get_logger(), "Planning algorithm %s failed to generate a valid"
      " path to (%.2f, %.2f)", planner_id.c_str(),
      goal.pose.position.x, goal.pose.position.y);
    action_server->terminate_current();
    return false;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Found valid path of size %lu to (%.2f, %.2f)",
    path.poses.size(), goal.pose.position.x,
    goal.pose.position.y);

  return true;
}

void
PlannerServer::computePlan()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePath goal and result
  auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<nav2_msgs::action::ComputePath::Result>();

  try {
    if (isServerInactive(action_server_) || isCancelRequested(action_server_)) {
      return;
    }

    // waitForCostmap();

    getPreemptedGoalIfRequested(action_server_, goal);
    geometry_msgs::msg::PoseStamped starting_cartesian_pose;
    auto start_cartesian_poses = goal->start_cartesian_poses;
    /**
     * @todo: In case that there is a set of starting poses we compute the last one.
     * However, this need to be generalized to consider multiple starting poses from
     * different robots.
     */
    if (!start_cartesian_poses.empty())
    {
      for (const auto& pose : start_cartesian_poses )
      {
        starting_cartesian_pose = pose;
      }
    }

    auto cartesian_path = goal->cartesian_path;
    geographic_msgs::msg::GeoPoseStamped starting_geographic_pose;
    auto start_geographic_poses = goal->start_geographic_poses;

    if (!start_geographic_poses.empty())
    {
      for (const auto &pose : start_geographic_poses)
      {
        starting_geographic_pose = pose;
      }
    }
    auto geographic_path = goal->geographic_path;
    auto number_of_robots = static_cast<int>(goal->robot_ids.size());

    if (!goal->use_start_poses)
    {
      /**
       * @brief TODO: We might have to revisit this about getRobotPose if we are using GPS.
       * 
       * For now, if starting poses are not available, it is handled by the client. In the
       * request message definition we throw an error, if the starting poses are not defined
       * by the user in the BT input port.
       */

      // if (!costmap_ros_->getRobotPose(start_local))
      // {
      //   action_server_->terminate_current();
      //   return;
      // }
    }

    if (goal->reference_mode == nav2_msgs::action::ComputePath::Goal::CARTESIAN)
    {
      RCLCPP_INFO(
          get_logger(),
          "Cartesian reference mode selected.");
      result->path_and_boundary = getPlan(starting_cartesian_pose, cartesian_path,
                                          goal->planner_id, number_of_robots);
    }
    else if (goal->reference_mode == nav2_msgs::action::ComputePath::Goal::GEOGRAPHIC)
    {
      RCLCPP_INFO(
          get_logger(),
          "Geographical reference mode selected.");
      result->path_and_boundary = getPlan(starting_geographic_pose, geographic_path,
                                          goal->planner_id, number_of_robots);
    }

    if (result->path_and_boundary.path_local.empty() &&
        result->path_and_boundary.path_global.empty())
    {
      RCLCPP_WARN(
          get_logger(), "Planning algorithm %s failed to generate a valid"
                        " path",
          goal->planner_id.c_str());
      action_server_->terminate_current();
      return;
    }

    RCLCPP_DEBUG(
        get_logger(),
        "Found valid path of size");

    publishPlan(result->path_and_boundary);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_)
    {
      RCLCPP_WARN(
          get_logger(),
          "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
          1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_->succeeded_current(result);
  }
  catch (std::exception &ex)
  {
    RCLCPP_WARN(
        get_logger(), "%s plugin failed to plan calculation: \"%s\"",
        goal->planner_id.c_str(), ex.what());
    // TODO(orduno): provide information about fail error to parent task,
    //               for example: couldn't get costmap update
    action_server_->terminate_current();
  }
}

template <typename PoseType, typename PathType>
nav2_msgs::msg::PathAndBoundary
PlannerServer::getPlan(
    const PoseType &start,
    const PathType &goals,
    const std::string &planner_id,
    const int &robots)
{
  RCLCPP_DEBUG(
      get_logger(), "Attempting to a find path from goal(s) to start");

  // RCLCPP_DEBUG(
  //     get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
  //                   "(%.2f, %.2f).",
  //     start.pose.position.x, start.pose.position.y,
  //     goal.pose.position.x, goal.pose.position.y);

  if (planners_.find(planner_id) != planners_.end()) {
    return planners_[planner_id]->createPlan(start, goals, robots);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
          get_logger(), "No planners specified in action call. "
                        "Server will use only plugin %s in server."
                        " This warning will appear once.",
          planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goals, robots);
    } else {
      RCLCPP_ERROR(
          get_logger(), "planner %s is not a valid planner. "
                        "Planner names are: %s",
          planner_id.c_str(),
          planner_ids_concat_.c_str());
    }
  }

  return nav2_msgs::msg::PathAndBoundary();
}

void
PlannerServer::publishPlan(const nav2_msgs::msg::PathAndBoundary &path_boundary)
{
  auto msg = std::make_unique<nav2_msgs::msg::PathAndBoundary>(path_boundary);
  if (
      plan_publisher_->is_activated() &&
      this->count_subscribers(plan_publisher_->get_topic_name()) > 0)
  {
    plan_publisher_->publish(std::move(msg));
  }
}

void PlannerServer::isPathValid(
  const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
  response->is_valid = true;

  if (request->path.poses.empty()) {
    response->is_valid = false;
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  unsigned int closest_point_index = 0;
  if (costmap_ros_->getRobotPose(current_pose)) {
    float current_distance = std::numeric_limits<float>::max();
    float closest_distance = current_distance;
    geometry_msgs::msg::Point current_point = current_pose.pose.position;
    for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
      geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

      current_distance = nav2_util::geometry_utils::euclidean_distance(
        current_point,
        path_point);

      if (current_distance < closest_distance) {
        closest_point_index = i;
        closest_distance = current_distance;
      }
    }

    /**
     * The lethal check starts at the closest point to avoid points that have already been passed
     * and may have become occupied
     */
    unsigned int mx = 0;
    unsigned int my = 0;
    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      costmap_->worldToMap(
        request->path.poses[i].pose.position.x,
        request->path.poses[i].pose.position.y, mx, my);
      unsigned int cost = costmap_->getCost(mx, my);

      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        response->is_valid = false;
      }
    }
  }
}

rcl_interfaces::msg::SetParametersResult
PlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
        } else {
          RCLCPP_WARN(
            get_logger(),
            "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
            " than 0.0 to turn on duration overrrun warning messages", parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_planner::PlannerServer)
