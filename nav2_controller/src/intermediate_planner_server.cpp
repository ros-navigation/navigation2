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
#include <exception>

#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "nav2_controller/intermediate_planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

IntermediatePlannerServer::IntermediatePlannerServer(
  nav2_util::LifecycleNode::SharedPtr parent,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: node_(parent),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_ids_{"GridBased"},
  default_types_{"nav2_navfn_planner/NavfnPlanner"},
  costmap_(nullptr)
{
  RCLCPP_INFO(logger_, "Creating");

  // Declare this node's parameters
  node_->declare_parameter("tolerance", 0.25);
  node_->declare_parameter("n_points_near_goal", 5);
  node_->declare_parameter("points_per_rotation", 10);
  node_->declare_parameter("planner_plugins", default_ids_);
  node_->declare_parameter("expected_planner_frequency", 1.0);
  node_->declare_parameter("publish_spiral_markers", false);

  node_->get_parameter("tolerance", tolerance_);
  node_->get_parameter("n_points_near_goal", n_points_near_goal_);
  node_->get_parameter("points_per_rotation", points_per_rotation_);
  node_->get_parameter("publish_spiral_markers", publish_spiral_markers_);

  node_->get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      node_->declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  costmap_ros_ = costmap_ros;
}

IntermediatePlannerServer::~IntermediatePlannerServer()
{
  /*
   * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
   * never called.
   */
  planners_.clear();
}

nav2_util::CallbackReturn
IntermediatePlannerServer::configure()
{
  RCLCPP_INFO(logger_, "Configuring");

  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_DEBUG(
    logger_, "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  planner_types_.resize(planner_ids_.size());

  RCLCPP_INFO(logger_, "Creating global planners");

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = nav2_util::get_plugin_type_param(
        node_, planner_ids_[i]);
      RCLCPP_INFO(
        logger_, "Found global planner plugin %s of type %s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      nav2_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        logger_, "Created intermediate planner plugin %s of type %s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->configure(node_, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        logger_, "Failed to create intermediate planner. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    logger_,
    "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  double expected_planner_frequency;
  node_->get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      logger_,
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  plan_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("intermediate_plan", 1);
  intermediate_goal_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "intermediate_goal", 1);
  if (publish_spiral_markers_) {
    spiral_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "spiral_markers", 1);
  }

  // Create the action server for path planning to a pose
  action_server_pose_ = std::make_unique<ActionServerToPose>(
    node_,
    "compute_local_path",
    std::bind(&IntermediatePlannerServer::computePlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
IntermediatePlannerServer::activate()
{
  RCLCPP_INFO(logger_, "Activating");

  plan_publisher_->on_activate();
  intermediate_goal_publisher_->on_activate();
  action_server_pose_->activate();
  if (publish_spiral_markers_) {
    spiral_markers_pub_->on_activate();
  }

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  is_path_valid_service_ = node_->create_service<nav2_msgs::srv::IsPathValid>(
    "is_intermediate_path_valid",
    std::bind(
      &IntermediatePlannerServer::isPathValid, this,
      std::placeholders::_1, std::placeholders::_2));

  // Add callback for dynamic parameters
  dyn_params_handler_ = node_->add_on_set_parameters_callback(
    std::bind(&IntermediatePlannerServer::dynamicParametersCallback, this, _1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
IntermediatePlannerServer::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating");

  action_server_pose_->deactivate();
  plan_publisher_->on_deactivate();
  intermediate_goal_publisher_->on_deactivate();
  if (publish_spiral_markers_) {
    spiral_markers_pub_->on_deactivate();
  }

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
IntermediatePlannerServer::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up");

  action_server_pose_.reset();
  plan_publisher_.reset();
  intermediate_goal_publisher_.reset();
  tf_.reset();
  spiral_markers_pub_.reset();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }

  planners_.clear();
  costmap_ = nullptr;
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool IntermediatePlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(logger_, "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void IntermediatePlannerServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool IntermediatePlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(logger_, "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void IntermediatePlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool IntermediatePlannerServer::getStartPose(
  typename std::shared_ptr<const typename T::Goal>,
  geometry_msgs::msg::PoseStamped & start)
{
  return costmap_ros_->getRobotPose(start);
}

bool IntermediatePlannerServer::transformPosesToGlobalFrame(
  geometry_msgs::msg::PoseStamped & curr_start,
  geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
  {
    return false;
  }

  return true;
}

template<typename T>
bool IntermediatePlannerServer::validatePath(
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  (void)planner_id;

  if (path.poses.size() <= 1) {
    return false;
  }

  // Check if end pose is within costmap bounds
  auto end_pose = path.poses.back().pose;
  unsigned int mx, my;
  if (!costmap_->worldToMap(end_pose.position.x, end_pose.position.y, mx, my)) {
    return false;
  }

  // Check if end pose is within tolerance
  if (nav2_util::geometry_utils::euclidean_distance(end_pose.position, goal.pose.position) >
    tolerance_)
  {
    return false;
  }

  return true;
}

void
IntermediatePlannerServer::computePlan()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  geometry_msgs::msg::PoseStamped start;

  try {
    if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_pose_, goal);

    // Create mutable local copies
    nav_msgs::msg::Path global_path = goal->global_path;
    std::string planner_id = goal->planner_id;

    // Check if received path is valid
    if (global_path.poses.empty()) {
      throw nav2_core::NoValidPathCouldBeFound("Received global path is empty");
    }

    if (global_path.header.frame_id.empty()) {
      RCLCPP_WARN(logger_, "Received global path frame_id is empty, using frame_id: map");
      global_path.header.frame_id = "map";
    }

    // Get robot pose
    if (!getStartPose<ActionToPose>(goal, start)) {
      throw nav2_core::PlannerTFError("Failed to get robot pose");
    }
    start.header.frame_id = costmap_ros_->getGlobalFrameID();

    // Transform received path into costmap frame
    nav_msgs::msg::Path transformed_path;
    transformed_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    transformed_path.header.stamp = global_path.header.stamp;
    transformed_path.poses.reserve(global_path.poses.size());
    for (auto & pose : global_path.poses) {
      pose.header.frame_id = global_path.header.frame_id;
      pose.header.stamp = rclcpp::Time(0);
      geometry_msgs::msg::PoseStamped transformed_pose;
      if (!costmap_ros_->transformPoseToGlobalFrame(pose, transformed_pose)) {
        throw nav2_core::PlannerTFError("Failed to transform global path to costmap frame");
      }
      transformed_path.poses.push_back(transformed_pose);
    }

    // Find the index of the closest pose to start on global path
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t curr_idx = 0; curr_idx < transformed_path.poses.size(); ++curr_idx) {
      double curr_dist = nav2_util::geometry_utils::euclidean_distance(
        start.pose.position, transformed_path.poses[curr_idx].pose.position);
      if (curr_dist < min_dist) {
        min_dist = curr_dist;
        closest_idx = curr_idx;
      }
    }
    RCLCPP_DEBUG(
      logger_, "Closest pose to start is at index %zu with distance %.2f",
      closest_idx, min_dist);

    // Find index (goal) where the path leaves the local path
    size_t border_idx = closest_idx;
    for (size_t curr_idx = closest_idx; curr_idx < transformed_path.poses.size(); ++curr_idx) {
      unsigned int mx, my;

      // Returns false if pose is outside the costmap
      if (!costmap_->worldToMap(
          transformed_path.poses[curr_idx].pose.position.x,
          transformed_path.poses[curr_idx].pose.position.y, mx, my))
      {
        break;
      }
      border_idx = curr_idx;
    }

    if (border_idx == closest_idx) {
      float goal_dist = nav2_util::geometry_utils::euclidean_distance(
        start.pose.position, transformed_path.poses[border_idx].pose.position);
      RCLCPP_DEBUG(
        logger_,
        "Failed to find a border pose, using end pose (dist from robot: %.2f)", goal_dist);
      border_idx = transformed_path.poses.size() - 1;
    }

    // Create goal pose and set orientation
    geometry_msgs::msg::PoseStamped goal_pose = transformed_path.poses[border_idx];
    if (border_idx + 1 < transformed_path.poses.size()) {
      geometry_msgs::msg::PoseStamped next_pose = transformed_path.poses[border_idx + 1];
      float dx = next_pose.pose.position.x - goal_pose.pose.position.x;
      float dy = next_pose.pose.position.y - goal_pose.pose.position.y;
      float yaw = std::atan2(dy, dx);
      tf2::Quaternion quat;
      quat.setRPY(0, 0, yaw);
      goal_pose.pose.orientation = tf2::toMsg(quat);
    } else {
      goal_pose.pose.orientation = transformed_path.poses.back().pose.orientation;
    }
    intermediate_goal_publisher_->publish(goal_pose);

    float dx = goal_pose.pose.position.x - start.pose.position.x;
    float dy = goal_pose.pose.position.y - start.pose.position.y;
    RCLCPP_DEBUG(logger_, "Goal dist from robot: %.2f, %.2f", dx, dy);

    if (planner_id.empty()) {
      if (planner_ids_.size() > 0) {
        planner_id = planner_ids_[0];
        RCLCPP_WARN_ONCE(
          logger_, "No planner specified in action call, will use %s",
          planner_id.c_str());
      } else {
        throw nav2_core::InvalidPlanner("No planner specified in action call");
      }
    }

    RCLCPP_DEBUG(logger_, "Getting plan...");

    std::exception_ptr ex;
    std::string ex_str;
    auto getPlanNoThrow = [this, &start, &planner_id, &ex, &ex_str](
      const geometry_msgs::msg::PoseStamped & goal, nav_msgs::msg::Path & path) -> bool
      {
        bool found_path = false;
        try {
          path = getPlan(start, goal, planner_id);
          found_path = validatePath<ActionToPose>(goal, path, planner_id);
        } catch (...) {
          ex = std::current_exception();
          ex_str = ex ? ex.__cxa_exception_type()->name() : "unknown";
        }
        return found_path;
      };

    nav_msgs::msg::Path path_out_local;
    if (!getPlanNoThrow(goal_pose, path_out_local)) {
      // If couldn't find a path to exact goal, try to find a point within tolerance
      // Search in a parametric spiral of equation
      // (tol * t * cos(t * n * 2pi), tol * t * sin(t * n * 2pi))
      // where n is the number of rotations the spiral makes.
      // It's calculated as n_points_near_goal / points_per_rotation.
      RCLCPP_INFO(
        logger_,
        "Failed to find path to exact goal (%s). Searching for a point within tolerance...",
        ex ? ex_str.c_str() : "unknown");

      visualization_msgs::msg::MarkerArray spiral_markers;
      visualization_msgs::msg::Marker spiral_marker;
      visualization_msgs::msg::Marker point_marker;

      if (publish_spiral_markers_) {
        spiral_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
        spiral_marker.header.stamp = node_->get_clock()->now();
        spiral_marker.ns = "spiral";
        spiral_marker.id = 0;
        spiral_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        spiral_marker.action = visualization_msgs::msg::Marker::ADD;
        spiral_marker.scale.x = 0.05;
        spiral_marker.color.g = 1.0;
        spiral_marker.color.a = 1.0;
        spiral_marker.points.reserve(n_points_near_goal_);

        point_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
        point_marker.header.stamp = node_->get_clock()->now();
        point_marker.ns = "points";
        point_marker.id = 0;
        point_marker.type = visualization_msgs::msg::Marker::POINTS;
        point_marker.action = visualization_msgs::msg::Marker::ADD;
        point_marker.scale.x = 0.1;
        point_marker.scale.y = 0.1;
        point_marker.color.r = 1.0;
        point_marker.color.a = 1.0;
        point_marker.points.reserve(n_points_near_goal_);
      }

      float n_rot = static_cast<float>(n_points_near_goal_) / points_per_rotation_;
      float dt = 1. / n_points_near_goal_;
      for (float t = dt; t < 1; t += dt) {
        float angle = t * n_rot * 2 * M_PI;
        float x = goal_pose.pose.position.x + tolerance_ * t * std::cos(angle);
        float y = goal_pose.pose.position.y + tolerance_ * t * std::sin(angle);

        RCLCPP_INFO(
          logger_,
          "Trying point (%.2f, %.2f) within tolerance... (t = %.2f)",
          x, y, t);

        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;

        if (publish_spiral_markers_) {
          point_marker.points.push_back(point);
          spiral_marker.points.push_back(point);
          spiral_markers.markers.push_back(spiral_marker);
          spiral_markers.markers.push_back(point_marker);
          spiral_markers_pub_->publish(spiral_markers);
        }

        unsigned int mx, my;
        if (costmap_->worldToMap(x, y, mx, my)) {
          auto cost = costmap_->getCost(mx, my);
          if (cost != nav2_costmap_2d::LETHAL_OBSTACLE &&
            cost != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          {
            geometry_msgs::msg::PoseStamped new_goal = goal_pose;
            new_goal.pose.position.x = x;
            new_goal.pose.position.y = y;
            if (getPlanNoThrow(new_goal, path_out_local)) {
              break;
            } else {
              RCLCPP_INFO(
                logger_, "Failed to plan to point (%s)",
                ex ? ex_str.c_str() : "unknown");
            }
          } else {
            RCLCPP_INFO(logger_, "Point is in an obstacle");
          }
        } else {
          RCLCPP_INFO(logger_, "Point is outside the costmap");
        }
      }

      if (!validatePath<ActionToPose>(goal_pose, path_out_local, planner_id)) {
        if (ex) {
          std::rethrow_exception(ex);
        }
        throw nav2_core::NoValidPathCouldBeFound("Failed to find path to point within tolerance");
      }
    }

    // Transform path back to global frame
    RCLCPP_DEBUG(
      logger_, "Transforming path to global (%s) frame...",
      costmap_ros_->getGlobalFrameID().c_str());
    nav_msgs::msg::Path path_out_global;
    path_out_global.header.frame_id = global_path.header.frame_id;
    path_out_global.header.stamp = node_->get_clock()->now();
    path_out_global.poses.reserve(path_out_local.poses.size());
    for (auto & pose : path_out_local.poses) {
      pose.header.frame_id = costmap_ros_->getGlobalFrameID();
      pose.header.stamp = rclcpp::Time(0);
      geometry_msgs::msg::PoseStamped transformed_pose;
      try {
        tf_->transform(pose, transformed_pose, global_path.header.frame_id);
      } catch (tf2::TransformException & ex) {
        throw nav2_core::PlannerTFError("Failed to transform local path to global frame");
      }
      path_out_global.poses.push_back(transformed_pose);
    }
    result->local_path = path_out_global;

    // Publish the plan for visualization purposes
    publishPlan(result->local_path);

    auto cycle_duration = steady_clock_.now() - start_time;
    RCLCPP_DEBUG(
      logger_, "Intermediate planner server cycle time: %.9f",
      cycle_duration.seconds());

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        logger_,
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }
    action_server_pose_->succeeded_current(result);
  } catch (nav2_core::InvalidPlanner & ex) {
    result->error_code = ActionToPoseGoal::UNKNOWN;
    RCLCPP_ERROR(logger_, "InvalidPlanner exception: %s", ex.what());
    // exceptionWarning(start, goal->goal, goal->planner_id, ex);
    // result->error_code = ActionToPoseGoal::INVALID_PLANNER;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::StartOccupied & ex) {
    result->error_code = ActionToPoseGoal::UNKNOWN;
    RCLCPP_ERROR(logger_, "StartOccupied exception: %s", ex.what());
    // exceptionWarning(start, goal->goal, goal->planner_id, ex);
    // result->error_code = ActionToPoseGoal::START_OCCUPIED;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::GoalOccupied & ex) {
    result->error_code = ActionToPoseGoal::UNKNOWN;
    RCLCPP_ERROR(logger_, "GoalOccupied exception: %s", ex.what());
    // exceptionWarning(start, goal->goal, goal->planner_id, ex);
    // result->error_code = ActionToPoseGoal::GOAL_OCCUPIED;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::NoValidPathCouldBeFound & ex) {
    result->error_code = ActionToPoseGoal::UNKNOWN;
    RCLCPP_ERROR(logger_, "NoValidPathCouldBeFound exception: %s", ex.what());
    // exceptionWarning(start, goal->goal, goal->planner_id, ex);
    // result->error_code = ActionToPoseGoal::NO_VALID_PATH;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::PlannerTimedOut & ex) {
    result->error_code = ActionToPoseGoal::UNKNOWN;
    RCLCPP_ERROR(logger_, "PlannerTimedOut exception: %s", ex.what());
    // exceptionWarning(start, goal->goal, goal->planner_id, ex);
    // result->error_code = ActionToPoseGoal::TIMEOUT;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::StartOutsideMapBounds & ex) {
    result->error_code = ActionToPoseGoal::UNKNOWN;
    RCLCPP_ERROR(logger_, "StartOutsideMapBounds exception: %s", ex.what());
    // exceptionWarning(start, goal->goal, goal->planner_id, ex);
    // result->error_code = ActionToPoseGoal::START_OUTSIDE_MAP;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::GoalOutsideMapBounds & ex) {
    result->error_code = ActionToPoseGoal::UNKNOWN;
    RCLCPP_ERROR(logger_, "GoalOutsideMapBounds exception: %s", ex.what());
    // exceptionWarning(start, goal->goal, goal->planner_id, ex);
    // result->error_code = ActionToPoseGoal::GOAL_OUTSIDE_MAP;
    action_server_pose_->terminate_current(result);
  } catch (nav2_core::PlannerTFError & ex) {
    result->error_code = ActionToPoseGoal::UNKNOWN;
    RCLCPP_ERROR(logger_, "PlannerTFError exception: %s", ex.what());
    // exceptionWarning(start, goal->goal, goal->planner_id, ex);
    // result->error_code = ActionToPoseGoal::TF_ERROR;
    action_server_pose_->terminate_current(result);
  } catch (std::exception & ex) {
    result->error_code = ActionToPoseGoal::UNKNOWN;
    RCLCPP_ERROR(logger_, "Exception: %s", ex.what());
    // exceptionWarning(start, goal->goal, goal->planner_id, ex);
    // result->error_code = ActionToPoseGoal::UNKNOWN;
    action_server_pose_->terminate_current(result);
  }
}

nav_msgs::msg::Path
IntermediatePlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id)
{
  RCLCPP_DEBUG(
    logger_, "Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  if (planners_.find(planner_id) != planners_.end()) {
    return planners_[planner_id]->createPlan(start, goal);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
        logger_, "No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    } else {
      RCLCPP_ERROR(
        logger_, "planner %s is not a valid planner. "
        "Planner names are: %s", planner_id.c_str(),
        planner_ids_concat_.c_str());
      throw nav2_core::InvalidPlanner("Planner id " + planner_id + " is invalid");
    }
  }

  return nav_msgs::msg::Path();
}

void
IntermediatePlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

void IntermediatePlannerServer::isPathValid(
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
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
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
IntermediatePlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
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
            logger_,
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

void IntermediatePlannerServer::exceptionWarning(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id,
  const std::exception & ex)
{
  RCLCPP_WARN(
    logger_, "%s plugin failed to plan from (%.2f, %.2f) to (%0.2f, %.2f): \"%s\"",
    planner_id.c_str(),
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y,
    ex.what());
}

}  // namespace nav2_controller
