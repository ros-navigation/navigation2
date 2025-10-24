// Copyright (c) 2021 Samsung Research America
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

#include <algorithm>
#include <string>
#include <memory>
#include <vector>
#include <utility>

#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"

using rcl_interfaces::msg::ParameterType;

namespace nav2_rotation_shim_controller
{

RotationShimController::RotationShimController()
: lp_loader_("nav2_core", "nav2_core::Controller"),
  primary_controller_(nullptr),
  path_updated_(false),
  in_rotation_(false)
{
}

void RotationShimController::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  position_goal_checker_ = std::make_unique<nav2_controller::PositionGoalChecker>();
  position_goal_checker_->initialize(parent, plugin_name_ + ".position_checker", costmap_ros);
  plugin_name_ = name;
  node_ = parent;
  auto node = parent.lock();

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<ParameterHandler>(
    node, plugin_name_, logger_);
  params_ = param_handler_->getParams();

  try {
    primary_controller_ = lp_loader_.createUniqueInstance(params_->primary_controller);
    RCLCPP_INFO(
      logger_, "Created internal controller for rotation shimming: %s of type %s",
      plugin_name_.c_str(), params_->primary_controller.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_,
      "Failed to create internal controller for rotation shimming. Exception: %s", ex.what());
    return;
  }

  primary_controller_->configure(parent, name + ".PrimaryController", tf, costmap_ros);

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_ros->getCostmap());
}

void RotationShimController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "nav2_rotation_shim_controller::RotationShimController",
    plugin_name_.c_str());

  primary_controller_->activate();
  in_rotation_ = false;
  last_angular_vel_ = std::numeric_limits<double>::max();
  position_goal_checker_->reset();
  param_handler_->activate();
}

void RotationShimController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "nav2_rotation_shim_controller::RotationShimController",
    plugin_name_.c_str());

  primary_controller_->deactivate();
  param_handler_->deactivate();
}

void RotationShimController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type "
    "nav2_rotation_shim_controller::RotationShimController",
    plugin_name_.c_str());

  primary_controller_->cleanup();
  primary_controller_.reset();
  position_goal_checker_.reset();
}

geometry_msgs::msg::TwistStamped RotationShimController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // Rotate to goal heading when in goal xy tolerance
  if (params_->rotate_to_goal_heading) {
    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

    try {
      geometry_msgs::msg::PoseStamped sampled_pt_goal = getSampledPathGoal();

      if (!nav2_util::transformPoseInTargetFrame(
          sampled_pt_goal, sampled_pt_goal, *tf_,
          pose.header.frame_id))
      {
        throw nav2_core::ControllerTFError("Failed to transform pose to base frame!");
      }

      geometry_msgs::msg::Pose pose_tolerance;
      geometry_msgs::msg::Twist vel_tolerance;
      goal_checker->getTolerances(pose_tolerance, vel_tolerance);
      position_goal_checker_->setXYGoalTolerance(pose_tolerance.position.x);

      if (position_goal_checker_->isGoalReached(pose.pose, sampled_pt_goal.pose, velocity)) {
        double pose_yaw = tf2::getYaw(pose.pose.orientation);
        double goal_yaw = tf2::getYaw(sampled_pt_goal.pose.orientation);

        double angular_distance_to_heading = angles::shortest_angular_distance(pose_yaw, goal_yaw);

        auto cmd_vel = computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
        last_angular_vel_ = cmd_vel.twist.angular.z;
        return cmd_vel;
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_INFO(
        logger_,
        "Rotation Shim Controller was unable to find a goal point,"
        " a rotational collision was detected, or TF failed to transform"
        " into base frame! what(): %s", e.what());
    }
  }

  if (path_updated_) {
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
    try {
      auto sampled_pt = getSampledPathPt();
      double angular_distance_to_heading;
      if (params_->use_path_orientations) {
        angular_distance_to_heading = angles::shortest_angular_distance(
          tf2::getYaw(pose.pose.orientation),
          tf2::getYaw(sampled_pt.pose.orientation));
      } else {
        geometry_msgs::msg::Pose sampled_pt_base = transformPoseToBaseFrame(sampled_pt);
        angular_distance_to_heading = std::atan2(
          sampled_pt_base.position.y,
          sampled_pt_base.position.x);
      }

      double angular_thresh =
        in_rotation_ ? params_->angular_disengage_threshold : params_->angular_dist_threshold;
      if (abs(angular_distance_to_heading) > angular_thresh) {
        RCLCPP_DEBUG(
          logger_,
          "Robot is not within the new path's rough heading, rotating to heading...");
        in_rotation_ = true;
        auto cmd_vel = computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
        last_angular_vel_ = cmd_vel.twist.angular.z;
        return cmd_vel;
      } else {
        RCLCPP_DEBUG(
          logger_,
          "Robot is at the new path's rough heading, passing to controller");
        path_updated_ = false;
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_DEBUG(
        logger_,
        "Rotation Shim Controller was unable to find a sampling point,"
        " a rotational collision was detected, or TF failed to transform"
        " into base frame! what(): %s", e.what());
      path_updated_ = false;
    }
  }

  // If at this point, use the primary controller to path track
  in_rotation_ = false;
  auto cmd_vel = primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
  last_angular_vel_ = cmd_vel.twist.angular.z;
  return cmd_vel;
}

geometry_msgs::msg::PoseStamped RotationShimController::getSampledPathPt()
{
  if (current_path_.poses.size() < 2) {
    throw nav2_core::ControllerException(
            "Path is too short to find a valid sampled path point for rotation.");
  }

  geometry_msgs::msg::Pose start = current_path_.poses.front().pose;
  double dx, dy;

  // Find the first point at least sampling distance away
  for (unsigned int i = 1; i != current_path_.poses.size(); i++) {
    dx = current_path_.poses[i].pose.position.x - start.position.x;
    dy = current_path_.poses[i].pose.position.y - start.position.y;
    if (hypot(dx, dy) >= params_->forward_sampling_distance) {
      current_path_.poses[i].header.frame_id = current_path_.header.frame_id;
      current_path_.poses[i].header.stamp = clock_->now();  // Get current time transformation
      return current_path_.poses[i];
    }
  }

  auto goal = current_path_.poses.back();
  goal.header.frame_id = current_path_.header.frame_id;
  goal.header.stamp = clock_->now();
  return goal;
}

geometry_msgs::msg::PoseStamped RotationShimController::getSampledPathGoal()
{
  if (current_path_.poses.empty()) {
    throw nav2_core::InvalidPath("Path is empty - cannot find a goal point");
  }

  auto goal = current_path_.poses.back();
  goal.header.frame_id = current_path_.header.frame_id;
  goal.header.stamp = clock_->now();
  return goal;
}

geometry_msgs::msg::Pose
RotationShimController::transformPoseToBaseFrame(const geometry_msgs::msg::PoseStamped & pt)
{
  geometry_msgs::msg::PoseStamped pt_base;
  if (!nav2_util::transformPoseInTargetFrame(pt, pt_base, *tf_, costmap_ros_->getBaseFrameID())) {
    throw nav2_core::ControllerTFError("Failed to transform pose to base frame!");
  }
  return pt_base.pose;
}

geometry_msgs::msg::TwistStamped
RotationShimController::computeRotateToHeadingCommand(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  auto current = params_->closed_loop ? velocity.angular.z : last_angular_vel_;
  if (current == std::numeric_limits<double>::max()) {
    current = 0.0;
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
  const double angular_vel = sign * params_->rotate_to_heading_angular_vel;
  const double & dt = params_->control_duration;
  const double min_feasible_angular_speed = current - params_->max_angular_accel * dt;
  const double max_feasible_angular_speed = current + params_->max_angular_accel * dt;
  cmd_vel.twist.angular.z =
    std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  // Check if we need to slow down to avoid overshooting
  double max_vel_to_stop = std::sqrt(2 * params_->max_angular_accel *
    fabs(angular_distance_to_heading));
  if (fabs(cmd_vel.twist.angular.z) > max_vel_to_stop) {
    cmd_vel.twist.angular.z = sign * max_vel_to_stop;
  }

  isCollisionFree(cmd_vel, angular_distance_to_heading, pose);
  return cmd_vel;
}

void RotationShimController::isCollisionFree(
  const geometry_msgs::msg::TwistStamped & cmd_vel,
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Simulate rotation ahead by time in control frequency increments
  double simulated_time = 0.0;
  double initial_yaw = tf2::getYaw(pose.pose.orientation);
  double yaw = 0.0;
  double footprint_cost = 0.0;
  double remaining_rotation_before_thresh =
    fabs(angular_distance_to_heading) - params_->angular_dist_threshold;

  while (simulated_time < params_->simulate_ahead_time) {
    simulated_time += params_->control_duration;
    yaw = initial_yaw + cmd_vel.twist.angular.z * simulated_time;

    // Stop simulating past the point it would be passed onto the primary controller
    if (angles::shortest_angular_distance(yaw, initial_yaw) >= remaining_rotation_before_thresh) {
      break;
    }

    using namespace nav2_costmap_2d;  // NOLINT
    footprint_cost = collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y,
      yaw, costmap_ros_->getRobotFootprint());

    if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
      throw nav2_core::NoValidControl(
              "RotationShimController detected a potential collision ahead!");
    }

    if (footprint_cost >= static_cast<double>(LETHAL_OBSTACLE)) {
      throw nav2_core::NoValidControl("RotationShimController detected collision ahead!");
    }
  }
}

bool RotationShimController::isGoalChanged(const nav_msgs::msg::Path & path)
{
  // Return true if rotating or if the current path is empty
  if (in_rotation_ || current_path_.poses.empty()) {
    return true;
  }

  // Check if the last pose of the current and new paths differ
  return current_path_.poses.back().pose != path.poses.back().pose;
}

void RotationShimController::setPlan(const nav_msgs::msg::Path & path)
{
  path_updated_ = params_->rotate_to_heading_once ? isGoalChanged(path) : true;
  current_path_ = path;
  primary_controller_->setPlan(path);
  position_goal_checker_->reset();
}

void RotationShimController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  primary_controller_->setSpeedLimit(speed_limit, percentage);
}

void RotationShimController::reset()
{
  last_angular_vel_ = std::numeric_limits<double>::max();
  primary_controller_->reset();
  position_goal_checker_->reset();
}

}  // namespace nav2_rotation_shim_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_rotation_shim_controller::RotationShimController,
  nav2_core::Controller)
