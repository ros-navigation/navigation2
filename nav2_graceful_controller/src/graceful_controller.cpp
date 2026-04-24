// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_graceful_controller/graceful_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/exceptions.hpp"

namespace nav2_graceful_controller
{

void GracefulController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  param_handler_ = std::make_unique<ParameterHandler>(
    node, plugin_name_, logger_,
    costmap_ros_->getCostmap()->getSizeInMetersX());
  params_ = param_handler_->getParams();

  path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_buffer_, costmap_ros_);

  control_law_ = std::make_unique<SmoothControlLaw>(
    params_->k_phi, params_->k_delta, params_->beta, params_->lambda, params_->slowdown_radius,
    params_->v_linear_min, params_->v_linear_max, params_->v_angular_max);

  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_ros_->getCostmap());

  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  motion_target_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("motion_target", 1);
  slowdown_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("slowdown", 1);

  RCLCPP_INFO(logger_, "Configured Graceful Motion Controller: %s", plugin_name_.c_str());
}

void GracefulController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type graceful_controller::GracefulController",
    plugin_name_.c_str());
  transformed_plan_pub_.reset();
  local_plan_pub_.reset();
  motion_target_pub_.reset();
  slowdown_pub_.reset();
  collision_checker_.reset();
  path_handler_.reset();
  param_handler_.reset();
  control_law_.reset();
}

void GracefulController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type nav2_graceful_controller::GracefulController",
    plugin_name_.c_str());
  transformed_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  motion_target_pub_->on_activate();
  slowdown_pub_->on_activate();
}

void GracefulController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type nav2_graceful_controller::GracefulController",
    plugin_name_.c_str());
  transformed_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  motion_target_pub_->on_deactivate();
  slowdown_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped GracefulController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tolerance_ = pose_tolerance.position.x;
  }

  // Update the smooth control law with the new params
  control_law_->setCurvatureConstants(
    params_->k_phi, params_->k_delta, params_->beta, params_->lambda);
  control_law_->setSlowdownRadius(params_->slowdown_radius);
  control_law_->setSpeedLimit(params_->v_linear_min, params_->v_linear_max, params_->v_angular_max);

  // Transform path to robot base frame
  auto transformed_plan = path_handler_->transformGlobalPlan(
    pose, params_->max_robot_pose_search_dist);

  // If planner emitted constant-yaw waypoints, retangent them so delta is meaningful
  validateOrientations(transformed_plan.poses);

  // Publish plan for visualization
  transformed_plan_pub_->publish(transformed_plan);

  // Transform local frame to global frame for collision checking
  geometry_msgs::msg::TransformStamped costmap_transform;
  try {
    costmap_transform = tf_buffer_->lookupTransform(
      costmap_ros_->getGlobalFrameID(), costmap_ros_->getBaseFrameID(),
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger_, "Could not transform %s to %s: %s",
      costmap_ros_->getBaseFrameID().c_str(), costmap_ros_->getGlobalFrameID().c_str(),
      ex.what());
    throw nav2_core::PlannerException(ex.what());
  }

  // Integrated distance to end of path
  double dist_to_goal = nav2_util::geometry_utils::calculate_path_length(transformed_plan);

  // If inside XY goal tolerance, try a collision-checked in-place rotation to goal heading.
  // If the rotation would collide, fall through and keep running the control law instead of
  // getting stuck.
  if (dist_to_goal < goal_dist_tolerance_ || goal_reached_) {
    goal_reached_ = true;
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    size_t num_steps =
      static_cast<size_t>(fabs(angle_to_goal) / params_->in_place_collision_resolution);
    num_steps = std::max(static_cast<size_t>(1), num_steps);
    bool collision_free = true;
    for (size_t i = 1; i <= num_steps; ++i) {
      double step = static_cast<double>(i) / static_cast<double>(num_steps);
      double yaw = step * angle_to_goal;
      geometry_msgs::msg::PoseStamped next_pose;
      next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
      next_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      geometry_msgs::msg::PoseStamped costmap_pose;
      tf2::doTransform(next_pose, costmap_pose, costmap_transform);
      if (inCollision(
          costmap_pose.pose.position.x, costmap_pose.pose.position.y,
          tf2::getYaw(costmap_pose.pose.orientation)))
      {
        collision_free = false;
        break;
      }
    }
    if (collision_free) {
      cmd_vel.twist = rotateToTarget(angle_to_goal);
      return cmd_vel;
    }
    // Else fall through to control-law path following
  }

  // Precompute integrated distance to each candidate pose on path
  std::vector<double> target_distances;
  computeDistanceAlongPath(transformed_plan.poses, target_distances);

  // Walk candidates from end of plan back toward robot. Pick the FARTHEST target whose
  // simulated trajectory is collision-free. min_lookahead caps 1/r -> keeps the curvature
  // bounded; max_lookahead keeps the target nearby so we don't sweep wide.
  for (int i = static_cast<int>(transformed_plan.poses.size()) - 1; i >= 0; --i) {
    geometry_msgs::msg::PoseStamped target_pose = transformed_plan.poses[i];
    double dist_to_target = target_distances[i];

    if (dist_to_target > params_->max_lookahead) {continue;}

    if (dist_to_goal < params_->max_lookahead) {
      if (params_->prefer_final_rotation) {
        // Near goal: ignore final heading, just point at the target to avoid big sweeping arcs.
        // Final heading alignment is handled by the goal-reached in-place rotation above.
        double yaw = std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);
        target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      }
    } else if (dist_to_target < params_->min_lookahead) {
      // Target too close: 1/r would blow up. Stop iterating (no collision-free target available).
      break;
    }

    // Backward motion: flip target orientation so the control law treats it as a forward
    // problem in a mirrored frame.
    bool reversing = false;
    if (params_->allow_backward && target_pose.pose.position.x < 0.0) {
      reversing = true;
      target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(target_pose.pose.orientation) + M_PI);
    }

    // Simulate forward from origin in base frame; first iteration sets cmd_vel.
    nav_msgs::msg::Path local_plan;
    if (simulateTrajectory(target_pose, costmap_transform, local_plan, cmd_vel, reversing)) {
      motion_target_pub_->publish(
        nav2_graceful_controller::createMotionTargetMsg(target_pose));
      auto slowdown_marker = nav2_graceful_controller::createSlowdownMarker(
        target_pose, params_->slowdown_radius);
      slowdown_pub_->publish(slowdown_marker);
      local_plan.header = transformed_plan.header;
      local_plan_pub_->publish(local_plan);
      return cmd_vel;
    }
  }

  throw nav2_core::PlannerException("Collision detected in trajectory");
}

void GracefulController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_->setPlan(path);
  goal_reached_ = false;
  do_initial_rotation_ = true;
}

void GracefulController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> param_lock(param_handler_->getMutex());

  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    params_->v_linear_max = params_->v_linear_max_initial;
    params_->v_angular_max = params_->v_angular_max_initial;
  } else {
    if (percentage) {
      params_->v_linear_max = std::max(
        params_->v_linear_max_initial * speed_limit / 100.0, params_->v_linear_min);
      params_->v_angular_max = params_->v_angular_max_initial * speed_limit / 100.0;
    } else {
      params_->v_linear_max = std::max(speed_limit, params_->v_linear_min);
      params_->v_angular_max = params_->v_angular_max_initial *
        speed_limit / params_->v_linear_max_initial;
    }
  }
}

geometry_msgs::msg::PoseStamped GracefulController::getMotionTarget(
  const double & motion_target_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Legacy helper. Not used in the new control flow (see computeVelocityCommands).
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return std::hypot(ps.pose.position.x, ps.pose.position.y) >= motion_target_dist;
    });
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }
  return *goal_pose_it;
}

bool GracefulController::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & motion_target,
  const geometry_msgs::msg::TransformStamped & costmap_transform,
  nav_msgs::msg::Path & trajectory,
  geometry_msgs::msg::TwistStamped & cmd_vel,
  bool backward)
{
  trajectory.poses.clear();

  // Simulate from origin in base frame
  geometry_msgs::msg::PoseStamped next_pose;
  next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
  next_pose.pose.orientation.w = 1.0;

  // Should we do an in-sim initial rotation first?
  bool sim_initial_rotation = do_initial_rotation_ && params_->initial_rotation;
  double angle_to_target =
    std::atan2(motion_target.pose.position.y, motion_target.pose.position.x);
  if (fabs(angle_to_target) < params_->initial_rotation_tolerance) {
    sim_initial_rotation = false;
    do_initial_rotation_ = false;
  }

  double distance = std::numeric_limits<double>::max();
  double resolution_ = costmap_ros_->getCostmap()->getResolution();
  double dt = (params_->v_linear_max > 0.0) ? resolution_ / params_->v_linear_max : 0.0;

  unsigned int max_iter = 3 *
    std::hypot(motion_target.pose.position.x, motion_target.pose.position.y) / resolution_;

  do {
    if (sim_initial_rotation) {
      double next_pose_yaw = tf2::getYaw(next_pose.pose.orientation);
      auto cmd = rotateToTarget(angle_to_target - next_pose_yaw);

      // First iteration -> this is the velocity command we return from computeVelocityCommands.
      if (trajectory.poses.empty()) {cmd_vel.twist = cmd;}

      if (fabs(angle_to_target - next_pose_yaw) < params_->initial_rotation_tolerance) {
        sim_initial_rotation = false;
      }

      next_pose_yaw += cmd_vel.twist.angular.z * dt;
      next_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(next_pose_yaw);
    } else {
      if (trajectory.poses.empty()) {
        cmd_vel.twist = control_law_->calculateRegularVelocity(
          motion_target.pose, next_pose.pose, backward);
      }
      next_pose.pose = control_law_->calculateNextPose(
        dt, motion_target.pose, next_pose.pose, backward);
    }

    trajectory.poses.push_back(next_pose);

    geometry_msgs::msg::PoseStamped global_pose;
    tf2::doTransform(next_pose, global_pose, costmap_transform);
    if (inCollision(
        global_pose.pose.position.x, global_pose.pose.position.y,
        tf2::getYaw(global_pose.pose.orientation)))
    {
      return false;
    }

    distance = nav2_util::geometry_utils::euclidean_distance(motion_target.pose, next_pose.pose);
  } while (distance > resolution_ && trajectory.poses.size() < max_iter);

  return true;
}

geometry_msgs::msg::Twist GracefulController::rotateToTarget(const double & angle_to_target)
{
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.0;
  vel.angular.z = params_->rotation_scaling_factor * angle_to_target * params_->v_angular_max;
  vel.angular.z = std::copysign(1.0, vel.angular.z) *
    std::max(fabs(vel.angular.z), params_->v_angular_min_in_place);
  return vel;
}

bool GracefulController::inCollision(const double & x, const double & y, const double & theta)
{
  unsigned int mx, my;
  if (!costmap_ros_->getCostmap()->worldToMap(x, y, mx, my)) {
    RCLCPP_WARN(
      logger_, "The path is not in the costmap. Cannot check for collisions. "
      "Proceed at your own risk, slow the robot, or increase your costmap size.");
    return false;
  }

  bool is_tracking_unknown =
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  bool consider_footprint = !costmap_ros_->getUseRadius();

  double footprint_cost;
  if (consider_footprint) {
    footprint_cost = collision_checker_->footprintCostAtPose(
      x, y, theta, costmap_ros_->getRobotFootprint());
  } else {
    footprint_cost = collision_checker_->pointCost(mx, my);
  }

  switch (static_cast<unsigned char>(footprint_cost)) {
    case (nav2_costmap_2d::LETHAL_OBSTACLE):
      return true;
    case (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
      return consider_footprint ? false : true;
    case (nav2_costmap_2d::NO_INFORMATION):
      return is_tracking_unknown ? false : true;
  }

  return false;
}

void GracefulController::computeDistanceAlongPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses,
  std::vector<double> & distances)
{
  distances.resize(poses.size());
  if (poses.empty()) {return;}

  // First pose: distance from robot origin (0,0) in base frame
  double d = std::hypot(poses[0].pose.position.x, poses[0].pose.position.y);
  distances[0] = d;
  for (size_t i = 1; i < poses.size(); ++i) {
    d += nav2_util::geometry_utils::euclidean_distance(poses[i - 1].pose, poses[i].pose);
    distances[i] = d;
  }
}

void GracefulController::validateOrientations(
  std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  // Leave first/last orientation alone. Need >= 3 poses to do anything.
  if (path.size() < 3) {return;}

  // If any intermediate pose already has a distinct yaw from poses[1], the planner set
  // meaningful orientations -> leave them alone.
  double initial_yaw = tf2::getYaw(path[1].pose.orientation);
  for (size_t i = 2; i < path.size() - 1; ++i) {
    double this_yaw = tf2::getYaw(path[i].pose.orientation);
    if (angles::shortest_angular_distance(this_yaw, initial_yaw) > 1e-6) {return;}
  }

  // Constant-yaw plan -> retangent each pose (except last) to point at next pose.
  // Reversing logic is handled downstream in the control loop.
  for (size_t i = 0; i < path.size() - 1; ++i) {
    double dx = path[i + 1].pose.position.x - path[i].pose.position.x;
    double dy = path[i + 1].pose.position.y - path[i].pose.position.y;
    double yaw = std::atan2(dy, dx);
    path[i].pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  }
}

}  // namespace nav2_graceful_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_graceful_controller::GracefulController,
  nav2_core::Controller)
