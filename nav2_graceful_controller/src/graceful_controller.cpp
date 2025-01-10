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

#include <memory>
#include <mutex>

#include "angles/angles.h"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_graceful_controller/graceful_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace nav2_graceful_controller
{

void GracefulController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  // Handles storage and dynamic configuration of parameters.
  // Returns pointer to data current param settings.
  param_handler_ = std::make_unique<ParameterHandler>(
    node, plugin_name_, logger_,
    costmap_ros_->getCostmap()->getSizeInMetersX());
  params_ = param_handler_->getParams();

  // Handles global path transformations
  path_handler_ = std::make_unique<PathHandler>(
    tf2::durationFromSec(params_->transform_tolerance), tf_buffer_, costmap_ros_);

  // Handles the control law to generate the velocity commands
  control_law_ = std::make_unique<SmoothControlLaw>(
    params_->k_phi, params_->k_delta, params_->beta, params_->lambda, params_->slowdown_radius,
    params_->v_linear_min, params_->v_linear_max, params_->v_angular_max);

  // Initialize footprint collision checker
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_ros_->getCostmap());

  // Publishers
  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  motion_target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("motion_target", 1);
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

  // Add proper orientations to plan, if needed
  validateOrientations(transformed_plan.poses);

  // Publish plan for visualization
  transformed_plan_pub_->publish(transformed_plan);

  // Transform local frame to global frame to use in collision checking
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
    throw ex;
  }

  // Compute distance to goal as the path's integrated distance to account for path curvatures
  double dist_to_goal = nav2_util::geometry_utils::calculate_path_length(transformed_plan);

  // If we've reached the XY goal tolerance, just rotate
  if (dist_to_goal < goal_dist_tolerance_ || goal_reached_) {
    goal_reached_ = true;
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    // Check for collisions between our current pose and goal pose
    size_t num_steps = fabs(angle_to_goal) / params_->in_place_collision_resolution;
    // Need to check at least the end pose
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
    // Compute velocity if rotation is possible
    if (collision_free) {
      cmd_vel.twist = rotateToTarget(angle_to_goal);
      return cmd_vel;
    }
    // Else, fall through and see if we should follow control law longer
  }

  // Precompute distance to candidate poses
  std::vector<double> target_distances;
  computeDistanceAlongPath(transformed_plan.poses, target_distances);

  // Work back from the end of plan to find valid target pose
  for (int i = transformed_plan.poses.size() - 1; i >= 0; --i) {
    // Underlying control law needs a single target pose, which should:
    //  * Be as far away as possible from the robot (for smoothness)
    //  * But no further than the max_lookahed_ distance
    //  * Be feasible to reach in a collision free manner
    geometry_msgs::msg::PoseStamped target_pose = transformed_plan.poses[i];
    double dist_to_target = target_distances[i];

    // Continue if target_pose is too far away from robot
    if (dist_to_target > params_->max_lookahead) {continue;}

    if (dist_to_goal < params_->max_lookahead) {
      if (params_->prefer_final_rotation) {
        // Avoid unstability and big sweeping turns at the end of paths by
        // ignoring final heading
        double yaw = std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);
        target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      }
    } else if (dist_to_target < params_->min_lookahead) {
      // Make sure target is far enough away to avoid instability
      break;
    }

    // Flip the orientation of the motion target if the robot is moving backwards
    bool reversing = false;
    if (params_->allow_backward && target_pose.pose.position.x < 0.0) {
      reversing = true;
      target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(target_pose.pose.orientation) + M_PI);
    }

    // Actually simulate our path
    nav_msgs::msg::Path local_plan;
    if (simulateTrajectory(target_pose, costmap_transform, local_plan, cmd_vel, reversing)) {
      // Successfully simulated to target_pose - compute velocity at this moment
      // Publish the selected target_pose
      motion_target_pub_->publish(target_pose);
      // Publish marker for slowdown radius around motion target for debugging / visualization
      auto slowdown_marker = nav2_graceful_controller::createSlowdownMarker(
        target_pose, params_->slowdown_radius);
      slowdown_pub_->publish(slowdown_marker);
      // Publish the local plan
      local_plan.header = transformed_plan.header;
      local_plan_pub_->publish(local_plan);
      // Successfully found velocity command
      return cmd_vel;
    }
  }

  throw nav2_core::NoValidControl("Collision detected in trajectory");
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
      // Speed limit is expressed in % from maximum speed of robot
      params_->v_linear_max = std::max(
        params_->v_linear_max_initial * speed_limit / 100.0, params_->v_linear_min);
      params_->v_angular_max = params_->v_angular_max_initial * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in m/s
      params_->v_linear_max = std::max(speed_limit, params_->v_linear_min);
      // Limit the angular velocity to be proportional to the linear velocity
      params_->v_angular_max = params_->v_angular_max_initial *
        speed_limit / params_->v_linear_max_initial;
    }
  }
}

bool GracefulController::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & motion_target,
  const geometry_msgs::msg::TransformStamped & costmap_transform,
  nav_msgs::msg::Path & trajectory,
  geometry_msgs::msg::TwistStamped & cmd_vel,
  bool backward)
{
  trajectory.poses.clear();

  // First pose is robot current pose
  geometry_msgs::msg::PoseStamped next_pose;
  next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
  next_pose.pose.orientation.w = 1.0;

  // Should we simulate rotation initially?
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

  // Set max iter to avoid infinite loop
  unsigned int max_iter = 3 *
    std::hypot(motion_target.pose.position.x, motion_target.pose.position.y) / resolution_;

  // Generate path
  do{
    if (sim_initial_rotation) {
      // Compute rotation velocity
      double next_pose_yaw = tf2::getYaw(next_pose.pose.orientation);
      auto cmd = rotateToTarget(angle_to_target - next_pose_yaw);

      // If this is first iteration, this is our current target velocity
      if (trajectory.poses.empty()) {cmd_vel.twist = cmd;}

      // Are we done simulating initial rotation?
      if (fabs(angle_to_target - next_pose_yaw) < params_->initial_rotation_tolerance) {
        sim_initial_rotation = false;
      }

      // Forward simulate rotation command
      next_pose_yaw += cmd_vel.twist.angular.z * dt;
      next_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(next_pose_yaw);
    } else {
      // If this is first iteration, this is our current target velocity
      if (trajectory.poses.empty()) {
        cmd_vel.twist = control_law_->calculateRegularVelocity(
          motion_target.pose, next_pose.pose, backward);
      }

      // Apply velocities to calculate next pose
      next_pose.pose = control_law_->calculateNextPose(
        dt, motion_target.pose, next_pose.pose, backward);
    }

    // Add the pose to the trajectory for visualization
    trajectory.poses.push_back(next_pose);

    // Check for collision
    geometry_msgs::msg::PoseStamped global_pose;
    tf2::doTransform(next_pose, global_pose, costmap_transform);
    if (inCollision(
        global_pose.pose.position.x, global_pose.pose.position.y,
        tf2::getYaw(global_pose.pose.orientation)))
    {
      return false;
    }

    // Check if we reach the goal
    distance = nav2_util::geometry_utils::euclidean_distance(motion_target.pose, next_pose.pose);
  }while(distance > resolution_ && trajectory.poses.size() < max_iter);

  return true;
}

geometry_msgs::msg::Twist GracefulController::rotateToTarget(double angle_to_target)
{
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.0;
  vel.angular.z = params_->rotation_scaling_factor * angle_to_target * params_->v_angular_max;
  vel.angular.z = std::copysign(1.0, vel.angular.z) * std::max(abs(vel.angular.z),
      params_->v_angular_min_in_place);
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

  // Calculate the cost of the footprint at the robot's current position depending
  // on the shape of the footprint
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
  // Do the first pose from robot
  double d = std::hypot(poses[0].pose.position.x, poses[0].pose.position.y);
  distances[0] = d;
  // Compute remaining poses
  for (size_t i = 1; i < poses.size(); ++i) {
    d += nav2_util::geometry_utils::euclidean_distance(poses[i - 1].pose, poses[i].pose);
    distances[i] = d;
  }
}

void GracefulController::validateOrientations(
  std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  // We never change the orientation of the first & last pose
  // So we need at least three poses to do anything here
  if (path.size() < 3) {return;}

  // Check if we actually need to add orientations
  double initial_yaw = tf2::getYaw(path[1].pose.orientation);
  for (size_t i = 2; i < path.size() - 1; ++i) {
    double this_yaw = tf2::getYaw(path[i].pose.orientation);
    if (angles::shortest_angular_distance(this_yaw, initial_yaw) > 1e-6) {return;}
  }

  // For each pose, point at the next one
  // NOTE: control loop will handle reversing logic
  for (size_t i = 0; i < path.size() - 1; ++i) {
    // Get relative yaw angle
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
