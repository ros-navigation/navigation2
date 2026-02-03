// Copyright 2024 Nav2 Contributors
// Licensed under the Apache License, Version 2.0

#include "example_controller/pure_pursuit_controller.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace example_controller
{

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("Failed to lock parent node");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  global_frame_ = costmap_ros_->getGlobalFrameID();
  robot_frame_ = costmap_ros_->getBaseFrameID();

  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_linear_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_linear_vel", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_velocity_scaled_lookahead", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_allowed_time_to_collision", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_collision_detection", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785));

  // Get parameters
  node->get_parameter(name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(name_ + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(name_ + ".use_velocity_scaled_lookahead", use_velocity_scaled_lookahead_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".max_allowed_time_to_collision", max_allowed_time_to_collision_);
  node->get_parameter(name_ + ".use_collision_detection", use_collision_detection_);
  node->get_parameter(name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(name_ + ".rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  node->get_parameter(name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);

  // Initialize speed limit
  speed_limit_ = max_linear_vel_;
  speed_limit_is_percentage_ = false;

  // Create publisher for lookahead visualization
  lookahead_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    name_ + "/lookahead_point", 1);

  RCLCPP_INFO(
    logger_,
    "Configured Pure Pursuit controller '%s': lookahead=%.2f, max_vel=%.2f",
    name_.c_str(), lookahead_dist_, max_linear_vel_);
}

void PurePursuitController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller: %s", name_.c_str());
  lookahead_pub_.reset();
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(logger_, "Activating controller: %s", name_.c_str());
  lookahead_pub_->on_activate();
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating controller: %s", name_.c_str());
  lookahead_pub_->on_deactivate();
}

void PurePursuitController::setPath(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  global_plan_ = path;
  RCLCPP_DEBUG(logger_, "Received path with %zu poses", path.poses.size());
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock(path_mutex_);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = robot_frame_;

  // Check for empty path
  if (global_plan_.poses.empty()) {
    throw nav2_core::ControllerException("Received empty path");
  }

  // Transform path to robot frame
  nav_msgs::msg::Path transformed_plan = transformGlobalPlan(pose);
  if (transformed_plan.poses.empty()) {
    throw nav2_core::ControllerException("Transformed plan is empty");
  }

  // Check if goal is reached
  if (goal_checker != nullptr) {
    geometry_msgs::msg::Pose goal = global_plan_.poses.back().pose;
    geometry_msgs::msg::Twist zero_vel;
    if (goal_checker->isGoalReached(pose.pose, goal, zero_vel)) {
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      return cmd_vel;
    }
  }

  // Calculate lookahead distance (optionally scaled by velocity)
  double current_lookahead = lookahead_dist_;
  if (use_velocity_scaled_lookahead_) {
    current_lookahead = std::abs(velocity.linear.x) * lookahead_time_;
    current_lookahead = std::clamp(current_lookahead, min_lookahead_dist_, max_lookahead_dist_);
  }

  // Find lookahead point
  geometry_msgs::msg::PoseStamped lookahead = getLookaheadPoint(transformed_plan, current_lookahead);
  publishLookaheadMarker(lookahead);

  // Check angle to lookahead - rotate in place if needed
  double angle_to_lookahead = std::atan2(
    lookahead.pose.position.y,
    lookahead.pose.position.x);

  if (use_rotate_to_heading_ && std::abs(angle_to_lookahead) > rotate_to_heading_min_angle_) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = std::copysign(rotate_to_heading_angular_vel_, angle_to_lookahead);
    RCLCPP_DEBUG(logger_, "Rotating to heading: angle=%.2f rad", angle_to_lookahead);
    return cmd_vel;
  }

  // Calculate curvature using Pure Pursuit formula
  double curvature = calculateCurvature(lookahead);

  // Calculate velocities
  double effective_max_linear;
  if (speed_limit_is_percentage_) {
    effective_max_linear = max_linear_vel_ * (speed_limit_ / 100.0);
  } else {
    effective_max_linear = std::min(max_linear_vel_, speed_limit_);
  }

  double linear_vel = desired_linear_vel_;

  // Reduce speed based on curvature
  double curvature_vel_constraint = 1.0 / (std::abs(curvature) + 0.001);
  linear_vel = std::min(linear_vel, curvature_vel_constraint);

  // Apply limits
  linear_vel = std::clamp(linear_vel, min_linear_vel_, effective_max_linear);

  // Calculate angular velocity
  double angular_vel = linear_vel * curvature;
  angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

  // Collision detection
  if (use_collision_detection_) {
    if (isCollisionImminent(pose, linear_vel, angular_vel, max_allowed_time_to_collision_)) {
      throw nav2_core::ControllerException("Collision imminent, stopping");
    }
  }

  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  return cmd_vel;
}

void PurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
  RCLCPP_INFO(logger_, "Speed limit set to %.2f (percentage: %d)", speed_limit, percentage);
}

void PurePursuitController::reset()
{
  RCLCPP_INFO(logger_, "Resetting controller: %s", name_.c_str());
  std::lock_guard<std::mutex> lock(path_mutex_);
  global_plan_ = nav_msgs::msg::Path();
}

nav_msgs::msg::Path PurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = robot_frame_;
  transformed_plan.header.stamp = pose.header.stamp;

  // Get transform from global to robot frame
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      robot_frame_, global_plan_.header.frame_id,
      tf2::TimePointZero,
      rclcpp::Duration::from_seconds(transform_tolerance_));
  } catch (tf2::TransformException & ex) {
    throw nav2_core::ControllerException("Could not transform path: " + std::string(ex.what()));
  }

  // Transform each pose
  for (const auto & global_pose : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped robot_pose;
    tf2::doTransform(global_pose, robot_pose, transform);
    robot_pose.header.frame_id = robot_frame_;
    robot_pose.header.stamp = pose.header.stamp;
    transformed_plan.poses.push_back(robot_pose);
  }

  // Prune poses behind the robot
  while (transformed_plan.poses.size() > 1) {
    if (transformed_plan.poses.front().pose.position.x > 0.0) {
      break;
    }
    transformed_plan.poses.erase(transformed_plan.poses.begin());
  }

  return transformed_plan;
}

geometry_msgs::msg::PoseStamped PurePursuitController::getLookaheadPoint(
  const nav_msgs::msg::Path & transformed_plan,
  double lookahead_dist)
{
  // Find point at lookahead distance along path
  double accumulated_dist = 0.0;

  for (size_t i = 0; i < transformed_plan.poses.size() - 1; ++i) {
    double dx = transformed_plan.poses[i + 1].pose.position.x -
      transformed_plan.poses[i].pose.position.x;
    double dy = transformed_plan.poses[i + 1].pose.position.y -
      transformed_plan.poses[i].pose.position.y;
    double segment_dist = std::hypot(dx, dy);

    if (accumulated_dist + segment_dist >= lookahead_dist) {
      // Interpolate within segment
      double remaining = lookahead_dist - accumulated_dist;
      double t = remaining / segment_dist;

      geometry_msgs::msg::PoseStamped lookahead;
      lookahead.header = transformed_plan.header;
      lookahead.pose.position.x = transformed_plan.poses[i].pose.position.x + t * dx;
      lookahead.pose.position.y = transformed_plan.poses[i].pose.position.y + t * dy;
      lookahead.pose.position.z = 0.0;
      lookahead.pose.orientation = transformed_plan.poses[i + 1].pose.orientation;

      return lookahead;
    }

    accumulated_dist += segment_dist;
  }

  // Return last point if path is shorter than lookahead
  return transformed_plan.poses.back();
}

double PurePursuitController::calculateCurvature(const geometry_msgs::msg::PoseStamped & lookahead)
{
  // Pure Pursuit curvature formula: k = 2 * y / L^2
  // where y is lateral offset and L is distance to lookahead
  double x = lookahead.pose.position.x;
  double y = lookahead.pose.position.y;
  double l_squared = x * x + y * y;

  if (l_squared < 0.001) {
    return 0.0;  // Avoid division by zero
  }

  return 2.0 * y / l_squared;
}

bool PurePursuitController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & /*pose*/,
  double linear_vel, double angular_vel, double check_time)
{
  // Simple forward projection collision check
  double dt = 0.1;  // Time step
  double x = 0.0, y = 0.0, theta = 0.0;

  for (double t = 0.0; t < check_time; t += dt) {
    // Update position
    x += linear_vel * std::cos(theta) * dt;
    y += linear_vel * std::sin(theta) * dt;
    theta += angular_vel * dt;

    // Convert to map coordinates
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) {
      return true;  // Outside costmap bounds
    }

    // Check cost
    unsigned char cost = costmap_->getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      RCLCPP_DEBUG(
        logger_, "Collision predicted at (%.2f, %.2f) in %.2f seconds", x, y, t);
      return true;
    }
  }

  return false;
}

void PurePursuitController::publishLookaheadMarker(const geometry_msgs::msg::PoseStamped & lookahead)
{
  visualization_msgs::msg::Marker marker;
  marker.header = lookahead.header;
  marker.ns = "lookahead";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = lookahead.pose;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  lookahead_pub_->publish(marker);
}

}  // namespace example_controller

// Register the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(example_controller::PurePursuitController, nav2_core::Controller)
