// Copyright (c) 2020 Samsung Research America
// Copyright (c) 2021 Alex Melkobrodov
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

#include <string>
#include <memory>
#include "nav2_util/geometry_utils.hpp"
#include "nav2_shim_controller/shim_controller.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_shim_controller
{

void ShimController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  plugin_name_ = name;
  logger_ = node->get_logger();
  tf_ = tf;

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update
  goal_yaw_tol_ = 0.25;

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.26));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angle_threshold", rclcpp::ParameterValue(0.785));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".default_plugin", rclcpp::PARAMETER_STRING);
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_path", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_dynamic_threshold", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angle_threshold", max_angle_threshold_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_path", use_rotate_to_path_);
  node->get_parameter(plugin_name_ + ".use_dynamic_threshold", use_dynamic_threshold_);
  node->get_parameter("controller_frequency", control_frequency);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  if (node->get_parameter(plugin_name_ + ".default_plugin", default_plugin_name_)) {
    std::string default_plugin_type = nav2_util::get_plugin_type_param(node, default_plugin_name_);
    default_plugin = plugin_loader_.createSharedInstance(default_plugin_type);
    RCLCPP_INFO(
      logger_, "Initializing default plugin \"%s\" with type \"%s\"",
      default_plugin_name_.c_str(), default_plugin_type.c_str());

    try {
      default_plugin->configure(node, default_plugin_name_, tf, costmap_ros);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Couldn't initialize default plugin! %s", e.what());
      throw nav2_core::PlannerException("Unable to load default plugin!");
    }
    RCLCPP_INFO(logger_, "Initialized default plugin \"%s\"", default_plugin_name_.c_str());
  } else {
    RCLCPP_ERROR(
      node->get_logger(), "Can not get 'default_plugin' param value for %s", plugin_name_.c_str());
  }
}

void ShimController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " shim_controller::ShimController",
    plugin_name_.c_str());

  if (default_plugin) {
    default_plugin->cleanup();
    default_plugin.reset();
  }
}

void ShimController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "shim_controller::ShimController",
    plugin_name_.c_str());
  if (default_plugin) {
    default_plugin->activate();
  }
}

void ShimController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "shim_controller::ShimController",
    plugin_name_.c_str());

  if (default_plugin) {
    default_plugin->deactivate();
  }
}

geometry_msgs::msg::TwistStamped ShimController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (goal_checker && goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    goal_dist_tol_ = pose_tolerance.position.x;
    goal_yaw_tol_ = tf2::getYaw(pose_tolerance.orientation);
  } else {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  }

  double angular_vel = 0.0;
  double angle_to_path;
  bool rotate_to_path = false, rotate_to_heading = false;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  double max_thresh = max_angle_threshold_;
  if (use_dynamic_threshold_) {
    // Calculates dynamic angular threshold.
    // 1) If the current speed = 0 the threshold is the minimum, it helps to turn the robot exactly
    //  on the planned path when receiving a plan or re-planning the path.
    // 2) If the current speed = maximum speed threshold is maximum, this allows the default planner
    //  to avoid obstacles without switching to a turn to the planned path.
    max_thresh = std::max(
      goal_yaw_tol_,
      max_angle_threshold_ * (fabs(speed.linear.x) / max_linear_vel_));
  }

  auto robot_pose_in_map = transformPoseToMapFrame(pose);

  if (isCloseToGoalHeading(robot_pose_in_map, global_plan_.poses.back())) {
    if (use_rotate_to_heading_) {
      double angle_to_goal = angles::normalize_angle(
        tf2::getYaw(global_plan_.poses.back().pose.orientation) -
        tf2::getYaw(robot_pose_in_map.pose.orientation));
      rotateToHeading(angular_vel, angle_to_goal, prev_cmd_vel_);
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = angular_vel;
      rotate_to_heading = true;
    }
  } else if (shouldRotateToPath(robot_pose_in_map, angle_to_path, max_thresh)) {
    if (use_rotate_to_path_) {
      rotateToHeading(angular_vel, angle_to_path, prev_cmd_vel_);
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = angular_vel;
      rotate_to_path = true;
    }
  }

  if (default_plugin && !rotate_to_path && !rotate_to_heading) {
    cmd_vel = default_plugin->computeVelocityCommands(pose, speed, goal_checker);
  }
  prev_cmd_vel_ = cmd_vel;
  return cmd_vel;
}

bool ShimController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & robot_pose_in_map, double & angle_to_path,
  const double & angle_thresh)
{
  geometry_msgs::msg::PoseStamped carrot_pose;
  for (auto pose : global_plan_.poses) {
    if (isCloseToGoalHeading(robot_pose_in_map, pose)) {
      global_plan_.poses.erase(global_plan_.poses.begin());
    } else {
      carrot_pose = pose;
      break;
    }
  }

  // Whether we should rotate robot to rough path heading
  angle_to_path = angles::normalize_angle(
    atan2(
      carrot_pose.pose.position.y - robot_pose_in_map.pose.position.y,
      carrot_pose.pose.position.x - robot_pose_in_map.pose.position.x) -
    tf2::getYaw(robot_pose_in_map.pose.orientation));

  return fabs(angle_to_path) > angle_thresh;
}

geometry_msgs::msg::PoseStamped ShimController::transformPoseToMapFrame(
  const geometry_msgs::msg::PoseStamped & input_pose)
{
  geometry_msgs::msg::PoseStamped output_pose;
  try {
    tf_->transform(input_pose, output_pose, global_plan_.header.frame_id, transform_tolerance_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPoseToMapFrame: %s", ex.what());
  }
  return output_pose;
}

bool ShimController::isCloseToGoalHeading(
  const geometry_msgs::msg::PoseStamped & robot_pose_in_map,
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(
    carrot_pose.pose.position.x - robot_pose_in_map.pose.position.x,
    carrot_pose.pose.position.y - robot_pose_in_map.pose.position.y);
  return dist_to_goal < goal_dist_tol_;
}

void ShimController::rotateToHeading(
  double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::TwistStamped & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * max_angular_vel_;

  // Calculation of angle required for stopping robot with deacceleration in desired angle
  double deceleration_angle = 0.5 *
    std::pow(curr_speed.twist.angular.z, 2) / max_angular_accel_ + goal_yaw_tol_;

  if (deceleration_angle > fabs(angle_to_path)) {
    // sets desired angular velocity
    angular_vel = 0.0;
  }

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.twist.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.twist.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
  RCLCPP_DEBUG(
    logger_, "speed = %f, speed_cur = %f, angle = %f, calc angle = %f",
    angular_vel,
    curr_speed.twist.angular.z,
    angle_to_path,
    deceleration_angle);
}

void ShimController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (default_plugin) {
    default_plugin->setPlan(path);
  }
}

void ShimController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (default_plugin) {
    default_plugin->setSpeedLimit(speed_limit, percentage);
  }
}

}  // namespace nav2_shim_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_shim_controller::ShimController,
  nav2_core::Controller)
