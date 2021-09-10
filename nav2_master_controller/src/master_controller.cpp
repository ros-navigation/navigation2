// Copyright (c) 2020 Shrijit Singh
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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <utility>

#include "nav2_master_controller/master_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_master_controller
{

void MasterController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

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
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
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
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_path", use_rotate_to_path_);
  node->get_parameter(plugin_name_ + ".use_dynamic_threshold", use_dynamic_threshold_);
  node->get_parameter("controller_frequency", control_frequency);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  if (node->get_parameter(plugin_name_ + ".default_plugin", default_plugin_name)) {
    std::string default_plugin_type = nav2_util::get_plugin_type_param(node, default_plugin_name);
    default_plugin = plugin_loader_.createSharedInstance(default_plugin_type);
    RCLCPP_INFO(
      logger_, "Initializing default plugin \"%s\" with type \"%s\"",
      default_plugin_name.c_str(), default_plugin_type.c_str());

    try {
      default_plugin->configure(node, default_plugin_name, tf_, costmap_ros_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Couldn't initialize default plugin! %s", e.what());
      throw;
    }
    RCLCPP_INFO(logger_, "Initialized default plugin \"%s\"", default_plugin_name.c_str());
  } else {
    RCLCPP_ERROR(
      node->get_logger(), "Can not get 'default_plugin' param value for %s", plugin_name_.c_str());
  }
}

void MasterController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " master_controller::MasterController",
    plugin_name_.c_str());

  if (default_plugin) {default_plugin->cleanup();}
}

void MasterController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "master_controller::MasterController",
    plugin_name_.c_str());
  if (default_plugin) {default_plugin->activate();}
}

void MasterController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "master_controller::MasterController",
    plugin_name_.c_str());

  if (default_plugin) {default_plugin->deactivate();}
}

geometry_msgs::msg::TwistStamped MasterController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
    goal_yaw_tol_ = tf2::getYaw(pose_tolerance.orientation);
  }

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path
  auto carrot_pose = getLookAheadPoint(lookahead_dist_, transformed_plan);

  double angular_vel;
  double angle_to_path;
  bool rotate_to_path = false, rotate_to_heading = false;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  double max_thresh = max_angle_threshold_;
  if (use_dynamic_threshold_) {
    max_thresh = std::max(
      goal_yaw_tol_,
      max_angle_threshold_ * (fabs(speed.linear.x) / max_linear_vel_));
  }

  if (shouldRotateToGoalHeading(carrot_pose)) {
    if (use_rotate_to_heading_) {
      double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
      rotateToHeading(angular_vel, angle_to_goal, prev_cmd_vel);
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = angular_vel;
      rotate_to_heading = true;
    }
  } else if (shouldRotateToPath(carrot_pose, angle_to_path, max_thresh)) {
    if (use_rotate_to_path_) {
      rotateToHeading(angular_vel, angle_to_path, prev_cmd_vel);
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = angular_vel;
      rotate_to_path = true;
    }
  }
  if (default_plugin && !rotate_to_path && !rotate_to_heading) {
    cmd_vel = default_plugin->computeVelocityCommands(pose, speed, goal_checker);
  }
  prev_cmd_vel = cmd_vel;
  return cmd_vel;
}

bool MasterController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path,
  const double & angle_thresh)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return fabs(angle_to_path) > angle_thresh;
}


bool MasterController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return dist_to_goal < goal_dist_tol_;
}

void MasterController::rotateToHeading(
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

geometry_msgs::msg::PoseStamped MasterController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }

  return *goal_pose_it;
}

void MasterController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (default_plugin) {default_plugin->setPlan(global_plan_);}
}

void MasterController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (default_plugin) {default_plugin->setSpeedLimit(speed_limit, percentage);}
}

nav_msgs::msg::Path MasterController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  const double max_costmap_dim = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  const double max_transform_dist = max_costmap_dim * costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > max_transform_dist;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool MasterController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

}  // namespace nav2_master_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_master_controller::MasterController,
  nav2_core::Controller)
