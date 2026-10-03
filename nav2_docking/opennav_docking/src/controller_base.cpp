// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2026 Karinca Robotics
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
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "opennav_docking/controller_base.hpp"

#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using rcl_interfaces::msg::ParameterType;

namespace opennav_docking
{

ControllerParameterHandler::ControllerParameterHandler(
  const nav2::LifecycleNode::SharedPtr & node, const std::string & name,
  const rclcpp::Logger & logger)
: nav2_util::ParameterHandler<ControllerParameters>(node, logger), name_(name)
{
  params_.rotate_to_heading_angular_vel = node->declare_or_get_parameter(
    name_ + ".rotate_to_heading_angular_vel", 1.0);
  params_.rotate_to_heading_max_angular_accel = node->declare_or_get_parameter(
    name_ + ".rotate_to_heading_max_angular_accel", 3.2);
  params_.projection_time = node->declare_or_get_parameter(name_ + ".projection_time", 5.0);
  params_.simulation_time_step = node->declare_or_get_parameter(
    name_ + ".simulation_time_step", 0.1);
  params_.dock_collision_threshold = node->declare_or_get_parameter(
    name_ + ".dock_collision_threshold", 0.3);
}

rcl_interfaces::msg::SetParametersResult
ControllerParameterHandler::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }
    if (param_type != ParameterType::PARAMETER_DOUBLE) {
      continue;
    }
    const bool non_negative =
      param_name == name_ + ".rotate_to_heading_angular_vel" ||
      param_name == name_ + ".rotate_to_heading_max_angular_accel" ||
      param_name == name_ + ".projection_time" ||
      param_name == name_ + ".dock_collision_threshold";
    if (param_name == name_ + ".simulation_time_step" && parameter.as_double() <= 0.0) {
      RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
      result.successful = false;
    } else if (non_negative && parameter.as_double() < 0.0) {
      RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >=0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
      result.successful = false;
    }
  }
  return result;
}

void ControllerParameterHandler::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + ".rotate_to_heading_angular_vel") {
        params_.rotate_to_heading_angular_vel = parameter.as_double();
      } else if (param_name == name_ + ".rotate_to_heading_max_angular_accel") {
        params_.rotate_to_heading_max_angular_accel = parameter.as_double();
      } else if (param_name == name_ + ".projection_time") {
        params_.projection_time = parameter.as_double();
      } else if (param_name == name_ + ".simulation_time_step") {
        params_.simulation_time_step = parameter.as_double();
      } else if (param_name == name_ + ".dock_collision_threshold") {
        params_.dock_collision_threshold = parameter.as_double();
      }
    }
  }
}

void ControllerBase::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & name, nav2::TransformBuffer::SharedPtr tf)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in docking controller " + name};
  }

  name_ = name;
  tf2_buffer_ = tf;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Frames are the server's; targets are given in its base frame
  fixed_frame_ = node->declare_or_get_parameter("fixed_frame", std::string("odom"));
  base_frame_ = node->declare_or_get_parameter("base_frame", std::string("base_link"));

  param_handler_ = std::make_unique<ControllerParameterHandler>(node, name_, logger_);
  params_ = param_handler_->getParams();

  // Configure time only parameters
  use_collision_detection_ = node->declare_or_get_parameter(
    name_ + ".use_collision_detection", true);
  auto costmap_topic = node->declare_or_get_parameter(
    name_ + ".costmap_topic", std::string("local_costmap/costmap_raw"));
  auto footprint_topic = node->declare_or_get_parameter(
    name_ + ".footprint_topic", std::string("local_costmap/published_footprint"));
  transform_tolerance_ = node->declare_or_get_parameter(name_ + ".transform_tolerance", 0.1);

  // Let the derived control law declare its own parameters
  onConfigure(node);

  if (use_collision_detection_) {
    configureCollisionChecker(node, costmap_topic, footprint_topic, transform_tolerance_);
  }

  trajectory_pub_ = node->create_publisher<nav_msgs::msg::Path>("docking_trajectory");
}

void ControllerBase::cleanup()
{
  params_ = nullptr;
  param_handler_.reset();
  trajectory_pub_.reset();
  collision_checker_.reset();
  costmap_sub_.reset();
  footprint_sub_.reset();
}

void ControllerBase::activate()
{
  trajectory_pub_->on_activate();
  param_handler_->activate();
}

void ControllerBase::deactivate()
{
  trajectory_pub_->on_deactivate();
  param_handler_->deactivate();
}

bool ControllerBase::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
  const geometry_msgs::msg::Twist & /*velocity*/,
  const geometry_msgs::msg::Pose & target,
  const DockingOptions & options,
  double dt,
  geometry_msgs::msg::Twist & cmd)
{
  std::lock_guard<std::mutex> lock(param_handler_->getMutex());
  cmd = computeCommand(target, options.reverse, dt);
  return isTrajectoryCollisionFree(target, !options.undocking, options.reverse);
}

geometry_msgs::msg::Twist ControllerBase::computeRotateToHeadingCommand(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::Twist & current_velocity,
  const double & dt)
{
  std::lock_guard<std::mutex> lock(param_handler_->getMutex());
  geometry_msgs::msg::Twist cmd_vel;
  const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
  const double angular_vel = sign * params_->rotate_to_heading_angular_vel;
  const double min_feasible_angular_speed =
    current_velocity.angular.z - params_->rotate_to_heading_max_angular_accel * dt;
  const double max_feasible_angular_speed =
    current_velocity.angular.z + params_->rotate_to_heading_max_angular_accel * dt;
  cmd_vel.angular.z =
    std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  // Check if we need to slow down to avoid overshooting
  double max_vel_to_stop =
    std::sqrt(
    2 * params_->rotate_to_heading_max_angular_accel * fabs(angular_distance_to_heading));
  if (fabs(cmd_vel.angular.z) > max_vel_to_stop) {
    cmd_vel.angular.z = sign * max_vel_to_stop;
  }

  return cmd_vel;
}

bool ControllerBase::isTrajectoryCollisionFree(
  const geometry_msgs::msg::Pose & target_pose, bool is_docking, bool backward)
{
  // Visualization of the trajectory
  auto trajectory = std::make_unique<nav_msgs::msg::Path>();
  trajectory->header.frame_id = base_frame_;
  trajectory->header.stamp = clock_->now();

  // First pose
  geometry_msgs::msg::PoseStamped next_pose;
  next_pose.header.frame_id = base_frame_;
  trajectory->poses.push_back(next_pose);

  // Get the transform from base_frame to fixed_frame
  geometry_msgs::msg::TransformStamped base_to_fixed_transform;
  try {
    base_to_fixed_transform = tf2_buffer_->lookupTransform(
      fixed_frame_, base_frame_, trajectory->header.stamp,
      tf2::durationFromSec(transform_tolerance_));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger_, "Could not get transform from %s to %s: %s",
      base_frame_.c_str(), fixed_frame_.c_str(), ex.what());
    return false;
  }

  // Generate path
  double distance = std::numeric_limits<double>::max();
  unsigned int max_iter = static_cast<unsigned int>(
    ceil(params_->projection_time / params_->simulation_time_step));

  do{
    // Apply velocities to calculate next pose
    next_pose.pose = predictNextPose(
      params_->simulation_time_step, target_pose, next_pose.pose, backward);

    // Add the pose to the trajectory for visualization
    trajectory->poses.push_back(next_pose);

    // Transform pose from base_frame into fixed_frame
    geometry_msgs::msg::PoseStamped local_pose = next_pose;
    local_pose.header.stamp = trajectory->header.stamp;
    tf2::doTransform(local_pose, local_pose, base_to_fixed_transform);

    // Determine the distance at which to check for collisions
    // Skip the final segment of the trajectory for docking
    // and the initial segment for undocking
    // This avoids false positives when the robot is at the dock
    double dock_collision_distance = is_docking ?
      nav2_util::geometry_utils::euclidean_distance(target_pose, next_pose.pose) :
      std::hypot(next_pose.pose.position.x, next_pose.pose.position.y);

    // If this distance is greater than the dock_collision_threshold, check for collisions
    if (use_collision_detection_ &&
      dock_collision_distance > params_->dock_collision_threshold &&
      !collision_checker_->isCollisionFree(local_pose.pose))
    {
      RCLCPP_WARN(
        logger_, "Collision detected at pose: (%.2f, %.2f, %.2f) in frame %s",
        local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z,
        local_pose.header.frame_id.c_str());
      trajectory_pub_->publish(std::move(trajectory));
      return false;
    }

    // Check if we reach the goal
    distance = nav2_util::geometry_utils::euclidean_distance(target_pose, next_pose.pose);
  }while(distance > 1e-2 && trajectory->poses.size() < max_iter);

  trajectory_pub_->publish(std::move(trajectory));

  return true;
}

void ControllerBase::configureCollisionChecker(
  const nav2::LifecycleNode::SharedPtr & node,
  std::string costmap_topic, std::string footprint_topic, double transform_tolerance)
{
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(node, costmap_topic);
  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
    node, footprint_topic, *tf2_buffer_, base_frame_, transform_tolerance);
  collision_checker_ = std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_sub_, node->get_name());
}

}  // namespace opennav_docking
