// Copyright (c) 2021 Anushree Sabnis
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
#include <chrono>
#include <memory>
#include <utility>
#include <string>
#include <vector>

#include "assisted_teleop.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace nav2_recoveries
{
using namespace std::chrono_literals; //NOLINT

AssistedTeleop::AssistedTeleop()
: action_server_(nullptr),
  cycle_frequency_(10.0)
{
}

void
AssistedTeleop::cleanup()
{
  action_server_.reset();
  vel_pub_.reset();
}

void
AssistedTeleop::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", recovery_name_.c_str());

  vel_pub_->on_activate();
  action_server_->activate();
}

void
AssistedTeleop::deactivate()
{
  vel_pub_->on_deactivate();
  action_server_->deactivate();
}

void
AssistedTeleop::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker)
{
  node_ = parent;
  auto node = node_.lock();
  logger_ = node->get_logger();

  RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

  recovery_name_ = name;
  tf_ = tf;
  collision_checker_ = collision_checker;

  nav2_util::declare_parameter_if_not_declared(
    node,
    "projection_time", rclcpp::ParameterValue(1.0));

  nav2_util::declare_parameter_if_not_declared(
    node,
    "linear_velocity_threshold_", rclcpp::ParameterValue(0.06));

  nav2_util::declare_parameter_if_not_declared(
    node,
    "cmd_vel_topic", rclcpp::ParameterValue(std::string("cmd_vel")));

  nav2_util::declare_parameter_if_not_declared(
    node,
    "input_vel_topic", rclcpp::ParameterValue(std::string("cmd_vel_input")));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("robot_base_frame", robot_base_frame_);
  node->get_parameter("transform_tolerance", transform_tolerance_);
  node->get_parameter("projection_time", projection_time_);
  node->get_parameter("linear_velocity_threshold", linear_velocity_threshold_);
  node->get_parameter("cmd_vel_topic", cmd_vel_topic_);
  node->get_parameter("input_vel_topic", input_vel_topic_);

  vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);

  action_server_ = std::make_shared<ActionServer>(
    node, recovery_name_,
    std::bind(&AssistedTeleop::execute, this));
}

void
AssistedTeleop::velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double angular_vel = msg->angular.z;
  geometry_msgs::msg::Vector3Stamped twist_speed;
  twist_speed.vector.x = msg->linear.x;
  twist_speed.vector.y = msg->linear.y;
  twist_speed.vector.z = 0;
  moveRobot(computeVelocity(twist_speed, angular_vel));
}

void
AssistedTeleop::projectPose(
  geometry_msgs::msg::Vector3Stamped & twist_speed,
  double angular_vel, double projection_time, geometry_msgs::msg::Pose2D & projected_pose)
{
  // Project Pose by time increment
  projected_pose.x += projection_time * (
    twist_speed.vector.x * cos(projected_pose.theta) +
    twist_speed.vector.y * sin(projected_pose.theta));
  projected_pose.y += projection_time * (
    twist_speed.vector.x * sin(projected_pose.theta) +
    twist_speed.vector.y * cos(projected_pose.theta));
  projected_pose.theta += projection_time *
    angular_vel;
}

geometry_msgs::msg::Twist::UniquePtr
AssistedTeleop::computeVelocity(
  geometry_msgs::msg::Vector3Stamped & twist_speed, double angular_vel)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  geometry_msgs::msg::Pose2D projected_pose;
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
  }
  projected_pose.x = current_pose.pose.position.x;
  projected_pose.y = current_pose.pose.position.y;
  projected_pose.theta = tf2::getYaw(
    current_pose.pose.orientation);

  cmd_vel->linear.x = twist_speed.vector.x;
  cmd_vel->linear.y = twist_speed.vector.y;
  cmd_vel->angular.z = angular_vel;

  double linear_vel = std::fabs(std::hypot(cmd_vel->linear.x, cmd_vel->linear.y));
  const double dt = collision_checker_->getCostmapResolution() / linear_vel;
  int loopcount = 1;
  double scaling_factor = 1;

  while (true) {
    double time_to_collision = loopcount * dt;
    if (time_to_collision >= projection_time_) {
      break;
    } else {
      scaling_factor = projection_time_ / time_to_collision;

      projectPose(twist_speed, angular_vel, time_to_collision, projected_pose);

      if (!collision_checker_->isCollisionFree(projected_pose)) {
        RCLCPP_WARN(logger_, "Collision approaching in %.2f seconds", time_to_collision);
        cmd_vel->linear.x /= scaling_factor;
        cmd_vel->linear.y /= scaling_factor;
        cmd_vel->angular.z /= scaling_factor;
        break;
      }
    }
    loopcount++;
  }
  return cmd_vel;
}

void
AssistedTeleop::moveRobot(geometry_msgs::msg::Twist::UniquePtr cmd_vel)
{
  if (std::fabs(std::hypot(cmd_vel->linear.x, cmd_vel->linear.y)) < linear_velocity_threshold_) {
    stopRobot();
  } else {
    vel_pub_->publish(std::move(cmd_vel));
  }
}

void AssistedTeleop::execute()
{
  RCLCPP_INFO(logger_, "Attempting %s", recovery_name_.c_str());

  auto node = node_.lock();
  // Log a message every second
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  auto timer = node->create_wall_timer(
    1s,
    [&]()
    {RCLCPP_INFO(logger_, "%s running...", recovery_name_.c_str());});


  auto start_time = node->now();

  // Initialize the ActionT goal and result
  auto at_goal = action_server_->get_current_goal();
  auto result = std::make_shared<AssistedTeleopAction::Result>();

  rclcpp::WallRate loop_rate(cycle_frequency_);
  auto assisted_teleop_end_ = (node->now() + at_goal->time).nanoseconds();

  vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    input_vel_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleop::velCallback,
      this, std::placeholders::_1));

  while (rclcpp::ok()) {
    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(logger_, "Canceling %s", recovery_name_.c_str());
      stopRobot();
      vel_sub_.reset();
      result->total_elapsed_time = node->now() - start_time;
      action_server_->terminate_all(result);
      return;
    }

    if (action_server_->is_preempt_requested()) {
      RCLCPP_ERROR(
        logger_, "Received a preemption request for %s,"
        " however feature is currently not implemented. Aborting and stopping.",
        recovery_name_.c_str());
      stopRobot();
      vel_sub_.reset();
      result->total_elapsed_time = node->now() - start_time;
      action_server_->terminate_current(result);
      return;
    }

    auto current_point = node->now().nanoseconds();

    auto time_left =
      assisted_teleop_end_ - current_point;

    // Enable recovery behavior if we haven't run out of time
    if (time_left < 0) {
      vel_sub_.reset();
      result->total_elapsed_time = node->now() - start_time;
      action_server_->succeeded_current(result);
      RCLCPP_INFO(
        logger_,
        "%s completed successfully", recovery_name_.c_str());
      return;
    }
  }
  loop_rate.sleep();
}

void AssistedTeleop::stopRobot()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = 0.0;
  cmd_vel->linear.y = 0.0;
  cmd_vel->angular.z = 0.0;

  vel_pub_->publish(std::move(cmd_vel));
}
}  // namespace nav2_recoveries

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_recoveries::AssistedTeleop, nav2_core::Recovery)
