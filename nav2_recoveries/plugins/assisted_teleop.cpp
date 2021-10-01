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

#include "assisted_teleop.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_recoveries
{
using namespace std::chrono_literals; //NOLINT
void
AssistedTeleop::cleanup()
{
  action_server_.reset();
  vel_pub_.reset();
  vel_sub_.reset();
  costmap_sub_.reset();
}

void
AssistedTeleop::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", recovery_name_.c_str());

  vel_pub_->on_activate();
  action_server_->activate();
  enabled_ = true;
}

void
AssistedTeleop::deactivate()
{
  vel_pub_->on_deactivate();
  action_server_->deactivate();
  enabled_ = false;
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
  node->get_parameter("linear_velocity_threshold_", linear_velocity_threshold_);
  node->get_parameter("cmd_vel_topic", cmd_vel_topic_);
  node->get_parameter("input_vel_topic", input_vel_topic_);

  vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    input_vel_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleop::vel_callback,
      this, std::placeholders::_1));

  vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);

  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    node_, "local_costmap/costmap_raw");

  action_server_ = std::make_shared<ActionServer>(
    node, recovery_name_,
    std::bind(&AssistedTeleop::execute, this));

  collision_checker_ = collision_checker;
}

void
AssistedTeleop::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  speed_x = msg->linear.x;
  speed_y = msg->linear.y;
  angular_vel_ = msg->angular.z;
  if (go) {
    if (!checkCollision()) {
      RCLCPP_INFO(logger_, "Reducing velocity by a factor of %.2f", scaling_factor);
    }
    moveRobot();
  }
}

bool
AssistedTeleop::updatePose()
{
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return false;
  }
  projected_pose.x = current_pose.pose.position.x;
  projected_pose.y = current_pose.pose.position.y;
  projected_pose.theta = tf2::getYaw(
    current_pose.pose.orientation);
  return true;
}

void
AssistedTeleop::projectPose(
  double speed_x, double speed_y,
  double angular_vel_, double projection_time)
{
  // Project Pose by time increment
  projected_pose.x += projection_time * (
    speed_x * cos(projected_pose.theta));
  projected_pose.y += projection_time * (
    speed_y * sin(projected_pose.theta));
  projected_pose.theta += projection_time *
    angular_vel_;
}

bool
AssistedTeleop::checkCollision()
{
  const double dt =
    (speed_x != 0) ? (costmap_ros_->getResolution() / std::fabs(speed_x)) : projection_time_;
  int loopcount = 1;

  while (true) {
    if (updatePose()) {
      double time_to_collision = loopcount * dt;
      if (time_to_collision >= projection_time_) {
        scaling_factor = 1;
        break;
      }
      scaling_factor = projection_time_ / (time_to_collision);
      loopcount++;

      projectPose(speed_x, speed_y, angular_vel_, time_to_collision);

      if (!collision_checker_->isCollisionFree(projected_pose)) {
        RCLCPP_WARN(logger_, "Collision approaching in %.2f seconds", time_to_collision);
        return false;
      }
    }
  }
  return true;
}

void
AssistedTeleop::moveRobot()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

  cmd_vel->linear.x = speed_x / scaling_factor;
  cmd_vel->linear.y = speed_y;
  cmd_vel->angular.z = angular_vel_ / scaling_factor;

  if (std::fabs(cmd_vel->linear.x) < linear_velocity_threshold_) {
    stopRobot();
  } else {
    vel_pub_->publish(std::move(cmd_vel));
  }
}

void AssistedTeleop::execute()
{
  RCLCPP_INFO(logger_, "Attempting %s", recovery_name_.c_str());

  if (!enabled_) {
    RCLCPP_WARN(
      logger_,
      "Called while inactive, ignoring request.");
    return;
  }

  // Log a message every second
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    auto timer = node->create_wall_timer(
      1s,
      [&]()
      {RCLCPP_INFO(logger_, "%s running...", recovery_name_.c_str());});
  }

  auto start_time = steady_clock_.now();

  // Initialize the ActionT goal, feedback and result
  auto at_goal = action_server_->get_current_goal();
  auto feedback_ = std::make_shared<AssistedTeleopAction::Feedback>();
  auto result = std::make_shared<AssistedTeleopAction::Result>();

  rclcpp::WallRate loop_rate(cycle_frequency_);
  assisted_teleop_end_ = std::chrono::steady_clock::now() +
    rclcpp::Duration(at_goal->time).to_chrono<std::chrono::nanoseconds>();

  while (rclcpp::ok()) {
    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(logger_, "Canceling %s", recovery_name_.c_str());
      go = false;
      stopRobot();
      result->total_elapsed_time = steady_clock_.now() - start_time;
      action_server_->terminate_all(result);
      return;
    }

    if (action_server_->is_preempt_requested()) {
      RCLCPP_ERROR(
        logger_, "Received a preemption request for %s,"
        " however feature is currently not implemented. Aborting and stopping.",
        recovery_name_.c_str());
      stopRobot();
      result->total_elapsed_time = steady_clock_.now() - start_time;
      action_server_->terminate_current(result);
      return;
    }

    auto current_point = std::chrono::steady_clock::now();

    auto time_left =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      assisted_teleop_end_ - current_point).count();

    feedback_->time_left = rclcpp::Duration(
      rclcpp::Duration::from_nanoseconds(time_left));

    action_server_->publish_feedback(feedback_);

    // Enable recovery behavior if we haven't run out of time
    if (time_left > 0) {
      go = true;
      costmap_ros_ = costmap_sub_->getCostmap();
    } else {
      go = false;
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
