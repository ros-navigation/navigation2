// Copyright (c) 2022 Joshua Wallace
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

#include <utility>

#include "nav2_behaviors/plugins/assisted_teleop.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_behaviors
{
AssistedTeleop::AssistedTeleop()
: TimedBehavior<AssistedTeleopAction>(),
  feedback_(std::make_shared<AssistedTeleopAction::Feedback>())
{}

void AssistedTeleop::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // set up parameters
  nav2::declare_parameter_if_not_declared(
    node,
    "projection_time", rclcpp::ParameterValue(1.0));

  nav2::declare_parameter_if_not_declared(
    node,
    "simulation_time_step", rclcpp::ParameterValue(0.1));

  nav2::declare_parameter_if_not_declared(
    node,
    "cmd_vel_teleop", rclcpp::ParameterValue(std::string("cmd_vel_teleop")));

  node->get_parameter("projection_time", projection_time_);
  node->get_parameter("simulation_time_step", simulation_time_step_);

  std::string cmd_vel_teleop;
  node->get_parameter("cmd_vel_teleop", cmd_vel_teleop);

  vel_sub_ = std::make_unique<nav2_util::TwistSubscriber>(
    node,
    cmd_vel_teleop,
    [&](geometry_msgs::msg::Twist::SharedPtr msg) {
      teleop_twist_.twist = *msg;
    }, [&](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      teleop_twist_ = *msg;
    });

  preempt_teleop_sub_ = node->create_subscription<std_msgs::msg::Empty>(
    "preempt_teleop",
    std::bind(
      &AssistedTeleop::preemptTeleopCallback,
      this, std::placeholders::_1));
}

ResultStatus AssistedTeleop::onRun(const std::shared_ptr<const AssistedTeleopAction::Goal> command)
{
  preempt_teleop_ = false;
  command_time_allowance_ = command->time_allowance;
  end_time_ = this->clock_->now() + command_time_allowance_;
  return ResultStatus{Status::SUCCEEDED, AssistedTeleopActionResult::NONE, ""};
}

void AssistedTeleop::onActionCompletion(std::shared_ptr<AssistedTeleopActionResult>/*result*/)
{
  teleop_twist_ = geometry_msgs::msg::TwistStamped();
  preempt_teleop_ = false;
}

ResultStatus AssistedTeleop::onCycleUpdate()
{
  feedback_->current_teleop_duration = elapsed_time_;
  action_server_->publish_feedback(feedback_);

  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    std::string error_msg = "Exceeded time allowance before reaching the " + behavior_name_ +
      "goal - Exiting " + behavior_name_;
    RCLCPP_WARN_STREAM(logger_, error_msg.c_str());
    return ResultStatus{Status::FAILED, AssistedTeleopActionResult::TIMEOUT, error_msg};
  }

  // user states that teleop was successful
  if (preempt_teleop_) {
    stopRobot();
    return ResultStatus{Status::SUCCEEDED, AssistedTeleopActionResult::NONE, ""};
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, local_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    std::string error_msg = "Current robot pose is not available for " + behavior_name_;
    RCLCPP_ERROR_STREAM(logger_, error_msg.c_str());
    return ResultStatus{Status::FAILED, AssistedTeleopActionResult::TF_ERROR, error_msg};
  }

  geometry_msgs::msg::Pose projected_pose = current_pose.pose;

  auto scaled_twist = std::make_unique<geometry_msgs::msg::TwistStamped>(teleop_twist_);
  for (double time = simulation_time_step_; time < projection_time_;
    time += simulation_time_step_)
  {
    projected_pose = projectPose(projected_pose, teleop_twist_.twist, simulation_time_step_);

    if (!local_collision_checker_->isCollisionFree(projected_pose)) {
      if (time == simulation_time_step_) {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          logger_,
          *clock_,
          1000,
          behavior_name_.c_str() << " collided on first time step, setting velocity to zero");
        scaled_twist->twist.linear.x = 0.0f;
        scaled_twist->twist.linear.y = 0.0f;
        scaled_twist->twist.angular.z = 0.0f;
        break;
      } else {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          logger_,
          *clock_,
          1000,
          behavior_name_.c_str() << " collision approaching in " << time << " seconds");
        double scale_factor = time / projection_time_;
        scaled_twist->twist.linear.x *= scale_factor;
        scaled_twist->twist.linear.y *= scale_factor;
        scaled_twist->twist.angular.z *= scale_factor;
        break;
      }
    }
  }
  vel_pub_->publish(std::move(scaled_twist));

  return ResultStatus{Status::RUNNING, AssistedTeleopActionResult::NONE, ""};
}

geometry_msgs::msg::Pose AssistedTeleop::projectPose(
  const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Twist & twist,
  double projection_time)
{
  geometry_msgs::msg::Pose projected_pose = pose;

  double theta = tf2::getYaw(pose.orientation);

  projected_pose.position.x += projection_time * (
    twist.linear.x * cos(theta) +
    twist.linear.y * sin(theta));

  projected_pose.position.y += projection_time * (
    twist.linear.x * sin(theta) -
    twist.linear.y * cos(theta));

  double new_theta = theta + projection_time * twist.angular.z;
  projected_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(new_theta);

  return projected_pose;
}

void AssistedTeleop::preemptTeleopCallback(const std_msgs::msg::Empty::SharedPtr)
{
  preempt_teleop_ = true;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::AssistedTeleop, nav2_core::Behavior)
