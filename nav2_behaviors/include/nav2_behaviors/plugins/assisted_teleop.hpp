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

#ifndef NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_
#define NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_util/twist_subscriber.hpp"

namespace nav2_behaviors
{
using AssistedTeleopAction = nav2_msgs::action::AssistedTeleop;

/**
 * @class nav2_behaviors::AssistedTeleop
 * @brief An action server behavior for assisted teleop
 */
class AssistedTeleop : public TimedBehavior<AssistedTeleopAction>
{
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  using AssistedTeleopActionGoal = AssistedTeleopAction::Goal;
  using AssistedTeleopActionResult = AssistedTeleopAction::Result;
  AssistedTeleop();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  ResultStatus onRun(const std::shared_ptr<const AssistedTeleopActionGoal> command) override;

  /**
   * @brief func to run at the completion of the action
   */
  void onActionCompletion(std::shared_ptr<AssistedTeleopActionResult>/*result*/) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  ResultStatus onCycleUpdate() override;

  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
   */
  CostmapInfoType getResourceInfo() override {return CostmapInfoType::LOCAL;}

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief project a position
   * @param pose initial pose to project
   * @param twist velocity to project pose by
   * @param projection_time time to project by
   */
  geometry_msgs::msg::Pose2D projectPose(
    const geometry_msgs::msg::Pose2D & pose,
    const geometry_msgs::msg::Twist & twist,
    double projection_time);

  /**
   * @brief Callback function to preempt assisted teleop
   * @param msg empty message
   */
  void preemptTeleopCallback(const std_msgs::msg::Empty::SharedPtr msg);

  AssistedTeleopAction::Feedback::SharedPtr feedback_;

  // parameters
  double projection_time_;
  double simulation_time_step_;

  geometry_msgs::msg::TwistStamped teleop_twist_;
  bool preempt_teleop_{false};

  // subscribers
  std::unique_ptr<nav2_util::TwistSubscriber> vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr preempt_teleop_sub_;

  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};
}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__ASSISTED_TELEOP_HPP_
