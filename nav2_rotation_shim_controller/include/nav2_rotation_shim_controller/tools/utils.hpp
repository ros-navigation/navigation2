// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#ifndef NAV2_ROTATION_SHIM_CONTROLLER__TOOLS__UTILS_HPP_
#define NAV2_ROTATION_SHIM_CONTROLLER__TOOLS__UTILS_HPP_

#include "nav2_core/goal_checker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_rotation_shim_controller::utils
{

/**
* @brief get the current pose of the robot
* @param goal_checker goal checker to get tolerances
* @param robot robot pose
* @param goal goal pose
* @return bool Whether the robot is in the distance tolerance ignoring rotation and speed
*/
inline bool withinPositionGoalTolerance(
  nav2_core::GoalChecker * goal_checker,
  const geometry_msgs::msg::Pose & robot,
  const geometry_msgs::msg::Pose & goal)
{
  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    goal_checker->getTolerances(pose_tolerance, velocity_tolerance);

    const auto pose_tolerance_sq = pose_tolerance.position.x * pose_tolerance.position.x;

    auto dx = robot.position.x - goal.position.x;
    auto dy = robot.position.y - goal.position.y;

    auto dist_sq = dx * dx + dy * dy;

    if (dist_sq < pose_tolerance_sq) {
      return true;
    }
  }

  return false;
}

}  // namespace nav2_rotation_shim_controller::utils

#endif  // NAV2_ROTATION_SHIM_CONTROLLER__TOOLS__UTILS_HPP_
