// Copyright (c) 2022 Samsung Research Russia
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

#ifndef NAV2_COLLISION_MONITOR__TYPES_HPP_
#define NAV2_COLLISION_MONITOR__TYPES_HPP_

#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

namespace nav2_collision_monitor
{

/// @brief Velocity for 2D model of motion
struct Velocity
{
  double x;  // x-component of linear velocity
  double y;  // y-component of linear velocity
  double tw;  // z-component of angular twist

  inline bool operator!=(const Velocity & second) const
  {
      return x != second.x || y != second.y || tw != second.tw;
  }

  inline bool operator<(const Velocity & second) const
  {
    const double first_vel = x*x + y*y;
    const double second_vel = second.x*second.x + second.y*second.y;
    return first_vel < second_vel;
  }

  inline Velocity operator*(const double & mul) const
  {
    return {x * mul, y * mul, tw * mul};
  }
};

/// @brief 2D point
struct Point
{
  double x;  // x-coordinate of point
  double y;  // y-coordinate of point
};

/// @brief 2D Pose
struct Pose
{
  double x;  // x-coordinate of pose
  double y;  // y-coordinate of pose
  double theta;  // rotation angle of pose
};

/// @brief Action type for robot
enum ActionType
{
  DO_NOTHING = 0,  // No action
  STOP = 1,  // Stop the robot
  SLOWDOWN = 2,  // Slowdown in percentage from current operating speed
  APPROACH = 3  // Keep constant time interval before collision
};

/// @brief Action for robot
struct Action
{
  ActionType action_type;
  Velocity req_vel;

  inline bool operator!=(const Action & second) const
  {
    return action_type != second.action_type || req_vel != second.req_vel;
  }
};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__TYPES_HPP_
