// Copyright (c) 2026 Polymath Robotics
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

#ifndef NAV2_COLLISION_MONITOR__TRAJECTORY_UTILS_HPP_
#define NAV2_COLLISION_MONITOR__TRAJECTORY_UTILS_HPP_

#include <string>

#include "nav_msgs/msg/trajectory.hpp"
#include "tf2/LinearMath/Transform.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Converts a trajectory message to a base frame trajectory.
 * The first point is time 0 and each later point is its stamp difference from the first.
 * Fails on an empty trajectory, non-finite poses, a point frame_id other than the header's,
 * non-increasing stamps, and a time step above max_time_step_s.
 * @param message Trajectory message
 * @param to_base Transform from the message frame to the base frame
 * @param max_time_step_s Largest allowed time between points
 * @param trajectory Output trajectory
 * @param reason Output description of the failure
 * @return True if the trajectory was converted
 */
bool toTrajectory(
  const nav_msgs::msg::Trajectory & message, const tf2::Transform & to_base,
  const double max_time_step_s, Trajectory & trajectory, std::string & reason);

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__TRAJECTORY_UTILS_HPP_
