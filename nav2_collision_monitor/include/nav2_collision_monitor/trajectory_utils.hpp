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
#include <vector>

#include "nav_msgs/msg/trajectory.hpp"
#include "rclcpp/time.hpp"
#include "tf2/LinearMath/Transform.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{
namespace trajectory_utils
{

/**
 * @brief Frame of a trajectory: the first point's frame_id, or the header's when that is empty
 * @param trajectory Trajectory to inspect
 * @return Frame ID, empty if the trajectory has no points
 */
std::string getFrameId(const nav_msgs::msg::Trajectory & trajectory);

/**
 * @brief Checks that a trajectory can be swept.
 * Rejects empty, non-finite, zero-stamped, non-monotonic and mixed-frame trajectories.
 * An empty point frame_id inherits the header's.
 * @param trajectory Trajectory to check
 * @param require_time_span If true, a trajectory with several points needs a non-zero time span
 * @param reason Output description of the first failure
 * @return True if the trajectory is usable
 */
bool validate(
  const nav_msgs::msg::Trajectory & trajectory, bool require_time_span, std::string & reason);

/**
 * @brief Poses of a trajectory in the base frame, labelled with a time.
 * With use_trajectory_time the label is stamp - now, points before now are dropped,
 * and a pose interpolated at label 0 is inserted when now lies between two points.
 * Otherwise the label is stamp - first stamp.
 * TODO: (zeerek) With use_trajectory_time false, drop poses before the pose closest to the origin, or require a tighter trajectory_timeout.
 * @param trajectory Validated trajectory
 * @param to_base Transform from the trajectory frame to the base frame
 * @param now Current time
 * @param use_trajectory_time Label by absolute stamps against now
 * @return Labelled poses, empty if no point is usable
 */
std::vector<TimedPose> toBasePoses(
  const nav_msgs::msg::Trajectory & trajectory, const tf2::Transform & to_base,
  const rclcpp::Time & now, bool use_trajectory_time);

/**
 * @brief Resamples labelled poses (linear xy, shortest-arc yaw).
 * @param poses Poses with non-decreasing labels
 * @param horizon_s Last label to keep, inclusive
 * @param step_s Sample spacing from the first label; 0 or less keeps the input poses
 * @return Poses with labels up to horizon_s
 */
std::vector<TimedPose> interpolate(
  const std::vector<TimedPose> & poses, double horizon_s, double step_s);

}  // namespace trajectory_utils
}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__TRAJECTORY_UTILS_HPP_
