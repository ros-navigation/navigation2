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

#include "nav2_collision_monitor/trajectory_utils.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include "tf2/utils.hpp"

namespace nav2_collision_monitor
{

namespace
{

// Allowed excess over the maximum time step, for stamps rounded by their producer
constexpr double STEP_TOLERANCE_S = 1e-6;

int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000 + static_cast<int64_t>(stamp.nanosec);
}

bool isFinitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
         q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.0;
}

}  // namespace

bool toTrajectory(
  const nav_msgs::msg::Trajectory & message, const tf2::Transform & to_base,
  const double max_time_step_s, Trajectory & trajectory, std::string & reason)
{
  trajectory.clear();
  if (message.points.empty()) {
    reason = "trajectory is empty";
    return false;
  }

  const int64_t first_ns = stampNanoseconds(message.points.front().header.stamp);
  int64_t previous_ns = first_ns;
  for (std::size_t i = 0; i < message.points.size(); i++) {
    const auto & point = message.points[i];
    if (!point.header.frame_id.empty() && point.header.frame_id != message.header.frame_id) {
      reason = "point " + std::to_string(i) + " is not in the trajectory frame";
      return false;
    }
    if (!isFinitePose(point.pose)) {
      reason = "point " + std::to_string(i) + " has an invalid pose";
      return false;
    }
    const int64_t stamp_ns = stampNanoseconds(point.header.stamp);
    if (i > 0 && stamp_ns <= previous_ns) {
      reason = "point " + std::to_string(i) + " does not increase the stamp";
      return false;
    }
    if (static_cast<double>(stamp_ns - previous_ns) * 1e-9 > max_time_step_s + STEP_TOLERANCE_S) {
      reason = "point " + std::to_string(i) + " is more than " +
        std::to_string(max_time_step_s) + " s after the previous point";
      return false;
    }
    previous_ns = stamp_ns;

    tf2::Quaternion rotation(
      point.pose.orientation.x, point.pose.orientation.y,
      point.pose.orientation.z, point.pose.orientation.w);
    rotation.normalize();
    const tf2::Transform in_base = to_base *
      tf2::Transform(rotation,
        tf2::Vector3(point.pose.position.x, point.pose.position.y, point.pose.position.z));
    trajectory.push_back(
      {static_cast<double>(stamp_ns - first_ns) * 1e-9,
        {in_base.getOrigin().x(), in_base.getOrigin().y(), tf2::getYaw(in_base.getRotation())}});
  }
  return true;
}

}  // namespace nav2_collision_monitor
