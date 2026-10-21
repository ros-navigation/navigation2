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

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "tf2/utils.hpp"

namespace nav2_collision_monitor
{
namespace trajectory_utils
{

namespace
{

// Tolerance when comparing sample labels
constexpr double LABEL_EPSILON = 1e-9;

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool isZeroStamp(const builtin_interfaces::msg::Time & stamp)
{
  return 0 == stamp.sec && 0 == stamp.nanosec;
}

bool isFinitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
         q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.0;
}

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

TimedPose lerp(const TimedPose & from, const TimedPose & to, double time_s)
{
  const double span = to.time_s - from.time_s;
  const double u = span > 0.0 ? (time_s - from.time_s) / span : 0.0;
  return {
    time_s,
    {from.pose.x + u * (to.pose.x - from.pose.x),
      from.pose.y + u * (to.pose.y - from.pose.y),
      wrapAngle(from.pose.theta + u * wrapAngle(to.pose.theta - from.pose.theta))}};
}

}  // namespace

std::string getFrameId(const nav_msgs::msg::Trajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return "";
  }
  const std::string & point_frame = trajectory.points.front().header.frame_id;
  return point_frame.empty() ? trajectory.header.frame_id : point_frame;
}

bool validate(
  const nav_msgs::msg::Trajectory & trajectory, bool require_time_span, std::string & reason)
{
  if (trajectory.points.empty()) {
    reason = "trajectory is empty";
    return false;
  }

  const std::string frame_id = getFrameId(trajectory);
  if (frame_id.empty()) {
    reason = "trajectory has no frame_id";
    return false;
  }

  double previous_stamp = 0.0;
  for (std::size_t i = 0; i < trajectory.points.size(); i++) {
    const auto & point = trajectory.points[i];
    const std::string & point_frame =
      point.header.frame_id.empty() ? trajectory.header.frame_id : point.header.frame_id;
    if (point_frame != frame_id) {
      reason = "trajectory mixes frames '" + frame_id + "' and '" + point_frame + "'";
      return false;
    }
    if (!isFinitePose(point.pose)) {
      reason = "point " + std::to_string(i) + " has a non-finite or zero-length pose";
      return false;
    }
    if (isZeroStamp(point.header.stamp)) {
      reason = "point " + std::to_string(i) + " has a zero stamp";
      return false;
    }
    const double stamp = stampSeconds(point.header.stamp);
    if (i > 0 && stamp < previous_stamp) {
      reason = "point " + std::to_string(i) + " has a stamp before the previous point";
      return false;
    }
    previous_stamp = stamp;
  }

  if (require_time_span && trajectory.points.size() > 1 &&
    stampSeconds(trajectory.points.front().header.stamp) == previous_stamp)
  {
    reason = "trajectory has no time span";
    return false;
  }

  return true;
}

std::vector<TimedPose> toBasePoses(
  const nav_msgs::msg::Trajectory & trajectory, const tf2::Transform & to_base,
  const rclcpp::Time & now, bool use_trajectory_time)
{
  std::vector<TimedPose> poses;
  if (trajectory.points.empty()) {
    return poses;
  }

  const double reference_s = use_trajectory_time ?
    now.seconds() : stampSeconds(trajectory.points.front().header.stamp);

  bool bracketing = false;
  TimedPose previous{0.0, {0.0, 0.0, 0.0}};
  for (const auto & point : trajectory.points) {
    tf2::Quaternion rotation(
      point.pose.orientation.x, point.pose.orientation.y,
      point.pose.orientation.z, point.pose.orientation.w);
    rotation.normalize();
    const tf2::Transform in_base = to_base *
      tf2::Transform(rotation, tf2::Vector3(point.pose.position.x, point.pose.position.y, 0.0));
    const TimedPose current{
      stampSeconds(point.header.stamp) - reference_s,
      {in_base.getOrigin().x(), in_base.getOrigin().y(), tf2::getYaw(in_base.getRotation())}};

    if (use_trajectory_time && current.time_s < 0.0) {
      previous = current;
      bracketing = true;
      continue;
    }
    if (bracketing && current.time_s > 0.0) {
      poses.push_back(lerp(previous, current, 0.0));
    }
    bracketing = false;
    poses.push_back(current);
  }
  return poses;
}

std::vector<TimedPose> interpolate(
  const std::vector<TimedPose> & poses, double horizon_s, double step_s)
{
  std::vector<TimedPose> sampled;
  if (poses.empty() || poses.front().time_s > horizon_s) {
    return sampled;
  }

  if (step_s <= 0.0 || 1 == poses.size()) {
    for (const TimedPose & pose : poses) {
      if (pose.time_s > horizon_s) {
        break;
      }
      sampled.push_back(pose);
    }
    return sampled;
  }

  const double start_s = poses.front().time_s;
  const double end_s = std::min(horizon_s, poses.back().time_s);
  std::size_t upper = 1;
  for (std::size_t k = 0;; k++) {
    const double time_s = std::min(start_s + static_cast<double>(k) * step_s, end_s);
    while (upper + 1 < poses.size() && poses[upper].time_s < time_s) {
      upper++;
    }
    sampled.push_back(lerp(poses[upper - 1], poses[upper], time_s));
    if (time_s >= end_s - LABEL_EPSILON) {
      break;
    }
  }
  return sampled;
}

}  // namespace trajectory_utils
}  // namespace nav2_collision_monitor
