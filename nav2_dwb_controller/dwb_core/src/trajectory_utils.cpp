/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <dwb_core/trajectory_utils.hpp>

#include <cmath>

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/duration.hpp"

#include "dwb_core/exceptions.hpp"

namespace dwb_core
{
const geometry_msgs::msg::Pose & getClosestPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset)
{
  rclcpp::Duration goal_time = rclcpp::Duration::from_seconds(time_offset);
  const unsigned int num_poses = trajectory.poses.size();
  if (num_poses == 0) {
    throw nav2_core::InvalidPath("Cannot call getClosestPose on empty trajectory.");
  }
  unsigned int closest_index = num_poses;
  double closest_diff = 0.0;
  for (unsigned int i = 0; i < num_poses; ++i) {
    double diff = std::fabs((rclcpp::Duration(trajectory.time_offsets[i]) - goal_time).seconds());
    if (closest_index == num_poses || diff < closest_diff) {
      closest_index = i;
      closest_diff = diff;
    }
    if (goal_time < rclcpp::Duration(trajectory.time_offsets[i])) {
      break;
    }
  }
  return trajectory.poses[closest_index];
}

geometry_msgs::msg::Pose projectPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset)
{
  rclcpp::Duration goal_time = rclcpp::Duration::from_seconds(time_offset);
  const unsigned int num_poses = trajectory.poses.size();
  if (num_poses == 0) {
    throw nav2_core::InvalidPath("Cannot call projectPose on empty trajectory.");
  }
  if (goal_time <= (trajectory.time_offsets[0])) {
    return trajectory.poses[0];
  } else if (goal_time >= rclcpp::Duration(trajectory.time_offsets[num_poses - 1])) {
    return trajectory.poses[num_poses - 1];
  }

  for (unsigned int i = 0; i < num_poses - 1; ++i) {
    if (goal_time >= rclcpp::Duration(trajectory.time_offsets[i]) &&
      goal_time < rclcpp::Duration(trajectory.time_offsets[i + 1]))
    {
      double time_diff =
        (rclcpp::Duration(trajectory.time_offsets[i + 1]) -
        rclcpp::Duration(trajectory.time_offsets[i])).seconds();
      double ratio = (goal_time - rclcpp::Duration(trajectory.time_offsets[i])).seconds() /
        time_diff;
      double inv_ratio = 1.0 - ratio;
      const auto & pose_a = trajectory.poses[i];
      const auto & pose_b = trajectory.poses[i + 1];

      geometry_msgs::msg::Pose projected;
      projected.position.x = pose_a.position.x * inv_ratio + pose_b.position.x * ratio;
      projected.position.y = pose_a.position.y * inv_ratio + pose_b.position.y * ratio;
      projected.position.z = pose_a.position.z * inv_ratio + pose_b.position.z * ratio;

      // Interpolate orientation using slerp
      tf2::Quaternion q1, q2;
      tf2::fromMsg(pose_a.orientation, q1);
      tf2::fromMsg(pose_b.orientation, q2);
      projected.orientation = tf2::toMsg(q1.slerp(q2, ratio));

      return projected;
    }
  }

  // Should not reach this point
  return trajectory.poses[num_poses - 1];
}


}  // namespace dwb_core
