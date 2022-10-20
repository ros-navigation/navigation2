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

#ifndef DWB_CORE__TRAJECTORY_UTILS_HPP_
#define DWB_CORE__TRAJECTORY_UTILS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"

namespace dwb_core
{

/**
 * @brief Helper function to find a pose in the trajectory with a particular time time_offset
 * @param trajectory The trajectory to search
 * @param time_offset The desired time_offset
 * @return reference to the pose that is closest to the particular time offset
 *
 * Linearly searches through the poses. Once the poses time_offset is greater than the desired time_offset,
 * the search ends, since the poses have increasing time_offsets.
 */
const geometry_msgs::msg::Pose2D & getClosestPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset);

/**
 * @brief Helper function to create a pose with an exact time_offset by linearly interpolating between existing poses
 * @param trajectory The trajectory with pose and time offset information
 * @param time_offset The desired time_offset
 * @return New Pose2D with interpolated values
 * @note If the given time offset is outside the bounds of the trajectory, the return pose will be either the first or last pose.
 */
geometry_msgs::msg::Pose2D projectPose(
  const dwb_msgs::msg::Trajectory2D & trajectory,
  const double time_offset);

}  // namespace dwb_core

#endif  // DWB_CORE__TRAJECTORY_UTILS_HPP_
