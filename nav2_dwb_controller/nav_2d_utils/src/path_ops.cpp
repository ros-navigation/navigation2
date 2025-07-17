/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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

#include <cmath>

#include "nav_2d_utils/path_ops.hpp"
#include "tf2/convert.hpp"
#include "tf2/utils.hpp"
#include "nav2_util/geometry_utils.hpp"

using std::sqrt;

namespace nav_2d_utils
{
nav_msgs::msg::Path adjustPlanResolution(
  const nav_msgs::msg::Path & global_plan_in,
  double resolution)
{
  nav_msgs::msg::Path global_plan_out;
  if (global_plan_in.poses.size() == 0) {
    return global_plan_out;
  }

  geometry_msgs::msg::PoseStamped last = global_plan_in.poses[0];
  global_plan_out.poses.push_back(last);

  // we can take "holes" in the plan smaller than 2 grid cells (squared = 4)
  double min_sq_resolution = resolution * resolution * 4.0;

  for (unsigned int i = 1; i < global_plan_in.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped loop = global_plan_in.poses[i];
    double sq_dist = (loop.pose.position.x - last.pose.position.x) *
      (loop.pose.position.x - last.pose.position.x) +
      (loop.pose.position.y - last.pose.position.y) *
      (loop.pose.position.y - last.pose.position.y);
    if (sq_dist > min_sq_resolution) {
      // add points in-between
      double diff = sqrt(sq_dist) - sqrt(min_sq_resolution);
      int steps = static_cast<int>(diff / resolution) - 1;
      double steps_double = static_cast<double>(steps);

      double theta_last = tf2::getYaw(last.pose.orientation);
      double theta_loop = tf2::getYaw(loop.pose.orientation);
      double delta_x = (loop.pose.position.x - last.pose.position.x) / steps_double;
      double delta_y = (loop.pose.position.y - last.pose.position.y) / steps_double;
      double delta_t = (theta_loop - theta_last) / steps_double;

      for (int j = 1; j < steps; ++j) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = last.pose.position.x + j * delta_x;
        pose.pose.position.y = last.pose.position.y + j * delta_y;
        pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
          theta_last + j * delta_t);
        global_plan_out.poses.push_back(pose);
      }
    }
    global_plan_out.poses.push_back(global_plan_in.poses[i]);
    last.pose.position.x = loop.pose.position.x;
    last.pose.position.y = loop.pose.position.y;
  }
  return global_plan_out;
}
}  // namespace nav_2d_utils
