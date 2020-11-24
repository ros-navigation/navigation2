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

#include "nav_2d_utils/path_ops.hpp"
#include <cmath>

using std::sqrt;

namespace nav_2d_utils
{
nav_2d_msgs::msg::Path2D adjustPlanResolution(
  const nav_2d_msgs::msg::Path2D & global_plan_in,
  double resolution)
{
  nav_2d_msgs::msg::Path2D global_plan_out;
  if (global_plan_in.poses.size() == 0) {
    return global_plan_out;
  }

  geometry_msgs::msg::Pose2D last = global_plan_in.poses[0];
  global_plan_out.poses.push_back(last);

  // we can take "holes" in the plan smaller than 2 grid cells (squared = 4)
  double min_sq_resolution = resolution * resolution * 4.0;

  for (unsigned int i = 1; i < global_plan_in.poses.size(); ++i) {
    geometry_msgs::msg::Pose2D loop = global_plan_in.poses[i];
    double sq_dist = (loop.x - last.x) * (loop.x - last.x) + (loop.y - last.y) * (loop.y - last.y);
    if (sq_dist > min_sq_resolution) {
      // add points in-between
      double diff = sqrt(sq_dist) - sqrt(min_sq_resolution);
      int steps = static_cast<int>(diff / resolution) - 1;
      double steps_double = static_cast<double>(steps);

      double delta_x = (loop.x - last.x) / steps_double;
      double delta_y = (loop.y - last.y) / steps_double;
      double delta_t = (loop.theta - last.theta) / steps_double;

      for (int j = 1; j < steps; ++j) {
        geometry_msgs::msg::Pose2D pose;
        pose.x = last.x + j * delta_x;
        pose.y = last.y + j * delta_y;
        pose.theta = last.theta + j * delta_t;
        global_plan_out.poses.push_back(pose);
      }
    }
    global_plan_out.poses.push_back(global_plan_in.poses[i]);
    last.x = loop.x;
    last.y = loop.y;
  }
  return global_plan_out;
}
}  // namespace nav_2d_utils
