/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Wilco Bonestroo
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
#include "gtest/gtest.h"
#include "nav_2d_utils/path_ops.hpp"

using std::sqrt;
using nav_2d_utils::adjustPlanResolution;

TEST(path_ops_test, AdjustResolutionEmpty)
{
  nav_2d_msgs::msg::Path2D in;
  nav_2d_msgs::msg::Path2D out = adjustPlanResolution(in, 2.0);
  EXPECT_EQ(out.poses.size(), 0ul);
}

TEST(path_ops_test, AdjustResolutionSimple)
{
  nav_2d_msgs::msg::Path2D in;
  const float RESOLUTION = 20.0;

  geometry_msgs::msg::Pose2D pose1;
  pose1.x = 0.0;
  pose1.y = 0.0;
  geometry_msgs::msg::Pose2D pose2;
  pose2.x = 100.0;
  pose2.y = 0.0;

  in.poses.push_back(pose1);
  in.poses.push_back(pose2);

  nav_2d_msgs::msg::Path2D out = adjustPlanResolution(in, RESOLUTION);
  float length = 100;
  uint32_t number_of_points = ceil(length / (2 * RESOLUTION));
  EXPECT_EQ(out.poses.size(), number_of_points);
  float max_length = length / (number_of_points - 1);

  for (unsigned int i = 1; i < out.poses.size(); i++) {
    pose1 = out.poses[i - 1];
    pose2 = out.poses[i];

    double sq_dist = (pose1.x - pose2.x) * (pose1.x - pose2.x) +
      (pose1.y - pose2.y) * (pose1.y - pose2.y);

    EXPECT_TRUE(sqrt(sq_dist) <= max_length);
  }
}
