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

#include "gtest/gtest.h"
#include "dwb_core/trajectory_utils.hpp"

using dwb_core::getClosestPose;
using dwb_core::projectPose;

TEST(Utils, ClosestPose)
{
  dwb_msgs::msg::Trajectory2D traj;
  traj.poses.resize(4);
  traj.time_offsets.resize(4);
  for (unsigned int i = 0; i < traj.poses.size(); i++) {
    double d = static_cast<double>(i);
    traj.poses[i].x = d;
    traj.time_offsets[i] = rclcpp::Duration::from_seconds(d);
  }

  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.0).x, traj.poses[0].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, -1.0).x, traj.poses[0].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.4).x, traj.poses[0].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.5).x, traj.poses[0].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.51).x, traj.poses[1].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 1.0).x, traj.poses[1].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 1.4999).x, traj.poses[1].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 2.0).x, traj.poses[2].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 2.51).x, traj.poses[3].x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 3.5).x, traj.poses[3].x);
}

TEST(Utils, ProjectPose)
{
  dwb_msgs::msg::Trajectory2D traj;
  traj.poses.resize(4);
  traj.time_offsets.resize(4);
  for (unsigned int i = 0; i < traj.poses.size(); i++) {
    double d = static_cast<double>(i);
    traj.poses[i].x = d;
    traj.poses[i].y = 30.0 - 2.0 * d;
    traj.poses[i].theta = 0.42;
    traj.time_offsets[i] = rclcpp::Duration::from_seconds(d);
  }

  EXPECT_DOUBLE_EQ(projectPose(traj, 0.0).x, 0.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.0).y, 30.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.0).theta, 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, -1.0).x, 0.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, -1.0).y, 30.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, -1.0).theta, 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.4).x, 0.4);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.4).y, 29.2);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.4).theta, 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.5).x, 0.5);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.5).y, 29.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.5).theta, 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.51).x, 0.51);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.51).y, 28.98);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.51).theta, 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.0).x, 1.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.0).y, 28.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.0).theta, 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.4999).x, 1.4999);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.4999).y, 27.0002);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.4999).theta, 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 2.0).x, 2.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 2.0).y, 26.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 2.0).theta, 0.42);
  EXPECT_FLOAT_EQ(projectPose(traj, 2.51).x, 2.51);
  EXPECT_FLOAT_EQ(projectPose(traj, 2.51).y, 24.98);
  EXPECT_DOUBLE_EQ(projectPose(traj, 2.51).theta, 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 3.5).x, 3.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 3.5).y, 24.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 3.5).theta, 0.42);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
