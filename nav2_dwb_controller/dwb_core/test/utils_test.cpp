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
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.hpp"

using dwb_core::getClosestPose;
using dwb_core::projectPose;

TEST(Utils, ClosestPose)
{
  dwb_msgs::msg::Trajectory2D traj;
  traj.poses.resize(4);
  traj.time_offsets.resize(4);
  for (unsigned int i = 0; i < traj.poses.size(); i++) {
    double d = static_cast<double>(i);
    traj.poses[i].position.x = d;
    traj.time_offsets[i] = rclcpp::Duration::from_seconds(d);
  }

  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.0).position.x, traj.poses[0].position.x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, -1.0).position.x, traj.poses[0].position.x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.4).position.x, traj.poses[0].position.x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.5).position.x, traj.poses[0].position.x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 0.51).position.x, traj.poses[1].position.x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 1.0).position.x, traj.poses[1].position.x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 1.4999).position.x, traj.poses[1].position.x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 2.0).position.x, traj.poses[2].position.x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 2.51).position.x, traj.poses[3].position.x);
  EXPECT_DOUBLE_EQ(getClosestPose(traj, 3.5).position.x, traj.poses[3].position.x);
}

TEST(Utils, ProjectPose)
{
  dwb_msgs::msg::Trajectory2D traj;
  traj.poses.resize(4);
  traj.time_offsets.resize(4);
  for (unsigned int i = 0; i < traj.poses.size(); i++) {
    double d = static_cast<double>(i);
    traj.poses[i].position.x = d;
    traj.poses[i].position.y = 30.0 - 2.0 * d;
    traj.poses[i].orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.42);
    traj.time_offsets[i] = rclcpp::Duration::from_seconds(d);
  }

  EXPECT_DOUBLE_EQ(projectPose(traj, 0.0).position.x, 0.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.0).position.y, 30.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(projectPose(traj, 0.0).orientation), 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, -1.0).position.x, 0.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, -1.0).position.y, 30.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(projectPose(traj, -1.0).orientation), 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.4).position.x, 0.4);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.4).position.y, 29.2);
  EXPECT_DOUBLE_EQ(tf2::getYaw(projectPose(traj, 0.4).orientation), 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.5).position.x, 0.5);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.5).position.y, 29.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(projectPose(traj, 0.5).orientation), 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.51).position.x, 0.51);
  EXPECT_DOUBLE_EQ(projectPose(traj, 0.51).position.y, 28.98);
  EXPECT_DOUBLE_EQ(tf2::getYaw(projectPose(traj, 0.51).orientation), 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.0).position.x, 1.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.0).position.y, 28.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(projectPose(traj, 1.0).orientation), 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.4999).position.x, 1.4999);
  EXPECT_DOUBLE_EQ(projectPose(traj, 1.4999).position.y, 27.0002);
  EXPECT_DOUBLE_EQ(tf2::getYaw(projectPose(traj, 1.4999).orientation), 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 2.0).position.x, 2.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 2.0).position.y, 26.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(projectPose(traj, 2.0).orientation), 0.42);
  EXPECT_FLOAT_EQ(projectPose(traj, 2.51).position.x, 2.51);
  EXPECT_FLOAT_EQ(projectPose(traj, 2.51).position.y, 24.98);
  EXPECT_DOUBLE_EQ(tf2::getYaw(projectPose(traj, 2.51).orientation), 0.42);
  EXPECT_DOUBLE_EQ(projectPose(traj, 3.5).position.x, 3.0);
  EXPECT_DOUBLE_EQ(projectPose(traj, 3.5).position.y, 24.0);
  EXPECT_DOUBLE_EQ(tf2::getYaw(projectPose(traj, 3.5).orientation), 0.42);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
