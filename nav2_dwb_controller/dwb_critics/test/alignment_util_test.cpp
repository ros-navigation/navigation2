/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Samsung Research America
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

#include <vector>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "dwb_critics/alignment_util.hpp"
#include "dwb_core/exceptions.hpp"

TEST(AlignmentUtil, TestProjection)
{
  geometry_msgs::msg::Pose pose, pose_out;
  pose.position.x = 1.0;
  pose.position.y = -1.0;
  tf2::Quaternion q1;
  q1.setRPY(0, 0, 0.0);  // theta = 0.0
  pose.orientation = tf2::toMsg(q1);

  double distance = 1.0;
  pose_out = dwb_critics::getForwardPose(pose, distance);
  EXPECT_EQ(pose_out.position.x, 2.0);
  EXPECT_EQ(pose_out.position.y, -1.0);
  EXPECT_NEAR(tf2::getYaw(pose_out.orientation), 0.0, 1e-6);

  pose.position.x = 2.0;
  pose.position.y = -10.0;
  tf2::Quaternion q2;
  q2.setRPY(0, 0, 0.54);
  pose.orientation = tf2::toMsg(q2);

  pose_out = dwb_critics::getForwardPose(pose, distance);
  EXPECT_NEAR(pose_out.position.x, 2.8577, 0.01);
  EXPECT_NEAR(pose_out.position.y, -9.4858, 0.01);
  EXPECT_NEAR(tf2::getYaw(pose_out.orientation), 0.54, 0.01);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
