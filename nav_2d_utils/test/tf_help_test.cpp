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

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "nav_2d_utils/tf_help.hpp"

TEST(TF_Help, TransformToSelf) {
  bool result;

  std::shared_ptr<tf2_ros::Buffer> tf;
  std::string frame = "frame_id";
  geometry_msgs::msg::PoseStamped in_pose;
  in_pose.header.frame_id = "frame_id";
  in_pose.pose.position.x = 1.0;
  in_pose.pose.position.y = 2.0;
  in_pose.pose.position.z = 3.0;
  tf2::Quaternion qt;
  qt.setRPY(0.5, 1.0, 1.5);
  in_pose.pose.orientation.w = qt.w();
  in_pose.pose.orientation.x = qt.x();
  in_pose.pose.orientation.y = qt.y();
  in_pose.pose.orientation.z = qt.z();

  geometry_msgs::msg::PoseStamped out_pose;
  rclcpp::Duration transform_tolerance(0, 500);

  result = nav_2d_utils::transformPose(tf, frame, in_pose, out_pose, transform_tolerance);

  EXPECT_TRUE(result);
  EXPECT_EQ(out_pose.header.frame_id, "frame_id");
  EXPECT_EQ(out_pose.pose.position.x, 1.0);
  EXPECT_EQ(out_pose.pose.position.y, 2.0);
  EXPECT_EQ(out_pose.pose.position.z, 3.0);
  EXPECT_EQ(out_pose.pose.orientation.w, qt.w());
  EXPECT_EQ(out_pose.pose.orientation.x, qt.x());
  EXPECT_EQ(out_pose.pose.orientation.y, qt.y());
  EXPECT_EQ(out_pose.pose.orientation.z, qt.z());
}

TEST(TF_Help, EmptyBuffer) {
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto buffer = std::make_shared<tf2_ros::Buffer>(clock);

  std::string frame = "frame_id";
  geometry_msgs::msg::PoseStamped in_pose;
  in_pose.header.frame_id = "other_frame_id";
  in_pose.pose.position.x = 1.0;
  in_pose.pose.position.y = 2.0;
  in_pose.pose.position.z = 3.0;
  tf2::Quaternion qt;
  qt.setRPY(0.5, 1.0, 1.5);
  in_pose.pose.orientation.w = qt.w();
  in_pose.pose.orientation.x = qt.x();
  in_pose.pose.orientation.y = qt.y();
  in_pose.pose.orientation.z = qt.z();

  geometry_msgs::msg::PoseStamped out_pose;
  rclcpp::Duration transform_tolerance(0, 500);

  bool result;
  result = nav_2d_utils::transformPose(buffer, frame, in_pose, out_pose, transform_tolerance);

  EXPECT_FALSE(result);
}
