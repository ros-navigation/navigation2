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

#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

#include "gtest/gtest.h"
#include "nav_2d_utils/conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

using nav_2d_utils::posesToPath;
using nav_2d_utils::pathToPath;

TEST(nav_2d_utils, PosesToPathEmpty)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  nav_msgs::msg::Path path = posesToPath(poses);

  EXPECT_EQ(path.poses.size(), 0ul);
}

TEST(nav_2d_utils, PosesToPathNonEmpty)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  geometry_msgs::msg::PoseStamped pose1;
  rclcpp::Time time1, time2;
  auto node = rclcpp::Node::make_shared("twod_utils_test_node");
  time1 = node->now();

  tf2::Quaternion quat1, quat2;
  quat1.setRPY(0, 0, 0.123);
  pose1.pose.position.x = 1.0;
  pose1.pose.position.y = 2.0;
  pose1.pose.orientation.w = quat1.w();
  pose1.pose.orientation.x = quat1.x();
  pose1.pose.orientation.y = quat1.y();
  pose1.pose.orientation.z = quat1.z();
  pose1.header.stamp = time1;
  pose1.header.frame_id = "frame1_id";

  geometry_msgs::msg::PoseStamped pose2;
  pose2.pose.position.x = 4.0;
  pose2.pose.position.y = 5.0;
  quat2.setRPY(0, 0, 0.987);
  pose2.pose.orientation.w = quat2.w();
  pose2.pose.orientation.x = quat2.x();
  pose2.pose.orientation.y = quat2.y();
  pose2.pose.orientation.z = quat2.z();

  time2 = node->now();
  pose2.header.stamp = time2;
  pose2.header.frame_id = "frame2_id";

  poses.push_back(pose1);
  poses.push_back(pose2);

  nav_msgs::msg::Path path = posesToPath(poses);

  EXPECT_EQ(path.poses.size(), 2ul);
  EXPECT_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_EQ(path.poses[0].pose.position.y, 2.0);
  EXPECT_EQ(path.poses[0].header.stamp, time1);
  EXPECT_EQ(path.poses[0].header.frame_id, "frame1_id");
  EXPECT_EQ(path.poses[1].pose.position.x, 4.0);
  EXPECT_EQ(path.poses[1].pose.position.y, 5.0);
  EXPECT_EQ(path.poses[1].header.frame_id, "frame2_id");

  EXPECT_EQ(path.header.stamp, time1);
}

TEST(nav_2d_utils, PathToPathEmpty)
{
  nav_2d_msgs::msg::Path2D path2d;
  nav_msgs::msg::Path path = pathToPath(path2d);
  EXPECT_EQ(path.poses.size(), 0ul);
}

TEST(nav_2d_utils, PathToPathNoNEmpty)
{
  nav_2d_msgs::msg::Path2D path2d;

  geometry_msgs::msg::Pose2D pose1;
  pose1.x = 1.0;
  pose1.y = 2.0;
  pose1.theta = M_PI / 2.0;

  geometry_msgs::msg::Pose2D pose2;
  pose2.x = 4.0;
  pose2.y = 5.0;
  pose2.theta = M_PI;

  path2d.poses.push_back(pose1);
  path2d.poses.push_back(pose2);

  nav_msgs::msg::Path path = pathToPath(path2d);
  EXPECT_EQ(path.poses.size(), 2ul);
  EXPECT_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_EQ(path.poses[0].pose.position.y, 2.0);

  tf2::Quaternion quat;
  quat.setRPY(0, 0, M_PI / 2.0);
  EXPECT_EQ(path.poses[0].pose.orientation.w, quat.w());
  EXPECT_EQ(path.poses[0].pose.orientation.x, quat.x());
  EXPECT_EQ(path.poses[0].pose.orientation.y, quat.x());
  EXPECT_EQ(path.poses[0].pose.orientation.z, quat.z());

  EXPECT_EQ(path.poses[1].pose.position.x, 4.0);
  EXPECT_EQ(path.poses[1].pose.position.y, 5.0);
  quat.setRPY(0, 0, M_PI);
  EXPECT_EQ(path.poses[1].pose.orientation.w, quat.w());
  EXPECT_EQ(path.poses[1].pose.orientation.x, quat.x());
  EXPECT_EQ(path.poses[1].pose.orientation.y, quat.x());
  EXPECT_EQ(path.poses[1].pose.orientation.z, quat.z());
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
