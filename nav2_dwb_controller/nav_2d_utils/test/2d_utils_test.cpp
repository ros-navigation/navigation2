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
#include <vector>
#include <tf2/LinearMath/Quaternion.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "gtest/gtest.h"
#include "nav_2d_utils/conversions.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using nav_2d_utils::posesToPath;

TEST(nav_2d_utils, PosesToPathEmpty)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  auto node = std::make_shared<nav2::LifecycleNode>("twod_utils_test_node");
  rclcpp::Time time = node->now();
  std::string frame = "test_frame";

  nav_msgs::msg::Path path = posesToPath(poses, frame, time);

  EXPECT_EQ(path.poses.size(), 0ul);
  EXPECT_EQ(path.header.frame_id, frame);
  EXPECT_EQ(path.header.stamp, time);
}

TEST(nav_2d_utils, PosesToPathNonEmpty)
{
  auto node = std::make_shared<nav2::LifecycleNode>("twod_utils_test_node");
  rclcpp::Time time = node->now();
  std::string frame = "map";

  tf2::Quaternion quat1, quat2;
  quat1.setRPY(0, 0, 0.123);
  quat2.setRPY(0, 0, 0.987);

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 1.0;
  pose1.position.y = 2.0;
  pose1.orientation = tf2::toMsg(quat1);

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 4.0;
  pose2.position.y = 5.0;
  pose2.orientation = tf2::toMsg(quat2);

  std::vector<geometry_msgs::msg::Pose> poses = {pose1, pose2};

  nav_msgs::msg::Path path = posesToPath(poses, frame, time);

  EXPECT_EQ(path.poses.size(), 2ul);
  EXPECT_EQ(path.poses[0].pose.position.x, 1.0);
  EXPECT_EQ(path.poses[0].pose.position.y, 2.0);
  EXPECT_EQ(path.poses[1].pose.position.x, 4.0);
  EXPECT_EQ(path.poses[1].pose.position.y, 5.0);

  for (const auto & stamped_pose : path.poses) {
    EXPECT_EQ(stamped_pose.header.frame_id, frame);
    EXPECT_EQ(stamped_pose.header.stamp, time);
  }

  EXPECT_EQ(path.header.frame_id, frame);
  EXPECT_EQ(path.header.stamp, time);
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
