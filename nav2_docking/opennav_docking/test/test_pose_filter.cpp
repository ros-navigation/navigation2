// Copyright (c) 2024 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "opennav_docking/pose_filter.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

// Testing the pose filter

namespace opennav_docking
{

TEST(PoseFilterTests, FilterTests)
{
  double coef = 0.5;
  double timeout = 1.0;
  PoseFilter filter(coef, timeout);

  // First measurement
  geometry_msgs::msg::PoseStamped meas1;
  meas1.header.frame_id = "test";
  meas1.header.stamp = rclcpp::Time(1, 0);
  meas1.pose.position.x = 1.0;
  meas1.pose.position.y = 3.0;
  meas1.pose.position.z = 5.0;
  meas1.pose.orientation.w = 1.0;

  // Update filter
  geometry_msgs::msg::PoseStamped pose = filter.update(meas1);

  // Header frame_id is inconsistent, so pose = measurment
  EXPECT_NEAR(pose.pose.position.x, 1.0, 0.0001);
  EXPECT_NEAR(pose.pose.position.y, 3.0, 0.0001);
  EXPECT_NEAR(pose.pose.position.z, 5.0, 0.0001);
  EXPECT_EQ(pose.pose.orientation.w, 1.0);

  // Create a second measurement
  geometry_msgs::msg::PoseStamped meas2 = meas1;
  meas2.header.stamp = rclcpp::Time(1, 500);
  meas2.pose.position.x = 2.0;
  meas2.pose.position.y = 4.0;
  meas2.pose.position.z = 6.0;
  double yaw = 0.5, pitch = 0.0, roll = 0.0;
  tf2::Quaternion quat;
  quat.setEuler(pitch, roll, yaw);
  meas2.pose.orientation = tf2::toMsg(quat);

  // Update filter, check expectations
  pose = filter.update(meas2);
  EXPECT_NEAR(pose.pose.position.x, 1.5, 0.0001);
  EXPECT_NEAR(pose.pose.position.y, 3.5, 0.0001);
  EXPECT_NEAR(pose.pose.position.z, 5.5, 0.0001);

  double pose_yaw = tf2::getYaw(pose.pose.orientation);
  EXPECT_NEAR(pose_yaw, 0.25, 0.0001);

  // Apply same measurement again
  pose = filter.update(meas2);
  EXPECT_NEAR(pose.pose.position.x, 1.75, 0.0001);
  EXPECT_NEAR(pose.pose.position.y, 3.75, 0.0001);
  EXPECT_NEAR(pose.pose.position.z, 5.75, 0.0001);

  pose_yaw = tf2::getYaw(pose.pose.orientation);
  EXPECT_NEAR(pose_yaw, 0.375, 0.0001);

  // Check timeout
  meas2.header.stamp = rclcpp::Time(3, 0);
  pose = filter.update(meas2);
  EXPECT_NEAR(pose.pose.position.x, 2.0, 0.0001);
  EXPECT_NEAR(pose.pose.position.y, 4.0, 0.0001);
  EXPECT_NEAR(pose.pose.position.z, 6.0, 0.0001);
}

}  // namespace opennav_docking
