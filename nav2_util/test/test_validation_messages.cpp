// Copyright (c) 2024 GoesM
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

#include <gtest/gtest.h>
#include "nav2_util/validate_messages.hpp"

TEST(ValidateMessagesTest, DoubleValueCheck) {
  // Test valid double value
  EXPECT_TRUE(nav2_util::validateMsg(3.14));
  // Test invalid double value (infinity)
  EXPECT_FALSE(nav2_util::validateMsg(std::numeric_limits<double>::infinity()));
  // Test invalid double value (NaN)
  EXPECT_FALSE(nav2_util::validateMsg(std::numeric_limits<double>::quiet_NaN()));
}

TEST(ValidateMessagesTest, TimeStampCheck)
{
  // Test valid time stamp
  builtin_interfaces::msg::Time valid_time_stamp;
  valid_time_stamp.sec = 123;
  valid_time_stamp.nanosec = 456789;
  EXPECT_TRUE(nav2_util::validateMsg(valid_time_stamp));
  // Test invalid time stamp (nanosec out of range)
  builtin_interfaces::msg::Time invalid_time_stamp;
  invalid_time_stamp.sec = 123;
  invalid_time_stamp.nanosec = 1e9;   // 1 second = 1e9 nanoseconds
  EXPECT_FALSE(nav2_util::validateMsg(invalid_time_stamp));
}

TEST(ValidateMessagesTest, HeaderCheck)
{
  // Test valid header with non-empty frame_id
  std_msgs::msg::Header valid_header;
  valid_header.stamp.sec = 123;
  valid_header.stamp.nanosec = 456789;
  valid_header.frame_id = "map";
  EXPECT_TRUE(nav2_util::validateMsg(valid_header));
  // Test invalid header with empty frame_id
  std_msgs::msg::Header invalid_header;
  invalid_header.stamp.sec = 123;
  invalid_header.stamp.nanosec = 456789;
  invalid_header.frame_id = "";
  EXPECT_FALSE(nav2_util::validateMsg(invalid_header));
  invalid_header.stamp.sec = 123;
  invalid_header.stamp.nanosec = 1e9;
  invalid_header.frame_id = "map";
  EXPECT_FALSE(nav2_util::validateMsg(invalid_header));
}

TEST(ValidateMessagesTest, PointCheck)
{
  // Test valid Point message
  geometry_msgs::msg::Point valid_point;
  valid_point.x = 1.0;
  valid_point.y = 2.0;
  valid_point.z = 3.0;
  EXPECT_TRUE(nav2_util::validateMsg(valid_point));
  // Test invalid Point message with NaN value
  geometry_msgs::msg::Point invalid_point;
  invalid_point.x = 1.0;
  invalid_point.y = std::numeric_limits<double>::quiet_NaN();
  invalid_point.z = 3.0;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_point));
  // Test invalid Point message with NaN value
  invalid_point.x = std::numeric_limits<double>::quiet_NaN();
  invalid_point.y = 2.0;
  invalid_point.z = 3.0;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_point));
  // Test invalid Point message with NaN value
  invalid_point.x = 1.0;
  invalid_point.y = 2.0;
  invalid_point.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(nav2_util::validateMsg(invalid_point));
}

TEST(ValidateMessagesTest, QuaternionCheck)
{
  // Test valid Quaternion message
  geometry_msgs::msg::Quaternion valid_quaternion;
  valid_quaternion.x = 0.0;
  valid_quaternion.y = 0.0;
  valid_quaternion.z = 0.0;
  valid_quaternion.w = 1.0;
  EXPECT_TRUE(nav2_util::validateMsg(valid_quaternion));
  // Test invalid Quaternion message with invalid magnitude
  geometry_msgs::msg::Quaternion invalid_quaternion;
  invalid_quaternion.x = 0.1;
  invalid_quaternion.y = 0.2;
  invalid_quaternion.z = 0.3;
  invalid_quaternion.w = 0.5;   // Invalid magnitude (should be 1.0)
  EXPECT_FALSE(nav2_util::validateMsg(invalid_quaternion));

  // One NaN value
  invalid_quaternion.x = 0.0;
  invalid_quaternion.y = std::numeric_limits<double>::quiet_NaN();
  invalid_quaternion.z = 0.0;
  invalid_quaternion.w = 1.0;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_quaternion));
  invalid_quaternion.x = std::numeric_limits<double>::quiet_NaN();
  invalid_quaternion.y = 0.0;
  invalid_quaternion.z = 0.0;
  invalid_quaternion.w = 1.0;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_quaternion));
  invalid_quaternion.x = 0.0;
  invalid_quaternion.y = 0.0;
  invalid_quaternion.z = std::numeric_limits<double>::quiet_NaN();
  invalid_quaternion.w = 1.0;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_quaternion));
  invalid_quaternion.x = 0.0;
  invalid_quaternion.y = 0.0;
  invalid_quaternion.z = 1.0;
  invalid_quaternion.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(nav2_util::validateMsg(invalid_quaternion));
}

TEST(ValidateMessagesTest, PoseCheck)
{
  // Test valid Pose message
  geometry_msgs::msg::Pose valid_pose;
  valid_pose.position.x = 1.0;
  valid_pose.position.y = 2.0;
  valid_pose.position.z = 3.0;
  valid_pose.orientation.x = 1.0;
  valid_pose.orientation.y = 0.0;
  valid_pose.orientation.z = 0.0;
  valid_pose.orientation.w = 0.0;
  EXPECT_TRUE(nav2_util::validateMsg(valid_pose));
  // Test invalid Pose message with invalid position
  geometry_msgs::msg::Pose invalid_pose;
  invalid_pose.position.x = 1.0;
  invalid_pose.position.y = std::numeric_limits<double>::quiet_NaN();
  invalid_pose.position.z = 3.0;
  invalid_pose.orientation.x = 1.0;
  invalid_pose.orientation.y = 0.0;
  invalid_pose.orientation.z = 0.0;
  invalid_pose.orientation.w = 0.0;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_pose));
  // Test invalid Pose message with invalid orientation
  invalid_pose.position.x = 1.0;
  invalid_pose.position.y = 2.0;
  invalid_pose.position.z = 3.0;
  invalid_pose.orientation.x = 0.1;
  invalid_pose.orientation.y = 0.2;
  invalid_pose.orientation.z = 0.3;
  invalid_pose.orientation.w = 0.4;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_pose));
}


TEST(ValidateMessagesTest, MapMetaDataCheck) {
  // Test valid MapMetaData message
  nav_msgs::msg::MapMetaData valid_map_meta_data;
  valid_map_meta_data.resolution = 0.05;
  valid_map_meta_data.width = 100;
  valid_map_meta_data.height = 100;
  geometry_msgs::msg::Pose valid_origin;
  valid_origin.position.x = 0.0;
  valid_origin.position.y = 0.0;
  valid_origin.position.z = 0.0;
  valid_origin.orientation.x = 0.0;
  valid_origin.orientation.y = 0.0;
  valid_origin.orientation.z = 0.0;
  valid_origin.orientation.w = 1.0;
  valid_map_meta_data.origin = valid_origin;
  EXPECT_TRUE(nav2_util::validateMsg(valid_map_meta_data));

  // Test invalid origin message
  nav_msgs::msg::MapMetaData invalid_map_meta_data;
  invalid_map_meta_data.resolution = 100.0;
  invalid_map_meta_data.width = 100;
  invalid_map_meta_data.height = 100;
  geometry_msgs::msg::Pose invalid_origin;
  invalid_origin.position.x = 0.0;
  invalid_origin.position.y = 0.0;
  invalid_origin.position.z = 0.0;
  invalid_origin.orientation.x = 0.0;
  invalid_origin.orientation.y = 0.0;
  invalid_origin.orientation.z = 1.0;
  invalid_origin.orientation.w = 1.0;
  invalid_map_meta_data.origin = invalid_origin;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_map_meta_data));

  // Test invalid resolution message
  invalid_map_meta_data.resolution = std::numeric_limits<double>::quiet_NaN();
  invalid_map_meta_data.width = 100;
  invalid_map_meta_data.height = 100;
  invalid_map_meta_data.origin = valid_origin;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_map_meta_data));

  // Test invalid MapMetaData message with zero width
  invalid_map_meta_data.resolution = 0.05;
  invalid_map_meta_data.width = 0;
  invalid_map_meta_data.height = 100;
  invalid_map_meta_data.origin = valid_origin;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_map_meta_data));
}

TEST(ValidateMessagesTest, OccupancyGridCheck) {
  // Test valid OccupancyGrid message
  nav_msgs::msg::OccupancyGrid valid_occupancy_grid;
  valid_occupancy_grid.header.frame_id = "map";
  valid_occupancy_grid.info.resolution = 0.05;
  valid_occupancy_grid.info.width = 100;
  valid_occupancy_grid.info.height = 100;
  std::vector<int8_t> data(100 * 100, 0);   // Initialize with zeros
  valid_occupancy_grid.data = data;
  EXPECT_TRUE(nav2_util::validateMsg(valid_occupancy_grid));

  // Test invalid header message with wrong data size
  nav_msgs::msg::OccupancyGrid invalid_occupancy_grid;
  invalid_occupancy_grid.header.frame_id = "";    // Incorrect id
  invalid_occupancy_grid.info.resolution = 0.05;
  invalid_occupancy_grid.info.width = 100;
  invalid_occupancy_grid.info.height = 100;
  invalid_occupancy_grid.data = data;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_occupancy_grid));

  // Test invalid info message with wrong data size
  invalid_occupancy_grid.header.frame_id = "map";
  invalid_occupancy_grid.info.resolution = 0.05;
  invalid_occupancy_grid.info.width = 0;    // Incorrect width
  invalid_occupancy_grid.info.height = 100;
  invalid_occupancy_grid.data = data;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_occupancy_grid));

  // Test invalid OccupancyGrid message with wrong data size
  invalid_occupancy_grid.header.frame_id = "map";
  invalid_occupancy_grid.info.resolution = 0.05;
  invalid_occupancy_grid.info.width = 100;
  invalid_occupancy_grid.info.height = 100;
  std::vector<int8_t> invalid_data(100 * 99, 0);   // Incorrect data size
  invalid_occupancy_grid.data = invalid_data;
  EXPECT_FALSE(nav2_util::validateMsg(invalid_occupancy_grid));
}

// Add more test cases for other validateMsg functions if needed
