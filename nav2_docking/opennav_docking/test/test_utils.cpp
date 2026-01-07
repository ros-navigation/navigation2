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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "opennav_docking/utils.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Test parsing dock plugins and database files (see test_dock_file.yaml).

namespace opennav_docking
{

TEST(UtilsTests, parseDockParams1)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test");
  DockMap db;
  std::vector<std::string> dock_str = {"dock_a", "dock_b"};
  node->declare_parameter("docks", rclcpp::ParameterValue(dock_str));

  std::vector<std::string> docks_param;
  node->get_parameter("docks", docks_param);

  // No type set
  EXPECT_FALSE(utils::parseDockParams(docks_param, node, db));
}

TEST(UtilsTests, parseDockParams2)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test2");
  DockMap db;
  std::vector<std::string> dock_str = {"dock_c", "dock_d"};
  node->declare_parameter("docks", rclcpp::ParameterValue(dock_str));

  // Don't declare dock_d.frame, check if "map" default
  node->declare_parameter("dock_c.frame", rclcpp::ParameterValue(std::string("map_a")));

  node->declare_parameter("dock_c.type", rclcpp::ParameterValue(std::string("type_a")));
  node->declare_parameter("dock_d.type", rclcpp::ParameterValue(std::string("type_b")));
  std::vector<double> dock_pose = {0.3, 0.3, 0.3};
  node->declare_parameter("dock_c.pose", rclcpp::ParameterValue(dock_pose));
  node->declare_parameter("dock_d.pose", rclcpp::ParameterValue(dock_pose));

  // Don't declare dock_c.id, check if empty string default
  node->declare_parameter("dock_d.id", rclcpp::ParameterValue("d"));

  std::vector<std::string> docks_param;
  node->get_parameter("docks", docks_param);

  EXPECT_TRUE(utils::parseDockParams(docks_param, node, db));
  EXPECT_EQ(db["dock_c"].frame, std::string("map_a"));
  EXPECT_EQ(db["dock_d"].frame, std::string("map"));
  EXPECT_EQ(db["dock_c"].type, std::string("type_a"));
  EXPECT_EQ(db["dock_c"].pose.position.x, 0.3);
  EXPECT_EQ(db["dock_c"].pose.position.y, 0.3);
  EXPECT_EQ(db["dock_c"].id, std::string(""));
  EXPECT_EQ(db["dock_d"].id, std::string("d"));
}

TEST(UtilsTests, parseDockParams3)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test3");
  DockMap db;
  std::vector<std::string> dock_str = {"dock_e", "dock_f"};
  node->declare_parameter("docks", rclcpp::ParameterValue(dock_str));

  node->declare_parameter("dock_e.type", rclcpp::ParameterValue(std::string("type_a")));
  node->declare_parameter("dock_f.type", rclcpp::ParameterValue(std::string("type_b")));

  std::vector<std::string> docks_param;
  node->get_parameter("docks", docks_param);

  // Incorrect pose size
  std::vector<double> dock_pose_err = {0.3, 0.3};
  node->declare_parameter("dock_e.pose", rclcpp::ParameterValue(dock_pose_err));
  EXPECT_FALSE(utils::parseDockParams(docks_param, node, db));
}

TEST(UtilsTests, parseDockFile)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test4");
  DockMap db;
  std::string filepath = ament_index_cpp::get_package_share_directory("opennav_docking") +
    "/dock_files/test_dock_file.yaml";
  EXPECT_TRUE(utils::parseDockFile(filepath, node, db));
  EXPECT_EQ(db.size(), 2u);
  EXPECT_EQ(db["dock_1"].frame, std::string("map_a"));
  EXPECT_EQ(db["dock_2"].frame, std::string("map"));
  EXPECT_EQ(db["dock_1"].type, std::string("dockv_3"));
  EXPECT_EQ(db["dock_2"].type, std::string("dockv_1"));
  EXPECT_EQ(db["dock_1"].pose.position.x, 0.3);
  EXPECT_EQ(db["dock_1"].pose.position.y, 0.3);
  EXPECT_EQ(db["dock_1"].pose.orientation.w, 1.0);
  EXPECT_EQ(db["dock_2"].pose.position.x, 0.0);
  EXPECT_EQ(db["dock_2"].pose.position.y, 0.0);
  EXPECT_NE(db["dock_2"].pose.orientation.w, 1.0);
  EXPECT_EQ(db["dock_1"].id, std::string(""));
  EXPECT_EQ(db["dock_2"].id, std::string("2"));
}

TEST(UtilsTests, parseDockFile2)
{
  auto node = std::make_shared<nav2::LifecycleNode>("test4");
  DockMap db;

  // Test with a file that has no docks
  std::string filepath = ament_index_cpp::get_package_share_directory("opennav_docking") +
    "/dock_files/test_no_docks_file.yaml";
  EXPECT_FALSE(utils::parseDockFile(filepath, node, db));

  // Test with a file that has no type
  filepath = ament_index_cpp::get_package_share_directory("opennav_docking") +
    "/dock_files/test_dock_no_type_file.yaml";
  EXPECT_FALSE(utils::parseDockFile(filepath, node, db));

  // Test with a file that has no pose
  filepath = ament_index_cpp::get_package_share_directory("opennav_docking") +
    "/dock_files/test_dock_no_pose_file.yaml";
  EXPECT_FALSE(utils::parseDockFile(filepath, node, db));

  // Test with a file that has wring pose array size
  filepath = ament_index_cpp::get_package_share_directory("opennav_docking") +
    "/dock_files/test_dock_bad_pose_file.yaml";
  EXPECT_FALSE(utils::parseDockFile(filepath, node, db));
}

TEST(UtilsTests, testgetDockPoseStamped)
{
  Dock d;
  d.frame = "dock_f";
  d.pose.position.x = 0.1;
  rclcpp::Time t(10, 10);
  auto pose = utils::getDockPoseStamped(&d, t);
  EXPECT_EQ(pose.header.frame_id, "dock_f");
  EXPECT_EQ(pose.pose.position.x, 0.1);
}

TEST(UtilsTests, testl2Norm)
{
  geometry_msgs::msg::Pose a, b;
  a.position.x = 1.0;
  EXPECT_NEAR(utils::l2Norm(a, b), 1.0, 1e-3);
  b.position.x = 0.5;
  EXPECT_NEAR(utils::l2Norm(a, b), 0.5, 1e-3);
  b.position.y = 0.5;
  EXPECT_NEAR(utils::l2Norm(a, b), 0.7071, 1e-3);
  a.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.5);
  EXPECT_NEAR(utils::l2Norm(a, b), 0.8660, 1e-3);
  b.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.7);
  EXPECT_NEAR(utils::l2Norm(a, b), 0.734, 1e-3);
}

TEST(UtilsTests, testGetDockDirectionFromString) {
  using opennav_docking_core::DockDirection;
  EXPECT_EQ(utils::getDockDirectionFromString("forward"), DockDirection::FORWARD);
  EXPECT_EQ(utils::getDockDirectionFromString("backward"), DockDirection::BACKWARD);
  EXPECT_EQ(utils::getDockDirectionFromString("other"), DockDirection::UNKNOWN);
}

}  // namespace opennav_docking

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
