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

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_docking
{

TEST(UtilsTests, parseDockParams1)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  DockMap db;
  std::vector<std::string> dock_str = {"dockA", "dockB"};
  node->declare_parameter("docks", rclcpp::ParameterValue(dock_str));

  std::vector<std::string> docks_param;
  node->get_parameter("docks", docks_param);

  // No type set
  EXPECT_FALSE(utils::parseDockParams(docks_param, node, db));
}

TEST(UtilsTests, parseDockParams2)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test2");
  DockMap db;
  std::vector<std::string> dock_str = {"dockC", "dockD"};
  node->declare_parameter("docks", rclcpp::ParameterValue(dock_str));

  // Don't declare B, check if "map" default
  node->declare_parameter("dockC.frame", rclcpp::ParameterValue(std::string("mapA")));

  node->declare_parameter("dockC.type", rclcpp::ParameterValue(std::string("typeA")));
  node->declare_parameter("dockD.type", rclcpp::ParameterValue(std::string("typeB")));
  std::vector<double> dock_pose = {0.3, 0.3, 0.3};
  node->declare_parameter("dockC.pose", rclcpp::ParameterValue(dock_pose));
  node->declare_parameter("dockD.pose", rclcpp::ParameterValue(dock_pose));

  // Don't declare C, check if empty string default
  node->declare_parameter("dockD.id", rclcpp::ParameterValue("D"));

  std::vector<std::string> docks_param;
  node->get_parameter("docks", docks_param);

  EXPECT_TRUE(utils::parseDockParams(docks_param, node, db));
  EXPECT_EQ(db["dockC"].frame, std::string("mapA"));
  EXPECT_EQ(db["dockD"].frame, std::string("map"));
  EXPECT_EQ(db["dockC"].type, std::string("typeA"));
  EXPECT_EQ(db["dockC"].pose.position.x, 0.3);
  EXPECT_EQ(db["dockC"].pose.position.y, 0.3);
  EXPECT_EQ(db["dockC"].id, std::string(""));
  EXPECT_EQ(db["dockD"].id, std::string("D"));
}

TEST(UtilsTests, parseDockParams3)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test3");
  DockMap db;
  std::vector<std::string> dock_str = {"dockE", "dockF"};
  node->declare_parameter("docks", rclcpp::ParameterValue(dock_str));

  node->declare_parameter("dockE.type", rclcpp::ParameterValue(std::string("typeA")));
  node->declare_parameter("dockF.type", rclcpp::ParameterValue(std::string("typeB")));

  std::vector<std::string> docks_param;
  node->get_parameter("docks", docks_param);

  // Incorrect pose size
  std::vector<double> dock_pose_err = {0.3, 0.3};
  node->declare_parameter("dockE.pose", rclcpp::ParameterValue(dock_pose_err));
  EXPECT_FALSE(utils::parseDockParams(docks_param, node, db));
}

TEST(UtilsTests, parseDockFile)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test4");
  DockMap db;
  std::string filepath = ament_index_cpp::get_package_share_directory("opennav_docking") +
    "/test_dock_file.yaml";
  EXPECT_TRUE(utils::parseDockFile(filepath, node, db));
  EXPECT_EQ(db.size(), 2u);
  EXPECT_EQ(db["dock1"].frame, std::string("mapA"));
  EXPECT_EQ(db["dock2"].frame, std::string("map"));
  EXPECT_EQ(db["dock1"].type, std::string("dockv3"));
  EXPECT_EQ(db["dock2"].type, std::string("dockv1"));
  EXPECT_EQ(db["dock1"].pose.position.x, 0.3);
  EXPECT_EQ(db["dock1"].pose.position.y, 0.3);
  EXPECT_EQ(db["dock1"].pose.orientation.w, 1.0);
  EXPECT_EQ(db["dock2"].pose.position.x, 0.0);
  EXPECT_EQ(db["dock2"].pose.position.y, 0.0);
  EXPECT_NE(db["dock2"].pose.orientation.w, 1.0);
  EXPECT_EQ(db["dock1"].id, std::string(""));
  EXPECT_EQ(db["dock2"].id, std::string("2"));
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

}  // namespace opennav_docking
