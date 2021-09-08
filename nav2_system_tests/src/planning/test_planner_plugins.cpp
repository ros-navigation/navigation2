// Copyright (c) 2020 Samsung Research
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
// limitations under the License. Reserved.

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "planner_tester.hpp"
#include "nav2_util/lifecycle_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

using namespace std::chrono_literals;

using nav2_system_tests::PlannerTester;
using nav2_util::TestCostmap;

using ComputePathToPoseCommand = geometry_msgs::msg::PoseStamped;
using ComputePathToPoseResult = nav_msgs::msg::Path;

void callback(const nav_msgs::msg::Path::ConstSharedPtr /*grid*/)
{
}

void testSmallPathValidityAndOrientation(std::string plugin, double length)
{
  auto obj = std::make_shared<nav2_system_tests::NavFnPlannerTester>();
  rclcpp_lifecycle::State state;
  obj->set_parameter(rclcpp::Parameter("GridBased.plugin", plugin));
  obj->declare_parameter(
    "GridBased.use_final_approach_orientation", rclcpp::ParameterValue(false));
  obj->onConfigure(state);

  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::PoseStamped goal;

  start.pose.position.x = 0.5;
  start.pose.position.y = 0.5;
  start.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI_2);
  start.header.frame_id = "map";

  goal.pose.position.x = 0.5;
  goal.pose.position.y = start.pose.position.y + length;
  goal.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(-M_PI);
  goal.header.frame_id = "map";

  // Test without use_final_approach_orientation
  // expecting end path pose orientation to be equal to goal orientation
  auto path = obj->getPlan(start, goal, "GridBased");
  EXPECT_GT((int)path.poses.size(), 0);
  EXPECT_NEAR(tf2::getYaw(path.poses.back().pose.orientation), -M_PI, 0.01);
  obj->onCleanup(state);

  // Test WITH use_final_approach_orientation
  // expecting end path pose orientation to be equal to approach orientation
  // which in the one pose corner case should be the start pose orientation
  obj->set_parameter(rclcpp::Parameter("GridBased.use_final_approach_orientation", true));
  obj->onConfigure(state);
  path = obj->getPlan(start, goal, "GridBased");
  EXPECT_GT((int)path.poses.size(), 0);

  int path_size = path.poses.size();
  if (path_size == 1) {
    EXPECT_NEAR(
      tf2::getYaw(path.poses.back().pose.orientation),
      tf2::getYaw(start.pose.orientation),
      0.01);
  } else {
    double dx = path.poses.back().pose.position.x - path.poses.front().pose.position.x;
    double dy = path.poses.back().pose.position.y - path.poses.front().pose.position.y;
    EXPECT_NEAR(
      tf2::getYaw(path.poses.back().pose.orientation),
      atan2(dy, dx),
      0.01);
  }
  obj->onCleanup(state);
}

TEST(testPluginMap, Failures)
{
  auto obj = std::make_shared<nav2_system_tests::NavFnPlannerTester>();
  rclcpp_lifecycle::State state;
  obj->set_parameter(rclcpp::Parameter("expected_planner_frequency", 100000.0));
  obj->onConfigure(state);
  obj->create_subscription<nav_msgs::msg::Path>(
    "plan", rclcpp::SystemDefaultsQoS(), callback);

  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::PoseStamped goal;
  std::string plugin_fake = "fake";
  std::string plugin_none = "";
  auto path = obj->getPlan(start, goal, plugin_none);
  EXPECT_EQ(path.header.frame_id, std::string("map"));

  path = obj->getPlan(start, goal, plugin_fake);
  EXPECT_EQ(path.poses.size(), 0ul);

  obj->onCleanup(state);
}

TEST(testPluginMap, Smac2dEqualStartGoal)
{
  testSmallPathValidityAndOrientation("nav2_smac_planner/SmacPlanner2D", 0.0);
}

TEST(testPluginMap, Smac2dVerySmallPath)
{
  testSmallPathValidityAndOrientation("nav2_smac_planner/SmacPlanner2D", 0.00001);
}

TEST(testPluginMap, Smac2dBelowCostmapResolution)
{
  testSmallPathValidityAndOrientation("nav2_smac_planner/SmacPlanner2D", 0.09);
}

TEST(testPluginMap, Smac2dJustAboveCostmapResolution)
{
  testSmallPathValidityAndOrientation("nav2_smac_planner/SmacPlanner2D", 0.102);
}

TEST(testPluginMap, Smac2dAboveCostmapResolution)
{
  testSmallPathValidityAndOrientation("nav2_smac_planner/SmacPlanner2D", 1.5);
}

TEST(testPluginMap, NavFnEqualStartGoal)
{
  testSmallPathValidityAndOrientation("nav2_navfn_planner/NavfnPlanner", 0.0);
}

TEST(testPluginMap, NavFnVerySmallPath)
{
  testSmallPathValidityAndOrientation("nav2_navfn_planner/NavfnPlanner", 0.00001);
}

TEST(testPluginMap, NavFnBelowCostmapResolution)
{
  testSmallPathValidityAndOrientation("nav2_navfn_planner/NavfnPlanner", 0.09);
}

TEST(testPluginMap, NavFnJustAboveCostmapResolution)
{
  testSmallPathValidityAndOrientation("nav2_navfn_planner/NavfnPlanner", 0.102);
}

TEST(testPluginMap, NavFnAboveCostmapResolution)
{
  testSmallPathValidityAndOrientation("nav2_navfn_planner/NavfnPlanner", 1.5);
}

TEST(testPluginMap, ThetaStarEqualStartGoal)
{
  testSmallPathValidityAndOrientation("nav2_theta_star_planner/ThetaStarPlanner", 0.0);
}

TEST(testPluginMap, ThetaStarVerySmallPath)
{
  testSmallPathValidityAndOrientation("nav2_theta_star_planner/ThetaStarPlanner", 0.00001);
}

TEST(testPluginMap, ThetaStarBelowCostmapResolution)
{
  testSmallPathValidityAndOrientation("nav2_theta_star_planner/ThetaStarPlanner", 0.09);
}

TEST(testPluginMap, ThetaStarJustAboveCostmapResolution)
{
  testSmallPathValidityAndOrientation("nav2_theta_star_planner/ThetaStarPlanner", 0.102);
}

TEST(testPluginMap, ThetaStarAboveCostmapResolution)
{
  testSmallPathValidityAndOrientation("nav2_theta_star_planner/ThetaStarPlanner", 1.5);
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
