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

using namespace std::chrono_literals;

using nav2_system_tests::PlannerTester;
using nav2_util::TestCostmap;

using ComputePathToPoseCommand = geometry_msgs::msg::PoseStamped;
using ComputePathToPoseResult = nav_msgs::msg::Path;

void callback(const nav_msgs::msg::Path::ConstSharedPtr /*grid*/)
{
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
