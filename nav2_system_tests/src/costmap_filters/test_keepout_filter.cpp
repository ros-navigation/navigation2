// Copyright (c) 2020 Samsung Research Russia
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

#include "rclcpp/rclcpp.hpp"
#include "filters_tester.hpp"

using namespace std::chrono_literals;

std::shared_ptr<nav2_system_tests::FiltersTester> filters_tester;

// Checks that roobt can produce valid path plan and reach the goal.
// The goal lies behind keepout area that should force
// path planner to re-planm so finally the goal should be reached.
TEST(TestKeepoutFilter, testValidPlan)
{
  geometry_msgs::msg::PoseStamped start;
  start.pose.position.x = -4.0;
  start.pose.position.y = -4.0;
  geometry_msgs::msg::PoseStamped end;
  end.pose.position.x = 4.0;
  end.pose.position.y = 4.0;

  nav2_system_tests::TestStatus test_result = filters_tester->testPlan(start, end);
  EXPECT_EQ(nav2_system_tests::SUCCESS, test_result);
}

// Checks that robot will fail to go inside a keepout area
TEST(TestKeepoutFilter, testInvalidPlan)
{
  geometry_msgs::msg::PoseStamped start;
  start.pose.position.x = -4.0;
  start.pose.position.y = -4.0;
  geometry_msgs::msg::PoseStamped end;
  end.pose.position.x = 4.0;
  end.pose.position.y = -4.0;

  nav2_system_tests::TestStatus test_result = filters_tester->testPlan(start, end);
  EXPECT_EQ(nav2_system_tests::NO_PATH, test_result);
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Make FiltersTester and wait until it will be active
  filters_tester = std::make_shared<nav2_system_tests::FiltersTester>();
  while (!filters_tester->isActive()) {
    filters_tester->spinTester();
  }

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();
  filters_tester.reset();

  return test_result;
}
