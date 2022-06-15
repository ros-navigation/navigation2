// Copyright (c) 2022 Joshua Wallace
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

#include "nav2_msgs/srv/is_path_valid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "planner_tester.hpp"
#include "nav2_util/lifecycle_utils.hpp"

using nav2_system_tests::PlannerTester;
using nav2_util::TestCostmap;

TEST(testIsPathValid, testIsPathValid)
{
  auto planner_tester = std::make_shared<PlannerTester>();
  planner_tester->activate();
  planner_tester->loadSimpleCostmap(TestCostmap::top_left_obstacle);

  nav_msgs::msg::Path path;

  // empty path
  bool is_path_valid = planner_tester->isPathValid(path);
  EXPECT_FALSE(is_path_valid);

  // invalid path
  for (float i = 0; i < 10; i += 1.0) {
    for (float j = 0; j < 10; j += 1.0) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = i;
      pose.pose.position.y = j;
      path.poses.push_back(pose);
    }
  }
  is_path_valid = planner_tester->isPathValid(path);
  EXPECT_FALSE(is_path_valid);

  // valid path
  path.poses.clear();
  for (float i = 0; i < 10; i += 1.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = i;
    path.poses.push_back(pose);
  }
  is_path_valid = planner_tester->isPathValid(path);
  EXPECT_TRUE(is_path_valid);
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
