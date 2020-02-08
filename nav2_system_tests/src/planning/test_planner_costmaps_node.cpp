// Copyright (c) 2018 Intel Corporation
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

#include "rclcpp/rclcpp.hpp"
#include "planner_tester.hpp"
#include "nav2_util/lifecycle_utils.hpp"

using namespace std::chrono_literals;

using nav2_system_tests::PlannerTester;
using nav2_util::TestCostmap;

using ComputePathToPoseCommand = geometry_msgs::msg::PoseStamped;
using ComputePathToPoseResult = nav_msgs::msg::Path;

TEST(testSimpleCostmaps, testSimpleCostmaps)
{
  auto obj = std::make_shared<PlannerTester>();

  std::vector<TestCostmap> costmaps = {
    TestCostmap::open_space,
    TestCostmap::bounded,
    TestCostmap::top_left_obstacle,
    TestCostmap::bottom_left_obstacle,
    TestCostmap::maze1,
    TestCostmap::maze2
  };

  ComputePathToPoseResult result;

  obj->activate();

  for (auto costmap : costmaps) {
    obj->loadSimpleCostmap(costmap);
    EXPECT_EQ(true, obj->defaultPlannerTest(result));
  }
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
