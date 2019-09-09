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
using ComputePathToPoseResult = nav2_msgs::msg::Path;

// rclcpp::init can only be called once per process, so this needs to be a global variable
class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

// #TODO(orduno) Multiple startups and shutdowns of navfn kills the navfn process #1100
//               Re-enable after this is resolved
// TEST_F(PlannerTester, testSimpleCostmaps)
// {
//   activate();

//   nav2_util::startup_lifecycle_nodes("/navfn_planner");

//   std::vector<TestCostmap> costmaps = {
//     TestCostmap::open_space,
//     TestCostmap::bounded,
//     TestCostmap::top_left_obstacle,
//     TestCostmap::bottom_left_obstacle,
//     TestCostmap::maze1,
//     TestCostmap::maze2
//   };

//   ComputePathToPoseResult result;

//   for (auto costmap : costmaps) {
//     loadSimpleCostmap(costmap);
//     EXPECT_EQ(true, defaultPlannerTest(result));
//   }

//   nav2_util::reset_lifecycle_nodes("/navfn_planner");

//   deactivate();
// }

TEST_F(PlannerTester, testWithHundredRandomEndPoints)
{
  activate();
  loadDefaultMap();

  nav2_util::startup_lifecycle_nodes("/navfn_planner");

  EXPECT_EQ(true, defaultPlannerRandomTests(100, 0.1));

  nav2_util::reset_lifecycle_nodes("/navfn_planner");

  deactivate();
}
