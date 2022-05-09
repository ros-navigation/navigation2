// Copyright (c) 2020 Samsung Research
// Copyright (c) 2020 Sarthak Mittal
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
#include <cmath>
#include <tuple>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "drive_on_heading_behavior_tester.hpp"

using namespace std::chrono_literals;

using nav2_system_tests::DriveOnHeadingBehaviorTester;

struct TestParameters
{
  float x;
  float y;
  float speed;
  float time_allowance;
  float tolerance;
};

std::string testNameGenerator(const testing::TestParamInfo<TestParameters> &)
{
  static int test_index = 0;
  std::string name = "DriveOnHeadingTest" + std::to_string(test_index);
  ++test_index;
  return name;
}

class DriveOnHeadingBehaviorTestFixture
  : public ::testing::TestWithParam<TestParameters>
{
public:
  static void SetUpTestCase()
  {
    drive_on_heading_behavior_tester = new DriveOnHeadingBehaviorTester();
    if (!drive_on_heading_behavior_tester->isActive()) {
      drive_on_heading_behavior_tester->activate();
    }
  }

  static void TearDownTestCase()
  {
    delete drive_on_heading_behavior_tester;
    drive_on_heading_behavior_tester = nullptr;
  }

protected:
  static DriveOnHeadingBehaviorTester * drive_on_heading_behavior_tester;
};

DriveOnHeadingBehaviorTester * DriveOnHeadingBehaviorTestFixture::drive_on_heading_behavior_tester =
  nullptr;

TEST_P(DriveOnHeadingBehaviorTestFixture, testBackupBehavior)
{
  auto test_params = GetParam();
  auto goal = nav2_msgs::action::DriveOnHeading::Goal();
  goal.target.x = test_params.x;
  goal.target.y = test_params.y;
  goal.speed = test_params.speed;
  goal.time_allowance.sec = test_params.time_allowance;
  float tolerance = test_params.tolerance;

  if (!drive_on_heading_behavior_tester->isActive()) {
    drive_on_heading_behavior_tester->activate();
  }

  bool success = false;
  success = drive_on_heading_behavior_tester->defaultDriveOnHeadingBehaviorTest(
    goal,
    tolerance);

  float dist_to_obstacle = 2.0f;
  if ( ((dist_to_obstacle - std::fabs(test_params.x)) < std::fabs(goal.speed)) ||
    std::fabs(goal.target.y) > 0 ||
    goal.time_allowance.sec < 2.0 ||
    !((goal.target.x > 0.0) == (goal.speed > 0.0)))
  {
    EXPECT_FALSE(success);
  } else {
    EXPECT_TRUE(success);
  }
}

std::vector<TestParameters> test_params = {TestParameters{-0.05, 0.0, -0.2, 10.0, 0.01},
  TestParameters{-0.05, 0.1, -0.2, 10.0, 0.01},
  TestParameters{-2.0, 0.0, -0.2, 10.0, 0.1},
  TestParameters{-0.05, 0.0, -0.01, 1.0, 0.01},
  TestParameters{0.05, 0.0, -0.2, 10.0, 0.01}};

INSTANTIATE_TEST_SUITE_P(
  DriveOnHeadingBehaviorTests,
  DriveOnHeadingBehaviorTestFixture,
  ::testing::Values(
    test_params[0],
    test_params[1],
    test_params[2],
    test_params[3],
    test_params[4]),
  testNameGenerator);

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
