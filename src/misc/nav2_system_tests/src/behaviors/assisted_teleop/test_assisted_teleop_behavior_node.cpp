// Copyright (c) 2020 Samsung Research
// Copyright (c) 2020 Sarthak Mittal
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
#include <cmath>
#include <tuple>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "assisted_teleop_behavior_tester.hpp"
#include "nav2_msgs/action/back_up.hpp"

using namespace std::chrono_literals;

using nav2_system_tests::AssistedTeleopBehaviorTester;

struct TestParameters
{
  float lin_vel;
  float ang_vel;
};


std::string testNameGenerator(const testing::TestParamInfo<TestParameters> &)
{
  static int test_index = 0;
  std::string name = "AssistedTeleopTest" + std::to_string(test_index);
  ++test_index;
  return name;
}

class AssistedTeleopBehaviorTestFixture
  : public ::testing::TestWithParam<TestParameters>
{
public:
  static void SetUpTestCase()
  {
    assisted_teleop_behavior_tester = new AssistedTeleopBehaviorTester();
    if (!assisted_teleop_behavior_tester->isActive()) {
      assisted_teleop_behavior_tester->activate();
    }
  }

  static void TearDownTestCase()
  {
    delete assisted_teleop_behavior_tester;
    assisted_teleop_behavior_tester = nullptr;
  }

protected:
  static AssistedTeleopBehaviorTester * assisted_teleop_behavior_tester;
};

AssistedTeleopBehaviorTester *
AssistedTeleopBehaviorTestFixture::assisted_teleop_behavior_tester = nullptr;

TEST_P(AssistedTeleopBehaviorTestFixture, testAssistedTeleopBehavior)
{
  auto test_params = GetParam();

  if (!assisted_teleop_behavior_tester->isActive()) {
    assisted_teleop_behavior_tester->activate();
  }

  bool success = false;
  success = assisted_teleop_behavior_tester->defaultAssistedTeleopTest(
    test_params.lin_vel,
    test_params.ang_vel);

  EXPECT_TRUE(success);
}

std::vector<TestParameters> test_params = {TestParameters{-0.1, 0.0},
  TestParameters{0.35, 0.05}};

INSTANTIATE_TEST_SUITE_P(
  TestAssistedTeleopBehavior,
  AssistedTeleopBehaviorTestFixture,
  ::testing::Values(
    test_params[0],
    test_params[1]),
  testNameGenerator
);


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
