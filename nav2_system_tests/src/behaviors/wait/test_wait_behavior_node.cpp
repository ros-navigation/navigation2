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

#include "wait_behavior_tester.hpp"

using namespace std::chrono_literals;

using nav2_system_tests::WaitBehaviorTester;

std::string testNameGenerator(const testing::TestParamInfo<std::tuple<float, float>> & param)
{
  std::string name = std::to_string(std::abs(std::get<0>(param.param))) + "_" + std::to_string(
    std::get<1>(param.param));
  name.erase(std::remove(name.begin(), name.end(), '.'), name.end());
  return name;
}

class WaitBehaviorTestFixture
  : public ::testing::TestWithParam<std::tuple<float, float>>
{
public:
  static void SetUpTestCase()
  {
    wait_behavior_tester = new WaitBehaviorTester();
    if (!wait_behavior_tester->isActive()) {
      wait_behavior_tester->activate();
    }
  }

  static void TearDownTestCase()
  {
    delete wait_behavior_tester;
    wait_behavior_tester = nullptr;
  }

protected:
  static WaitBehaviorTester * wait_behavior_tester;
};

WaitBehaviorTester * WaitBehaviorTestFixture::wait_behavior_tester = nullptr;

TEST_P(WaitBehaviorTestFixture, testSWaitBehavior)
{
  float wait_time = std::get<0>(GetParam());
  float cancel = std::get<1>(GetParam());

  bool success = false;
  int num_tries = 3;
  for (int i = 0; i != num_tries; i++) {
    if (cancel == 1.0) {
      success = success || wait_behavior_tester->behaviorTestCancel(wait_time);
    } else {
      success = success || wait_behavior_tester->behaviorTest(wait_time);
    }
    if (success) {
      break;
    }
  }

  EXPECT_EQ(true, success);
}

INSTANTIATE_TEST_SUITE_P(
  WaitBehaviorTests,
  WaitBehaviorTestFixture,
  ::testing::Values(
    std::make_tuple(1.0, 0.0),
    std::make_tuple(2.0, 0.0),
    std::make_tuple(5.0, 0.0),
    std::make_tuple(10.0, 1.0)),
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
