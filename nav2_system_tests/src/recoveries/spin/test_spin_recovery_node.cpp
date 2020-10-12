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

#include "spin_recovery_tester.hpp"

using namespace std::chrono_literals;

using nav2_system_tests::SpinRecoveryTester;

std::string testNameGenerator(const testing::TestParamInfo<std::tuple<float, float>> & param)
{
  std::string name = std::to_string(std::abs(std::get<0>(param.param))) + "_" + std::to_string(
    std::get<1>(param.param));
  name.erase(std::remove(name.begin(), name.end(), '.'), name.end());
  return name;
}

class SpinRecoveryTestFixture
  : public ::testing::TestWithParam<std::tuple<float, float>>
{
public:
  static void SetUpTestCase()
  {
    spin_recovery_tester = new SpinRecoveryTester();
    if (!spin_recovery_tester->isActive()) {
      spin_recovery_tester->activate();
    }
  }

  static void TearDownTestCase()
  {
    delete spin_recovery_tester;
    spin_recovery_tester = nullptr;
  }

protected:
  static SpinRecoveryTester * spin_recovery_tester;
};

SpinRecoveryTester * SpinRecoveryTestFixture::spin_recovery_tester = nullptr;

TEST_P(SpinRecoveryTestFixture, testSpinRecovery)
{
  float target_yaw = std::get<0>(GetParam());
  float tolerance = std::get<1>(GetParam());

  bool success = false;
  int num_tries = 3;
  for (int i = 0; i != num_tries; i++) {
    success = success || spin_recovery_tester->defaultSpinRecoveryTest(target_yaw, tolerance);
    if (success) {
      break;
    }
  }

  EXPECT_EQ(true, success);
}

INSTANTIATE_TEST_CASE_P(
  SpinRecoveryTests,
  SpinRecoveryTestFixture,
  ::testing::Values(
    std::make_tuple(-M_PIf32 / 6.0, 0.1),
    std::make_tuple(M_PI_4f32, 0.1),
    std::make_tuple(-M_PI_2f32, 0.1),
    std::make_tuple(M_PIf32, 0.1),
    std::make_tuple(3.0 * M_PIf32 / 2.0, 0.15),
    std::make_tuple(-2.0 * M_PIf32, 0.1),
    std::make_tuple(4.0 * M_PIf32, 0.15)),
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
