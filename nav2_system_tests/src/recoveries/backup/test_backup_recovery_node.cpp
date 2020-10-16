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

#include "backup_recovery_tester.hpp"

using namespace std::chrono_literals;

using nav2_system_tests::BackupRecoveryTester;

std::string testNameGenerator(const testing::TestParamInfo<std::tuple<float, float>> & param)
{
  std::string name = std::to_string(std::abs(std::get<0>(param.param))) + "_" + std::to_string(
    std::get<1>(param.param));
  name.erase(std::remove(name.begin(), name.end(), '.'), name.end());
  return name;
}

class BackupRecoveryTestFixture
  : public ::testing::TestWithParam<std::tuple<float, float>>
{
public:
  static void SetUpTestCase()
  {
    backup_recovery_tester = new BackupRecoveryTester();
    if (!backup_recovery_tester->isActive()) {
      backup_recovery_tester->activate();
    }
  }

  static void TearDownTestCase()
  {
    delete backup_recovery_tester;
    backup_recovery_tester = nullptr;
  }

protected:
  static BackupRecoveryTester * backup_recovery_tester;
};

BackupRecoveryTester * BackupRecoveryTestFixture::backup_recovery_tester = nullptr;

TEST_P(BackupRecoveryTestFixture, testBackupRecovery)
{
  float target_dist = std::get<0>(GetParam());
  float tolerance = std::get<1>(GetParam());

  if (!backup_recovery_tester->isActive()) {
    backup_recovery_tester->activate();
  }

  bool success = false;
  success = backup_recovery_tester->defaultBackupRecoveryTest(target_dist, tolerance);

  // if intentionally backing into an obstacle, should fail.
  // TODO(stevemacenski): uncomment this once note below completed.
  // if (target_dist < -1.0) {
  //   success = !success;
  // }

  EXPECT_EQ(true, success);
}

// TODO(stevemacenski): See issue #1779, while the 3rd test collides,
// it returns success due to technical debt in the BT. This test will
// remain as a reminder to update this to a `false` case once the
// recovery server returns true values.

INSTANTIATE_TEST_CASE_P(
  BackupRecoveryTests,
  BackupRecoveryTestFixture,
  ::testing::Values(
    std::make_tuple(-0.1, 0.1),
    std::make_tuple(-0.2, 0.1),
    std::make_tuple(-2.0, 0.1)),
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
