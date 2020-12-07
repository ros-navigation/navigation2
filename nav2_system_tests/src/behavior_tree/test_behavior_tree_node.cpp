// Copyright (c) 2020 Samsung Research
// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2020 Vinny Ruia
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

#include <cmath>
#include <tuple>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "behavior_tree_tester.hpp"

using namespace std::chrono_literals;

using nav2_system_tests::BehaviorTreeTester;

class BehaviorTreeTestFixture
  : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    behavior_tree_tester = new BehaviorTreeTester();
    if (!behavior_tree_tester->isActive()) {
      behavior_tree_tester->activate();
    }
  }

  static void TearDownTestCase()
  {
    delete behavior_tree_tester;
    behavior_tree_tester = nullptr;
  }

protected:
  static BehaviorTreeTester * behavior_tree_tester;
};

BehaviorTreeTester * BehaviorTreeTestFixture::behavior_tree_tester = nullptr;

TEST_F(BehaviorTreeTestFixture, TestAllSuccess)
{
  struct nav2_system_tests::should_action_server_return_success_t test_case_1;
  bool success = false;
  success = behavior_tree_tester->defaultBehaviorTreeTest(
    test_case_1
  );

  EXPECT_EQ(true, success);
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
