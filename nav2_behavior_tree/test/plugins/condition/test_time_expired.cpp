// Copyright (c) 2018 Intel Corporation
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
// limitations under the License.

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <set>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/condition/time_expired_condition.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class TimeExpiredConditionTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    config_->input_ports["seconds"] = 1.0;
    bt_node_ = std::make_shared<nav2_behavior_tree::TimeExpiredCondition>(
      "time_expired", *config_);
  }

  void TearDown()
  {
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::TimeExpiredCondition> bt_node_;
};

std::shared_ptr<nav2_behavior_tree::TimeExpiredCondition>
TimeExpiredConditionTestFixture::bt_node_ = nullptr;

TEST_F(TimeExpiredConditionTestFixture, test_behavior)
{
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  for (int i = 0; i < 20; ++i) {
    rclcpp::sleep_for(500ms);
    if (i % 2) {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
    } else {
      EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
    }
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
