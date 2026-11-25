// Copyright (c) 2024 Angsa Robotics
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

#include "rcl/time.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "nav2_util/odometry_utils.hpp"

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/action/check_stop_status_action.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

class TestOdomSmoother : public nav2_util::OdomSmoother
{
public:
  using OdomSmoother::OdomSmoother;

  void inject(const nav_msgs::msg::Odometry & msg)
  {
    odomCallback(std::make_shared<nav_msgs::msg::Odometry>(msg));
  }
};

class CheckStopStatusTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    auto clock_handle = node_->get_clock()->get_clock_handle();
    auto ret = rcl_enable_ros_time_override(clock_handle);
    (void) ret;

    odom_smoother_ = std::make_shared<TestOdomSmoother>(node_);
    config_->blackboard->set(
      "odom_smoother",
      std::static_pointer_cast<nav2_util::OdomSmoother>(odom_smoother_));  // NOLINT
    // shorten duration_stopped  from default to make the test faster
    std::chrono::milliseconds duration = 100ms;
    config_->input_ports["duration_stopped"] = std::to_string(duration.count()) + "ms";
    bt_node_ = std::make_shared<nav2_behavior_tree::CheckStopStatus>("check_stop_status", *config_);
  }

  void TearDown()
  {
    bt_node_.reset();
    odom_smoother_.reset();
  }

  void setClock(const rclcpp::Time & time)
  {
    auto clock_handle = node_->get_clock()->get_clock_handle();
    auto ret = rcl_set_ros_time_override(clock_handle, time.nanoseconds());
    (void) ret;
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::CheckStopStatus> bt_node_;
  static std::shared_ptr<TestOdomSmoother> odom_smoother_;
};

std::shared_ptr<nav2_behavior_tree::CheckStopStatus>
CheckStopStatusTestFixture::bt_node_ = nullptr;
std::shared_ptr<TestOdomSmoother>
CheckStopStatusTestFixture::odom_smoother_ = nullptr;


TEST_F(CheckStopStatusTestFixture, test_behavior)
{
  auto clock = node_->get_clock();

  // Start at t = 10s (arbitrary, avoids zero-timestamp edge cases)
  setClock(rclcpp::Time(10, 0, RCL_ROS_TIME));

  nav_msgs::msg::Odometry odom_msg;

  // Test FAILURE when robot is moving
  odom_msg.header.stamp = clock->now() - rclcpp::Duration::from_seconds(0.5);
  odom_msg.twist.twist.linear.x = 1.0;
  odom_smoother_->inject(odom_msg);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // Test RUNNING when robot is stopped but not long enough (stopped for 0.05s, need 0.1s)
  odom_msg.header.stamp = clock->now() - rclcpp::Duration::from_seconds(0.05);
  odom_msg.twist.twist.linear.x = 0.001;
  odom_smoother_->inject(odom_msg);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  // Advance the fake clock by 0.2s - now stopped for 0.25s, well past the 0.1s threshold
  setClock(rclcpp::Time(10, 0, RCL_ROS_TIME) + rclcpp::Duration::from_seconds(0.2));
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Test FAILURE immediately after robot starts moving
  odom_msg.header.stamp = clock->now();
  odom_msg.twist.twist.angular.z = 0.1;
  odom_smoother_->inject(odom_msg);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  // Test SUCCESS when robot is stopped for long with a back-dated timestamp
  odom_smoother_ = std::make_shared<TestOdomSmoother>(node_);
  config_->blackboard->set(
    "odom_smoother",
    std::static_pointer_cast<nav2_util::OdomSmoother>(odom_smoother_));
  bt_node_ = std::make_shared<nav2_behavior_tree::CheckStopStatus>("check_stop_status", *config_);

  odom_msg.header.stamp = clock->now() - rclcpp::Duration(2, 0);
  odom_msg.twist.twist.angular.z = 0;
  odom_msg.twist.twist.linear.x = 0.001;
  odom_smoother_->inject(odom_msg);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  // Test SUCCESS when robot is stopped for long enough without a stamp for odometry
  odom_smoother_ = std::make_shared<TestOdomSmoother>(node_);
  config_->blackboard->set(
    "odom_smoother",
    std::static_pointer_cast<nav2_util::OdomSmoother>(odom_smoother_));
  bt_node_ = std::make_shared<nav2_behavior_tree::CheckStopStatus>("check_stop_status", *config_);

  setClock(rclcpp::Time(20, 0, RCL_ROS_TIME));
  odom_msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  odom_msg.twist.twist.linear.x = 0.001;
  odom_smoother_->inject(odom_msg);
  // In the first tick, the timestamp is set to clock->now() (t=20s)
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  // Advance clock by 0.2s - stopped for 0.2s, past the 0.1s threshold
  setClock(rclcpp::Time(20, 0, RCL_ROS_TIME) + rclcpp::Duration::from_seconds(0.2));
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
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
