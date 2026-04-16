// Copyright (c) 2026 Dexory
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
#include <thread>

#include "rcl/time.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/utils/loop_rate.hpp"

using namespace std::chrono_literals;  // NOLINT

// Minimal action node so we can build a valid BT::Tree
class DummyAction : public BT::SyncActionNode
{
public:
  DummyAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {return BT::NodeStatus::SUCCESS;}

  static BT::PortsList providedPorts() {return {};}
};

class LoopRateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    factory_.registerNodeType<DummyAction>("DummyAction");
    std::string xml =
      R"(<root BTCPP_format="4">
           <BehaviorTree ID="MainTree">
             <DummyAction/>
           </BehaviorTree>
         </root>)";
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(xml));
  }

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
};

// Verify that sleep() returns true when it sleeps for the full period
TEST_F(LoopRateTest, test_sleep_returns_true)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  nav2_behavior_tree::LoopRate rate(10ms, tree_.get(), clock);
  bool result = rate.sleep();
  EXPECT_TRUE(result);
}

// Verify that sleep() returns false when the cycle was missed (overrun)
TEST_F(LoopRateTest, test_sleep_returns_false_when_overrun)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  nav2_behavior_tree::LoopRate rate(10ms, tree_.get(), clock);

  rate.sleep();

  // Simulate overrun: wall-time exceeds period so next_interval is in the past
  std::this_thread::sleep_for(20ms);

  bool result = rate.sleep();
  EXPECT_FALSE(result);
}

// Verify that sleep() does not overshoot the target interval with a steady clock
TEST_F(LoopRateTest, test_does_not_overshoot_with_steady_clock)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = 10ms;
  nav2_behavior_tree::LoopRate rate(period, tree_.get(), clock);

  auto before = std::chrono::steady_clock::now();
  rate.sleep();
  auto elapsed = std::chrono::steady_clock::now() - before;

  // Should sleep for approximately the period.
  // Allow up to 3ms of jitter (scheduling, context switches) but no large overshoot.
  EXPECT_GE(elapsed, period - 1ms);
  EXPECT_LE(elapsed, period + 3ms);
}

// Verify that emitWakeUpSignal() preempts an in-progress sleep() early
TEST_F(LoopRateTest, test_wake_up_signal_preempts_sleep)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = 100ms;
  nav2_behavior_tree::LoopRate rate(period, tree_.get(), clock);

  // Fire wake-up signal after 10ms
  std::thread waker([this]() {
      std::this_thread::sleep_for(10ms);
      tree_->rootNode()->emitWakeUpSignal();
    });

  auto before = std::chrono::steady_clock::now();
  bool result = rate.sleep();
  auto elapsed = std::chrono::steady_clock::now() - before;

  waker.join();

  // Should have returned early (much less than 100ms)
  EXPECT_TRUE(result);
  EXPECT_LE(elapsed, 50ms);
}

// Verify that multiple consecutive sleeps accumulate to N * period (steady cadence)
TEST_F(LoopRateTest, test_consecutive_sleeps_maintain_cadence)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  const auto period = 10ms;
  nav2_behavior_tree::LoopRate rate(period, tree_.get(), clock);

  constexpr int iterations = 5;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < iterations; ++i) {
    rate.sleep();
  }
  auto total = std::chrono::steady_clock::now() - start;

  auto expected = period * iterations;
  // Total time should be close to N * period
  EXPECT_GE(total, expected - 5ms);
  EXPECT_LE(total, expected + 10ms);
}

// Verify that RCL_ROS_TIME without sim time override behaves like wall clock
TEST_F(LoopRateTest, test_ros_time_without_sim_time)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  const auto period = 10ms;
  nav2_behavior_tree::LoopRate rate(period, tree_.get(), clock);

  auto before = std::chrono::steady_clock::now();
  rate.sleep();
  auto elapsed = std::chrono::steady_clock::now() - before;

  // Should behave like wall clock: sleep for approximately the period
  EXPECT_GE(elapsed, period - 1ms);
  EXPECT_LE(elapsed, period + 3ms);
}

// Verify that sleep() wakes up when sim time is advanced past the period
TEST_F(LoopRateTest, test_ros_time_with_sim_time)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto * rcl_clock = clock->get_clock_handle();
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(rcl_clock));

  // Set initial sim time to 1 second
  const rcl_time_point_value_t t0 = RCL_S_TO_NS(1);
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(rcl_clock, t0));

  const auto period = 50ms;
  nav2_behavior_tree::LoopRate rate(period, tree_.get(), clock);

  // Advance sim time past the period from a separate thread
  std::thread advancer([rcl_clock, t0]() {
      std::this_thread::sleep_for(10ms);
      auto ret = rcl_set_ros_time_override(rcl_clock, t0 + RCL_MS_TO_NS(60));
      EXPECT_EQ(RCL_RET_OK, ret);
    });

  auto before = std::chrono::steady_clock::now();
  bool result = rate.sleep();
  auto elapsed = std::chrono::steady_clock::now() - before;

  advancer.join();

  // Should wake up shortly after sim time advances (within poll_interval + jitter)
  EXPECT_TRUE(result);
  EXPECT_LE(elapsed, 30ms);
}

// Verify that when sim time jumps ahead, wall-time elapsed is much less than the period
TEST_F(LoopRateTest, test_ros_time_sim_time_does_not_overshoot)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto * rcl_clock = clock->get_clock_handle();
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(rcl_clock));

  const rcl_time_point_value_t t0 = RCL_S_TO_NS(1);
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(rcl_clock, t0));

  const auto period = 100ms;
  nav2_behavior_tree::LoopRate rate(period, tree_.get(), clock);

  // Advance sim time by exactly the period after a short wall delay
  std::thread advancer([rcl_clock, t0]() {
      std::this_thread::sleep_for(5ms);
      auto ret = rcl_set_ros_time_override(rcl_clock, t0 + RCL_MS_TO_NS(100));
      EXPECT_EQ(RCL_RET_OK, ret);
    });

  auto before = std::chrono::steady_clock::now();
  rate.sleep();
  auto elapsed = std::chrono::steady_clock::now() - before;

  advancer.join();

  // Wall time should be much less than 100ms since sim time jumped ahead
  EXPECT_LE(elapsed, 30ms);
}

// Verify that sleep() does not return early when sim time has not yet reached the period
TEST_F(LoopRateTest, test_ros_time_sim_time_respects_period)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto * rcl_clock = clock->get_clock_handle();
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(rcl_clock));

  const rcl_time_point_value_t t0 = RCL_S_TO_NS(1);
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(rcl_clock, t0));

  const auto period = 100ms;
  nav2_behavior_tree::LoopRate rate(period, tree_.get(), clock);

  std::atomic<bool> sleep_done{false};

  // Run sleep() in a background thread so we can observe it blocking
  std::thread sleeper([&]() {
      rate.sleep();
      sleep_done = true;
    });

  // Advance sim time to half the period — sleep should still be blocking
  std::this_thread::sleep_for(10ms);
  auto ret = rcl_set_ros_time_override(rcl_clock, t0 + RCL_MS_TO_NS(50));
  EXPECT_EQ(RCL_RET_OK, ret);
  std::this_thread::sleep_for(5ms);
  EXPECT_FALSE(sleep_done.load()) << "sleep() returned before sim time reached the period";

  // Now advance sim time past the period — sleep should complete
  ret = rcl_set_ros_time_override(rcl_clock, t0 + RCL_MS_TO_NS(110));
  EXPECT_EQ(RCL_RET_OK, ret);
  sleeper.join();
  EXPECT_TRUE(sleep_done.load());
}

// Verify that consecutive sleeps in sim time accumulate to N * period in sim time
TEST_F(LoopRateTest, test_consecutive_sleeps_maintain_cadence_sim_time)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto * rcl_clock = clock->get_clock_handle();
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(rcl_clock));

  const rcl_time_point_value_t t0 = RCL_S_TO_NS(1);
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(rcl_clock, t0));

  const auto period = 50ms;
  constexpr int iterations = 5;
  nav2_behavior_tree::LoopRate rate(period, tree_.get(), clock);

  // Advance sim time in a background thread at 10x real time:
  // each iteration advances 50ms sim time every 5ms wall time
  std::thread advancer([rcl_clock, t0, iterations]() {
      for (int i = 1; i <= iterations; ++i) {
        std::this_thread::sleep_for(5ms);
        auto ret = rcl_set_ros_time_override(
          rcl_clock, t0 + RCL_MS_TO_NS(50 * i));
        EXPECT_EQ(RCL_RET_OK, ret);
      }
    });

  auto wall_start = std::chrono::steady_clock::now();
  for (int i = 0; i < iterations; ++i) {
    rate.sleep();
  }
  auto wall_elapsed = std::chrono::steady_clock::now() - wall_start;

  advancer.join();

  // Sim time should have advanced by N * period = 250ms
  auto sim_elapsed = clock->now() - rclcpp::Time(t0, RCL_ROS_TIME);
  EXPECT_GE(sim_elapsed.nanoseconds(), RCL_MS_TO_NS(250));

  // Wall time should be much less than 250ms (running at ~10x)
  EXPECT_LE(wall_elapsed, 100ms);
}
