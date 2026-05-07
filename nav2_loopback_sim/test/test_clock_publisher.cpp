// Copyright (c) 2026, Dexory (Tony Najjar)
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

#include <memory>
#include <chrono>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "nav2_loopback_sim/clock_publisher.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

using namespace std::chrono_literals;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class ClockPublisherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("clock_test_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    // Drive the lifecycle to ACTIVE so create_subscription() in each test
    // auto-activates under the nav2::Subscription wrapper.
    node_->configure();
    node_->activate();
  }

  void TearDown() override
  {
    executor_->cancel();
    node_.reset();
    executor_.reset();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some(10ms);
    }
  }

  std::unique_ptr<nav2_loopback_sim::ClockPublisher> makeClockPublisher(
    double speed_factor = 1.0)
  {
    return std::make_unique<nav2_loopback_sim::ClockPublisher>(
      node_->weak_from_this(),
      speed_factor);
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// Verify that starting the clock publisher results in /clock messages being published
TEST_F(ClockPublisherTest, PublishesClockMessages)
{
  auto clock_pub = makeClockPublisher();

  int msg_count = 0;
  auto sub = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock",
    [&](const rosgraph_msgs::msg::Clock::SharedPtr) {
      msg_count++;
    },
    rclcpp::QoS(10));

  clock_pub->start();
  spinFor(500ms);

  EXPECT_GE(msg_count, 5);
}

// Verify that successive /clock timestamps are strictly increasing
TEST_F(ClockPublisherTest, ClockAdvancesMonotonically)
{
  auto clock_pub = makeClockPublisher();

  std::vector<int64_t> timestamps;
  auto sub = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock",
    [&](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
      timestamps.push_back(rclcpp::Time(msg->clock).nanoseconds());
    },
    rclcpp::QoS(10));

  clock_pub->start();
  spinFor(500ms);

  ASSERT_GE(timestamps.size(), 10u);
  for (size_t i = 1; i < timestamps.size(); ++i) {
    EXPECT_GT(timestamps[i], timestamps[i - 1]);
  }
}

// Verify that calling stop() ceases all /clock publication
TEST_F(ClockPublisherTest, StopStopsPublishing)
{
  auto clock_pub = makeClockPublisher();

  int msg_count = 0;
  auto sub = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock",
    [&](const rosgraph_msgs::msg::Clock::SharedPtr) {
      msg_count++;
    },
    rclcpp::QoS(10));

  clock_pub->start();
  spinFor(200ms);
  EXPECT_GT(msg_count, 0);

  clock_pub->stop();
  spinFor(50ms);   // drain any in-flight callbacks
  int count_after_stop = msg_count;
  spinFor(200ms);
  EXPECT_EQ(msg_count, count_after_stop);
}

// Verify that zero and negative speed factors are silently rejected (no crash, no change)
TEST_F(ClockPublisherTest, SetSpeedFactorRejectsNonPositive)
{
  auto clock_pub = makeClockPublisher(1.0);
  clock_pub->start();

  // These should be silently rejected (no crash)
  clock_pub->setSpeedFactor(0.0);
  clock_pub->setSpeedFactor(-1.0);

  // Valid value should be accepted (no crash)
  clock_pub->setSpeedFactor(5.0);

  // Verify clock still works after rejected values
  int msg_count = 0;
  auto sub = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock",
    [&](const rosgraph_msgs::msg::Clock::SharedPtr) {
      msg_count++;
    },
    rclcpp::QoS(10));
  spinFor(200ms);
  EXPECT_GT(msg_count, 0);
}

// Verify that a 0.5x speed factor produces sim time ≈ half of wall time
TEST_F(ClockPublisherTest, SpeedFactorAffectsRate)
{
  // Slow clock at 0.5x
  auto clock_pub = makeClockPublisher(0.5);

  int64_t last_ns = 0;
  int count = 0;
  auto sub = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock",
    [&](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
      last_ns = rclcpp::Time(msg->clock).nanoseconds();
      count++;
    },
    rclcpp::QoS(10));

  auto wall_start = std::chrono::steady_clock::now();
  clock_pub->start();

  while (count < 20 &&
    std::chrono::steady_clock::now() - wall_start < 3s)
  {
    executor_->spin_some(10ms);
  }

  EXPECT_GE(count, 20);
  auto wall_elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now() - wall_start).count();
  double ratio = static_cast<double>(last_ns) / static_cast<double>(wall_elapsed_ns);
  // With 0.5x, sim time should be roughly half of wall time
  EXPECT_GT(ratio, 0.45);
  EXPECT_LT(ratio, 0.55);
}
