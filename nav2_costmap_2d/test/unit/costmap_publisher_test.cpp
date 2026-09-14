// Copyright (c) 2026 Open Navigation LLC
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
#include <functional>
#include <memory>
#include <thread>

#include "nav2_costmap_2d/costmap_2d_publisher.hpp"

using namespace std::chrono_literals;

class CostmapPublisherTest : public ::testing::TestWithParam<bool>
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<nav2::LifecycleNode>("costmap_publisher_test");
    subscriber_node_ = std::make_shared<rclcpp::Node>("costmap_listener");
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(subscriber_node_);
    publisher_ = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
      node_, &costmap_, "map", "costmap");
    publisher_->on_activate();
    publisher_->publishCostmap();
  }

  void TearDown() override
  {
    grid_sub_.reset();
    raw_sub_.reset();
    publisher_.reset();
    executor_.reset();
    subscriber_node_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void subscribe()
  {
    if (GetParam()) {
      raw_sub_ = subscriber_node_->create_subscription<nav2_msgs::msg::Costmap>(
        "costmap_raw", rclcpp::QoS(1).reliable().transient_local(),
        [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {
          ++received_;
          value_ = msg->data.at(0);
        });
    } else {
      grid_sub_ = subscriber_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "costmap", rclcpp::QoS(1).reliable().transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          ++received_;
          value_ = msg->data.at(0);
        });
    }
  }

  bool waitFor(const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  void spinForDiscovery()
  {
    const auto deadline = std::chrono::steady_clock::now() + 300ms;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  void changeCost(unsigned char value)
  {
    costmap_.setCost(0, 0, value);
    publisher_->updateBounds(0, 1, 0, 1);
    publisher_->publishCostmap();
  }

  nav2_costmap_2d::Costmap2D costmap_{2, 2, 1.0, 0.0, 0.0, 0};
  nav2::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr subscriber_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr raw_sub_;
  size_t received_{0};
  int value_{-1};
};

TEST_P(CostmapPublisherTest, LateSubscriberReceivesChangesWithoutUpdateSubscribers)
{
  changeCost(254);
  subscribe();
  ASSERT_TRUE(waitFor([this]() {return received_ > 0;}));
  ASSERT_TRUE(waitFor([this]() {
      publisher_->publishCostmap();
      return value_ == (GetParam() ? 254 : 100);
    }));

  const auto count = received_;
  publisher_->publishCostmap();
  spinForDiscovery();
  EXPECT_EQ(received_, count);
}

TEST_P(CostmapPublisherTest, ReconnectingSubscriberReceivesCurrentSnapshot)
{
  subscribe();
  ASSERT_TRUE(waitFor([this]() {return received_ > 0;}));
  spinForDiscovery();
  publisher_->publishCostmap();
  grid_sub_.reset();
  raw_sub_.reset();
  spinForDiscovery();
  changeCost(254);
  subscribe();
  ASSERT_TRUE(waitFor([this]() {
      publisher_->publishCostmap();
      return value_ == (GetParam() ? 254 : 100);
    }));
}

INSTANTIATE_TEST_SUITE_P(FullCostmapTopics, CostmapPublisherTest, ::testing::Bool());
