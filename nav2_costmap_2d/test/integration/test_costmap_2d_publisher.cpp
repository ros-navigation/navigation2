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

#include <future>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestCostmap2dPublisher : public nav2_util::LifecycleNode
{
public:
  explicit TestCostmap2dPublisher(std::string name)
  : LifecycleNode(name)
  {
    RCLCPP_INFO(get_logger(), "Constructing");
  }

  ~TestCostmap2dPublisher() {}

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
      100, 100, 0.05, 0, 0,
      nav2_costmap_2d::LETHAL_OBSTACLE);

    costmap_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
      shared_from_this(),
      costmap_.get(),
      "map",
      "dummy_costmap",
      true);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    std::string topic_name = "dummy_costmap_raw";
    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      topic_name,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&TestCostmap2dPublisher::costmapCallback, this, std::placeholders::_1),
      sub_option);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, get_node_base_interface());
    executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating");
    costmap_pub_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    executor_thread_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void publishCostmap()
  {
    costmap_pub_->publishCostmap();
  }

  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr costmap)
  {
    promise_.set_value(costmap);
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;
  std::promise<nav2_msgs::msg::Costmap::SharedPtr> promise_;
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    costmap_publisher_ = std::make_shared<TestCostmap2dPublisher>("test_costmap_publisher");
    costmap_publisher_->on_configure(costmap_publisher_->get_current_state());
    costmap_publisher_->on_activate(costmap_publisher_->get_current_state());
  }

  ~TestNode()
  {
    costmap_publisher_->on_deactivate(costmap_publisher_->get_current_state());
    costmap_publisher_->on_cleanup(costmap_publisher_->get_current_state());
  }

protected:
  std::shared_ptr<TestCostmap2dPublisher> costmap_publisher_;
};

TEST_F(TestNode, costmap_pub_test)
{
  costmap_publisher_->publishCostmap();
  auto future = costmap_publisher_->promise_.get_future();
  auto status = future.wait_for(std::chrono::seconds(5));
  EXPECT_TRUE(status == std::future_status::ready);

  auto costmap_raw = future.get();
  for (const auto cost : costmap_raw->data) {
    EXPECT_TRUE(cost == nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}
