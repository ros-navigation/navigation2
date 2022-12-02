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
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

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
  explicit TestCostmap2dPublisher(const std::string & name)
  : LifecycleNode(name)
  {
    RCLCPP_INFO(get_logger(), "Constructing");
  }

  ~TestCostmap2dPublisher() override = default;

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    std::string topic_name = "/dummy_costmap/static_layer_raw";
    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      topic_name,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&TestCostmap2dPublisher::costmapCallback, this, std::placeholders::_1),
      sub_option);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, get_node_base_interface());
    executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "dummy_costmap",
      std::string{get_namespace()},
      "dummy_costmap",
      get_parameter("use_sim_time").as_bool());
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

    costmap_ros_->configure();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Activating");
    costmap_ros_->activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    costmap_ros_->deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    executor_thread_.reset();
    costmap_thread_.reset();
    costmap_ros_->deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr costmap)
  {
    promise_.set_value(costmap);
  }

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;
  std::promise<nav2_msgs::msg::Costmap::SharedPtr> promise_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
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

  ~TestNode() override
  {
    costmap_publisher_->on_deactivate(costmap_publisher_->get_current_state());
    costmap_publisher_->on_cleanup(costmap_publisher_->get_current_state());
  }

protected:
  std::shared_ptr<TestCostmap2dPublisher> costmap_publisher_;
};

TEST_F(TestNode, costmap_pub_test)
{
  auto future = costmap_publisher_->promise_.get_future();
  auto status = future.wait_for(std::chrono::seconds(5));
  EXPECT_TRUE(status == std::future_status::ready);

  auto costmap_raw = future.get();

  // Check that the first row is free space
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(costmap_raw->data[0], nav2_costmap_2d::FREE_SPACE);
  }
}
