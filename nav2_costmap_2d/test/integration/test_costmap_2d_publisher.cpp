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

class CostmapRosLifecycleNode : public nav2_util::LifecycleNode
{
public:
  explicit CostmapRosLifecycleNode(const std::string & name)
  : LifecycleNode(name),
    name_(name) {}

  ~CostmapRosLifecycleNode() override = default;

  nav2_util::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      name_,
      std::string{get_namespace()},
      name_,
      get_parameter("use_sim_time").as_bool());
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

    costmap_ros_->configure();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    costmap_ros_->activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    costmap_ros_->deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    costmap_thread_.reset();
    costmap_ros_->deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  const std::string name_;
};

class LayerSubscriber
{
public:
  explicit LayerSubscriber(const nav2_util::LifecycleNode::WeakPtr & parent)
  {
    auto node = parent.lock();

    callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    std::string topic_name = "/fake_costmap/static_layer_raw";
    layer_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
      topic_name,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&LayerSubscriber::layerCallback, this, std::placeholders::_1),
      sub_option);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, node->get_node_base_interface());
    executor_thread_ = std::make_unique<nav2_util::NodeThread>(executor_);
  }

  ~LayerSubscriber()
  {
    executor_thread_.reset();
  }

  std::promise<nav2_msgs::msg::Costmap::SharedPtr> layer_promise_;

protected:
  void layerCallback(const nav2_msgs::msg::Costmap::SharedPtr layer)
  {
    if (!callback_hit_ && (layer->data.size() == 100)) {
      layer_promise_.set_value(layer);
      callback_hit_ = true;
    }
  }

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr layer_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;
  bool callback_hit_{false};
};

class CostmapRosTestFixture : public ::testing::Test
{
public:
  CostmapRosTestFixture()
  {
    costmap_lifecycle_node_ = std::make_shared<CostmapRosLifecycleNode>("fake_costmap");
    layer_subscriber_ = std::make_shared<LayerSubscriber>(
      costmap_lifecycle_node_->shared_from_this());
    costmap_lifecycle_node_->on_configure(costmap_lifecycle_node_->get_current_state());
    costmap_lifecycle_node_->on_activate(costmap_lifecycle_node_->get_current_state());
  }

  ~CostmapRosTestFixture() override
  {
    costmap_lifecycle_node_->on_deactivate(costmap_lifecycle_node_->get_current_state());
    costmap_lifecycle_node_->on_cleanup(costmap_lifecycle_node_->get_current_state());
  }

protected:
  std::shared_ptr<CostmapRosLifecycleNode> costmap_lifecycle_node_;
  std::shared_ptr<LayerSubscriber> layer_subscriber_;
};

TEST_F(CostmapRosTestFixture, costmap_pub_test)
{
  auto future = layer_subscriber_->layer_promise_.get_future();
  auto status = future.wait_for(std::chrono::seconds(5));
  ASSERT_TRUE(status == std::future_status::ready);

  auto costmap_raw = future.get();

  // Check first 20 cells of the 10by10 map
  ASSERT_EQ(costmap_raw->data.size(), 100u);
  unsigned int i = 0;
  for (; i < 7; ++i) {
    EXPECT_EQ(costmap_raw->data.at(i), nav2_costmap_2d::FREE_SPACE);
  }
  for (; i < 10; ++i) {
    EXPECT_EQ(costmap_raw->data.at(i), nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  for (; i < 17; ++i) {
    EXPECT_EQ(costmap_raw->data.at(i), nav2_costmap_2d::FREE_SPACE);
  }
  for (; i < 20; ++i) {
    EXPECT_EQ(costmap_raw->data.at(i), nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  SUCCEED();
}
