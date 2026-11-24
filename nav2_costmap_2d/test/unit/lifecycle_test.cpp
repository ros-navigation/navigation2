// Copyright (c) 2020 Samsung Research
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

#include <chrono>
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"


TEST(LifecylceTest, CheckInitialTfTimeout) {
  rclcpp::init(0, nullptr);

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions());
  costmap->declare_parameter("initial_transform_timeout", 0.0);
  costmap->set_parameter({"initial_transform_timeout", 0.0});

  std::thread spin_thread{[costmap]() {rclcpp::spin(costmap->get_node_base_interface());}};

  {
    const auto state_after_configure = costmap->configure();
    ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    // Without providing the transform from global to robot base the activation should fail
    // and the costmap should transition into the inactive state.
    const auto state_after_activate = costmap->activate();
    ASSERT_EQ(state_after_activate.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }

  // Set a dummy transform from global to robot base
  geometry_msgs::msg::TransformStamped transform_global_to_robot{};
  transform_global_to_robot.header.frame_id = costmap->getGlobalFrameID();
  transform_global_to_robot.child_frame_id = costmap->getBaseFrameID();
  costmap->getTfManager()->getRawBuffer()->setTransform(transform_global_to_robot, "test", true);
  // Now the costmap should successful transition into the active state
  {
    const auto state_after_activate = costmap->activate();
    ASSERT_EQ(state_after_activate.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}

TEST(LifecylceTest, RejectInvalidCostmapSize) {
  rclcpp::init(0, nullptr);

  const auto expect_failure = [](const std::vector<rclcpp::Parameter> & parameters) {
      rclcpp::NodeOptions options;
      options.parameter_overrides(parameters);
      auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
      EXPECT_EQ(
        costmap->on_configure(rclcpp_lifecycle::State()),
        nav2::CallbackReturn::FAILURE);
    };

  expect_failure({rclcpp::Parameter("width", -1)});
  expect_failure({rclcpp::Parameter("height", -1)});
  expect_failure({rclcpp::Parameter("resolution", 0.0)});
  expect_failure(
  {
    rclcpp::Parameter("width", std::numeric_limits<int>::max()),
    rclcpp::Parameter("height", std::numeric_limits<int>::max()),
    rclcpp::Parameter("resolution", 0.01),
  });

  rclcpp::shutdown();
}

TEST(LifecylceTest, ObstacleFiltersRetainBufferAfterManagerCleanup) {
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      {"plugins", std::vector<std::string>{"obstacle_layer"}},
      {"obstacle_layer.plugin", "nav2_costmap_2d::ObstacleLayer"},
      {"obstacle_layer.observation_sources", "scan cloud"},
      {"obstacle_layer.scan.data_type", "LaserScan"},
      {"obstacle_layer.cloud.data_type", "PointCloud2"},
    });
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    EXPECT_EQ(costmap->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    std::weak_ptr<nav2::TransformManager> manager = costmap->getTfManager();
    std::weak_ptr<tf2_ros::Buffer> buffer = costmap->getTfManager()->getRawBuffer();
    auto obstacle_layer = costmap->getLayeredCostmap()->getPlugins()->at(0);

    EXPECT_EQ(costmap->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_TRUE(manager.expired());
    {
      const auto retained_buffer = buffer.lock();
      ASSERT_NE(retained_buffer, nullptr);
      EXPECT_FALSE(retained_buffer->isUsingDedicatedThread());
    }

    obstacle_layer.reset();
    EXPECT_TRUE(buffer.expired());
  }
  rclcpp::shutdown();
}

TEST(LifecylceTest, InternalListenerUsesProcessTfTopics) {
  const char * argv[] = {
    "test", "--ros-args", "-r", "__ns:=/robot",
    "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"};
  rclcpp::init(8, argv);
  {
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap", "/robot", false, rclcpp::NodeOptions());
    ASSERT_EQ(costmap->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_STREQ(costmap->get_namespace(), "/robot/local_costmap");
    auto parent = std::make_shared<rclcpp::Node>("parent");
    auto broadcaster = nav2::create_transform_manager(parent, false);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "parent";
    transform.child_frame_id = "dynamic_child";
    transform.header.stamp = parent->now();
    transform.transform.translation.x = 2.0;
    auto static_transform = transform;
    static_transform.child_frame_id = "static_child";
    static_transform.transform.translation.x = 3.0;
    broadcaster->sendStaticTransform(static_transform);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool received = false;
    while (std::chrono::steady_clock::now() < deadline) {
      broadcaster->sendTransform(transform);
      const auto dynamic_result = costmap->getTfManager()->getLatestTransform(
        "parent", "dynamic_child", 0.0);
      const auto static_result = costmap->getTfManager()->getLatestTransform(
        "parent", "static_child", 0.0);
      if (dynamic_result && static_result) {
        EXPECT_DOUBLE_EQ(dynamic_result->transform.translation.x, 2.0);
        EXPECT_DOUBLE_EQ(static_result->transform.translation.x, 3.0);
        received = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(received);
    EXPECT_EQ(costmap->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }
  rclcpp::shutdown();
}
