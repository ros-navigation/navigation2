// Copyright (c) 2021, Samsung Research America
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

#include <math.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

TEST(NavfnTest, testDynamicParameter)
{
  auto node = std::make_shared<nav2::LifecycleNode>("Navfntest");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());
  auto planner =
    std::make_unique<nav2_navfn_planner::NavfnPlanner>();
  auto tf = nav2::create_transform_buffer(node);
  planner->configure(node, "test", tf, costmap);
  planner->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.tolerance", 1.0),
      rclcpp::Parameter("test.use_astar", true),
      rclcpp::Parameter("test.max_cycles_factor", 6),
      rclcpp::Parameter("test.allow_unknown", true),
      rclcpp::Parameter("test.use_final_approach_orientation", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.tolerance").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.use_astar").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.max_cycles_factor").as_int(), 6);
  EXPECT_EQ(node->get_parameter("test.allow_unknown").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.use_final_approach_orientation").as_bool(), true);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.tolerance", -1.0)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  // Invalid value should not be set
  EXPECT_EQ(node->get_parameter("test.tolerance").as_double(), 1.0);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.max_cycles_factor", 0)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  // Invalid value should not be set
  EXPECT_EQ(node->get_parameter("test.max_cycles_factor").as_int(), 6);
}

// AI-generated: exercise configuration against concurrent resizing under ThreadSanitizer.
TEST(NavfnTest, testConfigureWhileCostmapResizes)
{
  auto node = std::make_shared<nav2::LifecycleNode>("NavfnConfigureTest");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  auto tf = nav2::create_transform_buffer(node);
  nav2_navfn_planner::NavfnPlanner planner;

  std::atomic<bool> running{true};
  std::atomic<unsigned int> resize_count{0};
  std::thread resize_thread([&]() {
      unsigned int size = 16;
      while (running.load(std::memory_order_relaxed)) {
        {
          std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
          costmap->resizeMap(size, size + 8, 0.1, 0.0, 0.0);
        }
        size = size == 16 ? 17 : 16;
        resize_count.fetch_add(1, std::memory_order_relaxed);
        std::this_thread::yield();
      }
    });

  while (resize_count.load(std::memory_order_relaxed) == 0) {
    std::this_thread::yield();
  }
  for (int i = 0; i < 100; ++i) {
    EXPECT_NO_THROW(planner.configure(node, "test", tf, costmap_ros));
    planner.cleanup();
  }

  running.store(false, std::memory_order_relaxed);
  resize_thread.join();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
