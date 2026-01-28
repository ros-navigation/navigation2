// Copyright (c) 2024 BCK
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

#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

using namespace std::chrono_literals;

class ClearCostmapServiceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap");
    clear_entire_client_ = costmap_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/costmap/clear_entirely_costmap");
    costmap_->on_configure(rclcpp_lifecycle::State());
    ASSERT_TRUE(clear_entire_client_->wait_for_service(10s));
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  nav2::ServiceClient<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_client_;
};

TEST_F(ClearCostmapServiceTest, ClearEntireDefaultBehavior)
{
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

  auto result_future = clear_entire_client_->async_call(request);
  if (rclcpp::spin_until_future_complete(
      costmap_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    EXPECT_TRUE(response->success) << "Default clear entire should succeed";
  } else {
    FAIL() << "Failed to call ClearEntireCostmap service";
  }
}

TEST_F(ClearCostmapServiceTest, ClearEntireWithInvalidPlugin)
{
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  request->plugins.push_back("nonexistent_layer");

  auto result_future = clear_entire_client_->async_call(request);
  if (rclcpp::spin_until_future_complete(
      costmap_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    EXPECT_FALSE(response->success) << "Clear with invalid plugin should fail";
  } else {
    FAIL() << "Failed to call ClearEntireCostmap service";
  }
}

TEST_F(ClearCostmapServiceTest, ClearEntireWithMultipleInvalidPlugins)
{
  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  request->plugins.push_back("nonexistent_layer_1");
  request->plugins.push_back("nonexistent_layer_2");

  auto result_future = clear_entire_client_->async_call(request);
  if (rclcpp::spin_until_future_complete(
      costmap_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    EXPECT_FALSE(response->success) << "Clear with multiple invalid plugins should fail";
  } else {
    FAIL() << "Failed to call ClearEntireCostmap service";
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
