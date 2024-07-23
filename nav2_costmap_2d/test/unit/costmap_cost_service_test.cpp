// Copyright (c) 2024 Jatin Patil
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

#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/srv/get_cost.hpp"

using namespace std::chrono_literals;

class GetCostServiceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("get_cost_service_test");
    local_cost_client_ = node_->create_client<nav2_msgs::srv::GetCost>(
      "/local_costmap/get_cost_local_costmap");
    global_cost_client_ = node_->create_client<nav2_msgs::srv::GetCost>(
      "/global_costmap/get_cost_global_costmap");

    // Wait for the service to be available
    ASSERT_TRUE(local_cost_client_->wait_for_service(10s));
    ASSERT_TRUE(global_cost_client_->wait_for_service(10s));
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::GetCost>::SharedPtr local_cost_client_;
  rclcpp::Client<nav2_msgs::srv::GetCost>::SharedPtr global_cost_client_;
};

TEST_F(GetCostServiceTest, TestGlobalWithoutFootprint)
{
  auto request = std::make_shared<nav2_msgs::srv::GetCost::Request>();
  request->x = 0.0;
  request->y = 0.0;
  request->use_footprint = false;

  auto result_future = global_cost_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      node_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    SUCCEED();
  } else {
    FAIL() << "Failed to call service";
  }
}

TEST_F(GetCostServiceTest, TestGlobalWithFootprint)
{
  auto request = std::make_shared<nav2_msgs::srv::GetCost::Request>();
  request->x = 1.0;
  request->y = 1.0;
  request->use_footprint = true;

  auto result_future = global_cost_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      node_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    SUCCEED();
  } else {
    FAIL() << "Failed to call service";
  }
}

TEST_F(GetCostServiceTest, TestLocalWithoutFootprint)
{
  auto request = std::make_shared<nav2_msgs::srv::GetCost::Request>();
  request->x = 0.0;
  request->y = 0.0;
  request->use_footprint = false;

  auto result_future = local_cost_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      node_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    SUCCEED();
  } else {
    FAIL() << "Failed to call service";
  }
}

TEST_F(GetCostServiceTest, TestLocalWithFootprint)
{
  auto request = std::make_shared<nav2_msgs::srv::GetCost::Request>();
  request->x = 1.0;
  request->y = 1.0;
  request->use_footprint = true;

  auto result_future = local_cost_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      node_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    SUCCEED();
  } else {
    FAIL() << "Failed to call service";
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
