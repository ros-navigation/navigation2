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
#include "nav2_msgs/srv/get_costs.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace std::chrono_literals;

class GetCostServiceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap");
    client_ = costmap_->create_client<nav2_msgs::srv::GetCosts>(
      "/costmap/get_cost_costmap");
    costmap_->on_configure(rclcpp_lifecycle::State());
    ASSERT_TRUE(client_->wait_for_service(10s));
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  rclcpp::Client<nav2_msgs::srv::GetCosts>::SharedPtr client_;
};

TEST_F(GetCostServiceTest, TestWithoutFootprint)
{
  auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 1.0;
  request->poses.poses.push_back(pose);
  request->use_footprint = false;

  auto result_future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      costmap_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    EXPECT_GE(response->costs[0], 0.0) << "Cost is less than 0";
    EXPECT_LE(response->costs[0], 255.0) << "Cost is greater than 255";
  } else {
    FAIL() << "Failed to call service";
  }
}

TEST_F(GetCostServiceTest, TestWithFootprint)
{
  auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 1.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0.5);
  pose.pose.orientation = tf2::toMsg(q);
  request->poses.poses.push_back(pose);
  request->use_footprint = true;

  auto result_future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(
      costmap_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    EXPECT_GE(response->costs[0], 0.0) << "Cost is less than 0";
    EXPECT_LE(response->costs[0], 255.0) << "Cost is greater than 255";
  } else {
    FAIL() << "Failed to call service";
  }
}
