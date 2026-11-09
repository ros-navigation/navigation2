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

#include "rclcpp/executors.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/utilities.hpp"
#include "nav2_msgs/srv/get_costs.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

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
  nav2::ServiceClient<nav2_msgs::srv::GetCosts>::SharedPtr client_;
};

TEST_F(GetCostServiceTest, TestWithoutFootprint)
{
  auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = costmap_->getGlobalFrameID();
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 1.0;
  request->poses.push_back(pose);
  request->use_footprint = false;

  unsigned int mx, my;
  ASSERT_TRUE(costmap_->getCostmap()->worldToMap(0.5, 1.0, mx, my));
  costmap_->getCostmap()->setCost(mx, my, 100);

  auto result_future = client_->async_call(request);
  if (rclcpp::spin_until_future_complete(
      costmap_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    EXPECT_TRUE(response->success);
    ASSERT_EQ(response->costs.size(), 1u);
    EXPECT_FLOAT_EQ(response->costs[0], 100.0f);
  } else {
    FAIL() << "Failed to call service";
  }
}

TEST_F(GetCostServiceTest, TestWithFootprint)
{
  auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = costmap_->getGlobalFrameID();
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 1.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0.5);
  pose.pose.orientation = tf2::toMsg(q);
  request->poses.push_back(pose);
  // Footprint partly off the map
  pose.pose.position.x = 0.0;
  request->poses.push_back(pose);
  request->use_footprint = true;

  auto result_future = client_->async_call(request);
  if (rclcpp::spin_until_future_complete(
      costmap_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    EXPECT_TRUE(response->success);
    ASSERT_EQ(response->costs.size(), 2u);
    EXPECT_FLOAT_EQ(response->costs[0], static_cast<float>(nav2_costmap_2d::FREE_SPACE));
    EXPECT_FLOAT_EQ(response->costs[1], static_cast<float>(nav2_costmap_2d::LETHAL_OBSTACLE));
  } else {
    FAIL() << "Failed to call service";
  }
}

TEST_F(GetCostServiceTest, TestUnknownFrame)
{
  auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "unknown_frame";
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 1.0;
  request->poses.push_back(pose);
  request->use_footprint = false;

  auto result_future = client_->async_call(request);
  if (rclcpp::spin_until_future_complete(
      costmap_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    EXPECT_FALSE(response->success);
    ASSERT_EQ(response->costs.size(), 1u);
    EXPECT_FLOAT_EQ(response->costs[0], static_cast<float>(nav2_costmap_2d::NO_INFORMATION));
  } else {
    FAIL() << "Failed to call service";
  }
}

TEST_F(GetCostServiceTest, TestFootprintFillCatchesInteriorObstacle)
{
  // Square footprint
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = -0.5;
  footprint[0].y = -0.5;
  footprint[1].x = 0.5;
  footprint[1].y = -0.5;
  footprint[2].x = 0.5;
  footprint[2].y = 0.5;
  footprint[3].x = -0.5;
  footprint[3].y = 0.5;
  costmap_->setRobotFootprint(footprint);

  unsigned int mx, my;
  ASSERT_TRUE(costmap_->getCostmap()->worldToMap(2.5, 2.5, mx, my));
  costmap_->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = costmap_->getGlobalFrameID();
  pose.pose.position.x = 2.5;
  pose.pose.position.y = 2.5;
  pose.pose.orientation.w = 1.0;

  const auto get_cost = [this, &pose](const bool footprint_fill) {
      auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
      request->poses.push_back(pose);
      request->use_footprint = true;
      request->footprint_fill = footprint_fill;

      auto result_future = client_->async_call(request);
      EXPECT_EQ(
        rclcpp::spin_until_future_complete(costmap_, result_future),
        rclcpp::FutureReturnCode::SUCCESS);
      auto response = result_future.get();
      EXPECT_TRUE(response->success);
      EXPECT_EQ(response->costs.size(), 1u);
      return response->costs.at(0);
    };

  EXPECT_FLOAT_EQ(get_cost(false), static_cast<float>(nav2_costmap_2d::FREE_SPACE));
  EXPECT_FLOAT_EQ(get_cost(true), static_cast<float>(nav2_costmap_2d::LETHAL_OBSTACLE));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
