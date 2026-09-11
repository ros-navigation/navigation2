// Copyright (c) 2025 Open Navigation LLC
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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"


/**
 * Test for mapToWorldNoBounds
 */

TEST(mapToWorldNoBounds, MapToWorldNoBoundsNegativeMapCoords)
{
  double wx, wy;

  std::unique_ptr<nav2_costmap_2d::Costmap2D> map;

  map = std::make_unique<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 0.0, 0.0);
  map->mapToWorldNoBounds(-1, -1, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -0.5);
  EXPECT_DOUBLE_EQ(wy, -0.5);

  map = std::make_unique<nav2_costmap_2d::Costmap2D>(10, 10, 1.0, 1.0, 2.0);
  map->mapToWorldNoBounds(-5, -5, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -3.5);
  EXPECT_DOUBLE_EQ(wy, -2.5);

  map = std::make_unique<nav2_costmap_2d::Costmap2D>(10, 10, 2.0, 3.0, 4.0);
  map->mapToWorldNoBounds(-10, -10, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -16.0);
  EXPECT_DOUBLE_EQ(wy, -15.0);
}


TEST(GetRobotPose, PreservesTransformAndRejectsStalePose)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("transform_staleness_threshold", 10.0)});
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  costmap->on_configure(rclcpp_lifecycle::State());

  geometry_msgs::msg::PoseStamped pose;
  EXPECT_FALSE(costmap->getRobotPose(pose));

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = costmap->getGlobalFrameID();
  transform.child_frame_id = costmap->getBaseFrameID();
  transform.header.stamp = costmap->now() - rclcpp::Duration::from_seconds(30.0);
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(costmap->getTfBuffer()->setTransform(transform, "test"));
  EXPECT_FALSE(costmap->getRobotPose(pose));

  transform.header.stamp = costmap->now();
  ASSERT_TRUE(costmap->getTfBuffer()->setTransform(transform, "test"));
  ASSERT_TRUE(costmap->getRobotPose(pose));
  EXPECT_EQ(pose.header, transform.header);
  EXPECT_DOUBLE_EQ(pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.pose.position.y, 2.0);
  EXPECT_EQ(pose.pose.orientation, transform.transform.rotation);

  costmap->on_cleanup(rclcpp_lifecycle::State());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
