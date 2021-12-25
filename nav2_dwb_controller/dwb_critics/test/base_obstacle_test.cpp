/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Wilco Bonestroo
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_core/exceptions.hpp"

TEST(BaseObstacle, IsValidCost)
{
  std::shared_ptr<dwb_critics::BaseObstacleCritic> critic =
    std::make_shared<dwb_critics::BaseObstacleCritic>();

  for (int i = 0; i < 256; i++) {
    // for these 3 values the cost is not "valid"
    if (i == nav2_costmap_2d::LETHAL_OBSTACLE ||
      i == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
      i == nav2_costmap_2d::NO_INFORMATION)
    {
      ASSERT_FALSE(critic->isValidCost(i));
    } else {
      ASSERT_TRUE(critic->isValidCost(i));
    }
  }
}

TEST(BaseObstacle, ScorePose)
{
  std::shared_ptr<dwb_critics::BaseObstacleCritic> critic =
    std::make_shared<dwb_critics::BaseObstacleCritic>();

  auto node = nav2_util::LifecycleNode::make_shared("base_obstacle_critic_tester");

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  costmap_ros->configure();

  std::string name = "name";
  std::string ns = "ns";

  critic->initialize(node, name, ns, costmap_ros);

  costmap_ros->getCostmap()->setCost(0, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap_ros->getCostmap()->setCost(0, 1, nav2_costmap_2d::NO_INFORMATION);
  const int some_other_cost = 128;
  costmap_ros->getCostmap()->setCost(0, 2, some_other_cost);

  // The pose is in "world" coordinates. The (default) resolution is 0.1 m.
  geometry_msgs::msg::Pose2D pose;
  pose.x = 0;
  pose.y = 0;

  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = 0;
  pose.y = 0.15;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.y = 0.25;
  pose.x = 0.05;
  ASSERT_EQ(critic->scorePose(pose), some_other_cost);

  // The theta should not influence the cost
  for (int i = -50; i < 150; i++) {
    pose.theta = (1.0 / 50) * i * M_PI;
    ASSERT_EQ(critic->scorePose(pose), some_other_cost);
  }

  // Poses outside the map should throw an exception.
  pose.x = 1.0;
  pose.y = -0.1;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = costmap_ros->getCostmap()->getSizeInMetersX() + 0.1;
  pose.y = 1.0;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = 1.0;
  pose.y = costmap_ros->getCostmap()->getSizeInMetersY() + 0.1;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);

  pose.x = -0.1;
  pose.y = 1.0;
  ASSERT_THROW(critic->scorePose(pose), dwb_core::IllegalTrajectoryException);
}

TEST(BaseObstacle, CriticVisualization)
{
  std::shared_ptr<dwb_critics::BaseObstacleCritic> critic =
    std::make_shared<dwb_critics::BaseObstacleCritic>();

  auto node = nav2_util::LifecycleNode::make_shared("base_obstacle_critic_tester");

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  costmap_ros->configure();

  std::string name = "name";
  std::string ns = "ns";

  critic->initialize(node, name, ns, costmap_ros);

  costmap_ros->getCostmap()->setCost(0, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap_ros->getCostmap()->setCost(0, 1, nav2_costmap_2d::NO_INFORMATION);
  // Some random values
  costmap_ros->getCostmap()->setCost(3, 2, 64);
  costmap_ros->getCostmap()->setCost(30, 12, 85);
  costmap_ros->getCostmap()->setCost(10, 49, 24);
  costmap_ros->getCostmap()->setCost(45, 2, 12);

  std::vector<std::pair<std::string, std::vector<float>>> cost_channels;
  critic->addCriticVisualization(cost_channels);

  unsigned int size_x = costmap_ros->getCostmap()->getSizeInCellsX();
  unsigned int size_y = costmap_ros->getCostmap()->getSizeInCellsY();

  // The values in the pointcloud should be equal to the values in the costmap
  for (unsigned int y = 0; y < size_y; y++) {
    for (unsigned int x = 0; x < size_x; x++) {
      float pointValue = cost_channels[0].second[y * size_y + x];
      ASSERT_EQ(static_cast<int>(pointValue), costmap_ros->getCostmap()->getCost(x, y));
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
