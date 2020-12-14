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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "dwb_critics/map_grid.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

class OpenMapGrid : public dwb_critics::MapGridCritic
{
public:
  bool aggregationTypeIsLast()
  {
    return dwb_critics::MapGridCritic::ScoreAggregationType::Last == aggregationType_;
  }

  bool aggregationTypeIsSum()
  {
    return dwb_critics::MapGridCritic::ScoreAggregationType::Sum == aggregationType_;
  }

  bool aggregationTypeIsProduct()
  {
    return dwb_critics::MapGridCritic::ScoreAggregationType::Product == aggregationType_;
  }

  double getValue(unsigned int index)
  {
    return cell_values_[index];
  }

  void reset()
  {
    dwb_critics::MapGridCritic::reset();
  }
};

TEST(MapGrid, Last)
{
  auto critic = std::make_shared<OpenMapGrid>();
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");
  node->configure();
  node->activate();
  costmap_ros->configure();

  std::string name = "test";
  std::string ns = "ns";

  nav2_util::declare_parameter_if_not_declared(
    node, ns + "." + name + ".aggregation_type",
    rclcpp::ParameterValue(std::string("last")));

  critic->initialize(node, name, ns, costmap_ros);

  EXPECT_TRUE(critic->aggregationTypeIsLast());
}

TEST(MapGrid, Sum)
{
  auto critic = std::make_shared<OpenMapGrid>();
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");
  node->configure();
  node->activate();
  costmap_ros->configure();

  std::string name = "test";
  std::string ns = "ns";

  nav2_util::declare_parameter_if_not_declared(
    node, ns + "." + name + ".aggregation_type",
    rclcpp::ParameterValue(std::string("sum")));

  critic->initialize(node, name, ns, costmap_ros);

  EXPECT_TRUE(critic->aggregationTypeIsSum());
}

TEST(MapGrid, Product)
{
  auto critic = std::make_shared<OpenMapGrid>();
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");
  node->configure();
  node->activate();
  costmap_ros->configure();

  std::string name = "test";
  std::string ns = "ns";

  nav2_util::declare_parameter_if_not_declared(
    node, ns + "." + name + ".aggregation_type",
    rclcpp::ParameterValue(std::string("product")));

  critic->initialize(node, name, ns, costmap_ros);

  EXPECT_TRUE(critic->aggregationTypeIsProduct());
}

TEST(MapGrid, InvalidValue)
{
  auto critic = std::make_shared<OpenMapGrid>();
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");
  node->configure();
  node->activate();
  costmap_ros->configure();

  std::string name = "test";
  std::string ns = "ns";

  nav2_util::declare_parameter_if_not_declared(
    node, ns + "." + name + ".aggregation_type",
    rclcpp::ParameterValue(std::string("invalid_value")));

  critic->initialize(node, name, ns, costmap_ros);

  // For an invalid type it should fall back to Last
  EXPECT_TRUE(critic->aggregationTypeIsLast());
}

TEST(MapGrid, ObstacleUnreachable)
{
  auto critic = std::make_shared<OpenMapGrid>();
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  auto node = nav2_util::LifecycleNode::make_shared("costmap_tester");
  node->configure();
  node->activate();
  costmap_ros->configure();

  std::string name = "test";
  std::string ns = "ns";

  nav2_util::declare_parameter_if_not_declared(
    node, ns + "." + name + ".aggregation_type",
    rclcpp::ParameterValue(std::string("last")));

  critic->initialize(node, name, ns, costmap_ros);
  critic->reset();

  // The value for obstacle should be the number of grid cell
  unsigned int size_x = costmap_ros->getCostmap()->getSizeInCellsX();
  unsigned int size_y = costmap_ros->getCostmap()->getSizeInCellsY();
  unsigned int obstacle_cost = size_x * size_y;
  unsigned int unreachable_cost = obstacle_cost + 1;

  std::cout << "Size (x * y) " << unreachable_cost << std::endl;
  critic->setAsObstacle(2);
  EXPECT_EQ(critic->getValue(1), unreachable_cost);
  EXPECT_EQ(critic->getValue(2), obstacle_cost);
  EXPECT_EQ(critic->getValue(3), unreachable_cost);
}

TEST(MapGrid, CriticVisualization)
{
  auto critic = std::make_shared<OpenMapGrid>();

  auto node = nav2_util::LifecycleNode::make_shared("base_obstacle_critic_tester");
  node->configure();
  node->activate();

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_global_costmap");
  costmap_ros->configure();

  std::string name = "name";
  std::string ns = "ns";

  critic->initialize(node, name, ns, costmap_ros);
  // This makes all cells "unreachable"
  critic->reset();

  sensor_msgs::msg::PointCloud pointcloud;
  critic->addCriticVisualization(pointcloud);

  int size_x = costmap_ros->getCostmap()->getSizeInCellsX();
  int size_y = costmap_ros->getCostmap()->getSizeInCellsY();
  int unreachable = size_x * size_y + 1;

  // The values in the pointcloud should be equal to the values in the costmap
  for (int y = 0; y < size_y; y++) {
    for (int x = 0; x < size_x; x++) {
      float pointValue = pointcloud.channels[0].values[y * size_y + x];
      ASSERT_EQ(static_cast<int>(pointValue), unreachable);
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
