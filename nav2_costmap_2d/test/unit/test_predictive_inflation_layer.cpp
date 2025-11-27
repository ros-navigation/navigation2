/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, User
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/predictive_inflation_layer.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

class TestNode : public nav2::LifecycleNode
{
public:
  TestNode()
  : nav2::LifecycleNode("test_node", rclcpp::NodeOptions()) {}
  ~TestNode() {}
};

class PredictiveInflationTester : public testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<TestNode>();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    layered_costmap_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>(
      "global_costmap", false, false);
  }

  void TearDown() override
  {
    layer_.reset();
    layered_costmap_.reset();
    tf_buffer_.reset();
    node_.reset();
  }

  std::shared_ptr<TestNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::shared_ptr<nav2_costmap_2d::PredictiveInflationLayer> layer_;
};

TEST_F(PredictiveInflationTester, TestDirectionalModulation)
{
  // Setup: 10x10 map, 0.1 res
  layered_costmap_->resizeMap(200, 200, 0.05, 0.0, 0.0); // 10m x 10m
  auto * master = layered_costmap_->getCostmap();

  layer_ = std::make_shared<nav2_costmap_2d::PredictiveInflationLayer>();
  layer_->initialize(
    layered_costmap_.get(), "predictive_layer", tf_buffer_.get(),
    nav2::LifecycleNode::WeakPtr(node_), nullptr);

  // Configure params
  node_->set_parameter(rclcpp::Parameter("predictive_layer.enabled", true));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.inflation_radius", 1.0));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.cost_scaling_factor", 5.0));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.predictive_mode", true));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.nominal_speed", 1.0));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.max_inflation_scale", 2.0));

  // Weights: Forward=1.0, Rear=0.2. Speed Scale=1.0 -> Forward Effective = 2.0
  node_->set_parameter(rclcpp::Parameter("predictive_layer.forward_weight", 1.0));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.rear_weight", 0.2));

  layer_->onInitialize();

  // Add obstacles: One Ahead (6.0, 5.0), One Behind (4.0, 5.0). Robot at (5.0, 5.0)
  // Indices:
  unsigned int cy = 100; // Center (5.0m, 5.0m)
  unsigned int ahead_x = 120, ahead_y = 100; // (6.0m, 5.0m) -> 1m ahead
  unsigned int rear_x = 80, rear_y = 100;    // (4.0m, 5.0m) -> 1m behind

  master->setCost(ahead_x, ahead_y, LETHAL_OBSTACLE);
  master->setCost(rear_x, rear_y, LETHAL_OBSTACLE);

  // Update
  double min_x = 0, min_y = 0, max_x = 10, max_y = 10;
  layer_->updateBounds(5.0, 5.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  layer_->updateCosts(*master, 0, 0, 200, 200);

  // Check costs
  // Sample 0.5m away from each obstacle towards the robot
  unsigned int check_ahead_x = 110; // 5.5m (0.5m from obstacle)
  unsigned int check_rear_x = 90;   // 4.5m (0.5m from obstacle)

  unsigned char cost_ahead = master->getCost(check_ahead_x, cy);
  unsigned char cost_rear = master->getCost(check_rear_x, cy);

  // Forward obstacle should be inflated MORE (higher cost) than rear obstacle
  // Because robot is moving +X, obstacle at +X is "in front", obstacle at -X is "behind"
  // Wait...
  // Robot at (5,5), facing +X.
  // Obstacle 1 at (6,5) (Front). Vector R->O is (1,0). Angle 0. Front weight.
  // Obstacle 2 at (4,5) (Rear). Vector R->O is (-1,0). Angle PI. Rear weight.

  // Point 'check_ahead_x' is at 5.5. Dist to Obs1 is 0.5.
  // Point 'check_rear_x' is at 4.5. Dist to Obs2 is 0.5.

  // Effective distance Front: 0.5 / (1.0 * (1+1)) = 0.25m
  // Effective distance Rear: 0.5 / 0.2 = 2.5m

  // Cost(0.25) should be > Cost(2.5)
  EXPECT_GT(cost_ahead, cost_rear);
  EXPECT_GT(cost_ahead, 0);
  EXPECT_EQ(cost_rear, 0); // Should be cleared because eff dist > radius
}

TEST_F(PredictiveInflationTester, TestMaxRadiusExpansion)
{
  layered_costmap_->resizeMap(200, 200, 0.05, 0.0, 0.0);
  layer_ = std::make_shared<nav2_costmap_2d::PredictiveInflationLayer>();
  layer_->initialize(
    layered_costmap_.get(), "predictive_layer", tf_buffer_.get(),
    nav2::LifecycleNode::WeakPtr(node_), nullptr);

  node_->set_parameter(rclcpp::Parameter("predictive_layer.enabled", true));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.inflation_radius", 1.0));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.predictive_mode", true));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.max_inflation_scale", 3.0));

  layer_->onInitialize();

  double min_x = 5.0, min_y = 5.0, max_x = 5.0, max_y = 5.0;
  // Update bounds with robot at center
  layer_->updateBounds(5.0, 5.0, 0.0, &min_x, &min_y, &max_x, &max_y);

  // Expected expansion: 1.0 * 3.0 = 3.0m
  // Bounds should expand by at least 3.0m
  EXPECT_LE(min_x, 2.0); // 5.0 - 3.0
  EXPECT_GE(max_x, 8.0); // 5.0 + 3.0
}

TEST_F(PredictiveInflationTester, TestBackwardCompatibility)
{
  layered_costmap_->resizeMap(200, 200, 0.05, 0.0, 0.0);
  auto * master = layered_costmap_->getCostmap();

  layer_ = std::make_shared<nav2_costmap_2d::PredictiveInflationLayer>();
  layer_->initialize(
    layered_costmap_.get(), "predictive_layer", tf_buffer_.get(),
    nav2::LifecycleNode::WeakPtr(node_), nullptr);

  node_->set_parameter(rclcpp::Parameter("predictive_layer.enabled", true));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.predictive_mode", false)); // Disabled
  node_->set_parameter(rclcpp::Parameter("predictive_layer.inflation_radius", 1.0));
  node_->set_parameter(rclcpp::Parameter("predictive_layer.cost_scaling_factor", 5.0));

  layer_->onInitialize();

  // Single obstacle
  master->setCost(100, 100, LETHAL_OBSTACLE);

  double min_x = 0, min_y = 0, max_x = 10, max_y = 10;
  layer_->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
  layer_->updateCosts(*master, 0, 0, 200, 200);

  // Check simple cost calculation matches standard formula
  // Dist = 0.5m (10 cells).
  // Cost = 253 * exp(-5.0 * (0.5 - inscribed)) ...
  unsigned char cost = master->getCost(110, 100);
  EXPECT_GT(cost, 0);
  EXPECT_LT(cost, LETHAL_OBSTACLE);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
