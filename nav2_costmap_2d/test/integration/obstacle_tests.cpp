/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author David Lu!!
 * Test harness for ObstacleLayer for Costmap2D
 */

#include <memory>
#include <string>
#include <algorithm>
#include <utility>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "../testing_helper.hpp"

using std::begin;
using std::end;
using std::for_each;
using std::all_of;
using std::none_of;
using std::pair;
using std::string;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  explicit TestLifecycleNode(const string & name)
  : nav2_util::LifecycleNode(name)
  {
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
};

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    node_ = std::make_shared<TestLifecycleNode>("obstacle_test_node");
    node_->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
    node_->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
    node_->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
    node_->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    node_->declare_parameter(
      "unknown_cost_value",
      rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
    node_->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
    node_->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  }

  ~TestNode() {}

protected:
  std::shared_ptr<TestLifecycleNode> node_;
};

/*
 * For reference, the static map looks like this:
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0 254 254 254   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   upper left is 0,0, lower right is 9,9
 */

#if (0)
/**
 * Test for ray tracing free space
 */
TEST_F(TestNode, testRaytracing) {
  tf2_ros::Buffer tf(node_->get_clock());

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf, node_);
  auto olayer = addObstacleLayer(layers, tf, node_);

  // Add a point at 0, 0, 0
  addObservation(olayer, 0.0, 0.0, MAX_Z / 2, 0, 0, MAX_Z / 2);

  // This actually puts the LETHAL (254) point in the costmap at (0,0)
  layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
  // printMap(*(layers.getCostmap()));

  int lethal_count = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);

  // We expect just one obstacle to be added (20 in static map)
  ASSERT_EQ(lethal_count, 21);
}

/**
 * Test for ray tracing free space
 */
TEST_F(TestNode, testRaytracing2) {
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf, node_);
  auto olayer = addObstacleLayer(layers, tf, node_);

  // If we print map now, it is 10x10 all value 0
  // printMap(*(layers.getCostmap()));

  // Update will fill in the costmap with the static map
  layers.updateMap(0, 0, 0);

  // If we print the map now, we get the static map
  // printMap(*(layers.getCostmap()));

  // Static map has 20 LETHAL cells (see diagram above)
  int obs_before = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(obs_before, 20);

  // The sensor origin will be <0,0>. So if we add an obstacle at 9,9,
  // we would expect cells <0, 0> thru <8, 8> to be traced through
  // however the static map is not cleared by obstacle layer
  addObservation(olayer, 9.5, 9.5, MAX_Z / 2, 0.5, 0.5, MAX_Z / 2);
  layers.updateMap(0, 0, 0);

  // If we print map now, we have static map + <9,9> is LETHAL
  // printMap(*(layers.getCostmap()));
  int obs_after = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Change from previous test:
  // No obstacles from the static map will be cleared, so the
  // net change is +1.
  ASSERT_EQ(obs_after, obs_before + 1);

  // Fill in the diagonal, <7,7> and <9,9> already filled in, <0,0> is robot
  for (int i = 0; i < olayer->getSizeInCellsY(); ++i) {
    olayer->setCost(i, i, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  // This will updateBounds, which will raytrace the static observation added
  // above, thus clearing out the diagonal again!
  layers.updateMap(0, 0, 0);

  // Map now has diagonal except <0,0> filled with LETHAL (254)
  // printMap(*(layers.getCostmap()));
  int with_static = countValues(*(layers.getCostmap()), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Should thus be the same
  ASSERT_EQ(with_static, obs_after);
  // If 21 are filled, 79 should be free
  ASSERT_EQ(79, countValues(*(layers.getCostmap()), nav2_costmap_2d::FREE_SPACE));
}

/**
 * Test for wave interference
 */
TEST_F(TestNode, testWaveInterference) {
  tf2_ros::Buffer tf(node_->get_clock());
  node_->set_parameter(rclcpp::Parameter("track_unknown_space", true));
  // Start with an empty map, no rolling window, tracking unknown
  nav2_costmap_2d::LayeredCostmap layers("frame", false, true);
  layers.resizeMap(10, 10, 1, 0, 0);
  auto olayer = addObstacleLayer(layers, tf, node_);

  // If we print map now, it is 10x10, all cells are 255 (NO_INFORMATION)
  // printMap(*(layers.getCostmap()));

  // Lay out 3 obstacles in a line - along the diagonal, separated by a cell.
  addObservation(olayer, 3.0, 3.0, MAX_Z);
  addObservation(olayer, 5.0, 5.0, MAX_Z);
  addObservation(olayer, 7.0, 7.0, MAX_Z);
  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  // 3 obstacle cells are filled, <1,1>,<2,2>,<4,4> and <6,6> are now free
  // <0,0> is footprint and is free
  // printMap(*costmap);
  ASSERT_EQ(3, countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE));
  ASSERT_EQ(92, countValues(*costmap, nav2_costmap_2d::NO_INFORMATION));
  ASSERT_EQ(5, countValues(*costmap, nav2_costmap_2d::FREE_SPACE));
}

/**
 * Make sure we ignore points outside of our z threshold
 */
TEST_F(TestNode, testZThreshold) {
  tf2_ros::Buffer tf(node_->get_clock());
  // Start with an empty map
  nav2_costmap_2d::LayeredCostmap layers("frame", false, true);
  layers.resizeMap(10, 10, 1, 0, 0);

  auto olayer = addObstacleLayer(layers, tf, node_);

  // A point cloud with 2 points falling in a cell with a non-lethal cost
  addObservation(olayer, 0.0, 5.0, 0.4);
  addObservation(olayer, 1.0, 5.0, 2.2);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 1);
}

/**
 * Verify that dynamic obstacles are added
 */
TEST_F(TestNode, testDynamicObstacles) {
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf, node_);

  auto olayer = addObstacleLayer(layers, tf, node_);

  // Add a point cloud and verify its insertion. There should be only one new one
  addObservation(olayer, 0.0, 0.0);
  addObservation(olayer, 0.0, 0.0);
  addObservation(olayer, 0.0, 0.0);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  // Should now have 1 insertion and no deletions
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 21);

  // Repeating the call - we should see no insertions or deletions
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 21);
}

/**
 * Verify that if we add a point that is already a static obstacle we do not end up with a new ostacle
 */
TEST_F(TestNode, testMultipleAdditions) {
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf, node_);

  auto olayer = addObstacleLayer(layers, tf, node_);

  // A point cloud with one point that falls within an existing obstacle
  addObservation(olayer, 9.5, 0.0);
  layers.updateMap(0, 0, 0);
  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  // printMap(*costmap);

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 20);
}
#endif
/**
 * Verify correct init/reset cycling of layer
 */
TEST_F(TestNode, testRepeatedResets) {
  tf2_ros::Buffer tf(node_->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer = nullptr;
  addStaticLayer(layers, tf, node_, slayer);

  // TODO(orduno) Add obstacle layer

  // Define a node-level parameter
  pair<string, string> node_dummy = {"node_dummy_param", "node_dummy_val"};
  node_->declare_parameter(node_dummy.first, rclcpp::ParameterValue(node_dummy.second));

  // Define a layer-level parameter
  pair<string, string> layer_dummy = {"dummy_param", "dummy_val"};

  // Set parameters
  auto plugins = layers.getPlugins();
  for_each(
    begin(*plugins), end(*plugins), [&layer_dummy](const auto & plugin) {
      string layer_param = layer_dummy.first + "_" + plugin->getName();

      // Notice we are using Layer::declareParameter
      plugin->declareParameter(layer_param, rclcpp::ParameterValue(layer_dummy.second));
    });

  // Check that all parameters have been set
  // node-level param
  ASSERT_TRUE(node_->has_parameter(node_dummy.first));

  // layer-level param
  ASSERT_TRUE(
    all_of(
      begin(*plugins), end(*plugins), [&layer_dummy](const auto & plugin) {
        string layer_param = layer_dummy.first + "_" + plugin->getName();
        return plugin->hasParameter(layer_param);
      }));

  // Reset all layers. Parameters should be declared if not declared, otherwise skipped.
  ASSERT_NO_THROW(
    for_each(
      begin(*plugins), end(*plugins), [](const auto & plugin) {
        plugin->reset();
      }));
}
