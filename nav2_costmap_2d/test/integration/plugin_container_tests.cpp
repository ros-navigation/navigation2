// Copyright (c) 2024 Polymath Robotics, Inc.
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

#include <memory>
#include <string>
#include <algorithm>
#include <utility>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "../testing_helper.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/plugin_container_layer.hpp"

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
    node_ = std::make_shared<TestLifecycleNode>("plugin_container_test_node");
    node_->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
    node_->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
    node_->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
    node_->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    node_->declare_parameter("unknown_cost_value",
      rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
    node_->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
    node_->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  }

  ~TestNode() {}

  void waitForMap(std::shared_ptr<nav2_costmap_2d::StaticLayer> & slayer)
  {
    while (!slayer->isCurrent()) {
      rclcpp::spin_some(node_->get_node_base_interface());
    }
  }

  std::vector<geometry_msgs::msg::Point> setRadii(
    nav2_costmap_2d::LayeredCostmap & layers,
    double length, double width)
  {
    std::vector<geometry_msgs::msg::Point> polygon;
    geometry_msgs::msg::Point p;
    p.x = width;
    p.y = length;
    polygon.push_back(p);
    p.x = width;
    p.y = -length;
    polygon.push_back(p);
    p.x = -width;
    p.y = -length;
    polygon.push_back(p);
    p.x = -width;
    p.y = length;
    polygon.push_back(p);
    layers.setFootprint(polygon);

    return polygon;
  }

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

/**
 * Test if combining layers different plugin container layers works
 */

TEST_F(TestNode, testObstacleLayers) {
  tf2_ros::Buffer tf(node_->get_clock());

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_a = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_a, "pclayer_a");

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_b = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_b, "pclayer_b");

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_a =
    std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_a->addPlugin(olayer_a, "pclayer_a.obstacles");

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_b =
    std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_b->addPlugin(olayer_b, "pclayer_b.obstacles");

  addObservation(olayer_a, 5.0, 5.0);
  addObservation(olayer_b, 3.0, 8.0);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 2);
}

TEST_F(TestNode, testObstacleAndStaticLayers) {
  tf2_ros::Buffer tf(node_->get_clock());

  node_->declare_parameter("pclayer_a.static.map_topic",
    rclcpp::ParameterValue(std::string("map")));

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_a = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_a, "pclayer_a");

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_b = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_b, "pclayer_b");

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer =
    std::make_shared<nav2_costmap_2d::StaticLayer>();
  pclayer_a->addPlugin(slayer, "pclayer_a.static");

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_b =
    std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_b->addPlugin(olayer_b, "pclayer_b.obstacles");

  addObservation(olayer_b, 3.0, 8.0);

  waitForMap(slayer);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 21);
}

TEST_F(TestNode, testDifferentInflationLayers) {
  tf2_ros::Buffer tf(node_->get_clock());

  node_->declare_parameter("pclayer_a.static.map_topic",
    rclcpp::ParameterValue(std::string("map")));

  node_->declare_parameter("pclayer_b.inflation.cost_scaling_factor",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_b.inflation.inflation_radius",
    rclcpp::ParameterValue(1.0));

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_a = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_a, "pclayer_a");

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_b = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_b, "pclayer_b");

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer =
    std::make_shared<nav2_costmap_2d::StaticLayer>();
  pclayer_a->addPlugin(slayer, "pclayer_a.static");

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_b =
    std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_b->addPlugin(olayer_b, "pclayer_b.obstacles");

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer_b =
    std::make_shared<nav2_costmap_2d::InflationLayer>();
  pclayer_b->addPlugin(ilayer_b, "pclayer_b.inflation");

  std::vector<geometry_msgs::msg::Point> polygon = setRadii(layers, 1, 1);
  layers.setFootprint(polygon);

  addObservation(olayer_b, 5.0, 4.0);

  waitForMap(slayer);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 21);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 4);
}

TEST_F(TestNode, testDifferentInflationLayers2) {
  tf2_ros::Buffer tf(node_->get_clock());

  node_->declare_parameter("pclayer_a.static.map_topic",
    rclcpp::ParameterValue(std::string("map")));

  node_->declare_parameter("pclayer_a.inflation.cost_scaling_factor",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_a.inflation.inflation_radius",
    rclcpp::ParameterValue(1.0));

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_a = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_a, "pclayer_a");

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_b = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_b, "pclayer_b");

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer =
    std::make_shared<nav2_costmap_2d::StaticLayer>();
  pclayer_a->addPlugin(slayer, "pclayer_a.static");

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer_a =
    std::make_shared<nav2_costmap_2d::InflationLayer>();
  pclayer_a->addPlugin(ilayer_a, "pclayer_a.inflation");

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_b =
    std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_b->addPlugin(olayer_b, "pclayer_b.obstacles");

  std::vector<geometry_msgs::msg::Point> polygon = setRadii(layers, 1, 1);
  layers.setFootprint(polygon);

  addObservation(olayer_b, 9.0, 9.0);

  waitForMap(slayer);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 21);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 28);
}

TEST_F(TestNode, testResetting) {
  tf2_ros::Buffer tf(node_->get_clock());

  node_->declare_parameter("pclayer_a.static.map_topic",
    rclcpp::ParameterValue(std::string("map")));

  node_->declare_parameter("pclayer_a.inflation.cost_scaling_factor",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_a.inflation.inflation_radius",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_b.inflation.cost_scaling_factor",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_b.inflation.inflation_radius",
    rclcpp::ParameterValue(1.0));

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_a = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_a, "pclayer_a");

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_b = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_b, "pclayer_b");

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer =
    std::make_shared<nav2_costmap_2d::StaticLayer>();
  pclayer_a->addPlugin(slayer, "pclayer_a.static");

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer_a =
    std::make_shared<nav2_costmap_2d::InflationLayer>();
  pclayer_a->addPlugin(ilayer_a, "pclayer_a.inflation");

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_b =
    std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_b->addPlugin(olayer_b, "pclayer_b.obstacles");

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer_b =
    std::make_shared<nav2_costmap_2d::InflationLayer>();
  pclayer_b->addPlugin(ilayer_b, "pclayer_a.inflation");

  std::vector<geometry_msgs::msg::Point> polygon = setRadii(layers, 1, 1);
  layers.setFootprint(polygon);

  addObservation(olayer_b, 9.0, 9.0);

  waitForMap(slayer);

  ASSERT_EQ(pclayer_a->isCurrent(), true);
  ASSERT_EQ(pclayer_b->isCurrent(), true);

  ASSERT_EQ(slayer->isCurrent(), true);
  ASSERT_EQ(ilayer_a->isCurrent(), true);
  ASSERT_EQ(olayer_b->isCurrent(), true);
  ASSERT_EQ(ilayer_b->isCurrent(), true);

  pclayer_a->reset();
  pclayer_b->reset();

  ASSERT_EQ(pclayer_a->isCurrent(), false);
  ASSERT_EQ(pclayer_b->isCurrent(), false);

  ASSERT_EQ(slayer->isCurrent(), false);
  ASSERT_EQ(ilayer_a->isCurrent(), false);
  ASSERT_EQ(olayer_b->isCurrent(), false);
  ASSERT_EQ(ilayer_b->isCurrent(), false);
}

TEST_F(TestNode, testClearing) {
  tf2_ros::Buffer tf(node_->get_clock());

  node_->declare_parameter("pclayer_a.static.map_topic",
    rclcpp::ParameterValue(std::string("map")));

  node_->declare_parameter("pclayer_a.inflation.cost_scaling_factor",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_a.inflation.inflation_radius",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_b.inflation.cost_scaling_factor",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_b.inflation.inflation_radius",
    rclcpp::ParameterValue(1.0));

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_a = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_a, "pclayer_a");

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_b = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_b, "pclayer_b");

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer =
    std::make_shared<nav2_costmap_2d::StaticLayer>();
  pclayer_a->addPlugin(slayer, "pclayer_a.static");

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer_a =
    std::make_shared<nav2_costmap_2d::InflationLayer>();
  pclayer_a->addPlugin(ilayer_a, "pclayer_a.inflation");

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_b =
    std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_b->addPlugin(olayer_b, "pclayer_b.obstacles");

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer_b =
    std::make_shared<nav2_costmap_2d::InflationLayer>();
  pclayer_b->addPlugin(ilayer_b, "pclayer_a.inflation");

  std::vector<geometry_msgs::msg::Point> polygon = setRadii(layers, 1, 1);
  layers.setFootprint(polygon);

  addObservation(olayer_b, 9.0, 9.0);

  waitForMap(slayer);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 21);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 29);
  ASSERT_EQ(olayer_b->getCost(9, 9), nav2_costmap_2d::LETHAL_OBSTACLE);

  pclayer_a->clearArea(-1, -1, 10, 10, false);
  pclayer_b->clearArea(-1, -1, 10, 10, false);

  ASSERT_EQ(slayer->getCost(9, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(olayer_b->getCost(9, 9), nav2_costmap_2d::NO_INFORMATION);
}

TEST_F(TestNode, testOverwriteCombinationMethods) {
  tf2_ros::Buffer tf(node_->get_clock());

  node_->declare_parameter("pclayer_a.static.map_topic",
    rclcpp::ParameterValue(std::string("map")));

  node_->declare_parameter("pclayer_a.inflation.cost_scaling_factor",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_a.inflation.inflation_radius",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_b.inflation.cost_scaling_factor",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_b.inflation.inflation_radius",
    rclcpp::ParameterValue(1.0));

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_a = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_a, "pclayer_a");

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_b = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_b, "pclayer_b");

  node_->set_parameter(rclcpp::Parameter("pclayer_b.combination_method", 0));

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer =
    std::make_shared<nav2_costmap_2d::StaticLayer>();
  pclayer_a->addPlugin(slayer, "pclayer_a.static");

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer_a =
    std::make_shared<nav2_costmap_2d::InflationLayer>();
  pclayer_a->addPlugin(ilayer_a, "pclayer_a.inflation");

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_b =
    std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_b->addPlugin(olayer_b, "pclayer_b.obstacles");

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer_b =
    std::make_shared<nav2_costmap_2d::InflationLayer>();
  pclayer_b->addPlugin(ilayer_b, "pclayer_a.inflation");

  std::vector<geometry_msgs::msg::Point> polygon = setRadii(layers, 1, 1);
  layers.setFootprint(polygon);

  addObservation(olayer_b, 9.0, 9.0);

  waitForMap(slayer);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::LETHAL_OBSTACLE), 1);
  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE), 2);
}

TEST_F(TestNode, testWithoutUnknownOverwriteCombinationMethods) {
  tf2_ros::Buffer tf(node_->get_clock());

  node_->declare_parameter("pclayer_a.static.map_topic",
    rclcpp::ParameterValue(std::string("map")));

  node_->declare_parameter("pclayer_a.inflation.cost_scaling_factor",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_a.inflation.inflation_radius",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_b.inflation.cost_scaling_factor",
    rclcpp::ParameterValue(1.0));

  node_->declare_parameter("pclayer_b.inflation.inflation_radius",
    rclcpp::ParameterValue(1.0));

  nav2_costmap_2d::LayeredCostmap layers("frame", false, true);

  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_a = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_a, "pclayer_a");

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_b = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_b, "pclayer_b");

  node_->set_parameter(rclcpp::Parameter("pclayer_a.combination_method", 2));
  node_->set_parameter(rclcpp::Parameter("pclayer_b.combination_method", 2));

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer =
    std::make_shared<nav2_costmap_2d::StaticLayer>();
  pclayer_a->addPlugin(slayer, "pclayer_a.static");

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer_a =
    std::make_shared<nav2_costmap_2d::InflationLayer>();
  pclayer_a->addPlugin(ilayer_a, "pclayer_a.inflation");

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_b =
    std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_b->addPlugin(olayer_b, "pclayer_b.obstacles");

  std::shared_ptr<nav2_costmap_2d::InflationLayer> ilayer_b =
    std::make_shared<nav2_costmap_2d::InflationLayer>();
  pclayer_b->addPlugin(ilayer_b, "pclayer_a.inflation");

  std::vector<geometry_msgs::msg::Point> polygon = setRadii(layers, 1, 1);
  layers.setFootprint(polygon);

  addObservation(olayer_b, 9.0, 9.0);

  waitForMap(slayer);

  layers.updateMap(0, 0, 0);

  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();

  ASSERT_EQ(countValues(*costmap, nav2_costmap_2d::NO_INFORMATION), 100);
}

TEST_F(TestNode, testClearable) {
  tf2_ros::Buffer tf(node_->get_clock());

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_a = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_a, "pclayer_a");

  std::shared_ptr<nav2_costmap_2d::PluginContainerLayer> pclayer_b = nullptr;
  addPluginContainerLayer(layers, tf, node_, pclayer_b, "pclayer_b");

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer =
    std::make_shared<nav2_costmap_2d::StaticLayer>();
  pclayer_a->addPlugin(slayer, "pclayer_a.static");

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer_a =
    std::make_shared<nav2_costmap_2d::ObstacleLayer>();
  pclayer_a->addPlugin(olayer_a, "pclayer_a.obstacles");

  std::shared_ptr<nav2_costmap_2d::StaticLayer> slayer_b =
    std::make_shared<nav2_costmap_2d::StaticLayer>();
  pclayer_b->addPlugin(slayer_b, "pclayer_b.obstacles");

  waitForMap(slayer);

  ASSERT_EQ(pclayer_a->isClearable(), true);
  ASSERT_EQ(pclayer_b->isClearable(), false);
}

TEST_F(TestNode, testDynParamsSetPluginContainerLayer)
{
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  // Add obstacle layer
  std::vector<std::string> plugins_str;
  plugins_str.push_back("plugin_container_layer_a");
  plugins_str.push_back("plugin_container_layer_b");
  costmap->set_parameter(rclcpp::Parameter("plugins", plugins_str));

  costmap->declare_parameter("plugin_container_layer_a.plugin",
    rclcpp::ParameterValue(std::string("nav2_costmap_2d::PluginContainerLayer")));
  costmap->declare_parameter("plugin_container_layer_b.plugin",
    rclcpp::ParameterValue(std::string("nav2_costmap_2d::PluginContainerLayer")));

  std::vector<std::string> plugins_str_a;
  plugins_str_a.push_back("obstacle_layer");
  plugins_str_a.push_back("inflation_layer");
  costmap->declare_parameter("plugin_container_layer_a.plugins",
    rclcpp::ParameterValue(plugins_str_a));

  costmap->declare_parameter("plugin_container_layer_a.obstacle_layer.plugin",
    rclcpp::ParameterValue(std::string("nav2_costmap_2d::ObstacleLayer")));
  costmap->declare_parameter("plugin_container_layer_a.inflation_layer.plugin",
    rclcpp::ParameterValue(std::string("nav2_costmap_2d::InflationLayer")));

  std::vector<std::string> plugins_str_b;
  plugins_str_b.push_back("static_layer");
  plugins_str_b.push_back("inflation_layer");
  costmap->declare_parameter("plugin_container_layer_b.plugins",
    rclcpp::ParameterValue(plugins_str_b));

  costmap->declare_parameter("plugin_container_layer_b.static_layer.plugin",
    rclcpp::ParameterValue(std::string("nav2_costmap_2d::StaticLayer")));
  costmap->declare_parameter("plugin_container_layer_b.inflation_layer.plugin",
    rclcpp::ParameterValue(std::string("nav2_costmap_2d::InflationLayer")));


  costmap->set_parameter(rclcpp::Parameter("global_frame", std::string("map")));
  costmap->set_parameter(rclcpp::Parameter("robot_base_frame", std::string("base_link")));

  costmap->on_configure(rclcpp_lifecycle::State());

  costmap->on_activate(rclcpp_lifecycle::State());

  auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
    costmap->get_node_base_interface(), costmap->get_node_topics_interface(),
    costmap->get_node_graph_interface(),
    costmap->get_node_services_interface());

  auto results = parameter_client->set_parameters_atomically(
  {
    rclcpp::Parameter("plugin_container_layer_a.combination_method", 2),
    rclcpp::Parameter("plugin_container_layer_b.combination_method", 1),
    rclcpp::Parameter("plugin_container_layer_a.enabled", false),
    rclcpp::Parameter("plugin_container_layer_b.enabled", true)
  });

  rclcpp::spin_until_future_complete(
    costmap->get_node_base_interface(),
    results);

  EXPECT_EQ(costmap->get_parameter("plugin_container_layer_a.combination_method").as_int(), 2);
  EXPECT_EQ(costmap->get_parameter("plugin_container_layer_b.combination_method").as_int(), 1);

  EXPECT_EQ(costmap->get_parameter("plugin_container_layer_a.enabled").as_bool(), false);
  EXPECT_EQ(costmap->get_parameter("plugin_container_layer_b.enabled").as_bool(), true);

  costmap->on_deactivate(rclcpp_lifecycle::State());
  costmap->on_cleanup(rclcpp_lifecycle::State());
  costmap->on_shutdown(rclcpp_lifecycle::State());
}
