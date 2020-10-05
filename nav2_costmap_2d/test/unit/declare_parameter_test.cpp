// Copyright (c) 2020 Samsung Research Russia
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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class LayerWrapper : public nav2_costmap_2d::Layer
{
  void reset() {}
  void updateBounds(double, double, double, double *, double *, double *, double *) {}
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) {}
};

TEST(DeclareParameter, useValidParameter2Args)
{
  LayerWrapper layer;
  nav2_util::LifecycleNode::SharedPtr node =
    std::make_shared<nav2_util::LifecycleNode>("test_node");
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layer.initialize(&layers, "test_layer", &tf, node, nullptr, nullptr);

  layer.declareParameter("test1", rclcpp::ParameterValue("test_val1"));
  try {
    std::string val = node->get_parameter("test_layer.test1").as_string();
    EXPECT_EQ(val, "test_val1");
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    FAIL() << "test_layer.test1 parameter is not set";
  }
}

TEST(DeclareParameter, useValidParameter1Arg)
{
  LayerWrapper layer;
  nav2_util::LifecycleNode::SharedPtr node =
    std::make_shared<nav2_util::LifecycleNode>("test_node");
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layer.initialize(&layers, "test_layer", &tf, node, nullptr, nullptr);

  layer.declareParameter("test2");
  try {
    node->set_parameter(rclcpp::Parameter("test_layer.test2", "test_val2"));
    std::string val = node->get_parameter("test_layer.test2").as_string();
    EXPECT_EQ(val, "test_val2");
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    FAIL() << "test_layer.test2 parameter is not set";
  }
}

TEST(DeclareParameter, useInvalidParameter1Arg)
{
  LayerWrapper layer;
  nav2_util::LifecycleNode::SharedPtr node =
    std::make_shared<nav2_util::LifecycleNode>("test_node");
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);

  layer.initialize(&layers, "test_layer", &tf, node, nullptr, nullptr);

  layer.declareParameter("test3");
  try {
    std::string val = node->get_parameter("test_layer.test3").as_string();
    FAIL() << "Incorrectly handling test_layer.test3 parameters which was not set";
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    SUCCEED();
  }
}
