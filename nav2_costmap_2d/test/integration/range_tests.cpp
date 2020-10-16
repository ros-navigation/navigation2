/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Bytes Robotics
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

#include <memory>
#include <string>
#include <algorithm>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "../testing_helper.hpp"
#include "sensor_msgs/msg/range.hpp"

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
  RclCppFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~RclCppFixture()
  {
    rclcpp::shutdown();
  }
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
  : node_(std::make_shared<TestLifecycleNode>("range_test_node")),
    tf_(node_->get_clock())
  {
    // Standard non-plugin specific parameters
    node_->declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
    node_->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
    node_->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
    node_->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    node_->declare_parameter(
      "unknown_cost_value",
      rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
    node_->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
    node_->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
    node_->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("range")));
    node_->declare_parameter("global_frame", rclcpp::ParameterValue(std::string("map")));


    // Range sensor specific parameters
    node_->declare_parameter(
      "range.topics",
      rclcpp::ParameterValue(
        std::vector<std::string>{"/range/topic"}));
    node_->declare_parameter("range.phi", rclcpp::ParameterValue(1.2));
    node_->declare_parameter("range.clear_on_max_reading", rclcpp::ParameterValue(true));
  }

  ~TestNode() {}

protected:
  std::shared_ptr<TestLifecycleNode> node_;
  tf2_ros::Buffer tf_;
};

// Test clearing at max range
TEST_F(TestNode, testClearingAtMaxRange) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = "frame";
  transform.child_frame_id = "base_link";
  transform.transform.translation.y = 5;
  transform.transform.translation.x = 2;
  tf_.setTransform(transform, "default_authority", true);

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::RangeSensorLayer> rlayer{nullptr};
  addRangeLayer(layers, tf_, node_, rlayer);

  sensor_msgs::msg::Range msg;
  msg.min_range = 1.0;
  msg.max_range = 7.0;
  msg.range = 2.0;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "base_link";
  msg.radiation_type = msg.ULTRASOUND;
  msg.field_of_view = 0.174533;  // 10 deg
  rlayer->bufferIncomingRangeMsg(std::make_shared<sensor_msgs::msg::Range>(msg));

  layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
//  printMap(*(layers.getCostmap()));

  ASSERT_EQ(layers.getCostmap()->getCost(4, 5), 254);

  msg.range = 7.0;
  msg.header.stamp = node_->now();
  rlayer->bufferIncomingRangeMsg(std::make_shared<sensor_msgs::msg::Range>(msg));
  layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
//  printMap(*(layers.getCostmap()));

  ASSERT_EQ(layers.getCostmap()->getCost(4, 5), 0);
}

// Testing fixed scan with robot forward motion
TEST_F(TestNode, testProbabalisticModelForward) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = "frame";
  transform.child_frame_id = "base_link";
  transform.transform.translation.y = 5;
  transform.transform.translation.x = 2;
  tf_.setTransform(transform, "default_authority", true);

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::RangeSensorLayer> rlayer{nullptr};
  addRangeLayer(layers, tf_, node_, rlayer);

  sensor_msgs::msg::Range msg;
  msg.min_range = 1.0;
  msg.max_range = 10.0;
  msg.range = 3.0;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "base_link";
  msg.radiation_type = msg.ULTRASOUND;
  msg.field_of_view = 0.174533;  // 10 deg
  rlayer->bufferIncomingRangeMsg(std::make_shared<sensor_msgs::msg::Range>(msg));

  layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
//  printMap(*(layers.getCostmap()));

  rlayer->bufferIncomingRangeMsg(std::make_shared<sensor_msgs::msg::Range>(msg));
  transform.transform.translation.y = 5;
  transform.transform.translation.x = 4;
  tf_.setTransform(transform, "default_authority", true);

  layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
//  printMap(*(layers.getCostmap()));

  rlayer->bufferIncomingRangeMsg(std::make_shared<sensor_msgs::msg::Range>(msg));

  transform.transform.translation.y = 5;
  transform.transform.translation.x = 6;
  tf_.setTransform(transform, "default_authority", true);

  layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
//  printMap(*(layers.getCostmap()));

  ASSERT_EQ(layers.getCostmap()->getCost(5, 5), 254);
  ASSERT_EQ(layers.getCostmap()->getCost(6, 5), 0);
  ASSERT_EQ(layers.getCostmap()->getCost(7, 5), 254);
  ASSERT_EQ(layers.getCostmap()->getCost(8, 5), 0);
  ASSERT_EQ(layers.getCostmap()->getCost(9, 5), 254);
}

// Testing fixed motion with downward movement
TEST_F(TestNode, testProbabalisticModelDownward) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = "frame";
  transform.child_frame_id = "base_link";
  transform.transform.translation.y = 3;
  transform.transform.translation.x = 2;
  tf_.setTransform(transform, "default_authority", true);

  nav2_costmap_2d::LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  std::shared_ptr<nav2_costmap_2d::RangeSensorLayer> rlayer{nullptr};
  addRangeLayer(layers, tf_, node_, rlayer);

  sensor_msgs::msg::Range msg;
  msg.min_range = 1.0;
  msg.max_range = 10.0;
  msg.range = 1.0;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "base_link";
  msg.radiation_type = msg.ULTRASOUND;
  msg.field_of_view = 0.174533;  // 10 deg
  rlayer->bufferIncomingRangeMsg(std::make_shared<sensor_msgs::msg::Range>(msg));

  layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
//  printMap(*(layers.getCostmap()));

  rlayer->bufferIncomingRangeMsg(std::make_shared<sensor_msgs::msg::Range>(msg));

  transform.transform.translation.y = 5;
  transform.transform.translation.x = 2;
  tf_.setTransform(transform, "default_authority", true);

  layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
//  printMap(*(layers.getCostmap()));

  rlayer->bufferIncomingRangeMsg(std::make_shared<sensor_msgs::msg::Range>(msg));

  transform.transform.translation.y = 7;
  transform.transform.translation.x = 2;
  tf_.setTransform(transform, "default_authority", true);

  layers.updateMap(0, 0, 0);  // 0, 0, 0 is robot pose
//  printMap(*(layers.getCostmap()));

  ASSERT_EQ(layers.getCostmap()->getCost(3, 3), 254);
  ASSERT_EQ(layers.getCostmap()->getCost(3, 4), 0);
  ASSERT_EQ(layers.getCostmap()->getCost(3, 5), 254);
  ASSERT_EQ(layers.getCostmap()->getCost(3, 6), 0);
  ASSERT_EQ(layers.getCostmap()->getCost(3, 7), 254);
}
