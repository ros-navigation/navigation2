// Copyright (c) 2021 Wyca Robotics
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
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_broadcaster.h"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class DynParamTestNode
{
public:
  DynParamTestNode() {}
  ~DynParamTestNode() {}
};

TEST(DynParamTestNode, testDynParamsSet)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dyn_param_tester");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());

  // Set tf between default global_frame and robot_base_frame in order not to block in on_activate
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";
  tf_broadcaster_->sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "test_frame";
  tf_broadcaster_->sendTransform(t);

  costmap->on_activate(rclcpp_lifecycle::State());

  auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
    node->shared_from_this(),
    "/test_costmap/test_costmap",
    rmw_qos_profile_parameters);
  auto results1 = parameter_client->set_parameters_atomically(
  {
    rclcpp::Parameter("robot_radius", 1.234),
    rclcpp::Parameter("footprint_padding", 2.345),
    rclcpp::Parameter("transform_tolerance", 3.456),
    rclcpp::Parameter("publish_frequency", 4.567),
    rclcpp::Parameter("resolution", 5.678),
    rclcpp::Parameter("origin_x", 6.789),
    rclcpp::Parameter("origin_y", 7.891),
    rclcpp::Parameter("width", 2),
    rclcpp::Parameter("height", 3),
    rclcpp::Parameter(
      "footprint",
      "[[-0.325, -0.325], [-0.325, 0.325], [0.325, 0.325], [0.46, 0.0], [0.325, -0.325]]"),
    rclcpp::Parameter("robot_base_frame", "test_frame"),
  });

  // Try setting robot_base_frame to an invalid frame, should be rejected
  auto results2 = parameter_client->set_parameters_atomically(
  {
    rclcpp::Parameter("robot_base_frame", "wrong_test_frame"),
  });

  rclcpp::spin_some(costmap->get_node_base_interface());

  EXPECT_EQ(costmap->get_parameter("robot_radius").as_double(), 1.234);
  EXPECT_EQ(costmap->get_parameter("footprint_padding").as_double(), 2.345);
  EXPECT_EQ(costmap->get_parameter("transform_tolerance").as_double(), 3.456);
  EXPECT_EQ(costmap->get_parameter("publish_frequency").as_double(), 4.567);
  EXPECT_EQ(costmap->get_parameter("resolution").as_double(), 5.678);
  EXPECT_EQ(costmap->get_parameter("origin_x").as_double(), 6.789);
  EXPECT_EQ(costmap->get_parameter("origin_y").as_double(), 7.891);
  EXPECT_EQ(costmap->get_parameter("width").as_int(), 2);
  EXPECT_EQ(costmap->get_parameter("height").as_int(), 3);
  EXPECT_EQ(
    costmap->get_parameter("footprint").as_string(),
    "[[-0.325, -0.325], [-0.325, 0.325], [0.325, 0.325], [0.46, 0.0], [0.325, -0.325]]");
  EXPECT_EQ(costmap->get_parameter("robot_base_frame").as_string(), "test_frame");

  costmap->on_deactivate(rclcpp_lifecycle::State());
  costmap->on_cleanup(rclcpp_lifecycle::State());
  costmap->on_shutdown(rclcpp_lifecycle::State());
}
