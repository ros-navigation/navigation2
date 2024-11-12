// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "opennav_docking/controller.hpp"
#include "tf2_ros/buffer.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Testing the controller at high level; the nav2_graceful_controller
// Where the control law derives has over 98% test coverage

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_docking
{

TEST(ControllerTests, ObjectLifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

  // Skip collision detection
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.use_collision_detection", rclcpp::ParameterValue(false));

  auto controller = std::make_unique<opennav_docking::Controller>(
    node, tf, "test_fixed_frame", "test_base_frame");

  // Set a transform between fixed_frame and base_frame
  geometry_msgs::msg::TransformStamped fixed_to_base;
  fixed_to_base.header.frame_id = "test_fixed_frame";
  fixed_to_base.header.stamp = node->get_clock()->now();
  fixed_to_base.child_frame_id = "test_base_frame";
  fixed_to_base.transform.translation.x = 0.0;
  fixed_to_base.transform.translation.y = 0.0;
  fixed_to_base.transform.translation.z = 0.0;
  tf->setTransform(fixed_to_base, "test", false);

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist cmd_out, cmd_init;
  EXPECT_TRUE(controller->computeVelocityCommand(pose, cmd_out));
  EXPECT_NE(cmd_init, cmd_out);
  controller.reset();
}

TEST(ControllerTests, DynamicParameters) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto controller = std::make_shared<opennav_docking::Controller>(
    node, nullptr, "test_fixed_frame", "test_base_frame");

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("controller.k_phi", 1.0),
      rclcpp::Parameter("controller.k_delta", 2.0),
      rclcpp::Parameter("controller.beta", 3.0),
      rclcpp::Parameter("controller.lambda", 4.0),
      rclcpp::Parameter("controller.v_linear_min", 5.0),
      rclcpp::Parameter("controller.v_linear_max", 6.0),
      rclcpp::Parameter("controller.v_angular_max", 7.0),
      rclcpp::Parameter("controller.slowdown_radius", 8.0),
      rclcpp::Parameter("controller.projection_time", 9.0),
      rclcpp::Parameter("controller.simulation_time_step", 10.0)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("controller.k_phi").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("controller.k_delta").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("controller.beta").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("controller.lambda").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("controller.v_linear_min").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("controller.v_linear_max").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("controller.v_angular_max").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("controller.slowdown_radius").as_double(), 8.0);
  EXPECT_EQ(node->get_parameter("controller.projection_time").as_double(), 9.0);
  EXPECT_EQ(node->get_parameter("controller.simulation_time_step").as_double(), 10.0);
}

}  // namespace opennav_docking
