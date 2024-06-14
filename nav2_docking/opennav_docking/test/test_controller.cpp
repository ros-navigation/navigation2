// Copyright (c) 2024 Open Navigation LLC
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
#include "opennav_docking/controller.hpp"
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
  auto controller = std::make_unique<opennav_docking::Controller>(node);

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist cmd_out, cmd_init;
  EXPECT_TRUE(controller->computeVelocityCommand(pose, cmd_out));
  EXPECT_NE(cmd_init, cmd_out);
  controller.reset();
}

TEST(ControllerTests, DynamicParameters) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto controller = std::make_shared<opennav_docking::Controller>(node);

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
      rclcpp::Parameter("controller.slowdown_radius", 8.0)});

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
}

}  // namespace opennav_docking
