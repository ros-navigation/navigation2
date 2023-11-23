// Copyright (c) 2023 Alberto J. Tudela Roldán
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
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_graceful_motion_controller/smooth_control_law.hpp"
#include "nav2_graceful_motion_controller/graceful_motion_controller.hpp"

class GMControllerFixture : public nav2_graceful_motion_controller::GracefulMotionController
{
public:
  GMControllerFixture()
  : nav2_graceful_motion_controller::GracefulMotionController() {}

  nav_msgs::msg::Path getPlan() {return path_handler_->getPlan();}
};

TEST(GracefulMotionControllerTest, configure) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();
  controller->deactivate();
  controller->cleanup();

  // Set the plan
  nav_msgs::msg::Path plan;
  plan.poses.resize(3);
  plan.poses[0].header.frame_id = "map";
  controller->setPlan(plan);
  EXPECT_EQ(controller->getPlan().poses.size(), 3u);
  EXPECT_EQ(controller->getPlan().poses[0].header.frame_id, "map");
}

TEST(GracefulMotionControllerTest, dynamicParameters) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

  // Create controller
  auto controller = std::make_shared<nav2_graceful_motion_controller::GracefulMotionController>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.transform_tolerance", 1.0),
      rclcpp::Parameter("test.motion_target_dist", 2.0),
      rclcpp::Parameter("test.max_robot_pose_search_dist", 3.0),
      rclcpp::Parameter("test.k_phi", 4.0),
      rclcpp::Parameter("test.k_delta", 5.0),
      rclcpp::Parameter("test.beta", 6.0),
      rclcpp::Parameter("test.lambda", 7.0),
      rclcpp::Parameter("test.v_linear_min", 8.0),
      rclcpp::Parameter("test.v_linear_max", 9.0),
      rclcpp::Parameter("test.v_angular_max", 10.0),
      rclcpp::Parameter("test.slowdown_radius", 11.0)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.transform_tolerance").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.motion_target_dist").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.max_robot_pose_search_dist").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("test.k_phi").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("test.k_delta").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("test.beta").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("test.lambda").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.v_linear_min").as_double(), 8.0);
  EXPECT_EQ(node->get_parameter("test.v_linear_max").as_double(), 9.0);
  EXPECT_EQ(node->get_parameter("test.v_angular_max").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("test.slowdown_radius").as_double(), 11.0);

  // Set max search distant to negative so it warns
  results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.max_robot_pose_search_dist", -1.0)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
}

TEST(SmoothControlLawTest, calculateRegularVelocity) {
  // Initialize SmoothControlLaw
  nav2_graceful_motion_controller::SmoothControlLaw scl(1.0, 10.0, 0.2, 2.0, 0.1, 0.0, 1.0, 1.0);

  // Initialize target
  geometry_msgs::msg::Pose target;
  target.position.x = 0.0;
  target.position.y = 5.0;
  target.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  // Initialize current
  geometry_msgs::msg::Pose current;
  current.position.x = 0.0;
  current.position.y = 0.0;
  current.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  // Calculate velocity
  auto cmd_vel = scl.calculateRegularVelocity(target, current);

  // Check results
  EXPECT_NEAR(cmd_vel.linear.x, 0.1460446, 0.0001);
  EXPECT_NEAR(cmd_vel.angular.z, 0.7896695, 0.0001);

  // Set a new target
  target.position.x = 4.5;
  target.position.y = -2.17;
  target.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, -0.785));
  // Set the same current
  current.position.x = 0.0;
  current.position.y = 0.0;
  current.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  // Calculate velocity
  cmd_vel = scl.calculateRegularVelocity(target, current);

  // Check results
  EXPECT_NEAR(cmd_vel.linear.x, 0.96651200, 0.0001);
  EXPECT_NEAR(cmd_vel.angular.z, -0.4022844, 0.0001);
}

TEST(SmoothControlLawTest, calculateNextPose) {
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
