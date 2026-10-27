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

#include <cmath>

#include "gtest/gtest.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/utilities.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_controller/plugins/feasible_path_handler.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_graceful_controller/ego_polar_coords.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"
#include "nav2_graceful_controller/graceful_controller.hpp"
#include "nav2_util/path_utils.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

class SCLFixture : public nav2_graceful_controller::SmoothControlLaw
{
public:
  SCLFixture(
    double k_phi, double k_delta, double beta, double lambda,
    double slowdown_radius, double deceleration_max,
    double v_linear_min, double v_linear_max, double v_angular_max)
  : nav2_graceful_controller::SmoothControlLaw(k_phi, k_delta, beta, lambda,
      slowdown_radius, deceleration_max, v_linear_min, v_linear_max, v_angular_max) {}

  double getCurvatureKPhi() {return k_phi_;}
  double getCurvatureKDelta() {return k_delta_;}
  double getCurvatureBeta() {return beta_;}
  double getCurvatureLambda() {return lambda_;}
  double getSlowdownRadius() {return slowdown_radius_;}
  double getDecelMax() {return deceleration_max_;}
  double getSpeedLinearMin() {return v_linear_min_;}
  double getSpeedLinearMax() {return v_linear_max_;}
  double getSpeedAngularMax() {return v_angular_max_;}
  double calculateCurvature(geometry_msgs::msg::Pose target, geometry_msgs::msg::Pose current)
  {
    auto ego_coords = nav2_graceful_controller::EgocentricPolarCoordinates(target, current);
    return nav2_graceful_controller::SmoothControlLaw::calculateCurvature(
      ego_coords.r, ego_coords.phi, ego_coords.delta);
  }
};

class GMControllerFixture : public nav2_graceful_controller::GracefulController
{
public:
  GMControllerFixture()
  : nav2_graceful_controller::GracefulController() {}

  bool getInitialRotation() {return params_->initial_rotation;}

  bool getAllowBackward() {return params_->allow_backward;}

  visualization_msgs::msg::Marker createSlowdownMarker(
    const geometry_msgs::msg::PoseStamped & motion_target)
  {
    return nav2_graceful_controller::createSlowdownMarker(
      motion_target,
      params_->slowdown_radius);
  }

  geometry_msgs::msg::Twist rotateToTarget(const double & angle_to_target)
  {
    return nav2_graceful_controller::GracefulController::rotateToTarget(
      angle_to_target);
  }

  double getSpeedLinearMax() {return params_->v_linear_max;}

  double getSpeedAngularMax() {return params_->v_angular_max;}

  bool validateTargetPoseOnApproach(
    geometry_msgs::msg::PoseStamped & target_pose, double dist_to_target, double dist_to_goal,
    nav_msgs::msg::Path & trajectory, geometry_msgs::msg::TransformStamped & costmap_transform,
    geometry_msgs::msg::TwistStamped & cmd_vel)
  {
    return nav2_graceful_controller::GracefulController::validateTargetPoseOnApproach(
      target_pose, dist_to_target, dist_to_goal, trajectory, costmap_transform, cmd_vel);
  }
};

TEST(SmoothControlLawTest, setCurvatureConstants) {
  // Initialize SmoothControlLaw
  SCLFixture scl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0);

  // Set curvature constants
  scl.setCurvatureConstants(1.0, 2.0, 3.0, 4.0);

  // Set slowdown radius
  scl.setSlowdownRadius(5.0);

  // Set max deceleration
  scl.setMaxDeceleration(6.0);

  // Check results
  EXPECT_EQ(scl.getCurvatureKPhi(), 1.0);
  EXPECT_EQ(scl.getCurvatureKDelta(), 2.0);
  EXPECT_EQ(scl.getCurvatureBeta(), 3.0);
  EXPECT_EQ(scl.getCurvatureLambda(), 4.0);
  EXPECT_EQ(scl.getSlowdownRadius(), 5.0);
  EXPECT_EQ(scl.getDecelMax(), 6.0);
}

TEST(SmoothControlLawTest, setSpeedLimits) {
  // Initialize SmoothControlLaw
  SCLFixture scl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Set speed limits
  scl.setSpeedLimit(1.0, 2.0, 3.0);

  // Check results
  EXPECT_EQ(scl.getSpeedLinearMin(), 1.0);
  EXPECT_EQ(scl.getSpeedLinearMax(), 2.0);
  EXPECT_EQ(scl.getSpeedAngularMax(), 3.0);
}

TEST(SmoothControlLawTest, calculateCurvature) {
  // Initialize SmoothControlLaw
  SCLFixture scl(1.0, 10.0, 0.2, 2.0, 0.1, 2.5, 0.0, 1.0, 1.0);

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
  // Calculate curvature
  double curvature = scl.calculateCurvature(target, current);

  // Check results: it must be positive
  EXPECT_NEAR(curvature, 5.407042, 0.0001);

  // Set a new target
  target.position.x = 4.5;
  target.position.y = -2.17;
  target.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, -0.785));
  // Set the same current
  current.position.x = 0.0;
  current.position.y = 0.0;
  current.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  // Calculate curvature
  curvature = scl.calculateCurvature(target, current);

  // Check results: it must be neggative
  EXPECT_NEAR(curvature, -0.416228, 0.0001);
}

TEST(SmoothControlLawTest, calculateRegularVelocity) {
  // Initialize SmoothControlLaw
  SCLFixture scl(1.0, 10.0, 0.2, 2.0, 0.1, 2.5, 0.0, 1.0, 1.0);

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

  // Check results: both linear and angular velocity must be positive
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

  // Check results: linear velocity must be positive and angular velocity must be negative
  EXPECT_NEAR(cmd_vel.linear.x, 0.96651200, 0.0001);
  EXPECT_NEAR(cmd_vel.angular.z, -0.4022844, 0.0001);
}

TEST(SmoothControlLawTest, calculateNextPose) {
  // Initialize SmoothControlLaw
  SCLFixture scl(1.0, 10.0, 0.2, 2.0, 0.1, 2.5, 0.0, 1.0, 1.0);

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
  // Calculate next pose
  float dt = 0.1;
  auto next_pose = scl.calculateNextPose(dt, target, current);

  // Check results
  EXPECT_NEAR(next_pose.position.x, 0.1, 0.1);
  EXPECT_NEAR(next_pose.position.y, 0.0, 0.1);
  EXPECT_NEAR(tf2::getYaw(next_pose.orientation), 0.0, 0.1);
}

TEST(GracefulControllerTest, dynamicParameters) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

  // Set initial rotation and allow backward to true so it warns and allow backward is false
  nav2::declare_parameter_if_not_declared(
    node, "test.initial_rotation", rclcpp::ParameterValue(true));
  nav2::declare_parameter_if_not_declared(
    node, "test.allow_backward", rclcpp::ParameterValue(true));
  nav2::declare_parameter_if_not_declared(
    node, "test.use_collision_detection", rclcpp::ParameterValue(true));

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Check first allowed backward is false
  EXPECT_EQ(controller->getAllowBackward(), false);

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.min_lookahead", 1.0),
      rclcpp::Parameter("test.max_lookahead", 2.0),
      rclcpp::Parameter("test.k_phi", 4.0),
      rclcpp::Parameter("test.k_delta", 5.0),
      rclcpp::Parameter("test.beta", 6.0),
      rclcpp::Parameter("test.lambda", 7.0),
      rclcpp::Parameter("test.v_linear_min", 8.0),
      rclcpp::Parameter("test.v_linear_max", 9.0),
      rclcpp::Parameter("test.v_angular_max", 10.0),
      rclcpp::Parameter("test.v_angular_min_in_place", 14.0),
      rclcpp::Parameter("test.slowdown_radius", 11.0),
      rclcpp::Parameter("test.deceleration_max", 21.0),
      rclcpp::Parameter("test.initial_rotation", false),
      rclcpp::Parameter("test.initial_rotation_tolerance", 12.0),
      rclcpp::Parameter("test.prefer_final_rotation", false),
      rclcpp::Parameter("test.rotation_scaling_factor", 13.0),
      rclcpp::Parameter("test.allow_backward", false),
      rclcpp::Parameter("test.use_collision_detection", false),
      rclcpp::Parameter("test.in_place_collision_resolution", 15.0),
      rclcpp::Parameter("test.footprint_scaling_linear_vel", 16.0),
      rclcpp::Parameter("test.footprint_scaling_factor", 17.0),
      rclcpp::Parameter("test.footprint_scaling_step", 18.0),
      rclcpp::Parameter("test.obstacle_cost_margin", 19),
      rclcpp::Parameter("test.final_rotation_search_step", 20.0)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.min_lookahead").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.max_lookahead").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.k_phi").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("test.k_delta").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("test.beta").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("test.lambda").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.v_linear_min").as_double(), 8.0);
  EXPECT_EQ(node->get_parameter("test.v_linear_max").as_double(), 9.0);
  EXPECT_EQ(node->get_parameter("test.v_angular_max").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("test.v_angular_min_in_place").as_double(), 14.0);
  EXPECT_EQ(node->get_parameter("test.slowdown_radius").as_double(), 11.0);
  EXPECT_EQ(node->get_parameter("test.deceleration_max").as_double(), 21.0);
  EXPECT_EQ(node->get_parameter("test.initial_rotation").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.initial_rotation_tolerance").as_double(), 12.0);
  EXPECT_EQ(node->get_parameter("test.prefer_final_rotation").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.rotation_scaling_factor").as_double(), 13.0);
  EXPECT_EQ(node->get_parameter("test.allow_backward").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_collision_detection").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.in_place_collision_resolution").as_double(), 15.0);
  EXPECT_EQ(node->get_parameter("test.footprint_scaling_linear_vel").as_double(), 16.0);
  EXPECT_EQ(node->get_parameter("test.footprint_scaling_factor").as_double(), 17.0);
  EXPECT_EQ(node->get_parameter("test.footprint_scaling_step").as_double(), 18.0);
  EXPECT_EQ(node->get_parameter("test.obstacle_cost_margin").as_int(), 19);
  EXPECT_EQ(node->get_parameter("test.final_rotation_search_step").as_double(), 20.0);

  // Set allow backward to true
  results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.allow_backward", true)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  EXPECT_EQ(controller->getInitialRotation(), false);
  EXPECT_EQ(controller->getAllowBackward(), true);

  // Set initial rotation to true
  results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.initial_rotation", true)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  // Check parameters. Initial rotation should be false as both cannot be true at the same time
  EXPECT_EQ(controller->getInitialRotation(), false);
  EXPECT_EQ(controller->getAllowBackward(), true);

  // Set both initial rotation and allow backward to false
  results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.initial_rotation", false),
      rclcpp::Parameter("test.allow_backward", false)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  // Check parameters.
  EXPECT_EQ(node->get_parameter("test.initial_rotation").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.allow_backward").as_bool(), false);

  // Set initial rotation to true
  results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.initial_rotation", true)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  EXPECT_EQ(controller->getInitialRotation(), true);

  // Set allow backward to true
  results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.allow_backward", true)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
  // Check parameters. Now allow backward should be false as both cannot be true at the same time
  EXPECT_EQ(controller->getInitialRotation(), true);
  EXPECT_EQ(controller->getAllowBackward(), false);
}

TEST(GracefulControllerTest, createSlowdownMsg) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Create motion target
  geometry_msgs::msg::PoseStamped motion_target;
  motion_target.header.frame_id = "map";
  motion_target.pose.position.x = 1.0;
  motion_target.pose.position.y = 2.0;
  motion_target.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set slowdown parameter
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.slowdown_radius", 0.2)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Create slowdown message
  auto slowdown_msg = controller->createSlowdownMarker(motion_target);

  // Check results
  EXPECT_EQ(slowdown_msg.header.frame_id, "map");
  EXPECT_EQ(slowdown_msg.ns, "slowdown");
  EXPECT_EQ(slowdown_msg.id, 0);
  EXPECT_EQ(slowdown_msg.type, visualization_msgs::msg::Marker::SPHERE);
  EXPECT_EQ(slowdown_msg.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_EQ(slowdown_msg.pose.position.x, 1.0);
  EXPECT_EQ(slowdown_msg.pose.position.y, 2.0);
  EXPECT_EQ(slowdown_msg.pose.position.z, 0.01);
  EXPECT_EQ(slowdown_msg.pose.orientation.x, 0.0);
  EXPECT_EQ(slowdown_msg.pose.orientation.y, 0.0);
  EXPECT_EQ(slowdown_msg.pose.orientation.z, 0.0);
  EXPECT_EQ(slowdown_msg.pose.orientation.w, 1.0);
  EXPECT_EQ(slowdown_msg.scale.x, 0.4);
  EXPECT_EQ(slowdown_msg.scale.y, 0.4);
  EXPECT_EQ(slowdown_msg.scale.z, 0.02);
  EXPECT_EQ(slowdown_msg.color.r, 0.0);
  EXPECT_EQ(slowdown_msg.color.g, 1.0);
  EXPECT_EQ(slowdown_msg.color.b, 0.0);
}

TEST(GracefulControllerTest, rotateToTarget) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Set max velocity
  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.v_angular_max", 1.0),
      rclcpp::Parameter("test.rotation_scaling_factor", 0.5)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Set angle to target and get velocity command
  double angle_to_target = 0.5;
  auto cmd_vel = controller->rotateToTarget(angle_to_target);

  // Check results: it must be a positive rotation
  EXPECT_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.angular.z, 0.25);

  // Set a new angle to target
  angle_to_target = -0.5;
  cmd_vel = controller->rotateToTarget(angle_to_target);

  // Check results: it must be a negative rotation
  EXPECT_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.angular.z, -0.25);

  // Set very high v_angular_min_in_place velocity
  results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.v_angular_min_in_place", 1.0)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Set a new angle to target
  angle_to_target = 0.5;
  cmd_vel = controller->rotateToTarget(angle_to_target);

  // Check results: positive velocity, at least as high as min_in_place
  EXPECT_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.angular.z, 1.0);
}

TEST(GracefulControllerTest, setSpeedLimit) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Set max velocity
  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.v_linear_min", 2.0),
      rclcpp::Parameter("test.v_linear_max", 5.0)
    });
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Set relative speed limit and get speed parameter
  controller->setSpeedLimit(50.0, true);
  double speed_limit = controller->getSpeedLinearMax();

  // Check results
  EXPECT_EQ(speed_limit, 2.5);

  // Set absolute speed limit and get speed parameter
  controller->setSpeedLimit(100.0, false);
  speed_limit = controller->getSpeedLinearMax();

  // Check results
  EXPECT_EQ(speed_limit, 100.0);

  // Set a new value below the minimum and get speed parameter
  controller->setSpeedLimit(1.0, false);
  speed_limit = controller->getSpeedLinearMax();

  // Check results
  EXPECT_EQ(speed_limit, 2.0);
}

// [AI generated] This test and the getSpeedAngularMax() accessor
// were generated by AI and reviewed by the author
TEST(GracefulControllerTest, setSpeedLimitWithConfiguredParameters) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

  // Configure the velocity limits only at startup, they are never changed
  // through the dynamic parameter callback in this test
  nav2::declare_parameter_if_not_declared(
    node, "test.v_linear_min", rclcpp::ParameterValue(0.05));
  nav2::declare_parameter_if_not_declared(
    node, "test.v_linear_max", rclcpp::ParameterValue(0.4));
  nav2::declare_parameter_if_not_declared(
    node, "test.v_angular_max", rclcpp::ParameterValue(0.8));

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Relative speed limit is a percentage of the configured maximums
  controller->setSpeedLimit(30.0, true);
  EXPECT_NEAR(controller->getSpeedLinearMax(), 0.12, 1e-9);
  EXPECT_NEAR(controller->getSpeedAngularMax(), 0.24, 1e-9);

  // Absolute speed limit scales the angular maximum with the linear one
  controller->setSpeedLimit(0.2, false);
  EXPECT_NEAR(controller->getSpeedLinearMax(), 0.2, 1e-9);
  EXPECT_NEAR(controller->getSpeedAngularMax(), 0.4, 1e-9);

  // Removing the speed limit restores the configured maximums
  controller->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);
  EXPECT_NEAR(controller->getSpeedLinearMax(), 0.4, 1e-9);
  EXPECT_NEAR(controller->getSpeedAngularMax(), 0.8, 1e-9);
}

TEST(GracefulControllerTest, computeVelocityCommandRotate) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);

  nav2::declare_parameter_if_not_declared(
    node, "test.v_angular_max", rclcpp::ParameterValue(1.0));
  nav2::declare_parameter_if_not_declared(
    node, "test.rotation_scaling_factor", rclcpp::ParameterValue(0.5));

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap_ros, tf);

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_robot_frame";
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  robot_pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = robot_pose.pose.position.x;
  global_to_robot.transform.translation.y = robot_pose.pose.position.y;
  global_to_robot.transform.translation.z = robot_pose.pose.position.z;
  tf->setTransform(global_to_robot, "test", false);

  // Set a plan
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "test_global_frame";
  plan.poses.resize(3);
  plan.poses[0].header.frame_id = "test_global_frame";
  plan.poses[0].pose.position.x = 0.0;
  plan.poses[0].pose.position.y = 0.0;
  plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[1].header.frame_id = "test_global_frame";
  plan.poses[1].pose.position.x = 0.5;
  plan.poses[1].pose.position.y = 0.5;
  plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[2].header.frame_id = "test_global_frame";
  plan.poses[2].pose.position.x = 1.0;
  plan.poses[2].pose.position.y = 1.0;
  plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  controller->newPathReceived(plan);
  path_handler.setPlan(plan);

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;

  // Set the goal checker
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);
  auto [closest_point, pruned_plan_end] = path_handler.findPlanSegment(robot_pose);
  nav_msgs::msg::Path transformed_global_plan = path_handler.transformLocalPlan(closest_point,
    pruned_plan_end);
  geometry_msgs::msg::PoseStamped goal;
  auto cmd_vel = controller->computeVelocityCommands(robot_pose, robot_velocity, &checker,
    transformed_global_plan, goal);
  transformed_global_plan.header.frame_id = "fake_frame";
  EXPECT_THROW(controller->computeVelocityCommands(robot_pose, robot_velocity, &checker,
    transformed_global_plan, goal), std::runtime_error);

  // Check results: the robot should rotate in place.
  // So, linear velocity should be zero and angular velocity should be a positive value below 0.5.
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_GT(cmd_vel.twist.angular.z, 0.0);
  EXPECT_LE(cmd_vel.twist.angular.z, 0.5);
}

TEST(GracefulControllerTest, computeVelocityCommandRegular) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap_ros, tf);

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_robot_frame";
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  robot_pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = robot_pose.pose.position.x;
  global_to_robot.transform.translation.y = robot_pose.pose.position.y;
  global_to_robot.transform.translation.z = robot_pose.pose.position.z;
  tf->setTransform(global_to_robot, "test", false);

  // Set a plan in a straight line from the robot
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "test_global_frame";
  plan.poses.resize(3);
  plan.poses[0].header.frame_id = "test_global_frame";
  plan.poses[0].pose.position.x = 0.0;
  plan.poses[0].pose.position.y = 0.0;
  plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[1].header.frame_id = "test_global_frame";
  plan.poses[1].pose.position.x = 1.0;
  plan.poses[1].pose.position.y = 0.0;
  plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[2].header.frame_id = "test_global_frame";
  plan.poses[2].pose.position.x = 2.0;
  plan.poses[2].pose.position.y = 0.0;
  plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  controller->newPathReceived(plan);
  path_handler.setPlan(plan);

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;

  // Set the goal checker
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);
  auto [closest_point, pruned_plan_end] = path_handler.findPlanSegment(robot_pose);
  nav_msgs::msg::Path transformed_global_plan = path_handler.transformLocalPlan(closest_point,
    pruned_plan_end);
  geometry_msgs::msg::PoseStamped goal;
  auto cmd_vel = controller->computeVelocityCommands(robot_pose, robot_velocity, &checker,
    transformed_global_plan, goal);

  // Check results: the robot should go straight to the target.
  // So, linear velocity should be some positive value and angular velocity should be zero.
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.twist.angular.z, 0.0);
}

TEST(GracefulControllerTest, computeVelocityCommandReusesProvidedPoseForCollisionChecking) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGracefulPoseTransform");
  auto tf = nav2::create_transform_buffer(node);

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_global_frame";
  robot_pose.header.stamp = rclcpp::Time(10, 0);
  robot_pose.pose.orientation.w = 1.0;

  nav_msgs::msg::Path transformed_global_plan;
  transformed_global_plan.header.frame_id = "test_robot_frame";
  transformed_global_plan.header.stamp = robot_pose.header.stamp;
  transformed_global_plan.poses.resize(3);
  for (size_t i = 0; i < transformed_global_plan.poses.size(); ++i) {
    transformed_global_plan.poses[i].header = transformed_global_plan.header;
    transformed_global_plan.poses[i].pose.position.x = static_cast<double>(i);
    transformed_global_plan.poses[i].pose.orientation.w = 1.0;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header = robot_pose.header;
  goal.pose.position.x = 2.0;
  goal.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist robot_velocity;
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  // No transform is installed in the buffer. Collision checking must use robot_pose instead of
  // independently looking up the latest global-to-base transform.
  geometry_msgs::msg::TwistStamped cmd_vel;
  EXPECT_NO_THROW(
    cmd_vel = controller->computeVelocityCommands(
      robot_pose, robot_velocity, &checker, transformed_global_plan, goal));
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
}

TEST(GracefulControllerTest, computeVelocityCommandMinLookahead) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGracefulMinLookahead");
  auto tf = nav2::create_transform_buffer(node);

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Place a wall 0.4 m in front of the robot, so only targets closer than that are valid
  auto costmap = costmap_ros->getCostmap();
  unsigned int mx, my;
  for (double y = 4.5; y <= 5.5; y += 0.1) {
    ASSERT_TRUE(costmap->worldToMap(5.45, y, mx, my));
    costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  // Robot in the middle of the costmap
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_global_frame";
  robot_pose.pose.position.x = 5.0;
  robot_pose.pose.position.y = 5.0;
  robot_pose.pose.orientation.w = 1.0;

  // Straight plan of 2 m in the robot frame, longer than max_lookahead (not approaching goal)
  nav_msgs::msg::Path transformed_global_plan;
  transformed_global_plan.header.frame_id = "test_robot_frame";
  transformed_global_plan.poses.resize(21);
  for (size_t i = 0; i < transformed_global_plan.poses.size(); ++i) {
    transformed_global_plan.poses[i].header = transformed_global_plan.header;
    transformed_global_plan.poses[i].pose.position.x = 0.1 * static_cast<double>(i);
    transformed_global_plan.poses[i].pose.orientation.w = 1.0;
  }
  controller->newPathReceived(transformed_global_plan);

  geometry_msgs::msg::PoseStamped goal;
  goal.header = robot_pose.header;
  goal.pose.position.x = 7.0;
  goal.pose.position.y = 5.0;
  goal.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist robot_velocity;
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(), node->get_node_services_interface());

  // Small slowdown radius so the simulated trajectories reach their targets and
  // the wall is found by every target beyond it.
  // The only collision free targets are closer than min_lookahead: no valid control
  auto fut = params->set_parameters_atomically(
    {rclcpp::Parameter("test.slowdown_radius", 0.1),
      rclcpp::Parameter("test.min_lookahead", 0.45)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), fut);
  EXPECT_THROW(
    controller->computeVelocityCommands(
      robot_pose, robot_velocity, &checker, transformed_global_plan, goal),
    nav2_core::NoValidControl);

  // A collision free target is now beyond min_lookahead: the robot moves towards it
  fut = params->set_parameters_atomically({rclcpp::Parameter("test.min_lookahead", 0.25)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), fut);
  geometry_msgs::msg::TwistStamped cmd_vel;
  EXPECT_NO_THROW(
    cmd_vel = controller->computeVelocityCommands(
      robot_pose, robot_velocity, &checker, transformed_global_plan, goal));
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
}

TEST(GracefulControllerTest, computeVelocityCommandMinLookaheadNearGoal) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGracefulMinLookaheadNearGoal");
  auto tf = nav2::create_transform_buffer(node);

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Robot in the middle of the costmap
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_global_frame";
  robot_pose.pose.position.x = 5.0;
  robot_pose.pose.position.y = 5.0;
  robot_pose.pose.orientation.w = 1.0;

  // Straight plan of 0.2 m in the robot frame, shorter than min_lookahead (approaching goal)
  nav_msgs::msg::Path transformed_global_plan;
  transformed_global_plan.header.frame_id = "test_robot_frame";
  transformed_global_plan.poses.resize(3);
  for (size_t i = 0; i < transformed_global_plan.poses.size(); ++i) {
    transformed_global_plan.poses[i].header = transformed_global_plan.header;
    transformed_global_plan.poses[i].pose.position.x = 0.1 * static_cast<double>(i);
    transformed_global_plan.poses[i].pose.orientation.w = 1.0;
  }
  controller->newPathReceived(transformed_global_plan);

  geometry_msgs::msg::PoseStamped goal;
  goal.header = robot_pose.header;
  goal.pose.position.x = 7.0;
  goal.pose.position.y = 5.0;
  goal.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist robot_velocity;
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(), node->get_node_services_interface());

  // Every target is closer than min_lookahead, but min_lookahead must not apply while
  // approaching the goal: the robot keeps moving towards it
  auto fut = params->set_parameters_atomically({rclcpp::Parameter("test.min_lookahead", 0.25)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), fut);
  geometry_msgs::msg::TwistStamped cmd_vel;
  EXPECT_NO_THROW(
    cmd_vel = controller->computeVelocityCommands(
      robot_pose, robot_velocity, &checker, transformed_global_plan, goal));
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
}

TEST(GracefulControllerTest, computeVelocityCommandRegularBackwards) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);
  tf->setUsingDedicatedThread(true);

  // Set initial rotation false and allow backward to true
  nav2::declare_parameter_if_not_declared(
    node, "test.initial_rotation", rclcpp::ParameterValue(false));
  nav2::declare_parameter_if_not_declared(
    node, "test.allow_backward", rclcpp::ParameterValue(true));

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap_ros, tf);

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_robot_frame";
  robot_pose.pose.position.x = 2.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  robot_pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = robot_pose.pose.position.x;
  global_to_robot.transform.translation.y = robot_pose.pose.position.y;
  global_to_robot.transform.translation.z = robot_pose.pose.position.z;
  tf->setTransform(global_to_robot, "test", false);

  // Set a plan in a straight line from the robot
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "test_global_frame";
  plan.poses.resize(3);
  plan.poses[0].header.frame_id = "test_global_frame";
  plan.poses[0].pose.position.x = 2.0;
  plan.poses[0].pose.position.y = 0.0;
  plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[1].header.frame_id = "test_global_frame";
  plan.poses[1].pose.position.x = 1.0;
  plan.poses[1].pose.position.y = 0.0;
  plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[2].header.frame_id = "test_global_frame";
  plan.poses[2].pose.position.x = 0.0;
  plan.poses[2].pose.position.y = 0.0;
  plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  controller->newPathReceived(plan);
  path_handler.setPlan(plan);

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;

  // Set the goal checker
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);
  auto [closest_point, pruned_plan_end] = path_handler.findPlanSegment(robot_pose);
  nav_msgs::msg::Path transformed_global_plan = path_handler.transformLocalPlan(closest_point,
    pruned_plan_end);
  geometry_msgs::msg::PoseStamped goal;
  auto cmd_vel = controller->computeVelocityCommands(robot_pose, robot_velocity, &checker,
    transformed_global_plan, goal);

  // Check results: the robot should go straight to the target.
  // So, both linear velocity should be some negative values.
  EXPECT_LT(cmd_vel.twist.linear.x, 0.0);
  // There might be a small bit of noise on angular velocity
  EXPECT_LT(cmd_vel.twist.angular.z, 0.01);
  EXPECT_GT(cmd_vel.twist.angular.z, -0.01);
}

TEST(GracefulControllerTest, computeVelocityCommandFinal) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap_ros, tf);

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_robot_frame";
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  robot_pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = robot_pose.pose.position.x;
  global_to_robot.transform.translation.y = robot_pose.pose.position.y;
  global_to_robot.transform.translation.z = robot_pose.pose.position.z;
  tf->setTransform(global_to_robot, "test", false);

  // Set a plan in a straight line from the robot
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "test_global_frame";
  plan.poses.resize(5);
  plan.poses[0].header.frame_id = "test_global_frame";
  plan.poses[0].pose.position.x = 0.0;
  plan.poses[0].pose.position.y = 0.0;
  plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[1].header.frame_id = "test_global_frame";
  plan.poses[1].pose.position.x = 0.1;
  plan.poses[1].pose.position.y = 0.0;
  plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[2].header.frame_id = "test_global_frame";
  plan.poses[2].pose.position.x = 0.15;
  plan.poses[2].pose.position.y = 0.0;
  plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[3].header.frame_id = "test_global_frame";
  plan.poses[3].pose.position.x = 0.18;
  plan.poses[3].pose.position.y = 0.0;
  plan.poses[3].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[4].header.frame_id = "test_global_frame";
  plan.poses[4].pose.position.x = 0.2;
  plan.poses[4].pose.position.y = 0.0;
  plan.poses[4].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  controller->newPathReceived(plan);
  path_handler.setPlan(plan);

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;

  // Set the goal checker
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  auto [closest_point, pruned_plan_end] = path_handler.findPlanSegment(robot_pose);
  nav_msgs::msg::Path transformed_global_plan = path_handler.transformLocalPlan(closest_point,
    pruned_plan_end);
  geometry_msgs::msg::PoseStamped goal;
  auto cmd_vel = controller->computeVelocityCommands(robot_pose, robot_velocity, &checker,
    transformed_global_plan, goal);

  // Check results: the robot should do a final rotation near the target.
  // So, linear velocity should be zero and angular velocity should be a positive value below 0.5.
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_GE(cmd_vel.twist.angular.z, 0.0);
  EXPECT_LE(cmd_vel.twist.angular.z, 0.5);
}

TEST(GracefulControllerTest, slowDownForObstacle) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  costmap_ros->declare_parameter(
    "footprint", rclcpp::ParameterValue("[[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [-0.2, 0.2]]"));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap_ros, tf);

  // Place an obstacle near the path
  auto costmap = costmap_ros->getCostmap();
  unsigned int mx, my;
  bool ok = costmap->worldToMap(5.5, 5.3, mx, my);
  ASSERT_TRUE(ok);
  costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Set parameters to enable footprint scaling logic
  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(), node->get_node_services_interface());
  double max_vel = 1.0;
  auto fut = params->set_parameters_atomically(
    {rclcpp::Parameter("test.v_linear_max", max_vel),
      rclcpp::Parameter("test.footprint_scaling_linear_vel", 0.2),
      rclcpp::Parameter("test.footprint_scaling_factor", 1.0),
      rclcpp::Parameter("test.footprint_scaling_step", 0.1),
      rclcpp::Parameter("test.slowdown_radius", 0.1)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), fut);

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_global_frame";
  robot_pose.pose.position.x = 5.0;
  robot_pose.pose.position.y = 5.0;
  robot_pose.pose.position.z = 0.0;
  robot_pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = 5.0;
  global_to_robot.transform.translation.y = 5.0;
  global_to_robot.transform.translation.z = 0.0;
  tf->setTransform(global_to_robot, "test", false);

  // Set a plan in a straight line from the robot
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "test_global_frame";
  plan.poses.resize(5);
  plan.poses[0].header.frame_id = "test_global_frame";
  plan.poses[0].pose.position.x = 5.0;
  plan.poses[0].pose.position.y = 5.0;
  plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[1].header.frame_id = "test_global_frame";
  plan.poses[1].pose.position.x = 5.25;
  plan.poses[1].pose.position.y = 5.0;
  plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[2].header.frame_id = "test_global_frame";
  plan.poses[2].pose.position.x = 5.5;
  plan.poses[2].pose.position.y = 5.0;
  plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[3].header.frame_id = "test_global_frame";
  plan.poses[3].pose.position.x = 5.75;
  plan.poses[3].pose.position.y = 5.0;
  plan.poses[3].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[4].header.frame_id = "test_global_frame";
  plan.poses[4].pose.position.x = 6.0;
  plan.poses[4].pose.position.y = 5.0;
  plan.poses[4].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  path_handler.setPlan(plan);

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;

  // Set the goal checker
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  auto [closest_point, pruned_plan_end] = path_handler.findPlanSegment(robot_pose);
  nav_msgs::msg::Path transformed_global_plan = path_handler.transformLocalPlan(closest_point,
    pruned_plan_end);
  geometry_msgs::msg::PoseStamped goal;
  auto cmd_vel =
    controller->computeVelocityCommands(robot_pose, robot_velocity, &checker,
      transformed_global_plan, goal);

  // And the produced command velocity should be <= configured max
  EXPECT_LT(cmd_vel.twist.linear.x, max_vel);
}

TEST(GracefulControllerTest, computeVelocityCommandObstacleMargin) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  costmap_ros->declare_parameter(
    "footprint", rclcpp::ParameterValue("[[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [-0.2, 0.2]]"));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap_ros, tf);
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());
  int cost_margin = 10;
  auto fut = params->set_parameters_atomically(
    {rclcpp::Parameter("test.prefer_final_rotation", true),
      rclcpp::Parameter("test.obstacle_cost_margin", cost_margin),
      rclcpp::Parameter("test.slowdown_radius", 0.1),
      rclcpp::Parameter("test.footprint_scaling_linear_vel", 0.2),
      rclcpp::Parameter("test.footprint_scaling_factor", 1.0),
      rclcpp::Parameter("test.footprint_scaling_step", 0.1),
      rclcpp::Parameter("test.max_lookahead", 2.0)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), fut);

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_global_frame";
  robot_pose.pose.position.x = 5.0;
  robot_pose.pose.position.y = 5.0;
  robot_pose.pose.position.z = 0.0;
  robot_pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = 5.0;
  global_to_robot.transform.translation.y = 5.0;
  global_to_robot.transform.translation.z = 0.0;
  tf->setTransform(global_to_robot, "test", false);

  // Set a plan in a straight line from the robot
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "test_global_frame";
  plan.poses.resize(5);
  plan.poses[0].header.frame_id = "test_global_frame";
  plan.poses[0].pose.position.x = 5.0;
  plan.poses[0].pose.position.y = 5.0;
  plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[1].header.frame_id = "test_global_frame";
  plan.poses[1].pose.position.x = 5.25;
  plan.poses[1].pose.position.y = 5.0;
  plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[2].header.frame_id = "test_global_frame";
  plan.poses[2].pose.position.x = 5.5;
  plan.poses[2].pose.position.y = 5.0;
  plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[3].header.frame_id = "test_global_frame";
  plan.poses[3].pose.position.x = 5.75;
  plan.poses[3].pose.position.y = 5.0;
  plan.poses[3].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[4].header.frame_id = "test_global_frame";
  plan.poses[4].pose.position.x = 6.0;
  plan.poses[4].pose.position.y = 5.0;
  plan.poses[4].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  path_handler.setPlan(plan);

  controller->newPathReceived(plan);
  path_handler.setPlan(plan);

  auto costmap = costmap_ros->getCostmap();
  unsigned int mx, my;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      costmap->worldToMap(5.25 + i * 0.1, 4.75 + j * 0.1, mx, my);
      // Case 1: Safe Cost
      // Safety threshold = 252 - 10 = 242.
      // We set obstacle cost to 230 (< 242).
      costmap->setCost(mx, my, 230);
    }
  }

  auto [closest_point1, pruned_plan_end1] = path_handler.findPlanSegment(robot_pose);
  nav_msgs::msg::Path transformed_plan1 = path_handler.transformLocalPlan(closest_point1,
    pruned_plan_end1);

  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = plan.poses.back().pose.position.x;
  goal.pose.position.y = plan.poses.back().pose.position.y;
  auto cmd_vel_safe = controller->computeVelocityCommands(
    robot_pose, robot_velocity, &checker, transformed_plan1, goal);

  // Expectation: Go straight.
  EXPECT_GT(cmd_vel_safe.twist.linear.x, 0.0);
  EXPECT_NEAR(cmd_vel_safe.twist.angular.z, 0.0, 1e-3);

  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      costmap->worldToMap(5.25 + i * 0.1, 4.75 + j * 0.1, mx, my);
      // Case 2: Risky Cost (but valid)
      // We set obstacle cost to 250 (>= 242).
      // This should trigger the avoidance logic.
      costmap->setCost(mx, my, 250);
    }
  }
  costmap->worldToMap(5.5, 5.4, mx, my);
  costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  auto [closest_point2, pruned_plan_end2] = path_handler.findPlanSegment(robot_pose);
  nav_msgs::msg::Path transformed_plan2 = path_handler.transformLocalPlan(closest_point2,
    pruned_plan_end2);

  auto cmd_vel_risky = controller->computeVelocityCommands(
    robot_pose, robot_velocity, &checker, transformed_plan2, goal);

  bool is_same = (std::abs(cmd_vel_safe.twist.linear.x - cmd_vel_risky.twist.linear.x) < 1e-4) &&
    (std::abs(cmd_vel_safe.twist.angular.z - cmd_vel_risky.twist.angular.z) < 1e-4);

  EXPECT_FALSE(is_same);

  // It is likely to turn, so angular z should be non-zero (or larger than safe case)
  EXPECT_GT(std::abs(cmd_vel_risky.twist.angular.z), 1e-3);
}

TEST(GracefulControllerTest, computeVelocityCommandCollisionDetectionDisabled) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGracefulCollisionDetectionDisabled");
  auto tf = nav2::create_transform_buffer(node);

  nav2::declare_parameter_if_not_declared(
    node, "test.use_collision_detection", rclcpp::ParameterValue(false));
  nav2::declare_parameter_if_not_declared(
    node, "test.slowdown_radius", rclcpp::ParameterValue(0.1));

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.05));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_global_frame";
  robot_pose.pose.position.x = 5.0;
  robot_pose.pose.position.y = 5.0;
  robot_pose.pose.orientation.w = 1.0;

  // The goal is within max_lookahead, but outside the XY goal tolerance
  nav_msgs::msg::Path transformed_global_plan;
  transformed_global_plan.header.frame_id = "test_robot_frame";
  transformed_global_plan.poses.resize(1);
  transformed_global_plan.poses[0].header = transformed_global_plan.header;
  transformed_global_plan.poses[0].pose.position.x = 0.8;
  transformed_global_plan.poses[0].pose.orientation.w = 1.0;
  controller->newPathReceived(transformed_global_plan);

  geometry_msgs::msg::PoseStamped goal;
  goal.header = robot_pose.header;
  goal.pose.position.x = 5.8;
  goal.pose.position.y = 5.0;
  goal.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist robot_velocity;
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  // Approach validation must work when collision detection is disabled at startup
  geometry_msgs::msg::TwistStamped cmd_vel;
  ASSERT_NO_THROW(
    cmd_vel = controller->computeVelocityCommands(
      robot_pose, robot_velocity, &checker, transformed_global_plan, goal));
  EXPECT_TRUE(std::isfinite(cmd_vel.twist.linear.x));
  EXPECT_TRUE(std::isfinite(cmd_vel.twist.angular.z));
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
  EXPECT_NEAR(cmd_vel.twist.angular.z, 0.0, 1e-6);

  // A high-cost cell must not change the approach when collision detection is disabled
  auto costmap = costmap_ros->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(5.4, 5.0, mx, my));
  costmap->setCost(mx, my, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  controller->newPathReceived(transformed_global_plan);
  geometry_msgs::msg::TwistStamped cmd_vel_high_cost;
  ASSERT_NO_THROW(
    cmd_vel_high_cost = controller->computeVelocityCommands(
      robot_pose, robot_velocity, &checker, transformed_global_plan, goal));
  EXPECT_DOUBLE_EQ(cmd_vel_high_cost.twist.linear.x, cmd_vel.twist.linear.x);
  EXPECT_DOUBLE_EQ(cmd_vel_high_cost.twist.angular.z, cmd_vel.twist.angular.z);
  costmap->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);

  // Enabling collision detection dynamically must also detect obstacles
  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(), node->get_node_services_interface());
  auto fut = params->set_parameters_atomically(
    {rclcpp::Parameter("test.use_collision_detection", true)});
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(node->get_node_base_interface(), fut),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(fut.get().successful);

  // Place a lethal cell at the first simulated pose
  ASSERT_TRUE(costmap->worldToMap(5.05, 5.0, mx, my));
  costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(
    controller->computeVelocityCommands(
      robot_pose, robot_velocity, &checker, transformed_global_plan, goal),
    nav2_core::NoValidControl);
}

TEST(GracefulControllerTest, computeVelocityCommandApproachFallbackUsesPlanHeading) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGracefulApproachFallback");
  auto tf = nav2::create_transform_buffer(node);

  nav2::declare_parameter_if_not_declared(
    node, "test.initial_rotation", rclcpp::ParameterValue(false));

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.05));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Small obstacle on the straight line to the goal: it blocks the approach shortcut, which
  // ends facing the goal, but not the curve needed to end with the plan's heading
  auto costmap = costmap_ros->getCostmap();
  unsigned int mx, my;
  for (double x = 5.2; x <= 5.3; x += 0.05) {
    for (double y = 5.1; y <= 5.2; y += 0.05) {
      ASSERT_TRUE(costmap->worldToMap(x, y, mx, my));
      costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_global_frame";
  robot_pose.pose.position.x = 5.0;
  robot_pose.pose.position.y = 5.0;
  robot_pose.pose.orientation.w = 1.0;

  // The goal is close, so the approach shortcut is tried first. The heading of the plan
  // (1.5 rad) differs a lot from the heading towards the goal (atan2(0.3, 0.5))
  nav_msgs::msg::Path transformed_global_plan;
  transformed_global_plan.header.frame_id = "test_robot_frame";
  transformed_global_plan.poses.resize(1);
  transformed_global_plan.poses[0].header = transformed_global_plan.header;
  transformed_global_plan.poses[0].pose.position.x = 0.5;
  transformed_global_plan.poses[0].pose.position.y = 0.3;
  transformed_global_plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 1.5));
  controller->newPathReceived(transformed_global_plan);

  geometry_msgs::msg::PoseStamped goal;
  goal.header = robot_pose.header;
  goal.pose.position.x = 5.5;
  goal.pose.position.y = 5.3;
  goal.pose.orientation = transformed_global_plan.poses[0].pose.orientation;
  geometry_msgs::msg::Twist robot_velocity;
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  // The fallback must try the plan's own heading instead of repeating the blocked shortcut
  geometry_msgs::msg::TwistStamped cmd_vel;
  EXPECT_NO_THROW(
    cmd_vel = controller->computeVelocityCommands(
      robot_pose, robot_velocity, &checker, transformed_global_plan, goal));
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
}

/**
 * @brief Outcome of one call to validateTargetPoseOnApproach.
 */
struct ApproachOutcome
{
  /** @brief Return value of validateTargetPoseOnApproach. */
  bool valid;
  /** @brief Yaw of the target pose before the call. */
  double yaw_before;
  /** @brief Yaw of the target pose after the call. */
  double yaw_after;
};

/**
 * @brief Call validateTargetPoseOnApproach on a target close to the goal.
 *
 * @param target_x X of the target in the robot frame.
 * @param target_y Y of the target in the robot frame.
 * @param allow_backward Whether the controller may drive backwards.
 * @param blocked Whether a lethal wall is placed between the robot and the target.
 * @return The return value and the yaw of the target pose before and after the call.
 */
ApproachOutcome runApproachValidation(
  double target_x, double target_y, bool allow_backward, bool blocked)
{
  auto node = std::make_shared<nav2::LifecycleNode>("testGraceful");
  auto tf = nav2::create_transform_buffer(node);

  nav2::declare_parameter_if_not_declared(
    node, "test.initial_rotation", rclcpp::ParameterValue(false));
  nav2::declare_parameter_if_not_declared(
    node, "test.prefer_final_rotation", rclcpp::ParameterValue(true));
  nav2::declare_parameter_if_not_declared(
    node, "test.allow_backward", rclcpp::ParameterValue(allow_backward));

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Robot at (5, 5) facing +x in the costmap frame
  geometry_msgs::msg::TransformStamped costmap_transform;
  costmap_transform.header.frame_id = "test_global_frame";
  costmap_transform.child_frame_id = "test_robot_frame";
  costmap_transform.transform.translation.x = 5.0;
  costmap_transform.transform.translation.y = 5.0;
  costmap_transform.transform.rotation.w = 1.0;

  if (blocked) {
    // Wall across the way to the target, wide enough to stop any heading
    auto costmap = costmap_ros->getCostmap();
    unsigned int mx, my;
    double wall_x = 5.0 + 0.6 * target_x;
    for (double wall_y = 4.0; wall_y <= 6.5; wall_y += 0.05) {
      costmap->worldToMap(wall_x, wall_y, mx, my);
      costmap->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  // The plan's own heading (0.2 rad) differs from the heading towards the target
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "test_robot_frame";
  target_pose.pose.position.x = target_x;
  target_pose.pose.position.y = target_y;
  target_pose.pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.2));

  ApproachOutcome outcome;
  outcome.yaw_before = tf2::getYaw(target_pose.pose.orientation);

  nav_msgs::msg::Path trajectory;
  geometry_msgs::msg::TwistStamped cmd_vel;
  double dist_to_target = std::hypot(target_x, target_y);
  outcome.valid = controller->validateTargetPoseOnApproach(
    target_pose, dist_to_target, dist_to_target, trajectory, costmap_transform, cmd_vel);
  outcome.yaw_after = tf2::getYaw(target_pose.pose.orientation);
  return outcome;
}

TEST(GracefulControllerTest, validateTargetPoseOnApproachKeepsPoseWhenBlocked) {
  auto outcome = runApproachValidation(0.5, 0.3, false, true);

  // The target is blocked, so the approach check fails and the caller falls back to the
  // original heading of the plan: the pose must not carry the shortcut heading
  EXPECT_FALSE(outcome.valid);
  EXPECT_NEAR(outcome.yaw_after, outcome.yaw_before, 1e-6);
}

TEST(GracefulControllerTest, validateTargetPoseOnApproachKeepsPoseWhenBlockedBackwards) {
  auto outcome = runApproachValidation(-0.5, 0.3, true, true);

  // A failed attempt must not leave the heading flipped by PI for the reversing fallback
  EXPECT_FALSE(outcome.valid);
  EXPECT_NEAR(outcome.yaw_after, outcome.yaw_before, 1e-6);
}

TEST(GracefulControllerTest, validateTargetPoseOnApproachWritesHeadingOnSuccess) {
  auto outcome = runApproachValidation(0.5, 0.3, false, false);

  // On success the pose carries the shortcut heading, pointing at the target
  EXPECT_TRUE(outcome.valid);
  EXPECT_NEAR(outcome.yaw_after, std::atan2(0.3, 0.5), 1e-6);
}

TEST(GracefulControllerTest, computeVelocityCommandUnsafeApproachDoesNotHideSafeOnes) {
  auto node = std::make_shared<nav2::LifecycleNode>("testGracefulUnsafeApproach");
  auto tf = nav2::create_transform_buffer(node);

  // Create a costmap of 10x10 meters with a polygonal footprint
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap_ros->declare_parameter("global_frame", rclcpp::ParameterValue("test_global_frame"));
  costmap_ros->declare_parameter("robot_base_frame", rclcpp::ParameterValue("test_robot_frame"));
  costmap_ros->declare_parameter("width", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("height", rclcpp::ParameterValue(10));
  costmap_ros->declare_parameter("resolution", rclcpp::ParameterValue(0.05));
  costmap_ros->declare_parameter(
    "footprint", rclcpp::ParameterValue("[[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [-0.2, 0.2]]"));
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Small slowdown radius so the simulated trajectories reach their targets
  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(), node->get_node_services_interface());
  auto fut = params->set_parameters_atomically(
    {rclcpp::Parameter("test.prefer_final_rotation", true),
      rclcpp::Parameter("test.slowdown_radius", 0.1)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), fut);

  // A single inscribed cell in the straight approach: valid for a polygonal footprint,
  // but unsafe. It makes the straight approach (the fastest one) risky and unsafe.
  auto costmap = costmap_ros->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(costmap->worldToMap(5.4, 5.0, mx, my));
  costmap->setCost(mx, my, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  // Robot in the middle of the costmap
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_global_frame";
  robot_pose.pose.position.x = 5.0;
  robot_pose.pose.position.y = 5.0;
  robot_pose.pose.orientation.w = 1.0;

  // Straight plan of 0.8 m in the robot frame, shorter than max_lookahead (approaching goal)
  nav_msgs::msg::Path transformed_global_plan;
  transformed_global_plan.header.frame_id = "test_robot_frame";
  transformed_global_plan.poses.resize(9);
  for (size_t i = 0; i < transformed_global_plan.poses.size(); ++i) {
    transformed_global_plan.poses[i].header = transformed_global_plan.header;
    transformed_global_plan.poses[i].pose.position.x = 0.1 * static_cast<double>(i);
    transformed_global_plan.poses[i].pose.orientation.w = 1.0;
  }
  controller->newPathReceived(transformed_global_plan);

  geometry_msgs::msg::PoseStamped goal;
  goal.header = robot_pose.header;
  goal.pose.position.x = 5.8;
  goal.pose.position.y = 5.0;
  goal.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist robot_velocity;
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  auto cmd_vel = controller->computeVelocityCommands(
    robot_pose, robot_velocity, &checker, transformed_global_plan, goal);

  // The straight approach is discarded and a safe curved one is selected
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
  EXPECT_GT(std::abs(cmd_vel.twist.angular.z), 1e-3);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
