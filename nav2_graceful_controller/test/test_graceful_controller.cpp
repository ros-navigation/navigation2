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
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_graceful_controller/ego_polar_coords.hpp"
#include "nav2_graceful_controller/smooth_control_law.hpp"
#include "nav2_graceful_controller/graceful_controller.hpp"

class SCLFixture : public nav2_graceful_controller::SmoothControlLaw
{
public:
  SCLFixture(
    double k_phi, double k_delta, double beta, double lambda,
    double slowdown_radius, double v_linear_min, double v_linear_max, double v_angular_max)
  : nav2_graceful_controller::SmoothControlLaw(k_phi, k_delta, beta, lambda,
      slowdown_radius, v_linear_min, v_linear_max, v_angular_max) {}

  double getCurvatureKPhi() {return k_phi_;}
  double getCurvatureKDelta() {return k_delta_;}
  double getCurvatureBeta() {return beta_;}
  double getCurvatureLambda() {return lambda_;}
  double getSlowdownRadius() {return slowdown_radius_;}
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

  nav_msgs::msg::Path getPlan() {return path_handler_->getPlan();}

  geometry_msgs::msg::PoseStamped getMotionTarget(
    const double & motion_target_distance, const nav_msgs::msg::Path & plan)
  {
    return nav2_graceful_controller::GracefulController::getMotionTarget(
      motion_target_distance, plan);
  }

  geometry_msgs::msg::PointStamped createMotionTargetMsg(
    const geometry_msgs::msg::PoseStamped & motion_target)
  {
    return nav2_graceful_controller::createMotionTargetMsg(motion_target);
  }

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

  bool simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & motion_target,
    const geometry_msgs::msg::TransformStamped & costmap_transform,
    nav_msgs::msg::Path & trajectory, const bool & backward)
  {
    return nav2_graceful_controller::GracefulController::simulateTrajectory(
      robot_pose, motion_target, costmap_transform, trajectory, backward);
  }

  double getSpeedLinearMax() {return params_->v_linear_max;}

  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    return path_handler_->transformGlobalPlan(pose, params_->max_robot_pose_search_dist);
  }
};

TEST(SmoothControlLawTest, setCurvatureConstants) {
  // Initialize SmoothControlLaw
  SCLFixture scl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0);

  // Set curvature constants
  scl.setCurvatureConstants(1.0, 2.0, 3.0, 4.0);

  // Set slowdown radius
  scl.setSlowdownRadius(5.0);

  // Check results
  EXPECT_EQ(scl.getCurvatureKPhi(), 1.0);
  EXPECT_EQ(scl.getCurvatureKDelta(), 2.0);
  EXPECT_EQ(scl.getCurvatureBeta(), 3.0);
  EXPECT_EQ(scl.getCurvatureLambda(), 4.0);
  EXPECT_EQ(scl.getSlowdownRadius(), 5.0);
}

TEST(SmoothControlLawTest, setSpeedLimits) {
  // Initialize SmoothControlLaw
  SCLFixture scl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Set speed limits
  scl.setSpeedLimit(1.0, 2.0, 3.0);

  // Check results
  EXPECT_EQ(scl.getSpeedLinearMin(), 1.0);
  EXPECT_EQ(scl.getSpeedLinearMax(), 2.0);
  EXPECT_EQ(scl.getSpeedAngularMax(), 3.0);
}

TEST(SmoothControlLawTest, calculateCurvature) {
  // Initialize SmoothControlLaw
  SCLFixture scl(1.0, 10.0, 0.2, 2.0, 0.1, 0.0, 1.0, 1.0);

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
  SCLFixture scl(1.0, 10.0, 0.2, 2.0, 0.1, 0.0, 1.0, 1.0);

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
  SCLFixture scl(1.0, 10.0, 0.2, 2.0, 0.1, 0.0, 1.0, 1.0);

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

TEST(GracefulControllerTest, configure) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Set the plan
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  plan.poses.resize(3);
  plan.poses[0].header.frame_id = "map";
  controller->setPlan(plan);
  EXPECT_EQ(controller->getPlan().poses.size(), 3u);
  EXPECT_EQ(controller->getPlan().poses[0].header.frame_id, "map");

  // Cleaning up
  controller->deactivate();
  controller->cleanup();
}

TEST(GracefulControllerTest, dynamicParameters) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

  // Set max search distant to negative so it warns
  nav2_util::declare_parameter_if_not_declared(
    node, "test.max_robot_pose_search_dist", rclcpp::ParameterValue(-2.0));

  // Set initial rotation and allow backward to true so it warns and allow backward is false
  nav2_util::declare_parameter_if_not_declared(
    node, "test.initial_rotation", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, "test.allow_backward", rclcpp::ParameterValue(true));

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
    {rclcpp::Parameter("test.transform_tolerance", 1.0),
      rclcpp::Parameter("test.motion_target_dist", 2.0),
      rclcpp::Parameter("test.k_phi", 4.0),
      rclcpp::Parameter("test.k_delta", 5.0),
      rclcpp::Parameter("test.beta", 6.0),
      rclcpp::Parameter("test.lambda", 7.0),
      rclcpp::Parameter("test.v_linear_min", 8.0),
      rclcpp::Parameter("test.v_linear_max", 9.0),
      rclcpp::Parameter("test.v_angular_max", 10.0),
      rclcpp::Parameter("test.slowdown_radius", 11.0),
      rclcpp::Parameter("test.initial_rotation", false),
      rclcpp::Parameter("test.initial_rotation_min_angle", 12.0),
      rclcpp::Parameter("test.final_rotation", false),
      rclcpp::Parameter("test.rotation_scaling_factor", 13.0),
      rclcpp::Parameter("test.allow_backward", true)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("test.transform_tolerance").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("test.motion_target_dist").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("test.k_phi").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("test.k_delta").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("test.beta").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("test.lambda").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.v_linear_min").as_double(), 8.0);
  EXPECT_EQ(node->get_parameter("test.v_linear_max").as_double(), 9.0);
  EXPECT_EQ(node->get_parameter("test.v_angular_max").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("test.slowdown_radius").as_double(), 11.0);
  EXPECT_EQ(node->get_parameter("test.initial_rotation").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.initial_rotation_min_angle").as_double(), 12.0);
  EXPECT_EQ(node->get_parameter("test.final_rotation").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.rotation_scaling_factor").as_double(), 13.0);
  EXPECT_EQ(node->get_parameter("test.allow_backward").as_bool(), true);

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

TEST(GracefulControllerTest, getDifferentMotionTargets) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Set the plan
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  plan.poses.resize(3);
  plan.poses[0].header.frame_id = "map";
  plan.poses[0].pose.position.x = 1.0;
  plan.poses[0].pose.position.y = 2.0;
  plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[1].header.frame_id = "map";
  plan.poses[1].pose.position.x = 3.0;
  plan.poses[1].pose.position.y = 4.0;
  plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[2].header.frame_id = "map";
  plan.poses[2].pose.position.x = 5.0;
  plan.poses[2].pose.position.y = 6.0;
  plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  controller->setPlan(plan);

  // Set distance and get motion target
  double motion_target_distance = 3.5;
  auto motion_target = controller->getMotionTarget(motion_target_distance, plan);

  // Check results, should be the second one
  EXPECT_EQ(motion_target.header.frame_id, "map");
  EXPECT_EQ(motion_target.pose.position.x, 3.0);
  EXPECT_EQ(motion_target.pose.position.y, 4.0);
  EXPECT_EQ(motion_target.pose.orientation, tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0)));

  // Set a new distance greater than the path length and get motion target
  motion_target_distance = 10.0;
  motion_target = controller->getMotionTarget(motion_target_distance, plan);

  // Check results: should be the last one
  EXPECT_EQ(motion_target.header.frame_id, "map");
  EXPECT_EQ(motion_target.pose.position.x, 5.0);
  EXPECT_EQ(motion_target.pose.position.y, 6.0);
  EXPECT_EQ(motion_target.pose.orientation, tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0)));
}

TEST(GracefulControllerTest, createMotionTargetMsg) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
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

  // Create motion target message
  auto motion_target_msg = controller->createMotionTargetMsg(motion_target);

  // Check results
  EXPECT_EQ(motion_target_msg.header.frame_id, "map");
  EXPECT_EQ(motion_target_msg.point.x, 1.0);
  EXPECT_EQ(motion_target_msg.point.y, 2.0);
  EXPECT_EQ(motion_target_msg.point.z, 0.01);
}

TEST(GracefulControllerTest, createSlowdownMsg) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
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
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
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
}

TEST(GracefulControllerTest, setSpeedLimit) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
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

TEST(GracefulControllerTest, emptyPlan) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  auto results = costmap_ros->set_parameters(
    {rclcpp::Parameter("global_frame", "test_global_frame"),
      rclcpp::Parameter("robot_base_frame", "test_robot_frame"),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 10),
      rclcpp::Parameter("resolution", 0.1)});
  for (const auto & result : results) {
    EXPECT_TRUE(result.successful) << result.reason;
  }
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Set max search distant
  nav2_util::declare_parameter_if_not_declared(
    node, "test.max_robot_pose_search_dist", rclcpp::ParameterValue(5.0));

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_robot_frame";
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = robot_pose.pose.position.x;
  global_to_robot.transform.translation.y = robot_pose.pose.position.y;
  global_to_robot.transform.translation.z = robot_pose.pose.position.z;
  tf->setTransform(global_to_robot, "test", false);

  // Set an empty global plan
  nav_msgs::msg::Path global_plan;
  global_plan.header.frame_id = "test_global_frame";
  controller->setPlan(global_plan);

  EXPECT_THROW(controller->transformGlobalPlan(robot_pose), nav2_core::PlannerException);
}

TEST(GracefulControllerTest, poseOutsideCostmap) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  auto results = costmap_ros->set_parameters(
    {rclcpp::Parameter("global_frame", "test_global_frame"),
      rclcpp::Parameter("robot_base_frame", "test_robot_frame"),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 10),
      rclcpp::Parameter("resolution", 0.1)});
  for (const auto & result : results) {
    EXPECT_TRUE(result.successful) << result.reason;
  }
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Set max search distant
  nav2_util::declare_parameter_if_not_declared(
    node, "test.max_robot_pose_search_dist", rclcpp::ParameterValue(5.0));

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_robot_frame";
  robot_pose.pose.position.x = 500.0;
  robot_pose.pose.position.y = 500.0;
  robot_pose.pose.position.z = 0.0;

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = robot_pose.pose.position.x;
  global_to_robot.transform.translation.y = robot_pose.pose.position.y;
  global_to_robot.transform.translation.z = robot_pose.pose.position.z;
  tf->setTransform(global_to_robot, "test", false);

  // Set an empty global plan
  nav_msgs::msg::Path global_plan;
  global_plan.header.frame_id = "test_global_frame";
  global_plan.poses.resize(1);
  global_plan.poses[0].header.frame_id = "test_global_frame";
  global_plan.poses[0].pose.position.x = 0.0;
  global_plan.poses[0].pose.position.y = 0.0;
  controller->setPlan(global_plan);

  EXPECT_THROW(controller->transformGlobalPlan(robot_pose), nav2_core::PlannerException);
}

TEST(GracefulControllerTest, noPruningPlan) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  auto results = costmap_ros->set_parameters(
    {rclcpp::Parameter("global_frame", "test_global_frame"),
      rclcpp::Parameter("robot_base_frame", "test_robot_frame"),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 10),
      rclcpp::Parameter("resolution", 0.1)});
  for (const auto & result : results) {
    EXPECT_TRUE(result.successful) << result.reason;
  }
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Set max search distant
  nav2_util::declare_parameter_if_not_declared(
    node, "test.max_robot_pose_search_dist", rclcpp::ParameterValue(5.0));

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_robot_frame";
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = robot_pose.pose.position.x;
  global_to_robot.transform.translation.y = robot_pose.pose.position.y;
  global_to_robot.transform.translation.z = robot_pose.pose.position.z;
  tf->setTransform(global_to_robot, "test", false);

  // Set a linear global plan
  nav_msgs::msg::Path global_plan;
  global_plan.header.frame_id = "test_global_frame";
  global_plan.poses.resize(3);
  global_plan.poses[0].header.frame_id = "test_global_frame";
  global_plan.poses[0].pose.position.x = 0.0;
  global_plan.poses[0].pose.position.y = 0.0;
  global_plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  global_plan.poses[1].header.frame_id = "test_global_frame";
  global_plan.poses[1].pose.position.x = 1.0;
  global_plan.poses[1].pose.position.y = 1.0;
  global_plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  global_plan.poses[2].header.frame_id = "test_global_frame";
  global_plan.poses[2].pose.position.x = 3.0;
  global_plan.poses[2].pose.position.y = 3.0;
  global_plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  controller->setPlan(global_plan);

  // Check results: the plan should not be pruned
  auto transformed_plan = controller->transformGlobalPlan(robot_pose);
  EXPECT_EQ(transformed_plan.poses.size(), global_plan.poses.size());
}

TEST(GracefulControllerTest, pruningPlan) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create a costmap of 20x20 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  auto results = costmap_ros->set_parameters(
    {rclcpp::Parameter("global_frame", "test_global_frame"),
      rclcpp::Parameter("robot_base_frame", "test_robot_frame"),
      rclcpp::Parameter("width", 20),
      rclcpp::Parameter("height", 20),
      rclcpp::Parameter("resolution", 0.1)});
  for (const auto & result : results) {
    EXPECT_TRUE(result.successful) << result.reason;
  }
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Set max search distant
  nav2_util::declare_parameter_if_not_declared(
    node, "test.max_robot_pose_search_dist", rclcpp::ParameterValue(9.0));

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_robot_frame";
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = robot_pose.pose.position.x;
  global_to_robot.transform.translation.y = robot_pose.pose.position.y;
  global_to_robot.transform.translation.z = robot_pose.pose.position.z;
  tf->setTransform(global_to_robot, "test", false);

  // Set a linear global plan
  nav_msgs::msg::Path global_plan;
  global_plan.header.frame_id = "test_global_frame";
  global_plan.poses.resize(6);
  global_plan.poses[0].header.frame_id = "test_global_frame";
  global_plan.poses[0].pose.position.x = 0.0;
  global_plan.poses[0].pose.position.y = 0.0;
  global_plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  global_plan.poses[1].header.frame_id = "test_global_frame";
  global_plan.poses[1].pose.position.x = 3.0;
  global_plan.poses[1].pose.position.y = 3.0;
  global_plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  global_plan.poses[2].header.frame_id = "test_global_frame";
  global_plan.poses[2].pose.position.x = 5.0;
  global_plan.poses[2].pose.position.y = 5.0;
  global_plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  global_plan.poses[3].header.frame_id = "test_global_frame";
  global_plan.poses[3].pose.position.x = 10.0;
  global_plan.poses[3].pose.position.y = 10.0;
  global_plan.poses[3].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  global_plan.poses[4].header.frame_id = "test_global_frame";
  global_plan.poses[4].pose.position.x = 20.0;
  global_plan.poses[4].pose.position.y = 20.0;
  global_plan.poses[4].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  global_plan.poses[5].pose.position.x = 500.0;
  global_plan.poses[5].pose.position.y = 500.0;
  global_plan.poses[5].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  controller->setPlan(global_plan);

  // Check results: the plan should be pruned
  auto transformed_plan = controller->transformGlobalPlan(robot_pose);
  EXPECT_EQ(transformed_plan.poses.size(), 3u);
}

TEST(GracefulControllerTest, pruningPlanOutsideCostmap) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  auto results = costmap_ros->set_parameters(
    {rclcpp::Parameter("global_frame", "test_global_frame"),
      rclcpp::Parameter("robot_base_frame", "test_robot_frame"),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 10),
      rclcpp::Parameter("resolution", 0.1)});
  for (const auto & result : results) {
    EXPECT_TRUE(result.successful) << result.reason;
  }
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Set max search distant
  nav2_util::declare_parameter_if_not_declared(
    node, "test.max_robot_pose_search_dist", rclcpp::ParameterValue(15.0));

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

  // Create the robot pose
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "test_robot_frame";
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;

  // Set transform between global and robot frame
  geometry_msgs::msg::TransformStamped global_to_robot;
  global_to_robot.header.frame_id = "test_global_frame";
  global_to_robot.header.stamp = node->get_clock()->now();
  global_to_robot.child_frame_id = "test_robot_frame";
  global_to_robot.transform.translation.x = robot_pose.pose.position.x;
  global_to_robot.transform.translation.y = robot_pose.pose.position.y;
  global_to_robot.transform.translation.z = robot_pose.pose.position.z;
  tf->setTransform(global_to_robot, "test", false);

  // Set a linear global plan
  nav_msgs::msg::Path global_plan;
  global_plan.header.frame_id = "test_global_frame";
  global_plan.poses.resize(3);
  global_plan.poses[0].header.frame_id = "test_global_frame";
  global_plan.poses[0].pose.position.x = 0.0;
  global_plan.poses[0].pose.position.y = 0.0;
  global_plan.poses[0].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  global_plan.poses[1].header.frame_id = "test_global_frame";
  global_plan.poses[1].pose.position.x = 3.0;
  global_plan.poses[1].pose.position.y = 3.0;
  global_plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  global_plan.poses[2].header.frame_id = "test_global_frame";
  global_plan.poses[2].pose.position.x = 200.0;
  global_plan.poses[2].pose.position.y = 200.0;
  global_plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  controller->setPlan(global_plan);

  // Check results: the plan should be pruned
  auto transformed_plan = controller->transformGlobalPlan(robot_pose);
  EXPECT_EQ(transformed_plan.poses.size(), 2u);
}

TEST(GracefulControllerTest, computeVelocityCommandRotate) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  nav2_util::declare_parameter_if_not_declared(
    node, "test.v_angular_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "test.rotation_scaling_factor", rclcpp::ParameterValue(0.5));

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  auto results = costmap_ros->set_parameters(
    {rclcpp::Parameter("global_frame", "test_global_frame"),
      rclcpp::Parameter("robot_base_frame", "test_robot_frame"),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 10),
      rclcpp::Parameter("resolution", 0.1)});
  for (const auto & result : results) {
    EXPECT_TRUE(result.successful) << result.reason;
  }
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

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
  plan.poses[1].pose.position.x = 1.0;
  plan.poses[1].pose.position.y = 1.0;
  plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[2].header.frame_id = "test_global_frame";
  plan.poses[2].pose.position.x = 2.0;
  plan.poses[2].pose.position.y = 2.0;
  plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  controller->setPlan(plan);

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;

  // Set the goal checker
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  auto cmd_vel = controller->computeVelocityCommands(robot_pose, robot_velocity, &checker);

  // Check results: the robot should rotate in place.
  // So, linear velocity should be zero and angular velocity should be a positive value below 0.5.
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_GE(cmd_vel.twist.angular.x, 0.0);
  EXPECT_LE(cmd_vel.twist.angular.x, 0.5);
}

TEST(GracefulControllerTest, computeVelocityCommandRegular) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  auto results = costmap_ros->set_parameters(
    {rclcpp::Parameter("global_frame", "test_global_frame"),
      rclcpp::Parameter("robot_base_frame", "test_robot_frame"),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 10),
      rclcpp::Parameter("resolution", 0.1)});
  for (const auto & result : results) {
    EXPECT_TRUE(result.successful) << result.reason;
  }
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

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
  controller->setPlan(plan);

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;

  // Set the goal checker
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  auto cmd_vel = controller->computeVelocityCommands(robot_pose, robot_velocity, &checker);

  // Check results: the robot should go straight to the target.
  // So, linear velocity should be some positive value and angular velocity should be zero.
  EXPECT_GT(cmd_vel.twist.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.twist.angular.z, 0.0);
}

TEST(GracefulControllerTest, computeVelocityCommandRegularBackwards) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Set initial rotation false and allow backward to true
  nav2_util::declare_parameter_if_not_declared(
    node, "test.initial_rotation", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, "test.allow_backward", rclcpp::ParameterValue(true));

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  auto results = costmap_ros->set_parameters(
    {rclcpp::Parameter("global_frame", "test_global_frame"),
      rclcpp::Parameter("robot_base_frame", "test_robot_frame"),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 10),
      rclcpp::Parameter("resolution", 0.1)});
  for (const auto & result : results) {
    EXPECT_TRUE(result.successful) << result.reason;
  }
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

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
  plan.poses[1].pose.position.x = -1.0;
  plan.poses[1].pose.position.y = 0.0;
  plan.poses[1].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  plan.poses[2].header.frame_id = "test_global_frame";
  plan.poses[2].pose.position.x = -2.0;
  plan.poses[2].pose.position.y = 0.0;
  plan.poses[2].pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0));
  controller->setPlan(plan);

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;

  // Set the goal checker
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  auto cmd_vel = controller->computeVelocityCommands(robot_pose, robot_velocity, &checker);

  // Check results: the robot should go straight to the target.
  // So, both linear velocity should be some negative values.
  EXPECT_LT(cmd_vel.twist.linear.x, 0.0);
  EXPECT_LT(cmd_vel.twist.angular.z, 0.0);
}

TEST(GracefulControllerTest, computeVelocityCommandFinal) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testGraceful");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create a costmap of 10x10 meters
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  auto results = costmap_ros->set_parameters(
    {rclcpp::Parameter("global_frame", "test_global_frame"),
      rclcpp::Parameter("robot_base_frame", "test_robot_frame"),
      rclcpp::Parameter("width", 10),
      rclcpp::Parameter("height", 10),
      rclcpp::Parameter("resolution", 0.1)});
  for (const auto & result : results) {
    EXPECT_TRUE(result.successful) << result.reason;
  }
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  // Create controller
  auto controller = std::make_shared<GMControllerFixture>();
  controller->configure(node, "test", tf, costmap_ros);
  controller->activate();

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
  controller->setPlan(plan);

  // Set velocity
  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;

  // Set the goal checker
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap_ros);

  auto cmd_vel = controller->computeVelocityCommands(robot_pose, robot_velocity, &checker);

  // Check results: the robot should do a final rotation near the target.
  // So, linear velocity should be zero and angular velocity should be a positive value below 0.5.
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_GE(cmd_vel.twist.angular.x, 0.0);
  EXPECT_LE(cmd_vel.twist.angular.x, 0.5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
