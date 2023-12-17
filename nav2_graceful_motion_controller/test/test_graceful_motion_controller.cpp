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
#include "nav2_graceful_motion_controller/ego_polar_coords.hpp"
#include "nav2_graceful_motion_controller/smooth_control_law.hpp"
#include "nav2_graceful_motion_controller/graceful_motion_controller.hpp"

class SCLFixture : public nav2_graceful_motion_controller::SmoothControlLaw
{
public:
  SCLFixture(
    double k_phi, double k_delta, double beta, double lambda,
    double slowdown_radius, double v_linear_min, double v_linear_max, double v_angular_max)
  : nav2_graceful_motion_controller::SmoothControlLaw(k_phi, k_delta, beta, lambda,
      slowdown_radius, v_linear_min, v_linear_max, v_angular_max) {}

  double getSpeedLinearMin() {return v_linear_min_;}
  double getSpeedLinearMax() {return v_linear_max_;}
  double getSpeedAngularMax() {return v_angular_max_;}
  double calculateCurvature(geometry_msgs::msg::Pose target, geometry_msgs::msg::Pose current)
  {
    auto ego_coords = nav2_graceful_motion_controller::EgocentricPolarCoordinates(target, current);
    return nav2_graceful_motion_controller::SmoothControlLaw::calculateCurvature(
      ego_coords.r, ego_coords.phi, ego_coords.delta);
  }
};

class GMControllerFixture : public nav2_graceful_motion_controller::GracefulMotionController
{
public:
  GMControllerFixture()
  : nav2_graceful_motion_controller::GracefulMotionController() {}

  nav_msgs::msg::Path getPlan() {return path_handler_->getPlan();}

  geometry_msgs::msg::PoseStamped getMotionTarget(
    const double & motion_target_distance, const nav_msgs::msg::Path & plan)
  {
    return nav2_graceful_motion_controller::GracefulMotionController::getMotionTarget(
      motion_target_distance, plan);
  }

  geometry_msgs::msg::PointStamped createMotionTargetMsg(
    const geometry_msgs::msg::PoseStamped & motion_target)
  {
    return nav2_graceful_motion_controller::GracefulMotionController::createMotionTargetMsg(
      motion_target);
  }

  visualization_msgs::msg::Marker createSlowdownMsg(
    const geometry_msgs::msg::PoseStamped & motion_target)
  {
    return nav2_graceful_motion_controller::GracefulMotionController::createSlowdownMsg(
      motion_target);
  }

  geometry_msgs::msg::Twist rotateToTarget(const double & angle_to_target)
  {
    return nav2_graceful_motion_controller::GracefulMotionController::rotateToTarget(
      angle_to_target);
  }

  double getSpeedLinearMax() {return params_->v_linear_max;}
};

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

  // Check results
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

  // Check results
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
      rclcpp::Parameter("test.slowdown_radius", 11.0),
      rclcpp::Parameter("test.initial_rotation", false),
      rclcpp::Parameter("test.initial_rotation_min_angle", 12.0),
      rclcpp::Parameter("test.final_rotation", false)});

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
  EXPECT_EQ(node->get_parameter("test.initial_rotation").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.initial_rotation_min_angle").as_double(), 12.0);
  EXPECT_EQ(node->get_parameter("test.final_rotation").as_bool(), false);

  // Set max search distant to negative so it warns
  results = params->set_parameters_atomically(
    {rclcpp::Parameter("test.max_robot_pose_search_dist", -1.0)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);
}

TEST(GracefulMotionControllerTest, getDifferentMotionTargets) {
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

  // Check results, should be the last one
  EXPECT_EQ(motion_target.header.frame_id, "map");
  EXPECT_EQ(motion_target.pose.position.x, 5.0);
  EXPECT_EQ(motion_target.pose.position.y, 6.0);
  EXPECT_EQ(motion_target.pose.orientation, tf2::toMsg(tf2::Quaternion({0, 0, 1}, 0.0)));
}

TEST(GracefulMotionControllerTest, createMotionTargetMsg) {
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

TEST(GracefulMotionControllerTest, createSlowdownMsg) {
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
  auto slowdown_msg = controller->createSlowdownMsg(motion_target);

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

TEST(GracefulMotionControllerTest, rotateToTarget) {
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
    {rclcpp::Parameter("test.v_angular_max", 1.0)});
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Set angle to target and get velocity command
  double angle_to_target = 0.5;
  auto cmd_vel = controller->rotateToTarget(angle_to_target);

  // Check results
  EXPECT_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.angular.z, 1.0);

  // Set a new angle to target
  angle_to_target = -0.5;
  cmd_vel = controller->rotateToTarget(angle_to_target);

  // Check results with negative sign
  EXPECT_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.angular.z, -1.0);
}

TEST(GracefulMotionControllerTest, setSpeedLimit) {
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool success = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return success;
}
