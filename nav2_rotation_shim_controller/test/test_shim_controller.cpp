// Copyright (c) 2021 Samsung Research America
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "tf2_ros/transform_broadcaster.h"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class RotationShimShim : public nav2_rotation_shim_controller::RotationShimController
{
public:
  RotationShimShim()
  : nav2_rotation_shim_controller::RotationShimController()
  {
  }

  nav2_core::Controller::Ptr getPrimaryController()
  {
    return primary_controller_;
  }

  nav_msgs::msg::Path getPath()
  {
    return current_path_;
  }

  bool isPathUpdated()
  {
    return path_updated_;
  }

  geometry_msgs::msg::PoseStamped getSampledPathPtWrapper()
  {
    return getSampledPathPt();
  }

  bool isGoalChangedWrapper(const nav_msgs::msg::Path & path)
  {
    return isGoalChanged(path);
  }

  geometry_msgs::msg::Pose transformPoseToBaseFrameWrapper(geometry_msgs::msg::PoseStamped pt)
  {
    return transformPoseToBaseFrame(pt);
  }

  geometry_msgs::msg::TwistStamped
  computeRotateToHeadingCommandWrapper(
    const double & param,
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity)
  {
    return computeRotateToHeadingCommand(param, pose, velocity);
  }
};

TEST(RotationShimControllerTest, lifecycleTransitions)
{
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  // Should not populate primary controller, does not exist
  EXPECT_THROW(ctrl->configure(node, name, tf, costmap), std::runtime_error);
  EXPECT_EQ(ctrl->getPrimaryController(), nullptr);

  // Add a controller to the setup
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());
  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter(
        "PathFollower.primary_controller",
        std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"))});
  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  ctrl->configure(node, name, tf, costmap);
  EXPECT_NE(ctrl->getPrimaryController(), nullptr);

  ctrl->activate();

  ctrl->setSpeedLimit(50.0, true);

  ctrl->deactivate();
  ctrl->cleanup();
}

TEST(RotationShimControllerTest, setPlanAndSampledPointsTests)
{
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "hi mate!";
  path.poses.resize(10);
  path.poses[1].pose.position.x = 0.1;
  path.poses[1].pose.position.y = 0.1;
  path.poses[2].pose.position.x = 1.0;
  path.poses[2].pose.position.y = 1.0;
  path.poses[3].pose.position.x = 10.0;
  path.poses[3].pose.position.y = 10.0;
  EXPECT_EQ(controller->isPathUpdated(), false);
  controller->setPlan(path);
  EXPECT_EQ(controller->getPath().header.frame_id, std::string("hi mate!"));
  EXPECT_EQ(controller->getPath().poses.size(), 10u);
  EXPECT_EQ(controller->isPathUpdated(), true);

  // Test getting a sampled point
  auto pose = controller->getSampledPathPtWrapper();
  EXPECT_EQ(pose.pose.position.x, 1.0);  // default forward sampling is 0.5
  EXPECT_EQ(pose.pose.position.y, 1.0);

  nav_msgs::msg::Path path_invalid_leng;
  controller->setPlan(path_invalid_leng);
  EXPECT_THROW(controller->getSampledPathPtWrapper(), std::runtime_error);

  nav_msgs::msg::Path path_invalid_dists;
  path.poses.resize(10);
  controller->setPlan(path_invalid_dists);
  EXPECT_THROW(controller->getSampledPathPtWrapper(), std::runtime_error);
}

TEST(RotationShimControllerTest, rotationAndTransformTests)
{
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  costmap->configure();

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));

  node->declare_parameter("controller_frequency", 1.0);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "fake_frame";
  path.poses.resize(10);
  path.poses[1].pose.position.x = 0.15;
  path.poses[1].pose.position.y = 0.15;
  path.poses[2].pose.position.x = 1.0;
  path.poses[2].pose.position.y = 1.0;
  path.poses[3].pose.position.x = 10.0;
  path.poses[3].pose.position.y = 10.0;
  controller->setPlan(path);

  const geometry_msgs::msg::Twist velocity;
  EXPECT_EQ(
    controller->computeRotateToHeadingCommandWrapper(
      0.7, path.poses[1], velocity).twist.angular.z, 1.8);
  EXPECT_EQ(
    controller->computeRotateToHeadingCommandWrapper(
      -0.7, path.poses[1], velocity).twist.angular.z, -1.8);

  EXPECT_EQ(
    controller->computeRotateToHeadingCommandWrapper(
      0.87, path.poses[1], velocity).twist.angular.z, 1.8);

  // in base_link, so should pass through values without issue
  geometry_msgs::msg::PoseStamped pt;
  pt.pose.position.x = 100.0;
  pt.header.frame_id = "base_link";
  pt.header.stamp = rclcpp::Time();
  auto rtn = controller->transformPoseToBaseFrameWrapper(pt);
  EXPECT_EQ(rtn.position.x, 100.0);

  // in frame that doesn't exist, shouldn't throw, but should fail
  geometry_msgs::msg::PoseStamped pt2;
  pt.pose.position.x = 100.0;
  pt.header.frame_id = "fake_frame2";
  pt.header.stamp = rclcpp::Time();
  EXPECT_THROW(controller->transformPoseToBaseFrameWrapper(pt2), std::runtime_error);
}

TEST(RotationShimControllerTest, computeVelocityTests)
{
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  costmap->set_parameter(rclcpp::Parameter("origin_x", -25.0));
  costmap->set_parameter(rclcpp::Parameter("origin_y", -25.0));
  costmap->configure();
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "odom";
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter("controller_frequency", 1.0);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "fake_frame";
  path.poses.resize(10);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  geometry_msgs::msg::Twist velocity;
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap);

  // send without setting a path - should go to RPP immediately
  // then it should throw an exception because the path is empty and invalid
  EXPECT_THROW(controller->computeVelocityCommands(pose, velocity, &checker), std::runtime_error);

  // Set with a path -- should attempt to find a sampled point but throw exception
  // because it cannot be found, then go to RPP and throw exception because it cannot be transformed
  controller->setPlan(path);
  EXPECT_THROW(controller->computeVelocityCommands(pose, velocity, &checker), std::runtime_error);

  path.header.frame_id = "base_link";
  path.poses[1].pose.position.x = 0.1;
  path.poses[1].pose.position.y = 0.1;
  path.poses[2].pose.position.x = -1.0;
  path.poses[2].pose.position.y = -1.0;
  path.poses[2].header.frame_id = "base_link";
  path.poses[3].pose.position.x = 10.0;
  path.poses[3].pose.position.y = 10.0;

  // this should allow it to find the sampled point, then transform to base_link
  // validly because we setup the TF for it. The -1.0 should be selected since default min
  // is 0.5 and that should cause a rotation in place
  controller->setPlan(path);
  tf_broadcaster->sendTransform(transform);
  auto effort = controller->computeVelocityCommands(pose, velocity, &checker);
  EXPECT_EQ(fabs(effort.twist.angular.z), 1.8);

  path.header.frame_id = "base_link";
  path.poses[1].pose.position.x = 0.1;
  path.poses[1].pose.position.y = 0.1;
  path.poses[2].pose.position.x = 1.0;
  path.poses[2].pose.position.y = 0.0;
  path.poses[2].header.frame_id = "base_link";
  path.poses[3].pose.position.x = 10.0;
  path.poses[3].pose.position.y = 10.0;

  // this should allow it to find the sampled point, then transform to base_link
  // validly because we setup the TF for it. The 1.0 should be selected since default min
  // is 0.5 and that should cause a pass off to the RPP controller which will throw
  // and exception because it is off of the costmap
  controller->setPlan(path);
  tf_broadcaster->sendTransform(transform);
  EXPECT_THROW(controller->computeVelocityCommands(pose, velocity, &checker), std::runtime_error);
}

TEST(RotationShimControllerTest, openLoopRotationTests) {
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "odom";
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter(
    "controller_frequency",
    20.0);
  node->declare_parameter(
    "PathFollower.rotate_to_goal_heading",
    true);
  node->declare_parameter(
    "PathFollower.closed_loop",
    false);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(4);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  geometry_msgs::msg::Twist velocity;
  nav2_controller::SimpleGoalChecker checker;
  node->declare_parameter(
    "checker.xy_goal_tolerance",
    1.0);
  checker.initialize(node, "checker", costmap);

  path.header.frame_id = "base_link";
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.05;
  path.poses[1].pose.position.y = 0.05;
  path.poses[2].pose.position.x = 0.10;
  path.poses[2].pose.position.y = 0.10;
  // goal position within checker xy_goal_tolerance
  path.poses[3].pose.position.x = 0.20;
  path.poses[3].pose.position.y = 0.20;
  // goal heading 45 degrees to the left
  path.poses[3].pose.orientation.z = -0.3826834;
  path.poses[3].pose.orientation.w = 0.9238795;
  path.poses[3].header.frame_id = "base_link";

  // Calculate first velocity command
  controller->setPlan(path);
  auto cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker);
  EXPECT_NEAR(cmd_vel.twist.angular.z, -0.16, 1e-4);

  // Test second velocity command with wrong odometry
  velocity.angular.z = 1.8;
  cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker);
  EXPECT_NEAR(cmd_vel.twist.angular.z, -0.32, 1e-4);
}

TEST(RotationShimControllerTest, computeVelocityGoalRotationTests) {
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "odom";
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter(
    "PathFollower.rotate_to_goal_heading",
    true);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(4);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  geometry_msgs::msg::Twist velocity;
  nav2_controller::SimpleGoalChecker checker;
  node->declare_parameter(
    "checker.xy_goal_tolerance",
    1.0);
  checker.initialize(node, "checker", costmap);

  path.header.frame_id = "base_link";
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.05;
  path.poses[1].pose.position.y = 0.05;
  path.poses[2].pose.position.x = 0.10;
  path.poses[2].pose.position.y = 0.10;
  // goal position within checker xy_goal_tolerance
  path.poses[3].pose.position.x = 0.20;
  path.poses[3].pose.position.y = 0.20;
  // goal heading 45 degrees to the left
  path.poses[3].pose.orientation.z = -0.3826834;
  path.poses[3].pose.orientation.w = 0.9238795;
  path.poses[3].header.frame_id = "base_link";

  controller->setPlan(path);
  auto cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker);
  EXPECT_EQ(cmd_vel.twist.angular.z, -1.8);

  // goal heading 45 degrees to the right
  path.poses[3].pose.orientation.z = 0.3826834;
  path.poses[3].pose.orientation.w = 0.9238795;
  controller->setPlan(path);
  cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker);
  EXPECT_EQ(cmd_vel.twist.angular.z, 1.8);
}

TEST(RotationShimControllerTest, accelerationTests) {
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "odom";
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter(
    "controller_frequency",
    20.0);
  node->declare_parameter(
    "PathFollower.rotate_to_goal_heading",
    true);
  node->declare_parameter(
    "PathFollower.max_angular_accel",
    0.5);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(4);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  geometry_msgs::msg::Twist velocity;
  nav2_controller::SimpleGoalChecker checker;
  node->declare_parameter(
    "checker.xy_goal_tolerance",
    1.0);
  checker.initialize(node, "checker", costmap);

  path.header.frame_id = "base_link";
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.05;
  path.poses[1].pose.position.y = 0.05;
  path.poses[2].pose.position.x = 0.10;
  path.poses[2].pose.position.y = 0.10;
  // goal position within checker xy_goal_tolerance
  path.poses[3].pose.position.x = 0.20;
  path.poses[3].pose.position.y = 0.20;
  // goal heading 45 degrees to the left
  path.poses[3].pose.orientation.z = -0.3826834;
  path.poses[3].pose.orientation.w = 0.9238795;
  path.poses[3].header.frame_id = "base_link";

  // Test acceleration limits
  controller->setPlan(path);
  auto cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker);
  EXPECT_EQ(cmd_vel.twist.angular.z, -0.025);

  // Test slowing down to avoid overshooting
  velocity.angular.z = -1.8;
  cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker);
  EXPECT_NEAR(cmd_vel.twist.angular.z, -std::sqrt(2 * 0.5 * M_PI / 4), 1e-4);
}

TEST(RotationShimControllerTest, isGoalChangedTest)
{
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "odom";
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter(
    "PathFollower.rotate_to_heading_once",
    true);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(2);
  path.poses.back().pose.position.x = 2.0;
  path.poses.back().pose.position.y = 2.0;

  // Test: Current path is empty, should return true
  EXPECT_EQ(controller->isGoalChangedWrapper(path), true);

  // Test: Last pose of the current path is the same, should return false
  controller->setPlan(path);
  EXPECT_EQ(controller->isGoalChangedWrapper(path), false);

  // Test: Last pose of the current path differs, should return true
  path.poses.back().pose.position.x = 3.0;
  EXPECT_EQ(controller->isGoalChangedWrapper(path), true);
}

TEST(RotationShimControllerTest, testDynamicParameter)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ShimControllerTest");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  std::string name = "test";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "test.primary_controller",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.angular_dist_threshold", 7.0),
      rclcpp::Parameter("test.forward_sampling_distance", 7.0),
      rclcpp::Parameter("test.rotate_to_heading_angular_vel", 7.0),
      rclcpp::Parameter("test.max_angular_accel", 7.0),
      rclcpp::Parameter("test.simulate_ahead_time", 7.0),
      rclcpp::Parameter("test.primary_controller", std::string("HI")),
      rclcpp::Parameter("test.rotate_to_goal_heading", true),
      rclcpp::Parameter("test.rotate_to_heading_once", true),
      rclcpp::Parameter("test.closed_loop", false),
      rclcpp::Parameter("test.use_path_orientations", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.angular_dist_threshold").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.forward_sampling_distance").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_angular_vel").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.max_angular_accel").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.simulate_ahead_time").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.rotate_to_goal_heading").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_once").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.closed_loop").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_path_orientations").as_bool(), true);
}
