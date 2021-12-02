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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_shim_controller/shim_controller.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class BasicAPIShimController : public nav2_shim_controller::ShimController
{
public:
  nav_msgs::msg::Path getPlan() {return global_plan_;}

  bool shouldRotateToPathWrapper(
    const geometry_msgs::msg::PoseStamped & robot_pose, double & angle_to_path,
    double & angle_thresh)
  {
    return shouldRotateToPath(robot_pose, angle_to_path, angle_thresh);
  }

  bool isCloseToGoalHeadingWrapper(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & carrot_pose)
  {
    return isCloseToGoalHeading(robot_pose, carrot_pose);
  }

  void rotateToHeadingWrapper(
    double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::TwistStamped & curr_speed)
  {
    return rotateToHeading(angular_vel, angle_to_path, curr_speed);
  }

  geometry_msgs::msg::PoseStamped transformPoseToMapFrameWrapper(
    const geometry_msgs::msg::PoseStamped & input_pose)
  {
    return transformPoseToMapFrame(input_pose);
  }
};

TEST(ShimControllerTest, configure)
{
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> bad_ptr;
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testShimController");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  auto ctrl = std::make_shared<BasicAPIShimController>();
  try {
    ctrl->configure(bad_ptr, name, tf, costmap);
  } catch (const std::exception & e) {
    EXPECT_EQ(e.what(), std::string("Unable to lock node!"));
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".default_plugin", rclcpp::ParameterValue("PathFollowerDefault"));

  nav2_util::declare_parameter_if_not_declared(
    node, "PathFollowerDefault.plugin", rclcpp::ParameterValue("dwb_core::DWBLocalPlanner"));

  try {
    ctrl->configure(node, name, tf, costmap);
  } catch (const std::exception & e) {
    EXPECT_EQ(e.what(), std::string("Unable to load default plugin!"));
  }
}

TEST(ShimControllerTest, basicAPI)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testShimController");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");

  // instantiate
  auto ctrl = std::make_shared<BasicAPIShimController>();
  ctrl->configure(node, name, tf, costmap);
  ctrl->activate();
  ctrl->setSpeedLimit(0.51, false);
  ctrl->deactivate();
  ctrl->cleanup();

  // setPlan and get plan
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[0].header.frame_id = "fake_frame";
  ctrl->setPlan(path);
  EXPECT_EQ(ctrl->getPlan().poses.size(), 2ul);
  EXPECT_EQ(ctrl->getPlan().poses[0].header.frame_id, std::string("fake_frame"));
}

TEST(ShimControllerTest, rotateTests)
{
  auto ctrl = std::make_shared<BasicAPIShimController>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testShimController");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  ctrl->configure(node, name, tf, costmap);

  // shouldRotateToPath
  geometry_msgs::msg::PoseStamped carrot;
  double angle_to_path_rtn, angle_thresh = 0.785;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, angle_thresh), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 0.25;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, angle_thresh), true);

  carrot.pose.position.x = 0.1;
  carrot.pose.position.y = 1.0;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, angle_thresh), true);

  // shouldRotateToGoalHeading
  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.0;
  EXPECT_EQ(ctrl->isCloseToGoalHeadingWrapper(carrot, carrot), true);

  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.2;
  EXPECT_EQ(ctrl->isCloseToGoalHeadingWrapper(carrot, carrot), true);

  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.36;
  EXPECT_EQ(ctrl->isCloseToGoalHeadingWrapper(carrot, carrot), true);

  // rotateToHeading
  double ang_v = 0.5;
  double angle_to_path = 0.8;
  geometry_msgs::msg::TwistStamped curr_speed;
  curr_speed.twist.angular.z = 1.75;

  // basic full speed at a speed
  ctrl->rotateToHeadingWrapper(ang_v, angle_to_path, curr_speed);
  EXPECT_EQ(ang_v, 1.69);

  // negative direction
  angle_to_path = -0.8;
  curr_speed.twist.angular.z = -1.75;
  ctrl->rotateToHeadingWrapper(ang_v, angle_to_path, curr_speed);
  EXPECT_EQ(ang_v, -1.69);

  // kinematic clamping, some speed accelerating, some speed decelerating
  angle_to_path = 0.4;
  curr_speed.twist.angular.z = 0.0;
  ctrl->rotateToHeadingWrapper(ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 0.06, 0.01);

  curr_speed.twist.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 0.94, 0.01);

  angle_to_path = 0.8;
  curr_speed.twist.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 1.06, 0.01);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 100.0;
  pose.header.frame_id = "fake_frame2";
  pose.header.stamp = rclcpp::Time();
  EXPECT_THROW(ctrl->transformPoseToMapFrameWrapper(pose), std::runtime_error);
}

TEST(ShimControllerTest, computeVelocityTests)
{
  auto ctrl = std::make_shared<BasicAPIShimController>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testShimController");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  costmap->configure();
  ctrl->configure(node, name, tf, costmap);

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;
  path.poses.resize(2);
  path.header.frame_id = "base_link";
  pose.header.frame_id = "base_link";
  path.poses[1].pose.position.x = 0.0;
  path.poses[1].pose.position.y = 0.1;
  path.poses[1].pose.orientation.w = 0.0;
  path.poses[1].pose.orientation.z = 1.0;
  ctrl->setPlan(path);
  geometry_msgs::msg::TwistStamped cmd_vel;
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker");
  cmd_vel = ctrl->computeVelocityCommands(pose, speed, &checker);
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.twist.angular.z, 0.06);

  path.poses[1].pose.position.x = 0.0;
  path.poses[1].pose.position.y = 1.0;

  ctrl->setPlan(path);
  cmd_vel = ctrl->computeVelocityCommands(pose, speed, &checker);
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.twist.angular.z, 0.12);

  path.poses[1].pose.position.x = 1.0;
  path.poses[1].pose.position.y = 0.0;

  ctrl->setPlan(path);
  cmd_vel = ctrl->computeVelocityCommands(pose, speed, &checker);
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.twist.angular.z, 0.0);
}
