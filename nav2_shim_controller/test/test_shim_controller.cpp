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

  double getSpeed() {return desired_linear_vel_;}

  geometry_msgs::msg::PoseStamped getLookAheadPointWrapper(
    const double & dist, const nav_msgs::msg::Path & path)
  {
    return getLookAheadPoint(dist, path);
  }

  bool shouldRotateToPathWrapper(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path,
    double & angle_thresh)
  {
    return shouldRotateToPath(carrot_pose, angle_to_path, angle_thresh);
  }

  bool shouldRotateToGoalHeadingWrapper(const geometry_msgs::msg::PoseStamped & carrot_pose)
  {
    return shouldRotateToGoalHeading(carrot_pose);
  }

  void rotateToHeadingWrapper(
    double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::TwistStamped & curr_speed)
  {
    return rotateToHeading(angular_vel, angle_to_path, curr_speed);
  }

  nav_msgs::msg::Path transformGlobalPlanWrapper(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    return transformGlobalPlan(pose);
  }

  bool transformPoseWrapper(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose)
  {
    return transformPose(frame, in_pose, out_pose);
  }

  geometry_msgs::msg::TwistStamped calcCmdVelWrapper(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity)
  {
    return calcCmdVel(pose, velocity);
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

TEST(ShimControllerTest, transformGlobalPlan)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testShimController");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  auto ctrl = std::make_shared<BasicAPIShimController>();
  costmap->configure();
  ctrl->configure(node, name, tf, costmap);

  // setPlan and get plan
  nav_msgs::msg::Path path;
  ctrl->setPlan(path);
  geometry_msgs::msg::PoseStamped pose;
  try {
    ctrl->transformGlobalPlanWrapper(pose);
  } catch (const std::exception & e) {
    EXPECT_EQ(e.what(), std::string("Received plan with zero length"));
  }

  path.poses.resize(2);
  path.header.frame_id = "fake_path_frame";
  pose.header.frame_id = "fake_frame";
  ctrl->setPlan(path);

  try {
    ctrl->transformGlobalPlanWrapper(pose);
  } catch (const std::exception & e) {
    EXPECT_EQ(e.what(), std::string("Unable to transform robot pose into global plan's frame"));
  }
  path.header.frame_id = "fake_frame";
  pose.header.frame_id = "fake_frame";
  path.poses[0].pose.position.x = 5.0;
  path.poses[0].pose.position.y = 5.0;
  path.poses[1].pose.position.x = 5.0;
  path.poses[1].pose.position.y = 5.0;
  ctrl->setPlan(path);
  try {
    ctrl->transformGlobalPlanWrapper(pose);
  } catch (const std::exception & e) {
    EXPECT_EQ(e.what(), std::string("Resulting plan has 0 poses in it."));
  }
}

TEST(ShimControllerTest, transformPose)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testShimController");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  auto ctrl = std::make_shared<BasicAPIShimController>();
  ctrl->configure(node, name, tf, costmap);

  geometry_msgs::msg::PoseStamped in_pose, out_pose;
  in_pose.header.frame_id = "fake_frame_id";
  in_pose.pose.position.x = 1.0;
  in_pose.pose.position.y = 1.0;

  bool result;
  result = ctrl->transformPoseWrapper(in_pose.header.frame_id, in_pose, out_pose);
  EXPECT_EQ(in_pose, out_pose);
  EXPECT_EQ(result, true);

  result = ctrl->transformPoseWrapper("test_fake_frame_id", in_pose, out_pose);
  EXPECT_NE(in_pose, out_pose);
  EXPECT_EQ(result, false);
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
  ctrl->deactivate();
  ctrl->cleanup();

  // setPlan and get plan
  nav_msgs::msg::Path path;
  path.poses.resize(2);
  path.poses[0].header.frame_id = "fake_frame";
  ctrl->setPlan(path);
  EXPECT_EQ(ctrl->getPlan().poses.size(), 2ul);
  EXPECT_EQ(ctrl->getPlan().poses[0].header.frame_id, std::string("fake_frame"));

  // set speed limit
  const double base_speed = ctrl->getSpeed();
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(0.51, false);
  EXPECT_EQ(ctrl->getSpeed(), 0.51);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
  ctrl->setSpeedLimit(50, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed * 0.5);
  ctrl->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, true);
  EXPECT_EQ(ctrl->getSpeed(), base_speed);
}

TEST(ShimControllerTest, lookaheadAPI)
{
  auto ctrl = std::make_shared<BasicAPIShimController>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testShimController");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  ctrl->configure(node, name, tf, costmap);

  // test getLookAheadPoint
  double dist = 1.0;
  nav_msgs::msg::Path path;
  path.poses.resize(10);
  for (uint i = 0; i != path.poses.size(); i++) {
    path.poses[i].pose.position.x = static_cast<double>(i);
  }

  // test exact hits
  auto pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 1.0);

  // test getting next closest point
  dist = 3.8;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 4.0);

  // test end of path
  dist = 100.0;
  pt = ctrl->getLookAheadPointWrapper(dist, path);
  EXPECT_EQ(pt.pose.position.x, 9.0);
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
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, angle_thresh), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 1.0;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn, angle_thresh), true);

  // shouldRotateToGoalHeading
  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.0;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), true);

  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.24;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), true);

  carrot.pose.position.x = 0.0;
  carrot.pose.position.y = 0.26;
  EXPECT_EQ(ctrl->shouldRotateToGoalHeadingWrapper(carrot), false);

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
}

TEST(ShimControllerTest, calcCmdVel)
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
  cmd_vel = ctrl->calcCmdVelWrapper(pose, speed);
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.twist.angular.z, 0.06);

  path.poses[1].pose.position.x = 0.0;
  path.poses[1].pose.position.y = 1.0;

  ctrl->setPlan(path);
  cmd_vel = ctrl->calcCmdVelWrapper(pose, speed);
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.twist.angular.z, 0.12);

  path.poses[1].pose.position.x = 1.0;
  path.poses[1].pose.position.y = 0.0;

  ctrl->setPlan(path);
  cmd_vel = ctrl->calcCmdVelWrapper(pose, speed);
  EXPECT_EQ(cmd_vel.twist.linear.x, 0.0);
  EXPECT_EQ(cmd_vel.twist.angular.z, 0.0);
}
