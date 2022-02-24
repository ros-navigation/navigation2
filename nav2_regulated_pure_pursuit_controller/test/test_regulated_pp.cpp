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
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class BasicAPIRPP : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  BasicAPIRPP() : nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController() {};

  nav_msgs::msg::Path getPlan() {return global_plan_;};

  double getSpeed() {return desired_linear_vel_;};

  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsgWrapper(
    const geometry_msgs::msg::PoseStamped & carrot_pose)
  {
    return createCarrotMsg(carrot_pose);
  };

  void setVelocityScaledLookAhead() {use_velocity_scaled_lookahead_dist_ = true;};
  void setCostRegulationScaling() {use_cost_regulated_linear_velocity_scaling_ = true;};
  void resetVelocityRegulationScaling() {use_regulated_linear_velocity_scaling_ = false;};
  void resetVelocityApproachScaling() {use_approach_vel_scaling_ = false;};

  double getLookAheadDistanceWrapper(const geometry_msgs::msg::Twist & twist)
  {
    return getLookAheadDistance(twist);
  };

  geometry_msgs::msg::PoseStamped getLookAheadPointWrapper(
    const double & dist, const nav_msgs::msg::Path & path)
  {
    return getLookAheadPoint(dist, path);
  };

  bool shouldRotateToPathWrapper(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
  {
    return shouldRotateToPath(carrot_pose, angle_to_path);
  }

  bool shouldRotateToGoalHeadingWrapper(const geometry_msgs::msg::PoseStamped & carrot_pose)
  {
    return shouldRotateToGoalHeading(carrot_pose);
  }

  void rotateToHeadingWrapper(double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
  {
    return rotateToHeading(linear_vel, angular_vel, angle_to_path, curr_speed);
  }

  void applyConstraintsWrapper(
  const double & dist_error, const double & lookahead_dist,
  const double & curvature, const geometry_msgs::msg::Twist & curr_speed,
  const double & pose_cost, double & linear_vel)
  {
    return applyConstraints(dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  }

};

TEST(RegulatedPurePursuitTest, basicAPI)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");

  // instantiate
  auto ctrl = std::make_shared<BasicAPIRPP>();
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
}

TEST(RegulatedPurePursuitTest, createCarrotMsg)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "Hi!";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 12.0;
  pose.pose.orientation.w = 0.5;

  auto rtn = ctrl->createCarrotMsgWrapper(pose);
  EXPECT_EQ(rtn->header.frame_id, std::string("Hi!"));
  EXPECT_EQ(rtn->point.x, 1.0);
  EXPECT_EQ(rtn->point.y, 12.0);
  EXPECT_EQ(rtn->point.z, 0.01);
}

TEST(RegulatedPurePursuitTest, lookaheadAPI)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  ctrl->configure(node, name, tf, costmap);

  geometry_msgs::msg::Twist twist;

  // test getLookAheadDistance
  double rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.6);  // default lookahead_dist

  // shouldn't be a function of speed
  twist.linear.x = 10.0;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.6);

  // now it should be a function of velocity, max out
  ctrl->setVelocityScaledLookAhead();
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.9);  // 10 speed maxes out at max_lookahead_dist

  // check normal range
  twist.linear.x = 0.35;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_NEAR(rtn, 0.525, 0.0001);  // 1.5 * 0.35

  // check minimum range
  twist.linear.x = 0.0;
  rtn = ctrl->getLookAheadDistanceWrapper(twist);
  EXPECT_EQ(rtn, 0.3);

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

TEST(RegulatedPurePursuitTest, rotateTests)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  ctrl->configure(node, name, tf, costmap);

  // shouldRotateToPath
  geometry_msgs::msg::PoseStamped carrot;
  double angle_to_path_rtn;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 0.25;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn), false);

  carrot.pose.position.x = 0.5;
  carrot.pose.position.y = 1.0;
  EXPECT_EQ(ctrl->shouldRotateToPathWrapper(carrot, angle_to_path_rtn), true);

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
  double lin_v = 10.0;
  double ang_v = 0.5;
  double angle_to_path = 0.4;
  geometry_msgs::msg::Twist curr_speed;
  curr_speed.angular.z = 1.75;

  // basic full speed at a speed
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_EQ(lin_v, 0.0);
  EXPECT_EQ(ang_v, 1.8);

  // negative direction
  angle_to_path = -0.4;
  curr_speed.angular.z = -1.75;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_EQ(ang_v, -1.8);

  // kinematic clamping, no speed, some speed accelerating, some speed decelerating
  angle_to_path = 0.4;
  curr_speed.angular.z = 0.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 0.16, 0.01);

  curr_speed.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 1.16, 0.01);

  angle_to_path = -0.4;
  curr_speed.angular.z = 1.0;
  ctrl->rotateToHeadingWrapper(lin_v, ang_v, angle_to_path, curr_speed);
  EXPECT_NEAR(ang_v, 0.84, 0.01);
}

TEST(RegulatedPurePursuitTest, applyConstraints)
{
  auto ctrl = std::make_shared<BasicAPIRPP>();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testRPP");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  ctrl->configure(node, name, tf, costmap);

  double dist_error = 0.0;
  double lookahead_dist = 0.6;
  double curvature = 0.5;
  geometry_msgs::msg::Twist curr_speed;
  double pose_cost = 0.0;
  double linear_vel = 0.0;

  // since costmaps here are bogus, we can't access them
  ctrl->resetVelocityApproachScaling();

  // test curvature regulation (default)
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  EXPECT_EQ(linear_vel, 0.25);  // min set speed

  linear_vel = 1.0;
  curvature = 0.7407;
  curr_speed.linear.x = 0.5;
  ctrl->applyConstraintsWrapper(dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  EXPECT_NEAR(linear_vel, 0.5, 0.01);  // lower by curvature

  linear_vel = 1.0;
  curvature = 1000.0;
  curr_speed.linear.x = 0.25;
  ctrl->applyConstraintsWrapper(dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  EXPECT_NEAR(linear_vel, 0.25, 0.01);  // min out by curvature


  // now try with cost regulation (turn off velocity and only cost)
  // ctrl->setCostRegulationScaling();
  // ctrl->resetVelocityRegulationScaling();
  // curvature = 0.0;

  // min changable cost
  // pose_cost = 1;
  // linear_vel = 0.5;
  // curr_speed.linear.x = 0.5;
  // ctrl->applyConstraintsWrapper(dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.498, 0.01);

  // max changing cost
  // pose_cost = 127;
  // curr_speed.linear.x = 0.255;
  // ctrl->applyConstraintsWrapper(dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.255, 0.01);

  // over max cost thresh
  // pose_cost = 200;
  // curr_speed.linear.x = 0.25;
  // ctrl->applyConstraintsWrapper(dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.25, 0.01);

  // test kinematic clamping
  // pose_cost = 200;
  // curr_speed.linear.x = 1.0;
  // ctrl->applyConstraintsWrapper(dist_error, lookahead_dist, curvature, curr_speed, pose_cost, linear_vel);
  // EXPECT_NEAR(linear_vel, 0.5, 0.01);
}
