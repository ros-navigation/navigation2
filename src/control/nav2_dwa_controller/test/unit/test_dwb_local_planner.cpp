// Copyright (c) 2018 Intel Corporation
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

#include <iostream>
#include <vector>
#include <memory>
#include "gtest/gtest.h"
#include "dwb_local_planner/dwb_local_planner.h"
#include "dwb_local_planner/exceptions.h"
#include "dwb_plugins/simple_goal_checker.h"
#include "dwb_plugins/limited_accel_generator.h"
#include "dwb_critics/alignment_util.h"
#include "dwb_critics/line_iterator.h"
#include "dwb_critics/path_align.h"
#include "dwb_critics/twirling.h"
#include "dwb_critics/base_obstacle.h"
#include "dwb_critics/map_grid.h"
#include "dwb_critics/path_dist.h"
#include "dwb_critics/goal_align.h"
#include "dwb_critics/obstacle_footprint.h"
#include "dwb_critics/prefer_forward.h"
#include "dwb_critics/goal_dist.h"
#include "dwb_critics/oscillation.h"
#include "dwb_critics/rotate_to_goal.h"

using std::vector;
using std::make_shared;
using std::shared_ptr;
using dwb_local_planner::TFListenerPtr;
using dwb_local_planner::CostmapROSPtr;
using dwb_local_planner::TrajectoryGenerator;
using dwb_local_planner::GoalChecker;
using dwb_local_planner::TrajectoryCritic;
using dwb_local_planner::DWBLocalPlanner;

TrajectoryGenerator::Ptr g_trajectoryGenerator =
  make_shared<dwb_plugins::LimitedAccelGenerator>();
GoalChecker::Ptr g_goalChecker = make_shared<dwb_plugins::SimpleGoalChecker>();
vector<TrajectoryCritic::Ptr> g_trajectoryCritics;

class rclFixture
{
public:
  rclFixture() {rclcpp::init(0, nullptr);}
  ~rclFixture() {rclcpp::shutdown();}
};

rclFixture g_rclFixture;


#define ADD_CRITIC(critic_type) g_trajectoryCritics.push_back(make_shared<critic_type>())

class UninitializedTransform : public ::testing::Test
{
public:
  UninitializedTransform()
  {
    nh = rclcpp::Node::make_shared("dwa");
    planner.initialize(*nh, tf, cm);
    robot_pose.pose.x = 0;
    robot_pose.pose.y = 0;
    robot_pose.pose.theta = 0;
    ADD_CRITIC(dwb_critics::PreferForwardCritic);
    // ADD_CRITIC(dwb_critics::GoalDistCritic);
    // ADD_CRITIC(dwb_critics::PathAlignCritic);
    // ADD_CRITIC(dwb_critics::GoalAlignCritic);
    // ADD_CRITIC(dwb_critics::PathDistCritic);
    ADD_CRITIC(dwb_critics::OscillationCritic);
    ADD_CRITIC(dwb_critics::RotateToGoalCritic);
    ADD_CRITIC(dwb_critics::BaseObstacleCritic);
    ADD_CRITIC(dwb_critics::ObstacleFootprintCritic);
    ADD_CRITIC(dwb_critics::TwirlingCritic);
  }

protected:
  shared_ptr<rclcpp::Node> nh;
  TFListenerPtr tf;
  CostmapROSPtr cm;
  DWBLocalPlanner planner;
  nav_2d_msgs::msg::Pose2DStamped robot_pose;
};

TEST_F(UninitializedTransform, DISABLED_NoPlan)
{
  planner.setPlan(nav_2d_msgs::msg::Path2D());
  ASSERT_THROW(planner.computeVelocityCommands(
      robot_pose, nav_2d_msgs::msg::Twist2D()),
    nav_core2::PlannerException);
}

TEST_F(UninitializedTransform, DISABLED_NoTransform)
{
  geometry_msgs::msg::Pose2D goal;
  goal.x = 1;
  goal.y = 1;
  goal.theta = 0;
  vector<geometry_msgs::msg::Pose2D> path(1, goal);
  nav_2d_msgs::msg::Path2D plan;
  plan.poses = path;
  planner.setPlan(plan);
  ASSERT_THROW(planner.computeVelocityCommands(
      robot_pose, nav_2d_msgs::msg::Twist2D()),
    nav_core2::PlannerTFException);
}

TEST_F(UninitializedTransform, DISABLED_GoToOneOne)
{
  geometry_msgs::msg::Pose2D goal;
  goal.x = 0.5;
  goal.y = 0.5;
  goal.theta = 0;
  vector<geometry_msgs::msg::Pose2D> path(1, goal);
  nav_2d_msgs::msg::Path2D plan;
  plan.poses = path;
  planner.setPlan(plan);
  nav_2d_msgs::msg::Twist2D cmdVel;
  planner.computeVelocityCommands(robot_pose, cmdVel);
  FAIL();
}
