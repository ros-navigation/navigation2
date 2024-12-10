/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "nav2_controller/plugins/path_complete_goal_checker.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"

using nav2_controller::SimpleGoalChecker;
using nav2_controller::StoppedGoalChecker;
using nav2_controller::PathCompleteGoalChecker;

void checkMacro(
  nav2_core::GoalChecker & gc,
  double x0, double y0, double theta0,
  double x1, double y1, double theta1,
  double xv, double yv, double thetav,
  const nav_msgs::msg::Path & path,
  bool expected_result)
{
  gc.reset();
  geometry_msgs::msg::Pose2D pose0, pose1;
  pose0.x = x0;
  pose0.y = y0;
  pose0.theta = theta0;
  pose1.x = x1;
  pose1.y = y1;
  pose1.theta = theta1;
  nav_2d_msgs::msg::Twist2D v;
  v.x = xv;
  v.y = yv;
  v.theta = thetav;
  if (expected_result) {
    EXPECT_TRUE(
      gc.isGoalReached(
        nav_2d_utils::pose2DToPose(pose0),
        nav_2d_utils::pose2DToPose(pose1), nav_2d_utils::twist2Dto3D(v), path));
  } else {
    EXPECT_FALSE(
      gc.isGoalReached(
        nav_2d_utils::pose2DToPose(pose0),
        nav_2d_utils::pose2DToPose(pose1), nav_2d_utils::twist2Dto3D(v), path));
  }
}

class TestLifecycleNode : public nav2_util::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2_util::LifecycleNode(name)
  {
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {
    return nav2_util::CallbackReturn::SUCCESS;
  }
};

TEST(SimpleGoalChecker, goal_checker_reset)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  nav2_core::GoalChecker * gc = new SimpleGoalChecker;
  gc->reset();
  delete gc;
  EXPECT_TRUE(true);
}

TEST(StoppedGoalChecker, stopped_goal_checker_reset)
{
  auto x = std::make_shared<TestLifecycleNode>("stopped_goal_checker");

  nav2_core::GoalChecker * sgc = new StoppedGoalChecker;
  sgc->reset();
  delete sgc;
  EXPECT_TRUE(true);
}

TEST(StoppedGoalChecker, path_complete_goal_checker_reset)
{
  auto x = std::make_shared<TestLifecycleNode>("path_complete_goal_checker");

  nav2_core::GoalChecker * pcgc = new PathCompleteGoalChecker;
  pcgc->reset();
  delete pcgc;
  EXPECT_TRUE(true);
}


TEST(SimpleGoalChecker, test_goal_checking)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  SimpleGoalChecker gc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  nav_msgs::msg::Path path;
  gc.initialize(x, "nav2_controller", costmap);

  checkMacro(gc, 0, 0, 0, 0, 0, 0, 0, 0, 0, path, true);
  checkMacro(gc, 0, 0, 0, 1, 0, 0, 0, 0, 0, path, false);
  checkMacro(gc, 0, 0, 0, 0, 1, 0, 0, 0, 0, path, false);
  checkMacro(gc, 0, 0, 0, 0, 0, 1, 0, 0, 0, path, false);
  checkMacro(gc, 0, 0, 3.14, 0, 0, -3.14, 0, 0, 0, path, true);
  checkMacro(gc, 0, 0, 3.14, 0, 0, -3.14, 1, 0, 0, path, true);
  checkMacro(gc, 0, 0, 0, 0, 0, 0, 1, 0, 0, path, true);
  checkMacro(gc, 0, 0, 0, 0, 0, 0, 0, 1, 0, path, true);
  checkMacro(gc, 0, 0, 0, 0, 0, 0, 0, 0, 1, path, true);
}

TEST(StoppedGoalChecker, test_goal_checking)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  StoppedGoalChecker sgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  nav_msgs::msg::Path path;

  sgc.initialize(x, "nav2_controller", costmap);

  checkMacro(sgc, 0, 0, 0, 0, 0, 0, 0, 0, 0, path, true);
  checkMacro(sgc, 0, 0, 0, 1, 0, 0, 0, 0, 0, path, false);
  checkMacro(sgc, 0, 0, 0, 0, 1, 0, 0, 0, 0, path, false);
  checkMacro(sgc, 0, 0, 0, 0, 0, 1, 0, 0, 0, path, false);
  checkMacro(sgc, 0, 0, 3.14, 0, 0, -3.14, 0, 0, 0, path, true);
  checkMacro(sgc, 0, 0, 3.14, 0, 0, -3.14, 1, 0, 0, path, false);
  checkMacro(sgc, 0, 0, 0, 0, 0, 0, 1, 0, 0, path, false);
  checkMacro(sgc, 0, 0, 0, 0, 0, 0, 0, 1, 0, path, false);
  checkMacro(sgc, 0, 0, 0, 0, 0, 0, 0, 0, 1, path, false);

  // todo: add some where path complete goal checker differs
}

TEST(PathCompleteGoalChecker, test_goal_checking)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  PathCompleteGoalChecker pcgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  nav_msgs::msg::Path path;
  pcgc.initialize(x, "nav2_controller", costmap);

  // add one default constructed pose to the path
  // this should have no impact on the results vrs. SimpleGoalChecker
  path.poses.emplace_back();
  checkMacro(pcgc, 0, 0, 0, 0, 0, 0, 0, 0, 0, path, true);
  checkMacro(pcgc, 0, 0, 0, 1, 0, 0, 0, 0, 0, path, false);
  checkMacro(pcgc, 0, 0, 0, 0, 1, 0, 0, 0, 0, path, false);
  checkMacro(pcgc, 0, 0, 0, 0, 0, 1, 0, 0, 0, path, false);
  checkMacro(pcgc, 0, 0, 3.14, 0, 0, -3.14, 0, 0, 0, path, true);
  checkMacro(pcgc, 0, 0, 3.14, 0, 0, -3.14, 1, 0, 0, path, true);
  checkMacro(pcgc, 0, 0, 0, 0, 0, 0, 1, 0, 0, path, true);
  checkMacro(pcgc, 0, 0, 0, 0, 0, 0, 0, 1, 0, path, true);
  checkMacro(pcgc, 0, 0, 0, 0, 0, 0, 0, 0, 1, path, true);

  // add a second default constructed pose to the path
  // this should prevent any completions due to path_length_tolerence=1
  path.poses.emplace_back();

  checkMacro(pcgc, 0, 0, 0, 0, 0, 0, 0, 0, 0, path, false);
  checkMacro(pcgc, 0, 0, 0, 1, 0, 0, 0, 0, 0, path, false);
  checkMacro(pcgc, 0, 0, 0, 0, 1, 0, 0, 0, 0, path, false);
  checkMacro(pcgc, 0, 0, 0, 0, 0, 1, 0, 0, 0, path, false);
  checkMacro(pcgc, 0, 0, 3.14, 0, 0, -3.14, 0, 0, 0, path, false);
  checkMacro(pcgc, 0, 0, 3.14, 0, 0, -3.14, 1, 0, 0, path, false);
  checkMacro(pcgc, 0, 0, 0, 0, 0, 0, 1, 0, 0, path, false);
  checkMacro(pcgc, 0, 0, 0, 0, 0, 0, 0, 1, 0, path, false);
  checkMacro(pcgc, 0, 0, 0, 0, 0, 0, 0, 0, 1, path, false);
}


TEST(StoppedGoalChecker, get_tol_and_dynamic_params)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  StoppedGoalChecker sgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  sgc.initialize(x, "test", costmap);
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;

  // Test stopped goal checker's tolerance API
  EXPECT_TRUE(sgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(pose_tol.position.x, 0.25);
  EXPECT_EQ(pose_tol.position.y, 0.25);
  auto p2d = nav_2d_utils::poseToPose2D(pose_tol);
  EXPECT_NEAR(p2d.theta, 0.25, 1e-6);

  // Test Stopped goal checker's dynamic parameters
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    x->get_node_base_interface(), x->get_node_topics_interface(),
    x->get_node_graph_interface(),
    x->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.rot_stopped_velocity", 100.0),
      rclcpp::Parameter("test.trans_stopped_velocity", 100.0)});

  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);

  EXPECT_EQ(x->get_parameter("test.rot_stopped_velocity").as_double(), 100.0);
  EXPECT_EQ(x->get_parameter("test.trans_stopped_velocity").as_double(), 100.0);


  // Test the dynamic parameters impacted the tolerances
  EXPECT_TRUE(sgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(vel_tol.linear.x, 100.0);
  EXPECT_EQ(vel_tol.linear.y, 100.0);
  EXPECT_EQ(vel_tol.angular.z, 100.0);
}


TEST(SimpleGoalChecker, get_tol_and_dynamic_params)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  SimpleGoalChecker gc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  gc.initialize(x, "test2", costmap);
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;

  // Test stopped goal checker's tolerance API
  EXPECT_TRUE(gc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(pose_tol.position.x, 0.25);
  EXPECT_EQ(pose_tol.position.y, 0.25);

  // Test normal goal checker's dynamic parameters
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    x->get_node_base_interface(), x->get_node_topics_interface(),
    x->get_node_graph_interface(),
    x->get_node_services_interface());
  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test2.xy_goal_tolerance", 200.0),
      rclcpp::Parameter("test2.yaw_goal_tolerance", 200.0),
      rclcpp::Parameter("test2.stateful", true)});

  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);

  EXPECT_EQ(x->get_parameter("test2.xy_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test2.yaw_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test2.stateful").as_bool(), true);

  // Test the dynamic parameters impacted the tolerances
  EXPECT_TRUE(gc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(pose_tol.position.x, 200.0);
  EXPECT_EQ(pose_tol.position.y, 200.0);
}

TEST(PathCompleteGoalChecker, get_tol_and_dynamic_params)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  PathCompleteGoalChecker pcgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  pcgc.initialize(x, "test3", costmap);
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;

  // Test stopped goal checker's tolerance API
  EXPECT_TRUE(pcgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(pose_tol.position.x, 0.25);
  EXPECT_EQ(pose_tol.position.y, 0.25);

  // Test normal goal checker's dynamic parameters
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    x->get_node_base_interface(), x->get_node_topics_interface(),
    x->get_node_graph_interface(),
    x->get_node_services_interface());
  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test3.xy_goal_tolerance", 200.0),
      rclcpp::Parameter("test3.yaw_goal_tolerance", 200.0),
      rclcpp::Parameter("test3.path_length_tolerence", 3),
      rclcpp::Parameter("test3.stateful", true)});

  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);

  EXPECT_EQ(x->get_parameter("test3.xy_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test3.yaw_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test3.path_length_tolerence").as_int(), 3);
  EXPECT_EQ(x->get_parameter("test3.stateful").as_bool(), true);

  // Test the dynamic parameters impacted the tolerances
  EXPECT_TRUE(pcgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(pose_tol.position.x, 200.0);
  EXPECT_EQ(pose_tol.position.y, 200.0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
