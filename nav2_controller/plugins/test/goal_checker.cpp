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
#include "nav2_controller/plugins/position_goal_checker.hpp"
#include "nav2_controller/plugins/adaptive_tolerance_goal_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"

using nav2_controller::SimpleGoalChecker;
using nav2_controller::StoppedGoalChecker;
using nav2_controller::PositionGoalChecker;
using nav2_controller::AdaptiveToleranceGoalChecker;

void checkMacro(
  nav2_core::GoalChecker & gc,
  double x0, double y0, double theta0,
  double x1, double y1, double theta1,
  double xv, double yv, double thetav,
  bool expected_result)
{
  gc.reset();

  geometry_msgs::msg::Pose pose0, pose1;
  pose0.position.x = x0;
  pose0.position.y = y0;
  pose0.position.z = 0.0;
  pose0.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta0);

  pose1.position.x = x1;
  pose1.position.y = y1;
  pose1.position.z = 0.0;
  pose1.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta1);

  geometry_msgs::msg::Twist v;
  v.linear.x = xv;
  v.linear.y = yv;
  v.angular.z = thetav;

  nav_msgs::msg::Path transformed_global_plan;
  if (expected_result) {
    EXPECT_TRUE(gc.isGoalReached(pose0, pose1, v, transformed_global_plan));
  } else {
    EXPECT_FALSE(gc.isGoalReached(pose0, pose1, v, transformed_global_plan));
  }
}

void sameResult(
  nav2_core::GoalChecker & gc0, nav2_core::GoalChecker & gc1,
  double x0, double y0, double theta0,
  double x1, double y1, double theta1,
  double xv, double yv, double thetav,
  bool expected_result)
{
  checkMacro(gc0, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, expected_result);
  checkMacro(gc1, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, expected_result);
}

void trueFalse(
  nav2_core::GoalChecker & gc0, nav2_core::GoalChecker & gc1,
  double x0, double y0, double theta0,
  double x1, double y1, double theta1,
  double xv, double yv, double thetav)
{
  checkMacro(gc0, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, true);
  checkMacro(gc1, x0, y0, theta0, x1, y1, theta1, xv, yv, thetav, false);
}
class TestLifecycleNode : public nav2::LifecycleNode
{
public:
  explicit TestLifecycleNode(const std::string & name)
  : nav2::LifecycleNode(name)
  {
  }

  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn onShutdown(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn onError(const rclcpp_lifecycle::State &)
  {
    return nav2::CallbackReturn::SUCCESS;
  }
};

TEST(VelocityIterator, goal_checker_reset)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  nav2_core::GoalChecker * gc = new SimpleGoalChecker;
  gc->reset();
  delete gc;
  EXPECT_TRUE(true);
}

TEST(VelocityIterator, stopped_goal_checker_reset)
{
  auto x = std::make_shared<TestLifecycleNode>("stopped_goal_checker");

  nav2_core::GoalChecker * sgc = new StoppedGoalChecker;
  sgc->reset();
  delete sgc;
  EXPECT_TRUE(true);
}

TEST(VelocityIterator, position_goal_checker_reset)
{
  auto x = std::make_shared<TestLifecycleNode>("position_goal_checker");

  nav2_core::GoalChecker * pgc = new PositionGoalChecker;
  pgc->reset();
  delete pgc;
  EXPECT_TRUE(true);
}

TEST(VelocityIterator, adaptive_tolerance_goal_checker_reset)
{
  auto x = std::make_shared<TestLifecycleNode>("adaptive_tolerance_goal_checker");

  nav2_core::GoalChecker * prgc = new AdaptiveToleranceGoalChecker;
  prgc->reset();
  delete prgc;
  EXPECT_TRUE(true);
}

TEST(VelocityIterator, two_checks)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  SimpleGoalChecker gc;
  StoppedGoalChecker sgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  gc.initialize(x, "nav2_controller", costmap);
  sgc.initialize(x, "nav2_controller", costmap);
  sameResult(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 0, 0, true);
  sameResult(gc, sgc, 0, 0, 0, 1, 0, 0, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 0, 0, 1, 0, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 0, 0, 0, 1, 0, 0, 0, false);
  sameResult(gc, sgc, 0, 0, 3.14, 0, 0, -3.14, 0, 0, 0, true);
  trueFalse(gc, sgc, 0, 0, 3.14, 0, 0, -3.14, 1, 0, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 1, 0, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 1, 0);
  trueFalse(gc, sgc, 0, 0, 0, 0, 0, 0, 0, 0, 1);
}

TEST(StoppedGoalChecker, get_tol_and_dynamic_params)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  SimpleGoalChecker gc;
  StoppedGoalChecker sgc;
  PositionGoalChecker pgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  sgc.initialize(x, "test", costmap);
  gc.initialize(x, "test2", costmap);
  pgc.initialize(x, "test3", costmap);
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;

  // Test stopped goal checker's tolerance API
  EXPECT_TRUE(sgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(vel_tol.linear.x, 0.25);
  EXPECT_EQ(vel_tol.linear.y, 0.25);
  EXPECT_EQ(vel_tol.angular.z, 0.25);

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

  // Test normal goal checker's dynamic parameters
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test2.xy_goal_tolerance", 200.0),
      rclcpp::Parameter("test2.yaw_goal_tolerance", 200.0),
      rclcpp::Parameter("test2.path_length_tolerance", 200.0),
      rclcpp::Parameter("test2.stateful", true),
      rclcpp::Parameter("test2.symmetric_yaw_tolerance", true)});

  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);

  EXPECT_EQ(x->get_parameter("test2.xy_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test2.yaw_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test2.path_length_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test2.stateful").as_bool(), true);
  EXPECT_EQ(x->get_parameter("test2.symmetric_yaw_tolerance").as_bool(), true);

  // Test the dynamic parameters impacted the tolerances
  EXPECT_TRUE(sgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(vel_tol.linear.x, 100.0);
  EXPECT_EQ(vel_tol.linear.y, 100.0);
  EXPECT_EQ(vel_tol.angular.z, 100.0);

  EXPECT_TRUE(gc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(pose_tol.position.x, 200.0);
  EXPECT_EQ(pose_tol.position.y, 200.0);

  // Test position goal checker's dynamic parameters
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test3.xy_goal_tolerance", 200.0),
      rclcpp::Parameter("test3.path_length_tolerance", 200.0),
      rclcpp::Parameter("test3.stateful", true)});

  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);

  EXPECT_EQ(x->get_parameter("test3.xy_goal_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test3.path_length_tolerance").as_double(), 200.0);
  EXPECT_EQ(x->get_parameter("test3.stateful").as_bool(), true);

  EXPECT_TRUE(pgc.getTolerances(pose_tol, vel_tol));
  EXPECT_EQ(pose_tol.position.x, 200.0);
  EXPECT_EQ(pose_tol.position.y, 200.0);

  // Test setting invalid values
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.rot_stopped_velocity", -1.0)}
  );
  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);
  // Value should remain unchanged
  EXPECT_EQ(x->get_parameter("test.rot_stopped_velocity").as_double(), 100.0);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test2.xy_goal_tolerance", -1.0)}
  );
  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);
  // Value should remain unchanged
  EXPECT_EQ(x->get_parameter("test2.xy_goal_tolerance").as_double(), 200.0);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test3.path_length_tolerance", -1.0)}
  );
  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);
  // Value should remain unchanged
  EXPECT_EQ(x->get_parameter("test3.path_length_tolerance").as_double(), 200.0);
}

TEST(StoppedGoalChecker, is_reached)
{
  auto x = std::make_shared<TestLifecycleNode>("goal_checker");

  SimpleGoalChecker gc;
  StoppedGoalChecker sgc;
  PositionGoalChecker pgc;
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  sgc.initialize(x, "test", costmap);
  gc.initialize(x, "test2", costmap);
  pgc.initialize(x, "test3", costmap);
  geometry_msgs::msg::Pose goal_pose;
  geometry_msgs::msg::Twist velocity;
  geometry_msgs::msg::Pose current_pose;

  // Current linear x position is tolerance away from goal
  current_pose.position.x = 0.25;
  velocity.linear.x = 0.25;
  nav_msgs::msg::Path transformed_global_plan;
  EXPECT_TRUE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  sgc.reset();
  gc.reset();
  pgc.reset();

  // Current linear x speed exceeds tolerance
  velocity.linear.x = 0.25 + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  sgc.reset();
  gc.reset();
  pgc.reset();

  // Current linear x position is further than tolerance away from goal
  current_pose.position.x = 0.25 + std::numeric_limits<double>::epsilon();
  velocity.linear.x = 0.25;
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_FALSE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_FALSE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  sgc.reset();
  pgc.reset();
  current_pose.position.x = 0.0;
  velocity.linear.x = 0.0;

  // Current linear position is tolerance away from goal
  current_pose.position.x = 0.25 / std::sqrt(2);
  current_pose.position.y = 0.25 / std::sqrt(2);
  velocity.linear.x = 0.25 / std::sqrt(2);
  velocity.linear.y = 0.25 / std::sqrt(2);
  EXPECT_TRUE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  sgc.reset();
  gc.reset();
  pgc.reset();

  // Current linear speed exceeds tolerance
  velocity.linear.x = 0.25 / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  velocity.linear.y = 0.25 / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  sgc.reset();
  gc.reset();
  pgc.reset();

  // Current linear position is further than tolerance away from goal
  current_pose.position.x = 0.25 / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  current_pose.position.y = 0.25 / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  velocity.linear.x = 0.25 / std::sqrt(2);
  velocity.linear.y = 0.25 / std::sqrt(2);
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_FALSE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_FALSE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  sgc.reset();
  gc.reset();
  pgc.reset();

  current_pose.position.x = 0.0;
  velocity.linear.x = 0.0;

  // Current angular position is tolerance away from goal
  current_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.25);
  velocity.angular.z = 0.25;
  EXPECT_TRUE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  sgc.reset();
  gc.reset();
  pgc.reset();

  // Current angular speed exceeds tolerance
  velocity.angular.z = 0.25 + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  sgc.reset();
  gc.reset();
  pgc.reset();

  // Current angular position is further than tolerance away from goal
  current_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.25 + 1e-15);
  velocity.angular.z = 0.25;
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_FALSE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  sgc.reset();
  gc.reset();
  pgc.reset();

  // Looping path, xy yaw tolerance reached but path longer than path_length_tolerance
  current_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.25);
  geometry_msgs::msg::PoseStamped first_pose, second_pose, last_pose;
  first_pose.pose.position.x = 9.55;
  first_pose.pose.position.y = 7.77;
  second_pose.pose.position.x = 13.7;
  second_pose.pose.position.y = 7.84;
  last_pose.pose.position.x = 9.54;
  last_pose.pose.position.y = 7.77;
  transformed_global_plan.poses.push_back(first_pose);
  transformed_global_plan.poses.push_back(second_pose);
  transformed_global_plan.poses.push_back(last_pose);
  EXPECT_FALSE(sgc.isGoalReached(first_pose.pose, last_pose.pose, velocity,
    transformed_global_plan));
  EXPECT_FALSE(gc.isGoalReached(first_pose.pose, last_pose.pose, velocity,
    transformed_global_plan));
  EXPECT_FALSE(pgc.isGoalReached(first_pose.pose, last_pose.pose, velocity,
    transformed_global_plan));

  current_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.25 + M_PI);
  transformed_global_plan.poses.clear();
  EXPECT_FALSE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_FALSE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    x->get_node_base_interface(), x->get_node_topics_interface(),
    x->get_node_graph_interface(),
    x->get_node_services_interface());
  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test2.symmetric_yaw_tolerance", true),
      rclcpp::Parameter("test.symmetric_yaw_tolerance", true)});
  rclcpp::spin_until_future_complete(
    x->get_node_base_interface(),
    results);
  EXPECT_TRUE(sgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(gc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
  EXPECT_TRUE(pgc.isGoalReached(current_pose, goal_pose, velocity, transformed_global_plan));
}

TEST(AdaptiveToleranceGoalChecker, goal_reached)
{
  auto x = std::make_shared<TestLifecycleNode>("adaptive_tol_gc");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  geometry_msgs::msg::Twist vel;
  geometry_msgs::msg::Twist zero_vel;
  geometry_msgs::msg::Twist diff_trans_vel;
  diff_trans_vel.linear.x = 0.5;
  geometry_msgs::msg::Twist omni_vel;
  omni_vel.linear.x = 0.5;
  omni_vel.linear.y = 0.5;
  omni_vel.angular.z = 0.5;

  nav_msgs::msg::Path empty_plan;
  geometry_msgs::msg::Pose goal;
  geometry_msgs::msg::Pose current;

  const double fine_xy_tol = 0.10;
  const double coarse_xy_tol = 0.25;
  const double yaw_tol = 0.25;
  const double path_length_tol = 1.0;
  const double trans_stopped_vel = 0.10;
  const double rot_stopped_vel = 0.10;
  x->declare_parameter("pgc.fine_xy_goal_tolerance", fine_xy_tol);
  x->declare_parameter("pgc.coarse_xy_goal_tolerance", coarse_xy_tol);
  x->declare_parameter("pgc.yaw_goal_tolerance", yaw_tol);
  x->declare_parameter("pgc.path_length_tolerance", path_length_tol);
  x->declare_parameter("pgc.stateful", true);
  x->declare_parameter("pgc.symmetric_yaw_tolerance", false);
  x->declare_parameter("pgc.trans_stopped_velocity", trans_stopped_vel);
  x->declare_parameter("pgc.rot_stopped_velocity", rot_stopped_vel);
  x->declare_parameter("pgc.required_stagnation_cycles", 3);
  AdaptiveToleranceGoalChecker gc;
  gc.initialize(x, "pgc", costmap);

  // Fine tolerance: immediate accept regardless of velocity
  gc.reset();
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));
  EXPECT_TRUE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));
  EXPECT_TRUE(gc.isGoalReached(current, goal, omni_vel, empty_plan));

  gc.reset();
  current.position.x = fine_xy_tol / 2;
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));

  // Fine tolerance boundary: exactly at tolerance → accept
  gc.reset();
  current.position.x = fine_xy_tol;
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));

  // Fine tolerance boundary: just past → falls into coarse zone
  gc.reset();
  current.position.x = fine_xy_tol + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));

  // Fine tolerance diagonal: sqrt(x²+y²) = fine_xy_tol → accept
  gc.reset();
  current.position.x = fine_xy_tol / std::sqrt(2);
  current.position.y = fine_xy_tol / std::sqrt(2);
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));

  // Fine tolerance diagonal: just past → coarse zone
  gc.reset();
  current.position.x = fine_xy_tol / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  current.position.y = fine_xy_tol / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));
  current.position.y = 0.0;

  // Coarse tolerance boundary: exactly at tolerance → coarse zone, stagnates
  gc.reset();
  current.position.x = coarse_xy_tol;
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 1
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));   // count 3 → accept

  // Coarse tolerance boundary: just past → outside both, rejected
  gc.reset();
  current.position.x = coarse_xy_tol + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));

  // Coarse tolerance diagonal: sqrt(x²+y²) = coarse_xy_tol → coarse zone
  gc.reset();
  current.position.x = coarse_xy_tol / std::sqrt(2);
  current.position.y = coarse_xy_tol / std::sqrt(2);
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 1
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));   // count 3 → accept

  // Coarse tolerance diagonal: just past → outside
  gc.reset();
  current.position.x = coarse_xy_tol / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  current.position.y = coarse_xy_tol / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));
  current.position.y = 0.0;

  // Outside both tolerances: rejected
  gc.reset();
  current.position.x = coarse_xy_tol * 2;
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));

  // Coarse zone + moving at fixed position: stagnates (distance not improving)
  gc.reset();
  const double tol_diff = abs(coarse_xy_tol - fine_xy_tol);
  current.position.x = coarse_xy_tol - tol_diff / 2;
  EXPECT_FALSE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));  // count 1
  EXPECT_FALSE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));   // count 3 → accept

  // Coarse zone + stopped: accepts after stagnation cycles
  gc.reset();
  current.position.x = coarse_xy_tol - tol_diff / 2;
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 1
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));   // count 3 → accept

  // Improving distance while moving resets stall counter
  gc.reset();
  current.position.x = coarse_xy_tol - tol_diff / 2;
  EXPECT_FALSE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));  // count 1
  current.position.x = coarse_xy_tol - tol_diff / 2 - tol_diff / 4;         // closer
  // improving + moving → reset
  EXPECT_FALSE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));
  EXPECT_FALSE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));  // count 1
  EXPECT_FALSE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));   // count 3 → accept

  // Improving distance while stopped does NOT reset counter
  gc.reset();
  current.position.x = coarse_xy_tol - tol_diff / 2;
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 1
  current.position.x = coarse_xy_tol - tol_diff / 2 - tol_diff / 4;   // closer but stopped
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));   // count 3 → accept

  // Leaving zone resets counter
  gc.reset();
  current.position.x = coarse_xy_tol - tol_diff / 2;
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 1
  current.position.x = coarse_xy_tol * 2;
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // leave
  current.position.x = coarse_xy_tol - tol_diff / 2;
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // re-enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 1
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));   // count 3 → accept

  // Velocity threshold: stopped at fixed position → stagnates
  gc.reset();
  current.position.x = coarse_xy_tol - tol_diff / 2;
  vel.linear.x = trans_stopped_vel;
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // count 1
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, vel, empty_plan));   // count 3 → accept
  vel.linear.x = 0.0;

  // Velocity threshold: moving at fixed position → still stagnates (distance not improving)
  gc.reset();
  vel.linear.x = trans_stopped_vel + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // count 1
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, vel, empty_plan));   // count 3 → accept
  vel.linear.x = 0.0;

  // Rotational velocity: moving at fixed position → still stagnates
  gc.reset();
  vel.angular.z = rot_stopped_vel + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // count 1
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, vel, empty_plan));   // count 3 → accept
  vel.angular.z = 0.0;

  // Omni velocity: moving at fixed position → still stagnates
  gc.reset();
  vel.linear.x = trans_stopped_vel / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  vel.linear.y = trans_stopped_vel / std::sqrt(2) + std::numeric_limits<double>::epsilon();
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // enter
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // count 1
  EXPECT_FALSE(gc.isGoalReached(current, goal, vel, empty_plan));  // count 2
  EXPECT_TRUE(gc.isGoalReached(current, goal, vel, empty_plan));   // count 3 → accept
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;

  // Converges from coarse to fine while moving → immediate accept
  gc.reset();
  current.position.x = coarse_xy_tol - tol_diff / 2;
  EXPECT_FALSE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));
  current.position.x = fine_xy_tol / 2;
  EXPECT_TRUE(gc.isGoalReached(current, goal, diff_trans_vel, empty_plan));

  // Yaw: rejected when too far, accepted when within tolerance
  gc.reset();
  current.position.x = 0.0;
  current.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw_tol * 2);
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));
  gc.reset();
  current.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw_tol / 2);
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));

  // Symmetric yaw tolerance
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    x->get_node_base_interface(), x->get_node_topics_interface(),
    x->get_node_graph_interface(), x->get_node_services_interface());
  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("pgc.symmetric_yaw_tolerance", true)});
  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);

  gc.reset();
  current.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI + 0.1);
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));
  gc.reset();
  current.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI / 2.0);
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));

  // Stateful: XY latches after acceptance
  gc.reset();
  current.orientation = geometry_msgs::msg::Quaternion();
  current.position.x = 0.05;
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));
  current.position.x = 0.50;
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));
  gc.reset();
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, empty_plan));

  // Path length tolerance: reject if plan is too long
  gc.reset();
  current.position.x = 0.0;
  nav_msgs::msg::Path long_plan;
  geometry_msgs::msg::PoseStamped p1, p2;
  p1.pose.position.x = 0.0;
  p2.pose.position.x = 5.0;
  long_plan.poses = {p1, p2};
  EXPECT_FALSE(gc.isGoalReached(current, goal, zero_vel, long_plan));
  EXPECT_TRUE(gc.isGoalReached(current, goal, zero_vel, empty_plan));
}

TEST(AdaptiveToleranceGoalChecker, get_tol_and_dynamic_params)
{
  auto x = std::make_shared<TestLifecycleNode>("adaptive_tol_gc");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");

  const double fine_xy_tol = 0.10;
  const double coarse_xy_tol = 0.25;
  const double yaw_tol = 0.25;
  const double path_length_tol = 1.0;
  const double trans_stopped_vel = 0.10;
  const double rot_stopped_vel = 0.10;
  x->declare_parameter("pgc.fine_xy_goal_tolerance", fine_xy_tol);
  x->declare_parameter("pgc.coarse_xy_goal_tolerance", coarse_xy_tol);
  x->declare_parameter("pgc.yaw_goal_tolerance", yaw_tol);
  x->declare_parameter("pgc.path_length_tolerance", path_length_tol);
  x->declare_parameter("pgc.stateful", true);
  x->declare_parameter("pgc.symmetric_yaw_tolerance", false);
  x->declare_parameter("pgc.trans_stopped_velocity", trans_stopped_vel);
  x->declare_parameter("pgc.rot_stopped_velocity", rot_stopped_vel);
  x->declare_parameter("pgc.required_stagnation_cycles", 15);
  AdaptiveToleranceGoalChecker gc;
  gc.initialize(x, "pgc", costmap);

  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;
  EXPECT_TRUE(gc.getTolerances(pose_tol, vel_tol));
  EXPECT_DOUBLE_EQ(pose_tol.position.x, coarse_xy_tol);
  EXPECT_DOUBLE_EQ(pose_tol.position.y, coarse_xy_tol);
  EXPECT_DOUBLE_EQ(vel_tol.linear.x, trans_stopped_vel);
  EXPECT_DOUBLE_EQ(vel_tol.linear.y, trans_stopped_vel);
  EXPECT_DOUBLE_EQ(vel_tol.angular.z, rot_stopped_vel);

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    x->get_node_base_interface(), x->get_node_topics_interface(),
    x->get_node_graph_interface(), x->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("pgc.fine_xy_goal_tolerance", 0.05),
      rclcpp::Parameter("pgc.coarse_xy_goal_tolerance", 0.50),
      rclcpp::Parameter("pgc.yaw_goal_tolerance", 0.10),
      rclcpp::Parameter("pgc.path_length_tolerance", 2.0),
      rclcpp::Parameter("pgc.stateful", false),
      rclcpp::Parameter("pgc.symmetric_yaw_tolerance", true),
      rclcpp::Parameter("pgc.trans_stopped_velocity", 0.05),
      rclcpp::Parameter("pgc.rot_stopped_velocity", 0.05),
      rclcpp::Parameter("pgc.required_stagnation_cycles", 5)});
  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);

  EXPECT_EQ(x->get_parameter("pgc.fine_xy_goal_tolerance").as_double(), 0.05);
  EXPECT_EQ(x->get_parameter("pgc.coarse_xy_goal_tolerance").as_double(), 0.50);
  EXPECT_EQ(x->get_parameter("pgc.yaw_goal_tolerance").as_double(), 0.10);
  EXPECT_EQ(x->get_parameter("pgc.path_length_tolerance").as_double(), 2.0);
  EXPECT_EQ(x->get_parameter("pgc.stateful").as_bool(), false);
  EXPECT_EQ(x->get_parameter("pgc.symmetric_yaw_tolerance").as_bool(), true);
  EXPECT_EQ(x->get_parameter("pgc.trans_stopped_velocity").as_double(), 0.05);
  EXPECT_EQ(x->get_parameter("pgc.rot_stopped_velocity").as_double(), 0.05);
  EXPECT_EQ(x->get_parameter("pgc.required_stagnation_cycles").as_int(), 5);

  EXPECT_TRUE(gc.getTolerances(pose_tol, vel_tol));
  EXPECT_DOUBLE_EQ(pose_tol.position.x, 0.50);
  EXPECT_DOUBLE_EQ(vel_tol.linear.x, 0.05);

  // Invalid values rejected
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("pgc.fine_xy_goal_tolerance", -1.0)});
  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);
  EXPECT_EQ(x->get_parameter("pgc.fine_xy_goal_tolerance").as_double(), 0.05);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("pgc.coarse_xy_goal_tolerance", -1.0)});
  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);
  EXPECT_EQ(x->get_parameter("pgc.coarse_xy_goal_tolerance").as_double(), 0.50);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("pgc.yaw_goal_tolerance", -1.0)});
  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);
  EXPECT_EQ(x->get_parameter("pgc.yaw_goal_tolerance").as_double(), 0.10);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("pgc.path_length_tolerance", -1.0)});
  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);
  EXPECT_EQ(x->get_parameter("pgc.path_length_tolerance").as_double(), 2.0);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("pgc.trans_stopped_velocity", -1.0)});
  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);
  EXPECT_EQ(x->get_parameter("pgc.trans_stopped_velocity").as_double(), 0.05);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("pgc.rot_stopped_velocity", -1.0)});
  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);
  EXPECT_EQ(x->get_parameter("pgc.rot_stopped_velocity").as_double(), 0.05);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("pgc.required_stagnation_cycles", 0)});
  rclcpp::spin_until_future_complete(x->get_node_base_interface(), results);
  EXPECT_EQ(x->get_parameter("pgc.required_stagnation_cycles").as_int(), 5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
