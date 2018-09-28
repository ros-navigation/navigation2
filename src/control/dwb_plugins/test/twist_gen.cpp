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

#include <cmath>
#include <vector>
#include <algorithm>
#include "gtest/gtest.h"
#include "dwb_plugins/standard_traj_generator.h"
#include "dwb_plugins/limited_accel_generator.h"
#include "dwb_core/exceptions.h"

using std::hypot;
using std::fabs;
using dwb_plugins::StandardTrajectoryGenerator;

geometry_msgs::msg::Pose2D origin;
nav_2d_msgs::msg::Twist2D zero;
nav_2d_msgs::msg::Twist2D forward;

void checkLimits(
  const std::vector<nav_2d_msgs::msg::Twist2D> & twists,
  double exp_min_x, double exp_max_x, double exp_min_y, double exp_max_y,
  double exp_min_theta, double exp_max_theta,
  double exp_max_xy = -1.0,
  double exp_min_xy = -1.0, double exp_min_speed_theta = -1.0)
{
  ASSERT_GT(twists.size(), 0);
  nav_2d_msgs::msg::Twist2D first = twists[0];

  double min_x = first.x, max_x = first.x, min_y = first.y, max_y = first.y;
  double min_theta = first.theta, max_theta = first.theta;
  double max_xy = hypot(first.x, first.y);

  for (nav_2d_msgs::msg::Twist2D twist : twists) {
    min_x = std::min(min_x, twist.x);
    min_y = std::min(min_y, twist.y);
    min_theta = std::min(min_theta, twist.theta);
    max_x = std::max(max_x, twist.x);
    max_y = std::max(max_y, twist.y);
    max_theta = std::max(max_theta, twist.theta);
    double hyp = hypot(twist.x, twist.y);
    max_xy = std::max(max_xy, hyp);

    if (exp_min_xy >= 0 && exp_min_speed_theta >= 0) {
      EXPECT_TRUE(fabs(twist.theta) >= exp_min_speed_theta || hyp >= exp_min_xy);
    }
  }
  EXPECT_DOUBLE_EQ(min_x, exp_min_x);
  EXPECT_DOUBLE_EQ(max_x, exp_max_x);
  EXPECT_DOUBLE_EQ(min_y, exp_min_y);
  EXPECT_DOUBLE_EQ(max_y, exp_max_y);
  EXPECT_DOUBLE_EQ(min_theta, exp_min_theta);
  EXPECT_DOUBLE_EQ(max_theta, exp_max_theta);
  if (exp_max_xy >= 0) {
    EXPECT_DOUBLE_EQ(max_xy, exp_max_xy);
  }
}

double durationToSec(builtin_interfaces::msg::Duration d)
{
  return d.sec + d.nanosec * 1e-9;
}

TEST(VelocityIterator, standard_gen)
{
  auto nh = rclcpp::Node::make_shared("st_gen");
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  EXPECT_EQ(twists.size(), 1926);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0, 0.55, 0.1, 0.4);
}

TEST(VelocityIterator, max_xy)
{
  auto nh = rclcpp::Node::make_shared("max_xy");
  nh->set_parameters({rclcpp::Parameter("max_speed_xy", 1.0)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);

  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  // Expect more twists since max_speed_xy is now beyond feasible limits
  EXPECT_EQ(twists.size(), 2010);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0, hypot(0.55, 0.1));
}

TEST(VelocityIterator, min_xy)
{
  auto nh = rclcpp::Node::make_shared("min_xy");
  nh->set_parameters({rclcpp::Parameter("min_speed_xy", -1)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  // Expect even more since theres no min_speed_xy
  EXPECT_EQ(twists.size(), 2015);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0);
}

TEST(VelocityIterator, min_theta)
{
  auto nh = rclcpp::Node::make_shared("min_theta");
  nh->set_parameters({rclcpp::Parameter("min_speed_theta", -1)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  // Expect even more since theres no min_speed_xy
  EXPECT_EQ(twists.size(), 2015);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0);
}

TEST(VelocityIterator, no_limits)
{
  auto nh = rclcpp::Node::make_shared("no_limits");
  nh->set_parameters({rclcpp::Parameter("max_speed_xy", -1.0)});
  nh->set_parameters({rclcpp::Parameter("min_speed_xy", -1)});
  nh->set_parameters({rclcpp::Parameter("min_speed_theta", -1)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  // vx_samples * vtheta_samples * vy_samples + added zero theta samples - (0,0,0)
  EXPECT_EQ(twists.size(), 20 * 20 * 5 + 100 - 1);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0, hypot(0.55, 0.1), 0.0, 0.0);
}

TEST(VelocityIterator, no_limits_samples)
{
  auto nh = rclcpp::Node::make_shared("no_limits_samples");
  nh->set_parameters({rclcpp::Parameter("max_speed_xy", -1.0)});
  nh->set_parameters({rclcpp::Parameter("min_speed_xy", -1)});
  nh->set_parameters({rclcpp::Parameter("min_speed_theta", -1)});
  int x_samples = 10, y_samples = 3, theta_samples = 5;
  nh->set_parameters({rclcpp::Parameter("vx_samples", x_samples)});
  nh->set_parameters({rclcpp::Parameter("vy_samples", y_samples)});
  nh->set_parameters({rclcpp::Parameter("vtheta_samples", theta_samples)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  EXPECT_EQ(twists.size(), x_samples * y_samples * theta_samples - 1);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0, hypot(0.55, 0.1), 0.0, 0.0);
}

TEST(VelocityIterator, dwa_gen_exception)
{
  auto nh = rclcpp::Node::make_shared("dwa_gen_exception");
  nh->set_parameters({rclcpp::Parameter("use_dwa", true)});
  StandardTrajectoryGenerator gen;
  EXPECT_THROW(gen.initialize(nh), nav_core2::PlannerException);
}

TEST(VelocityIterator, no_dwa_gen_exception)
{
  auto nh = rclcpp::Node::make_shared("no_dwa_gen_exception");
  nh->set_parameters({rclcpp::Parameter("use_dwa", false)});
  dwb_plugins::LimitedAccelGenerator gen;
  EXPECT_THROW(gen.initialize(nh), nav_core2::PlannerException);
}

TEST(VelocityIterator, dwa_gen)
{
  auto nh = rclcpp::Node::make_shared("dwa_gen");
  nh->set_parameters({rclcpp::Parameter("use_dwa", true)});
  nh->set_parameters({rclcpp::Parameter("min_speed_theta", -1)});
  dwb_plugins::LimitedAccelGenerator gen;
  gen.initialize(nh);
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  // Same as no-limits since everything is within our velocity limits
  EXPECT_EQ(twists.size(), 20 * 20 * 5 + 100 - 1);
  checkLimits(twists, 0.0, 0.125, -0.1, 0.1, -0.16, 0.16, hypot(0.125, 0.1), 0.0, 0.1);
}

TEST(VelocityIterator, dwa_gen_no_param)
{
  auto nh = rclcpp::Node::make_shared("dwa_gen_no_param");
  nh->set_parameters({rclcpp::Parameter("min_speed_theta", -1)});
  dwb_plugins::LimitedAccelGenerator gen;
  gen.initialize(nh);
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  EXPECT_EQ(twists.size(), 20 * 20 * 5 + 100 - 1);
  checkLimits(twists, 0.0, 0.125, -0.1, 0.1, -0.16, 0.16, hypot(0.125, 0.1), 0.0, 0.1);
}

TEST(VelocityIterator, nonzero)
{
  auto nh = rclcpp::Node::make_shared("nonzero");
  nh->set_parameters({rclcpp::Parameter("use_dwa", true)});
  nh->set_parameters({rclcpp::Parameter("min_speed_theta", -1)});
  dwb_plugins::LimitedAccelGenerator gen;
  gen.initialize(nh);
  nav_2d_msgs::msg::Twist2D initial;
  initial.x = 0.1;
  initial.y = -0.08;
  initial.theta = 0.05;
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(initial);
  EXPECT_EQ(twists.size(), 2519);
  checkLimits(twists, 0.0, 0.225, -0.1, 0.045, -0.11000000000000003, 0.21,
    0.24622144504490268, 0.0, 0.1);
}

void matchPose(const geometry_msgs::msg::Pose2D & a, const geometry_msgs::msg::Pose2D & b)
{
  EXPECT_DOUBLE_EQ(a.x, b.x);
  EXPECT_DOUBLE_EQ(a.y, b.y);
  EXPECT_DOUBLE_EQ(a.theta, b.theta);
}

void matchPose(
  const geometry_msgs::msg::Pose2D & a, const double x, const double y,
  const double theta)
{
  EXPECT_DOUBLE_EQ(a.x, x);
  EXPECT_DOUBLE_EQ(a.y, y);
  EXPECT_DOUBLE_EQ(a.theta, theta);
}

void matchTwist(const nav_2d_msgs::msg::Twist2D & a, const nav_2d_msgs::msg::Twist2D & b)
{
  EXPECT_DOUBLE_EQ(a.x, b.x);
  EXPECT_DOUBLE_EQ(a.y, b.y);
  EXPECT_DOUBLE_EQ(a.theta, b.theta);
}

void matchTwist(
  const nav_2d_msgs::msg::Twist2D & a, const double x, const double y,
  const double theta)
{
  EXPECT_DOUBLE_EQ(a.x, x);
  EXPECT_DOUBLE_EQ(a.y, y);
  EXPECT_DOUBLE_EQ(a.theta, theta);
}

TEST(TrajectoryGenerator, basic)
{
  auto nh = rclcpp::Node::make_shared("basic");
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, forward, forward);
  matchTwist(res.velocity, forward);
  EXPECT_DOUBLE_EQ(durationToSec(res.duration), 1.7);
  int n = res.poses.size();
  EXPECT_EQ(n, 2);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
  matchPose(res.poses[n - 1], 0.255, 0, 0);
}

TEST(TrajectoryGenerator, too_slow)
{
  auto nh = rclcpp::Node::make_shared("too_slow");
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);
  nav_2d_msgs::msg::Twist2D cmd;
  cmd.x = 0.2;
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, cmd, cmd);
  matchTwist(res.velocity, cmd);
  EXPECT_DOUBLE_EQ(durationToSec(res.duration), 1.7);
  int n = res.poses.size();
  EXPECT_EQ(n, 1);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
}

TEST(TrajectoryGenerator, holonomic)
{
  auto nh = rclcpp::Node::make_shared("holonomic");
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);
  nav_2d_msgs::msg::Twist2D cmd;
  cmd.x = 0.3;
  cmd.y = 0.2;
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, cmd, cmd);
  matchTwist(res.velocity, cmd);
  EXPECT_DOUBLE_EQ(durationToSec(res.duration), 1.7);
  int n = res.poses.size();
  EXPECT_EQ(n, 2);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
  matchPose(res.poses[n - 1], 0.255, 0.17, 0);
}

TEST(TrajectoryGenerator, twisty)
{
  auto nh = rclcpp::Node::make_shared("twisty");
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);
  nav_2d_msgs::msg::Twist2D cmd;
  cmd.x = 0.3;
  cmd.y = -0.2;
  cmd.theta = 0.111;
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, cmd, cmd);
  matchTwist(res.velocity, cmd);
  EXPECT_DOUBLE_EQ(durationToSec(res.duration), 1.7);
  int n = res.poses.size();
  EXPECT_EQ(n, 8);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
  matchPose(res.poses[n - 1], 0.4656489295054273, -0.2649090438962528, 0.16511250000000002);
}

TEST(TrajectoryGenerator, sim_time)
{
  auto nh = rclcpp::Node::make_shared("sim_time");
  nh->set_parameters({rclcpp::Parameter("sim_time", 2.5)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, forward, forward);
  matchTwist(res.velocity, forward);
  EXPECT_DOUBLE_EQ(durationToSec(res.duration), 2.5);
  int n = res.poses.size();
  EXPECT_EQ(n, 2);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
  matchPose(res.poses[n - 1], 0.375, 0, 0);
}

TEST(TrajectoryGenerator, accel)
{
  auto nh = rclcpp::Node::make_shared("accel");
  nh->set_parameters({rclcpp::Parameter("sim_time", 5.0)});
  nh->set_parameters({rclcpp::Parameter("discretize_by_time", true)});
  nh->set_parameters({rclcpp::Parameter("sim_granularity", 1.0)});
  nh->set_parameters({rclcpp::Parameter("acc_lim_x", 0.1)});
  nh->set_parameters({rclcpp::Parameter("min_speed_xy", -1.0)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh);

  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, zero, forward);
  matchTwist(res.velocity, forward);
  EXPECT_DOUBLE_EQ(durationToSec(res.duration), 5.0);
  ASSERT_EQ(res.poses.size(), 5);
  matchPose(res.poses[0], origin);
  matchPose(res.poses[1], 0.1, 0, 0);
  matchPose(res.poses[2], 0.3, 0, 0);
  matchPose(res.poses[3], 0.6, 0, 0);
  matchPose(res.poses[4], 0.9, 0, 0);
}

TEST(TrajectoryGenerator, dwa)
{
  auto nh = rclcpp::Node::make_shared("dwa");
  nh->set_parameters({rclcpp::Parameter("use_dwa", true)});
  nh->set_parameters({rclcpp::Parameter("sim_period", 1.0)});
  nh->set_parameters({rclcpp::Parameter("sim_time", 5.0)});
  nh->set_parameters({rclcpp::Parameter("discretize_by_time", true)});
  nh->set_parameters({rclcpp::Parameter("sim_granularity", 1.0)});
  nh->set_parameters({rclcpp::Parameter("acc_lim_x", 0.1)});
  nh->set_parameters({rclcpp::Parameter("min_speed_xy", -1.0)});
  dwb_plugins::LimitedAccelGenerator gen;
  gen.initialize(nh);

  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, zero, forward);
  matchTwist(res.velocity, forward);
  EXPECT_DOUBLE_EQ(durationToSec(res.duration), 5.0);
  ASSERT_EQ(res.poses.size(), 5);
  matchPose(res.poses[0], origin);
  matchPose(res.poses[1], 0.3, 0, 0);
  matchPose(res.poses[2], 0.6, 0, 0);
  matchPose(res.poses[3], 0.9, 0, 0);
  matchPose(res.poses[4], 1.2, 0, 0);
}

int main(int argc, char ** argv)
{
  forward.x = 0.3;
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
