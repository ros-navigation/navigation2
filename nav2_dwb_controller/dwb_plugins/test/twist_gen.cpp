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
#include <string>

#include "gtest/gtest.h"
#include "dwb_plugins/standard_traj_generator.hpp"
#include "dwb_plugins/limited_accel_generator.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"

using std::hypot;
using std::fabs;
using dwb_plugins::StandardTrajectoryGenerator;

geometry_msgs::msg::Pose2D origin;
nav_2d_msgs::msg::Twist2D zero;
nav_2d_msgs::msg::Twist2D forward;

class LimitedAccelGeneratorTest : public dwb_plugins::LimitedAccelGenerator
{
public:
  double getAccelerationTime()
  {
    return acceleration_time_;
  }
};

std::vector<rclcpp::Parameter> getDefaultKinematicParameters()
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("dwb.min_vel_x", 0.0));
  parameters.push_back(rclcpp::Parameter("dwb.max_vel_x", 0.55));
  parameters.push_back(rclcpp::Parameter("dwb.min_vel_y", -0.1));
  parameters.push_back(rclcpp::Parameter("dwb.max_vel_y", 0.1));
  parameters.push_back(rclcpp::Parameter("dwb.max_vel_theta", 1.0));

  parameters.push_back(rclcpp::Parameter("dwb.acc_lim_x", 2.5));
  parameters.push_back(rclcpp::Parameter("dwb.acc_lim_y", 2.5));
  parameters.push_back(rclcpp::Parameter("dwb.acc_lim_theta", 3.2));
  parameters.push_back(rclcpp::Parameter("dwb.decel_lim_x", -2.5));
  parameters.push_back(rclcpp::Parameter("dwb.decel_lim_y", -2.5));
  parameters.push_back(rclcpp::Parameter("dwb.decel_lim_theta", -3.2));

  parameters.push_back(rclcpp::Parameter("dwb.min_speed_xy", 0.1));
  parameters.push_back(rclcpp::Parameter("dwb.max_speed_xy", 0.55));
  parameters.push_back(rclcpp::Parameter("dwb.min_speed_theta", 0.4));

  return parameters;
}

rclcpp_lifecycle::LifecycleNode::SharedPtr makeTestNode(
  const std::string & name,
  const std::vector<rclcpp::Parameter> & overrides = {})
{
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(getDefaultKinematicParameters());
  node_options.parameter_overrides().insert(
    node_options.parameter_overrides().end(), overrides.begin(), overrides.end());

  auto node = rclcpp_lifecycle::LifecycleNode::make_shared(name, node_options);
  node->on_configure(node->get_current_state());
  node->on_activate(node->get_current_state());

  return node;
}

void checkLimits(
  const std::vector<nav_2d_msgs::msg::Twist2D> & twists,
  double exp_min_x, double exp_max_x, double exp_min_y, double exp_max_y,
  double exp_min_theta, double exp_max_theta,
  double exp_max_xy = -1.0,
  double exp_min_xy = -1.0, double exp_min_speed_theta = -1.0)
{
  ASSERT_GT(twists.size(), 0u);
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
  auto nh = makeTestNode("st_gen");
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  EXPECT_EQ(twists.size(), 1926u);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0, 0.55, 0.1, 0.4);
}

TEST(VelocityIterator, max_xy)
{
  auto nh = makeTestNode("max_xy", {rclcpp::Parameter("dwb.max_speed_xy", 1.0)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");

  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  // Expect more twists since max_speed_xy is now beyond feasible limits
  EXPECT_EQ(twists.size(), 2010u);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0, hypot(0.55, 0.1));
}

TEST(VelocityIterator, min_xy)
{
  auto nh = makeTestNode("min_xy", {rclcpp::Parameter("dwb.min_speed_xy", -1.0)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  // Expect even more since theres no min_speed_xy
  EXPECT_EQ(twists.size(), 2015u);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0);
}

TEST(VelocityIterator, min_theta)
{
  auto nh = makeTestNode("min_theta", {rclcpp::Parameter("dwb.min_speed_theta", -1.0)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  // Expect even more since theres no min_speed_xy
  EXPECT_EQ(twists.size(), 2015u);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0);
}

TEST(VelocityIterator, no_limits)
{
  auto nh = makeTestNode(
    "no_limits", {
    rclcpp::Parameter("dwb.max_speed_xy", -1.0),
    rclcpp::Parameter("dwb.min_speed_xy", -1.0),
    rclcpp::Parameter("dwb.min_speed_theta", -1.0)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  // vx_samples * vtheta_samples * vy_samples + added zero theta samples - (0,0,0)
  EXPECT_EQ(twists.size(), 20u * 20u * 5u + 100u - 1u);
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0, hypot(0.55, 0.1), 0.0, 0.0);
}

TEST(VelocityIterator, no_limits_samples)
{
  const int x_samples = 10, y_samples = 3, theta_samples = 5;
  auto nh = makeTestNode(
    "no_limits_samples", {
    rclcpp::Parameter("dwb.max_speed_xy", -1.0),
    rclcpp::Parameter("dwb.min_speed_xy", -1.0),
    rclcpp::Parameter("dwb.min_speed_theta", -1.0),
    rclcpp::Parameter("dwb.vx_samples", x_samples),
    rclcpp::Parameter("dwb.vy_samples", y_samples),
    rclcpp::Parameter("dwb.vtheta_samples", theta_samples)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  EXPECT_EQ(twists.size(), static_cast<unsigned>(x_samples * y_samples * theta_samples - 1));
  checkLimits(twists, 0.0, 0.55, -0.1, 0.1, -1.0, 1.0, hypot(0.55, 0.1), 0.0, 0.0);
}

TEST(VelocityIterator, dwa_gen)
{
  auto nh = makeTestNode("dwa_gen", {rclcpp::Parameter("dwb.min_speed_theta", -1.0)});
  dwb_plugins::LimitedAccelGenerator gen;
  gen.initialize(nh, "dwb");
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(zero);
  // Same as no-limits since everything is within our velocity limits
  EXPECT_EQ(twists.size(), 20u * 20u * 5u + 100u - 1u);
  checkLimits(twists, 0.0, 0.125, -0.1, 0.1, -0.16, 0.16, hypot(0.125, 0.1), 0.0, 0.1);
}

TEST(VelocityIterator, dwa_gen_zero_frequency)
{
  auto nh = makeTestNode("dwa_gen");
  nh->declare_parameter("controller_frequency", 0.0);
  LimitedAccelGeneratorTest gen;
  gen.initialize(nh, "dwb");
  // Default value should be 0.05
  EXPECT_EQ(gen.getAccelerationTime(), 0.05);
}

TEST(VelocityIterator, dwa_gen_one_frequency)
{
  auto nh = makeTestNode("dwa_gen");
  nh->declare_parameter("controller_frequency", 1.0);
  LimitedAccelGeneratorTest gen;
  gen.initialize(nh, "dwb");
  EXPECT_EQ(gen.getAccelerationTime(), 1.0);
}

TEST(VelocityIterator, dwa_gen_ten_frequency)
{
  auto nh = makeTestNode("dwa_gen");
  nh->declare_parameter("controller_frequency", 10.0);
  LimitedAccelGeneratorTest gen;
  gen.initialize(nh, "dwb");
  EXPECT_EQ(gen.getAccelerationTime(), 0.1);
}

TEST(VelocityIterator, dwa_gen_fifty_frequency)
{
  auto nh = makeTestNode("dwa_gen");
  nh->declare_parameter("controller_frequency", 50.0);
  LimitedAccelGeneratorTest gen;
  gen.initialize(nh, "dwb");
  EXPECT_EQ(gen.getAccelerationTime(), 0.02);
}

TEST(VelocityIterator, dwa_gen_hundred_frequency)
{
  auto nh = makeTestNode("dwa_gen");
  nh->declare_parameter("controller_frequency", 100.0);
  LimitedAccelGeneratorTest gen;
  gen.initialize(nh, "dwb");
  EXPECT_EQ(gen.getAccelerationTime(), 0.01);
}

TEST(VelocityIterator, nonzero)
{
  auto nh = makeTestNode("nonzero", {rclcpp::Parameter("dwb.min_speed_theta", -1.0)});
  dwb_plugins::LimitedAccelGenerator gen;
  gen.initialize(nh, "dwb");
  nav_2d_msgs::msg::Twist2D initial;
  initial.x = 0.1;
  initial.y = -0.08;
  initial.theta = 0.05;
  std::vector<nav_2d_msgs::msg::Twist2D> twists = gen.getTwists(initial);
  EXPECT_EQ(twists.size(), 2519u);
  checkLimits(
    twists, 0.0, 0.225, -0.1, 0.045, -0.11000000000000003, 0.21,
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

const double DEFAULT_SIM_TIME = 1.7;

TEST(TrajectoryGenerator, basic)
{
  auto nh = makeTestNode("basic", {rclcpp::Parameter("dwb.linear_granularity", 0.5)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, forward, forward);
  matchTwist(res.velocity, forward);
  EXPECT_DOUBLE_EQ(durationToSec(res.time_offsets.back()), DEFAULT_SIM_TIME);
  int n = res.poses.size();
  EXPECT_EQ(n, 4);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
  matchPose(res.poses[n - 1], DEFAULT_SIM_TIME * forward.x, 0, 0);
}

TEST(TrajectoryGenerator, basic_no_last_point)
{
  auto nh = makeTestNode(
    "basic_no_last_point", {
    rclcpp::Parameter("dwb.include_last_point", false),
    rclcpp::Parameter("dwb.linear_granularity", 0.5)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, forward, forward);
  matchTwist(res.velocity, forward);
  EXPECT_DOUBLE_EQ(durationToSec(res.time_offsets.back()), DEFAULT_SIM_TIME / 2);
  int n = res.poses.size();
  EXPECT_EQ(n, 3);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
  matchPose(res.poses[n - 2], 0.255, 0, 0);
}

TEST(TrajectoryGenerator, too_slow)
{
  auto nh = makeTestNode("too_slow", {rclcpp::Parameter("dwb.linear_granularity", 0.5)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  nav_2d_msgs::msg::Twist2D cmd;
  cmd.x = 0.2;
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, cmd, cmd);
  matchTwist(res.velocity, cmd);
  EXPECT_DOUBLE_EQ(durationToSec(res.time_offsets.back()), DEFAULT_SIM_TIME);
  int n = res.poses.size();
  EXPECT_EQ(n, 3);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
}

TEST(TrajectoryGenerator, holonomic)
{
  auto nh = makeTestNode("holonomic", {rclcpp::Parameter("dwb.linear_granularity", 0.5)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  nav_2d_msgs::msg::Twist2D cmd;
  cmd.x = 0.3;
  cmd.y = 0.2;
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, cmd, cmd);
  matchTwist(res.velocity, cmd);
  EXPECT_DOUBLE_EQ(durationToSec(res.time_offsets.back()), DEFAULT_SIM_TIME);
  int n = res.poses.size();
  EXPECT_EQ(n, 4);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
  matchPose(res.poses[n - 1], cmd.x * DEFAULT_SIM_TIME, cmd.y * DEFAULT_SIM_TIME, 0);
}

TEST(TrajectoryGenerator, twisty)
{
  auto nh = makeTestNode(
    "twisty", {
    rclcpp::Parameter("dwb.linear_granularity", 0.5),
    rclcpp::Parameter("dwb.angular_granularity", 0.025)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  nav_2d_msgs::msg::Twist2D cmd;
  cmd.x = 0.3;
  cmd.y = -0.2;
  cmd.theta = 0.111;
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, cmd, cmd);
  matchTwist(res.velocity, cmd);
  EXPECT_NEAR(durationToSec(res.time_offsets.back()), DEFAULT_SIM_TIME, 1.0E-5);
  int n = res.poses.size();
  EXPECT_EQ(n, 10);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
  matchPose(
    res.poses[n - 1], 0.5355173615993063, -0.29635287789821596,
    cmd.theta * DEFAULT_SIM_TIME);
}

TEST(TrajectoryGenerator, sim_time)
{
  const double sim_time = 2.5;
  auto nh = makeTestNode(
    "sim_time", {
    rclcpp::Parameter("dwb.sim_time", sim_time),
    rclcpp::Parameter("dwb.linear_granularity", 0.5)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");
  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, forward, forward);
  matchTwist(res.velocity, forward);
  EXPECT_DOUBLE_EQ(durationToSec(res.time_offsets.back()), sim_time);
  int n = res.poses.size();
  EXPECT_EQ(n, 4);
  ASSERT_GT(n, 0);

  matchPose(res.poses[0], origin);
  matchPose(res.poses[n - 2], sim_time * forward.x, 0, 0);
}

TEST(TrajectoryGenerator, accel)
{
  auto nh = makeTestNode(
    "accel", {
    rclcpp::Parameter("dwb.sim_time", 5.0),
    rclcpp::Parameter("dwb.discretize_by_time", true),
    rclcpp::Parameter("dwb.time_granularity", 1.0),
    rclcpp::Parameter("dwb.acc_lim_x", 0.1),
    rclcpp::Parameter("dwb.min_speed_xy", -1.0)});
  StandardTrajectoryGenerator gen;
  gen.initialize(nh, "dwb");

  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, zero, forward);
  matchTwist(res.velocity, forward);
  EXPECT_DOUBLE_EQ(durationToSec(res.time_offsets.back()), 5.0);
  ASSERT_EQ(res.poses.size(), 7u);
  matchPose(res.poses[0], origin);
  matchPose(res.poses[1], 0.1, 0, 0);
  matchPose(res.poses[2], 0.3, 0, 0);
  matchPose(res.poses[3], 0.6, 0, 0);
  matchPose(res.poses[4], 0.9, 0, 0);
  matchPose(res.poses[5], 1.2, 0, 0);
}

TEST(TrajectoryGenerator, dwa)
{
  auto nh = makeTestNode(
    "dwa", {
    rclcpp::Parameter("dwb.sim_period", 1.0),
    rclcpp::Parameter("dwb.sim_time", 5.0),
    rclcpp::Parameter("dwb.discretize_by_time", true),
    rclcpp::Parameter("dwb.time_granularity", 1.0),
    rclcpp::Parameter("dwb.acc_lim_x", 0.1),
    rclcpp::Parameter("dwb.min_speed_xy", -1.0)});
  dwb_plugins::LimitedAccelGenerator gen;
  gen.initialize(nh, "dwb");

  dwb_msgs::msg::Trajectory2D res = gen.generateTrajectory(origin, zero, forward);
  matchTwist(res.velocity, forward);
  EXPECT_DOUBLE_EQ(durationToSec(res.time_offsets.back()), 5.0);
  ASSERT_EQ(res.poses.size(), 7u);
  matchPose(res.poses[0], origin);
  matchPose(res.poses[1], 0.3, 0, 0);
  matchPose(res.poses[2], 0.6, 0, 0);
  matchPose(res.poses[3], 0.9, 0, 0);
  matchPose(res.poses[4], 1.2, 0, 0);
  matchPose(res.poses[5], 1.5, 0, 0);
}

int main(int argc, char ** argv)
{
  forward.x = 0.3;
  rclcpp::init(0, nullptr);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
