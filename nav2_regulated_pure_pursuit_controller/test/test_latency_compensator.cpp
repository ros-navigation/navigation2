// Copyright (c) 2026 Open Navigation LLC
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

#include <cmath>

#include "gtest/gtest.h"
#include "nav2_regulated_pure_pursuit_controller/latency_compensator.hpp"

using nav2_regulated_pure_pursuit_controller::AxisModel;
using nav2_regulated_pure_pursuit_controller::LatencyCompensator;

namespace
{
geometry_msgs::msg::Twist twist(double v, double w)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = v;
  t.angular.z = w;
  return t;
}

rclcpp::Time stamp(double t)
{
  return rclcpp::Time(static_cast<int64_t>(t * 1e9));
}
}  // namespace

TEST(LatencyCompensatorTest, identityModel)
{
  AxisModel model;
  EXPECT_TRUE(model.isIdentity());
  LatencyCompensator comp(model, model);
  comp.recordCommand(twist(1.0, 0.5), stamp(10.0));

  auto pred = comp.predict(twist(0.3, 0.2), stamp(10.05));
  EXPECT_EQ(pred.x, 0.0);
  EXPECT_EQ(pred.y, 0.0);
  EXPECT_EQ(pred.yaw, 0.0);
  EXPECT_EQ(pred.linear_vel, 0.3);
  EXPECT_EQ(pred.angular_vel, 0.2);
}

TEST(LatencyCompensatorTest, pureDelayReplaysInFlightCommands)
{
  AxisModel model;
  model.delay = 0.3;
  LatencyCompensator comp(model, model);

  // Commands issued over the last 0.3s will execute over the next 0.3s
  for (int i = 0; i < 20; ++i) {
    comp.recordCommand(twist(i < 14 ? 0.0 : 1.0, 0.0), stamp(10.0 + 0.05 * i));
  }
  // Now is 11.0. The 1 m/s commands were issued from 10.7, so act from 11.0 to 11.3.
  auto pred = comp.predict(twist(0.0, 0.0), stamp(11.0));
  EXPECT_NEAR(pred.x, 0.3, 0.02);
  EXPECT_NEAR(pred.y, 0.0, 1e-9);
  EXPECT_NEAR(pred.linear_vel, 1.0, 1e-9);

  // With no history, the robot is assumed to hold its measured velocity
  comp.reset();
  pred = comp.predict(twist(0.5, 0.0), stamp(11.0));
  EXPECT_NEAR(pred.x, 0.15, 1e-6);
}

TEST(LatencyCompensatorTest, rotationIntegratesArc)
{
  AxisModel model;
  model.delay = 0.5;
  LatencyCompensator comp(model, model);
  for (int i = 0; i <= 20; ++i) {
    comp.recordCommand(twist(1.0, M_PI), stamp(9.0 + 0.05 * i));
  }
  // Half a second at pi rad/s is a semicircle of radius 1/pi
  auto pred = comp.predict(twist(1.0, M_PI), stamp(10.0));
  EXPECT_NEAR(pred.yaw, M_PI / 2.0, 1e-6);
  EXPECT_NEAR(pred.x, 1.0 / M_PI, 1e-3);
  EXPECT_NEAR(pred.y, 1.0 / M_PI, 1e-3);
}

TEST(LatencyCompensatorTest, firstOrderLag)
{
  AxisModel model;
  model.delay = 0.4;
  model.time_constant = 0.2;
  LatencyCompensator comp(model, model);
  // A step to 1 m/s that has been in flight since well before the delay
  comp.recordCommand(twist(1.0, 0.0), stamp(5.0));
  auto pred = comp.predict(twist(0.0, 0.0), stamp(10.0));
  EXPECT_NEAR(pred.linear_vel, 1.0 - std::exp(-0.4 / 0.2), 1e-3);
  // Distance is the integral of 1 - e^(-t/tau)
  EXPECT_NEAR(pred.x, 0.4 - 0.2 * (1.0 - std::exp(-0.4 / 0.2)), 1e-3);
}

TEST(LatencyCompensatorTest, modelUpdatesApplyImmediately)
{
  AxisModel model;
  LatencyCompensator comp(model, model);
  comp.recordCommand(twist(1.0, 0.0), stamp(10.0));
  EXPECT_NEAR(comp.predict(twist(1.0, 0.0), stamp(10.05)).x, 0.0, 1e-9);

  // The command history is kept when the model changes
  model.delay = 0.2;
  EXPECT_NEAR(comp.predict(twist(1.0, 0.0), stamp(10.05)).x, 0.2, 1e-6);
}

TEST(LatencyCompensatorTest, gainAndDeadband)
{
  AxisModel model;
  model.delay = 0.2;
  model.gain = 0.5;
  model.deadband = 0.1;
  EXPECT_DOUBLE_EQ(model.steadyState(0.05), 0.0);
  EXPECT_DOUBLE_EQ(model.steadyState(-0.4), -0.2);

  LatencyCompensator comp(model, model);
  comp.recordCommand(twist(0.05, 0.0), stamp(1.0));
  auto pred = comp.predict(twist(0.0, 0.0), stamp(2.0));
  EXPECT_NEAR(pred.x, 0.0, 1e-9);
}

TEST(LatencyCompensatorTest, shapeCommandInvertsModel)
{
  AxisModel model;
  model.time_constant = 0.3;
  model.gain = 0.8;
  model.deadband = 0.1;
  const double h = 0.05;

  // No shaping
  EXPECT_DOUBLE_EQ(LatencyCompensator::shapeCommand(0.5, 0.0, model, h, false, false), 0.5);
  EXPECT_DOUBLE_EQ(LatencyCompensator::shapeCommand(0.0, 0.4, model, h, true, true), 0.0);

  // Lead: a constant command from the prediction reaches the desired velocity after h
  const double v0 = 0.2, desired = 0.5;
  const double u = LatencyCompensator::shapeCommand(desired, v0, model, h, true, false);
  const double decay = std::exp(-h / model.time_constant);
  EXPECT_NEAR(u + (v0 - u) * decay, desired, 1e-9);

  // Lead never reverses direction to brake
  EXPECT_DOUBLE_EQ(LatencyCompensator::shapeCommand(0.1, 0.5, model, h, true, false), 0.0);
  EXPECT_DOUBLE_EQ(LatencyCompensator::shapeCommand(-0.1, -0.5, model, h, true, false), 0.0);

  // Gain inversion and steady state round trip
  const double cmd = LatencyCompensator::shapeCommand(0.4, 0.0, model, h, false, true);
  EXPECT_NEAR(model.steadyState(cmd), 0.4, 1e-9);

  // Deadband rounding to the edge or to zero
  EXPECT_DOUBLE_EQ(LatencyCompensator::shapeCommand(0.06, 0.0, model, h, false, true), 0.1);
  EXPECT_DOUBLE_EQ(LatencyCompensator::shapeCommand(-0.06, 0.0, model, h, false, true), -0.1);
  EXPECT_DOUBLE_EQ(LatencyCompensator::shapeCommand(0.02, 0.0, model, h, false, true), 0.0);
}

TEST(LatencyCompensatorTest, rotateStoppingSpeed)
{
  // Without latency this is the usual constant deceleration stopping speed
  EXPECT_DOUBLE_EQ(
    LatencyCompensator::rotateStoppingSpeed(0.5, 3.2, 0.0), std::sqrt(2.0 * 3.2 * 0.5));
  EXPECT_DOUBLE_EQ(
    LatencyCompensator::rotateStoppingSpeed(-0.5, 3.2, 0.0), std::sqrt(2.0 * 3.2 * 0.5));

  // With a lag, the robot's current speed carries it tau * w0 further
  const double a = 3.2, tau = 0.3, angle = 0.7, w0 = 1.0;
  const double w = LatencyCompensator::rotateStoppingSpeed(angle, a, tau, w0);
  EXPECT_NEAR(w * w / (2.0 * a) + tau * w0, angle, 1e-9);
  EXPECT_LT(w, std::sqrt(2.0 * a * angle));
  EXPECT_DOUBLE_EQ(LatencyCompensator::rotateStoppingSpeed(0.0, a, tau, 0.0), 0.0);
  // Already carried past the target by the lag, so command no further rotation
  EXPECT_DOUBLE_EQ(LatencyCompensator::rotateStoppingSpeed(0.2, a, tau, w0), 0.0);
  // Rotating away from the target leaves more room
  EXPECT_GT(
    LatencyCompensator::rotateStoppingSpeed(-0.7, a, tau, w0), std::sqrt(2.0 * a * angle));

  // Without deceleration available, never rotate
  EXPECT_DOUBLE_EQ(LatencyCompensator::rotateStoppingSpeed(0.7, 0.0, tau, w0), 0.0);

  // Simulate the ramp-down through the lag to confirm the robot stops at the target
  double cmd = LatencyCompensator::rotateStoppingSpeed(angle, a, tau, w0);
  double vel = w0, traveled = 0.0;
  const double dt = 1e-4;
  for (int i = 0; i < 100000; ++i) {
    cmd = std::max(0.0, cmd - a * dt);
    vel += (1.0 - std::exp(-dt / tau)) * (cmd - vel);
    traveled += vel * dt;
  }
  EXPECT_NEAR(traveled, angle, 1e-3);
}

TEST(LatencyCompensatorTest, historyHandlesTimeJumps)
{
  AxisModel model;
  model.delay = 0.2;
  LatencyCompensator comp(model, model);
  comp.recordCommand(twist(1.0, 0.0), stamp(100.0));
  // Time going backwards clears stale history
  comp.recordCommand(twist(0.0, 0.0), stamp(1.0));
  auto pred = comp.predict(twist(0.0, 0.0), stamp(1.05));
  EXPECT_NEAR(pred.x, 0.0, 1e-9);

  // Long histories are pruned without losing the command still in flight
  for (int i = 0; i < 5000; ++i) {
    comp.recordCommand(twist(0.5, 0.0), stamp(2.0 + 0.01 * i));
  }
  pred = comp.predict(twist(0.5, 0.0), stamp(52.0));
  EXPECT_NEAR(pred.x, 0.1, 1e-6);
}
