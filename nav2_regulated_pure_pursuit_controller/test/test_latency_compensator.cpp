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

  auto pred = comp.predict(twist(0.3, 0.2), stamp(10.05), stamp(10.05));
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
  auto pred = comp.predict(twist(0.0, 0.0), stamp(11.0), stamp(11.0));
  EXPECT_NEAR(pred.x, 0.3, 0.02);
  EXPECT_NEAR(pred.y, 0.0, 1e-9);
  EXPECT_NEAR(pred.linear_vel, 1.0, 1e-9);

  // With no history, the robot is assumed to hold its measured velocity
  comp.reset();
  pred = comp.predict(twist(0.5, 0.0), stamp(11.0), stamp(11.0));
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
  auto pred = comp.predict(twist(1.0, M_PI), stamp(10.0), stamp(10.0));
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
  auto pred = comp.predict(twist(0.0, 0.0), stamp(10.0), stamp(10.0));
  EXPECT_NEAR(pred.linear_vel, 1.0 - std::exp(-0.4 / 0.2), 1e-3);
  // Distance is the integral of 1 - e^(-t/tau)
  EXPECT_NEAR(pred.x, 0.4 - 0.2 * (1.0 - std::exp(-0.4 / 0.2)), 1e-3);
}

TEST(LatencyCompensatorTest, modelUpdatesApplyImmediately)
{
  AxisModel model;
  LatencyCompensator comp(model, model);
  comp.recordCommand(twist(1.0, 0.0), stamp(10.0));
  EXPECT_NEAR(comp.predict(twist(1.0, 0.0), stamp(10.05), stamp(10.05)).x, 0.0, 1e-9);

  // The command history is kept when the model changes
  model.delay = 0.2;
  EXPECT_NEAR(comp.predict(twist(1.0, 0.0), stamp(10.05), stamp(10.05)).x, 0.2, 1e-6);
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
  auto pred = comp.predict(twist(0.0, 0.0), stamp(2.0), stamp(2.0));
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
  EXPECT_DOUBLE_EQ(
    LatencyCompensator::shapeCommand(0.5, 0.0, model, h, false, false, false), 0.5);
  EXPECT_DOUBLE_EQ(
    LatencyCompensator::shapeCommand(0.0, 0.4, model, h, true, true, false), 0.0);

  // Lead: a constant command from the prediction reaches the desired velocity after h
  const double v0 = 0.2, desired = 0.5;
  const double u = LatencyCompensator::shapeCommand(desired, v0, model, h, true, false, false);
  const double decay = std::exp(-h / model.time_constant);
  EXPECT_NEAR(u + (v0 - u) * decay, desired, 1e-9);

  // Unless allowed, the lead never reverses direction to brake
  EXPECT_DOUBLE_EQ(
    LatencyCompensator::shapeCommand(0.1, 0.5, model, h, true, false, false), 0.0);
  EXPECT_DOUBLE_EQ(
    LatencyCompensator::shapeCommand(-0.1, -0.5, model, h, true, false, false), 0.0);
  const double brake = LatencyCompensator::shapeCommand(0.1, 0.5, model, h, true, false, true);
  EXPECT_LT(brake, 0.0);
  EXPECT_NEAR(brake + (0.5 - brake) * decay, 0.1, 1e-9);

  // Gain inversion and steady state round trip, leaving the deadband to be handled separately
  const double cmd = LatencyCompensator::shapeCommand(0.4, 0.0, model, h, false, true, false);
  EXPECT_NEAR(model.steadyState(cmd), 0.4, 1e-9);
  EXPECT_DOUBLE_EQ(
    LatencyCompensator::shapeCommand(0.04, 0.0, model, h, false, true, false), 0.05);
}

TEST(LatencyCompensatorTest, roundToDeadband)
{
  EXPECT_DOUBLE_EQ(LatencyCompensator::roundToDeadband(0.06, 0.1), 0.1);
  EXPECT_DOUBLE_EQ(LatencyCompensator::roundToDeadband(-0.06, 0.1), -0.1);
  EXPECT_DOUBLE_EQ(LatencyCompensator::roundToDeadband(0.02, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(LatencyCompensator::roundToDeadband(0.3, 0.1), 0.3);
  EXPECT_DOUBLE_EQ(LatencyCompensator::roundToDeadband(0.02, 0.0), 0.02);
}

TEST(LatencyCompensatorTest, modulateDeadband)
{
  AxisModel model;
  LatencyCompensator comp(model, model);
  const double d = 0.15;

  // Commands outside of the deadband pass through
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.3, d), 0.3);
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(-0.2, d), -0.2);
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.0, d), 0.0);

  // Commands inside of the deadband switch between the edge and zero, averaging to the command,
  // where rounding alone would stall at zero
  for (double desired : {0.05, 0.02, -0.06, 0.1}) {
    comp.reset();
    double sum = 0.0;
    const int n = 300;
    for (int i = 0; i < n; ++i) {
      const double out = comp.modulateDeadband(desired, d);
      EXPECT_TRUE(out == 0.0 || std::fabs(out) == d);
      EXPECT_GE(out * desired, 0.0);
      sum += out;
    }
    EXPECT_NEAR(sum / n, desired, d / n + 1e-9) << desired;
  }
  EXPECT_DOUBLE_EQ(LatencyCompensator::roundToDeadband(0.05, d), 0.0);

  // A stop, a command outside of the deadband, or a reset discards the carried error
  comp.reset();
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.05, d), 0.0);
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.0, d), 0.0);
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.05, d), 0.0);
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.05, d), d);
  comp.reset();
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.05, d), 0.0);
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.05, d), d);
  comp.reset();
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.05, d), 0.0);
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.3, d), 0.3);
  EXPECT_DOUBLE_EQ(comp.modulateDeadband(0.05, d), 0.0);
}

TEST(LatencyCompensatorTest, predictsToTheAngularDelay)
{
  // A robot whose speed responds slowly but which turns promptly is predicted to when a turning
  // command takes effect, not to the longer linear delay
  AxisModel linear, angular;
  linear.delay = 0.4;
  angular.delay = 0.1;
  LatencyCompensator comp(linear, angular);
  comp.recordCommand(twist(1.0, 0.0), stamp(5.0));
  EXPECT_NEAR(comp.predict(twist(1.0, 0.0), stamp(10.0), stamp(10.0)).x, 0.1, 1e-6);

  // Without an angular delay the turning command acts at once, so only the pose's age is predicted
  angular.delay = 0.0;
  EXPECT_NEAR(comp.predict(twist(1.0, 0.0), stamp(10.0), stamp(10.0)).x, 0.0, 1e-9);
  EXPECT_NEAR(comp.predict(twist(1.0, 0.0), stamp(10.0), stamp(10.05)).x, 0.05, 1e-6);
}

TEST(LatencyCompensatorTest, predictsFromMeasurementTime)
{
  AxisModel model;
  model.delay = 0.2;
  LatencyCompensator comp(model, model);

  // Commands issued after a stale measurement, which would share its timestamp, are both kept
  // and act on the robot
  comp.recordCommand(twist(1.0, 0.0), stamp(10.0));
  comp.recordCommand(twist(0.0, 0.0), stamp(10.05));
  comp.recordCommand(twist(1.0, 0.0), stamp(10.1));

  // Measured at 10.0 and issuing at 10.1, so simulate until 10.3. The first command acts over
  // [10.2, 10.25) for 0.05m, the second stops the robot and the third is not yet in effect.
  auto pred = comp.predict(twist(0.0, 0.0), stamp(10.0), stamp(10.1));
  // Within the integration timestep
  EXPECT_NEAR(pred.x, 0.05, 5e-3);

  // The horizon grows with the age of the measurement
  comp.reset();
  comp.recordCommand(twist(0.5, 0.0), stamp(9.0));
  EXPECT_NEAR(comp.predict(twist(0.5, 0.0), stamp(10.0), stamp(10.0)).x, 0.1, 1e-6);
  EXPECT_NEAR(comp.predict(twist(0.5, 0.0), stamp(10.0), stamp(10.1)).x, 0.15, 1e-6);
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
  auto pred = comp.predict(twist(0.0, 0.0), stamp(1.05), stamp(1.05));
  EXPECT_NEAR(pred.x, 0.0, 1e-9);

  // Long histories are pruned without losing the command still in flight
  for (int i = 0; i < 5000; ++i) {
    comp.recordCommand(twist(0.5, 0.0), stamp(2.0 + 0.01 * i));
  }
  pred = comp.predict(twist(0.5, 0.0), stamp(52.0), stamp(52.0));
  EXPECT_NEAR(pred.x, 0.1, 1e-6);
}
