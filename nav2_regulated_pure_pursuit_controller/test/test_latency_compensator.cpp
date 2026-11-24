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

#include "nav2_regulated_pure_pursuit_controller/latency_compensator.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.hpp"
#include "gtest/gtest.h"

using nav2_regulated_pure_pursuit_controller::LatencyCompensator;
using nav2_regulated_pure_pursuit_controller::Parameters;
using nav2_regulated_pure_pursuit_controller::PredictedState;

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
  return rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
}

Parameters model(double delay, double tau = 0.0)
{
  Parameters p{};
  p.model_delay_vx = delay;
  p.model_delay_wz = delay;
  p.model_time_constant_vx = tau;
  p.model_time_constant_wz = tau;
  return p;
}

double yaw(const PredictedState & s)
{
  return tf2::getYaw(s.pose.orientation);
}
}  // namespace

TEST(LatencyCompensatorTest, noModel)
{
  Parameters params = model(0.0);
  LatencyCompensator comp(&params);
  EXPECT_FALSE(comp.isEnabled());
  comp.recordCommand(twist(1.0, 0.5), stamp(10.0));
  auto pred = comp.predict(twist(0.3, 0.2), stamp(10.05), stamp(10.05));
  EXPECT_EQ(pred.pose.position.x, 0.0);
  EXPECT_EQ(pred.pose.position.y, 0.0);
  EXPECT_EQ(yaw(pred), 0.0);
  EXPECT_EQ(pred.velocity.linear.x, 0.3);
  EXPECT_EQ(pred.velocity.angular.z, 0.2);
  params.model_time_constant_wz = 0.1;
  EXPECT_TRUE(comp.isEnabled());
}

TEST(LatencyCompensatorTest, pureDelayReplaysInFlightCommands)
{
  // Commands issued over the last 0.3s will execute over the next 0.3s
  Parameters params = model(0.3);
  LatencyCompensator comp(&params);
  for (int i = 0; i < 20; ++i) {
    comp.recordCommand(twist(i < 14 ? 0.0 : 1.0, 0.0), stamp(10.0 + 0.05 * i));
  }
  // Now is 11.0. The 1 m/s commands were issued from 10.7, so act from 11.0 to 11.3.
  auto pred = comp.predict(twist(0.0, 0.0), stamp(11.0), stamp(11.0));
  EXPECT_NEAR(pred.pose.position.x, 0.3, 0.02);
  EXPECT_NEAR(pred.pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(pred.velocity.linear.x, 1.0, 1e-9);

  // With no history, the robot is assumed to hold its measured velocity
  comp.reset();
  pred = comp.predict(twist(0.5, 0.0), stamp(11.0), stamp(11.0));
  EXPECT_NEAR(pred.pose.position.x, 0.15, 1e-6);
}

TEST(LatencyCompensatorTest, rotationIntegratesArc)
{
  Parameters params = model(0.5);
  LatencyCompensator comp(&params);
  for (int i = 0; i <= 20; ++i) {
    comp.recordCommand(twist(1.0, M_PI), stamp(9.0 + 0.05 * i));
  }
  // Half a second at pi rad/s is a semicircle of radius 1/pi
  auto pred = comp.predict(twist(1.0, M_PI), stamp(10.0), stamp(10.0));
  EXPECT_NEAR(yaw(pred), M_PI / 2.0, 1e-6);
  EXPECT_NEAR(pred.pose.position.x, 1.0 / M_PI, 1e-3);
  EXPECT_NEAR(pred.pose.position.y, 1.0 / M_PI, 1e-3);
}

TEST(LatencyCompensatorTest, firstOrderLag)
{
  // A step to 1 m/s that has been in flight since well before the delay
  Parameters params = model(0.4, 0.2);
  LatencyCompensator comp(&params);
  comp.recordCommand(twist(1.0, 0.0), stamp(5.0));
  auto pred = comp.predict(twist(0.0, 0.0), stamp(10.0), stamp(10.0));
  EXPECT_NEAR(pred.velocity.linear.x, 1.0 - std::exp(-0.4 / 0.2), 1e-3);
  // Distance is the integral of 1 - e^(-t/tau)
  EXPECT_NEAR(pred.pose.position.x, 0.4 - 0.2 * (1.0 - std::exp(-0.4 / 0.2)), 1e-3);
}

TEST(LatencyCompensatorTest, predictsToTheAngularDelay)
{
  // A robot whose speed responds slowly but which turns promptly is predicted to when a turning
  // command takes effect, not to the longer linear delay
  Parameters params = model(0.4);
  params.model_delay_wz = 0.1;
  LatencyCompensator comp(&params);
  comp.recordCommand(twist(1.0, 0.0), stamp(5.0));
  EXPECT_NEAR(comp.predict(twist(1.0, 0.0), stamp(10.0), stamp(10.0)).pose.position.x, 0.1, 1e-6);
  // Without an angular delay the turning command acts at once, so only the pose's age is predicted
  params.model_delay_wz = 0.0;
  EXPECT_NEAR(comp.predict(twist(1.0, 0.0), stamp(10.0), stamp(10.0)).pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(
    comp.predict(twist(1.0, 0.0), stamp(10.0), stamp(10.05)).pose.position.x, 0.05, 1e-6);
}

TEST(LatencyCompensatorTest, predictsFromMeasurementTime)
{
  // Commands issued after a stale measurement, which would share its timestamp, are all kept
  // and act on the robot
  Parameters params = model(0.2);
  LatencyCompensator comp(&params);
  comp.recordCommand(twist(1.0, 0.0), stamp(10.0));
  comp.recordCommand(twist(0.0, 0.0), stamp(10.05));
  comp.recordCommand(twist(1.0, 0.0), stamp(10.1));
  // Measured at 10.0 and issuing at 10.1, so simulate until 10.3. The first command acts over
  // [10.2, 10.25) for 0.05m, the second stops the robot and the third is not yet in effect.
  EXPECT_NEAR(
    comp.predict(twist(0.0, 0.0), stamp(10.0), stamp(10.1)).pose.position.x, 0.05, 5e-3);

  // The horizon grows with the age of the measurement
  comp.reset();
  comp.recordCommand(twist(0.5, 0.0), stamp(9.0));
  EXPECT_NEAR(comp.predict(twist(0.5, 0.0), stamp(10.0), stamp(10.0)).pose.position.x, 0.1, 1e-6);
  EXPECT_NEAR(
    comp.predict(twist(0.5, 0.0), stamp(10.0), stamp(10.1)).pose.position.x, 0.15, 1e-6);
}

TEST(LatencyCompensatorTest, recordCommand)
{
  Parameters params = model(0.2);
  LatencyCompensator comp(&params);
  // The latest command issued at a time is the one acting on the robot
  comp.recordCommand(twist(1.0, 0.0), stamp(9.0));
  comp.recordCommand(twist(0.0, 0.0), stamp(10.0));
  comp.recordCommand(twist(0.5, 0.0), stamp(10.0));
  EXPECT_NEAR(comp.predict(twist(0.5, 0.0), stamp(10.2), stamp(10.2)).pose.position.x, 0.1, 1e-6);

  // Commands still in flight are kept as older ones are dropped
  for (int i = 1; i <= 100; ++i) {
    comp.recordCommand(twist(i < 98 ? 0.5 : 0.0, 0.0), stamp(10.0 + 0.05 * i));
  }
  // Issued 0.5 m/s until 14.85 and stopped since, so moves for 0.1s of the next 0.2s
  EXPECT_NEAR(comp.predict(twist(0.5, 0.0), stamp(15.0), stamp(15.0)).pose.position.x, 0.05, 5e-3);

  // After time jumps backwards, the commands issued since are used
  comp.recordCommand(twist(1.0, 0.0), stamp(1.0));
  EXPECT_NEAR(comp.predict(twist(1.0, 0.0), stamp(1.2), stamp(1.2)).pose.position.x, 0.2, 1e-6);
}

TEST(LatencyCompensatorTest, transformPlanToPredictedFrame)
{
  // Straight path along x in the robot frame, 0.1m spacing
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = -0.5 + 0.1 * i;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }

  PredictedState prediction;
  prediction.pose.position.x = 0.3;
  prediction.pose.position.y = 0.1;
  prediction.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.2);
  auto predicted = nav2_regulated_pure_pursuit_controller::transformPlanToPredictedFrame(
    path, prediction);

  // The path behind the predicted pose is dropped, starting from the closest point to it
  ASSERT_EQ(predicted.poses.size(), 13u);
  EXPECT_EQ(predicted.poses.front().header.frame_id, "base_link");
  // Re-expressed in the predicted frame
  const double dx = 0.3 - 0.3, dy = -0.1;
  const double c = std::cos(0.2), s = std::sin(0.2);
  EXPECT_NEAR(predicted.poses.front().pose.position.x, c * dx + s * dy, 1e-9);
  EXPECT_NEAR(predicted.poses.front().pose.position.y, -s * dx + c * dy, 1e-9);
  EXPECT_NEAR(tf2::getYaw(predicted.poses.front().pose.orientation), -0.2, 1e-9);

  // Predicted past the end of the path, the last 2 poses are kept for the path's direction
  prediction.pose.position.x = 1.7;
  prediction.pose.position.y = 0.0;
  prediction.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.0);
  EXPECT_EQ(
    nav2_regulated_pure_pursuit_controller::transformPlanToPredictedFrame(path, prediction)
    .poses.size(), 2u);
}
