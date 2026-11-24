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

#include "nav2_regulated_pure_pursuit_controller/latency_compensation_functions.hpp"
#include "gtest/gtest.h"

using namespace nav2_regulated_pure_pursuit_controller::latency_compensation; // NOLINT

namespace
{
geometry_msgs::msg::Twist twist(double v, double w)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = v;
  t.angular.z = w;
  return t;
}

PredictedState predict(
  const CommandHistory & history, const geometry_msgs::msg::Twist & speed,
  double measurement_time, double now, double delay, double tau = 0.0)
{
  return predictState(history, speed, measurement_time, now, delay, delay, tau, tau);
}
}  // namespace

TEST(LatencyCompensationTest, noModel)
{
  CommandHistory history;
  recordCommand(history, twist(1.0, 0.5), 10.0, 0.0);
  auto pred = predict(history, twist(0.3, 0.2), 10.05, 10.05, 0.0);
  EXPECT_EQ(pred.x, 0.0);
  EXPECT_EQ(pred.y, 0.0);
  EXPECT_EQ(pred.yaw, 0.0);
  EXPECT_EQ(pred.linear_vel, 0.3);
  EXPECT_EQ(pred.angular_vel, 0.2);
}

TEST(LatencyCompensationTest, pureDelayReplaysInFlightCommands)
{
  // Commands issued over the last 0.3s will execute over the next 0.3s
  CommandHistory history;
  for (int i = 0; i < 20; ++i) {
    recordCommand(history, twist(i < 14 ? 0.0 : 1.0, 0.0), 10.0 + 0.05 * i, 0.3);
  }
  // Now is 11.0. The 1 m/s commands were issued from 10.7, so act from 11.0 to 11.3.
  auto pred = predict(history, twist(0.0, 0.0), 11.0, 11.0, 0.3);
  EXPECT_NEAR(pred.x, 0.3, 0.02);
  EXPECT_NEAR(pred.y, 0.0, 1e-9);
  EXPECT_NEAR(pred.linear_vel, 1.0, 1e-9);

  // With no history, the robot is assumed to hold its measured velocity
  pred = predict(CommandHistory(), twist(0.5, 0.0), 11.0, 11.0, 0.3);
  EXPECT_NEAR(pred.x, 0.15, 1e-6);
}

TEST(LatencyCompensationTest, rotationIntegratesArc)
{
  CommandHistory history;
  for (int i = 0; i <= 20; ++i) {
    recordCommand(history, twist(1.0, M_PI), 9.0 + 0.05 * i, 0.5);
  }
  // Half a second at pi rad/s is a semicircle of radius 1/pi
  auto pred = predict(history, twist(1.0, M_PI), 10.0, 10.0, 0.5);
  EXPECT_NEAR(pred.yaw, M_PI / 2.0, 1e-6);
  EXPECT_NEAR(pred.x, 1.0 / M_PI, 1e-3);
  EXPECT_NEAR(pred.y, 1.0 / M_PI, 1e-3);
}

TEST(LatencyCompensationTest, firstOrderLag)
{
  // A step to 1 m/s that has been in flight since well before the delay
  CommandHistory history;
  recordCommand(history, twist(1.0, 0.0), 5.0, 0.4);
  auto pred = predict(history, twist(0.0, 0.0), 10.0, 10.0, 0.4, 0.2);
  EXPECT_NEAR(pred.linear_vel, 1.0 - std::exp(-0.4 / 0.2), 1e-3);
  // Distance is the integral of 1 - e^(-t/tau)
  EXPECT_NEAR(pred.x, 0.4 - 0.2 * (1.0 - std::exp(-0.4 / 0.2)), 1e-3);
}

TEST(LatencyCompensationTest, predictsToTheAngularDelay)
{
  // A robot whose speed responds slowly but which turns promptly is predicted to when a turning
  // command takes effect, not to the longer linear delay
  CommandHistory history;
  recordCommand(history, twist(1.0, 0.0), 5.0, 0.4);
  auto speed = twist(1.0, 0.0);
  EXPECT_NEAR(predictState(history, speed, 10.0, 10.0, 0.4, 0.1, 0.0, 0.0).x, 0.1, 1e-6);
  // Without an angular delay the turning command acts at once, so only the pose's age is predicted
  EXPECT_NEAR(predictState(history, speed, 10.0, 10.0, 0.4, 0.0, 0.0, 0.0).x, 0.0, 1e-9);
  EXPECT_NEAR(predictState(history, speed, 10.0, 10.05, 0.4, 0.0, 0.0, 0.0).x, 0.05, 1e-6);
}

TEST(LatencyCompensationTest, predictsFromMeasurementTime)
{
  // Commands issued after a stale measurement, which would share its timestamp, are all kept
  // and act on the robot
  CommandHistory history;
  recordCommand(history, twist(1.0, 0.0), 10.0, 0.2);
  recordCommand(history, twist(0.0, 0.0), 10.05, 0.2);
  recordCommand(history, twist(1.0, 0.0), 10.1, 0.2);
  // Measured at 10.0 and issuing at 10.1, so simulate until 10.3. The first command acts over
  // [10.2, 10.25) for 0.05m, the second stops the robot and the third is not yet in effect.
  EXPECT_NEAR(predict(history, twist(0.0, 0.0), 10.0, 10.1, 0.2).x, 0.05, 5e-3);

  // The horizon grows with the age of the measurement
  history.clear();
  recordCommand(history, twist(0.5, 0.0), 9.0, 0.2);
  EXPECT_NEAR(predict(history, twist(0.5, 0.0), 10.0, 10.0, 0.2).x, 0.1, 1e-6);
  EXPECT_NEAR(predict(history, twist(0.5, 0.0), 10.0, 10.1, 0.2).x, 0.15, 1e-6);
}

TEST(LatencyCompensationTest, recordCommand)
{
  CommandHistory history;
  // A command at the same time replaces the previous one
  recordCommand(history, twist(1.0, 0.0), 10.0, 0.2);
  recordCommand(history, twist(0.5, 0.0), 10.0, 0.2);
  ASSERT_EQ(history.size(), 1u);
  EXPECT_EQ(history.back().linear, 0.5);

  // Commands no longer acting on the robot are dropped, keeping the one still in effect
  for (int i = 1; i <= 100; ++i) {
    recordCommand(history, twist(1.0, 0.0), 10.0 + 0.05 * i, 0.2);
  }
  EXPECT_LE(history.size(), 26u);
  EXPECT_LE(history.front().stamp, 15.0 - 0.2 - 1.0);

  // Time jumping backwards clears the history
  recordCommand(history, twist(0.0, 0.0), 1.0, 0.2);
  ASSERT_EQ(history.size(), 1u);
  EXPECT_EQ(history.front().stamp, 1.0);
}

TEST(LatencyCompensationTest, transformPlanToPredictedFrame)
{
  // Straight path along x in the robot frame, 0.1m spacing
  nav_msgs::msg::Path path;
  for (int i = 0; i <= 20; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = -0.5 + 0.1 * i;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }

  PredictedState prediction;
  prediction.x = 0.3;
  prediction.y = 0.1;
  prediction.yaw = 0.2;
  auto predicted = transformPlanToPredictedFrame(path, prediction);

  // The path behind the predicted pose is dropped, starting from the closest point to it
  EXPECT_EQ(predicted.poses.size(), 13u);
  // Re-expressed in the predicted frame
  const double dx = 0.3 - prediction.x, dy = -prediction.y;
  const double c = std::cos(prediction.yaw), s = std::sin(prediction.yaw);
  EXPECT_NEAR(predicted.poses.front().pose.position.x, c * dx + s * dy, 1e-9);
  EXPECT_NEAR(predicted.poses.front().pose.position.y, -s * dx + c * dy, 1e-9);
  EXPECT_NEAR(tf2::getYaw(predicted.poses.front().pose.orientation), -prediction.yaw, 1e-9);

  // Predicted past the end of the path, the last 2 poses are kept for the path's direction
  prediction.x = 1.7;
  prediction.y = 0.0;
  prediction.yaw = 0.0;
  EXPECT_EQ(transformPlanToPredictedFrame(path, prediction).poses.size(), 2u);
}
