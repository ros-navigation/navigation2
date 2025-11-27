// Copyright (c) 2025 Fumiya Ohnishi
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

#include "nav2_regulated_pure_pursuit_controller/dynamic_window_pure_pursuit_functions.hpp"
#include "gtest/gtest.h"

using namespace nav2_regulated_pure_pursuit_controller::dynamic_window_pure_pursuit; // NOLINT


TEST(DynamicWindowPurePursuitTest, computeDynamicWindow)
{
  double max_linear_vel = 0.5;
  double min_linear_vel = -0.5;
  double max_angular_vel = 1.0;
  double min_angular_vel = -1.0;
  double max_linear_accel = 0.5;
  double max_linear_decel = 1.0;
  double max_angular_accel = 1.0;
  double max_angular_decel = 2.0;
  double control_duration = 1.0 / 20.0;

  double dynamic_window_max_linear_vel, dynamic_window_min_linear_vel,
    dynamic_window_max_angular_vel, dynamic_window_min_angular_vel;
  geometry_msgs::msg::Twist current_speed = geometry_msgs::msg::Twist();

  // case1: current linear velocity is positive, angular velocity is positive,
  // clip by max linear vel
  current_speed.linear.x = 0.5; current_speed.angular.z = 0.2;

  std::tie(dynamic_window_max_linear_vel, dynamic_window_min_linear_vel,
    dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel) = computeDynamicWindow(
    current_speed, max_linear_vel, min_linear_vel,
    max_angular_vel, min_angular_vel,
    max_linear_accel, max_linear_decel,
    max_angular_accel, max_angular_decel, control_duration);

  EXPECT_EQ(dynamic_window_max_linear_vel, 0.5);
  EXPECT_EQ(dynamic_window_min_linear_vel, 0.45);
  EXPECT_EQ(dynamic_window_max_angular_vel, 0.25);
  EXPECT_EQ(dynamic_window_min_angular_vel, 0.10);

  // case2: current linear velocity is positive, angular velocity is negative
  current_speed.linear.x = 0.3; current_speed.angular.z = -0.2;

  std::tie(dynamic_window_max_linear_vel, dynamic_window_min_linear_vel,
    dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel) = computeDynamicWindow(
    current_speed, max_linear_vel, min_linear_vel,
    max_angular_vel, min_angular_vel,
    max_linear_accel, max_linear_decel,
    max_angular_accel, max_angular_decel, control_duration);

  EXPECT_EQ(dynamic_window_max_linear_vel, 0.325);
  EXPECT_EQ(dynamic_window_min_linear_vel, 0.25);
  EXPECT_EQ(dynamic_window_max_angular_vel, -0.1);
  EXPECT_EQ(dynamic_window_min_angular_vel, -0.25);

  // case3: current linear velocity is zero, angular velocity is zero
  current_speed.linear.x = 0.0; current_speed.angular.z = -0.0;

  std::tie(dynamic_window_max_linear_vel, dynamic_window_min_linear_vel,
    dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel) = computeDynamicWindow(
    current_speed, max_linear_vel, min_linear_vel,
    max_angular_vel, min_angular_vel,
    max_linear_accel, max_linear_decel,
    max_angular_accel, max_angular_decel, control_duration);

  EXPECT_EQ(dynamic_window_max_linear_vel, 0.025);
  EXPECT_EQ(dynamic_window_min_linear_vel, -0.025);
  EXPECT_EQ(dynamic_window_max_angular_vel, 0.05);
  EXPECT_EQ(dynamic_window_min_angular_vel, -0.05);

  // case4: current linear velocity is negative, angular velocity is positive
  current_speed.linear.x = -0.3; current_speed.angular.z = 0.2;

  std::tie(dynamic_window_max_linear_vel, dynamic_window_min_linear_vel,
    dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel) = computeDynamicWindow(
    current_speed, max_linear_vel, min_linear_vel,
    max_angular_vel, min_angular_vel,
    max_linear_accel, max_linear_decel,
    max_angular_accel, max_angular_decel, control_duration);

  EXPECT_EQ(dynamic_window_max_linear_vel, -0.25);
  EXPECT_EQ(dynamic_window_min_linear_vel, -0.325);
  EXPECT_EQ(dynamic_window_max_angular_vel, 0.25);
  EXPECT_EQ(dynamic_window_min_angular_vel, 0.10);

  // case5: current linear velocity is negative, angular velocity is negative,
  // clipped by min lenear vel
  current_speed.linear.x = -0.5; current_speed.angular.z = -0.2;

  std::tie(dynamic_window_max_linear_vel, dynamic_window_min_linear_vel,
    dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel) = computeDynamicWindow(
    current_speed, max_linear_vel, min_linear_vel,
    max_angular_vel, min_angular_vel,
    max_linear_accel, max_linear_decel,
    max_angular_accel, max_angular_decel, control_duration);

  EXPECT_EQ(dynamic_window_max_linear_vel, -0.45);
  EXPECT_EQ(dynamic_window_min_linear_vel, -0.5);
  EXPECT_EQ(dynamic_window_max_angular_vel, -0.1);
  EXPECT_EQ(dynamic_window_min_angular_vel, -0.25);
}

TEST(DynamicWindowPurePursuitTest, applyRegulationToDynamicWindow)
{
  double regulated_linear_vel;
  double dynamic_window_max_linear_vel, dynamic_window_min_linear_vel;

  double regulated_dynamic_window_max_linear_vel, regulated_dynamic_window_min_linear_vel;

  regulated_linear_vel = 0.3;
  dynamic_window_max_linear_vel = 0.5, dynamic_window_min_linear_vel = 0.4;

  std::tie(regulated_dynamic_window_max_linear_vel,
    regulated_dynamic_window_min_linear_vel) = applyRegulationToDynamicWindow(
    regulated_linear_vel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  EXPECT_EQ(regulated_dynamic_window_max_linear_vel, 0.4);
  EXPECT_EQ(regulated_dynamic_window_min_linear_vel, 0.4);

  dynamic_window_max_linear_vel = 0.4, dynamic_window_min_linear_vel = 0.2;

  std::tie(regulated_dynamic_window_max_linear_vel,
    regulated_dynamic_window_min_linear_vel) = applyRegulationToDynamicWindow(
    regulated_linear_vel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  EXPECT_EQ(regulated_dynamic_window_max_linear_vel, 0.3);
  EXPECT_EQ(regulated_dynamic_window_min_linear_vel, 0.2);

  dynamic_window_max_linear_vel = 0.2, dynamic_window_min_linear_vel = 0.1;

  std::tie(regulated_dynamic_window_max_linear_vel,
    regulated_dynamic_window_min_linear_vel) = applyRegulationToDynamicWindow(
    regulated_linear_vel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  EXPECT_EQ(regulated_dynamic_window_max_linear_vel, 0.2);
  EXPECT_EQ(regulated_dynamic_window_min_linear_vel, 0.1);

  dynamic_window_max_linear_vel = 0.1, dynamic_window_min_linear_vel = -0.2;

  std::tie(regulated_dynamic_window_max_linear_vel,
    regulated_dynamic_window_min_linear_vel) = applyRegulationToDynamicWindow(
    regulated_linear_vel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  EXPECT_EQ(regulated_dynamic_window_max_linear_vel, 0.1);
  EXPECT_EQ(regulated_dynamic_window_min_linear_vel, 0.0);

  dynamic_window_max_linear_vel = -0.1, dynamic_window_min_linear_vel = -0.3;

  std::tie(regulated_dynamic_window_max_linear_vel,
    regulated_dynamic_window_min_linear_vel) = applyRegulationToDynamicWindow(
    regulated_linear_vel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  EXPECT_EQ(regulated_dynamic_window_max_linear_vel, -0.1);
  EXPECT_EQ(regulated_dynamic_window_min_linear_vel, -0.1);

  regulated_linear_vel = -0.3;
  dynamic_window_max_linear_vel = 0.3, dynamic_window_min_linear_vel = 0.1;

  std::tie(regulated_dynamic_window_max_linear_vel,
    regulated_dynamic_window_min_linear_vel) = applyRegulationToDynamicWindow(
    regulated_linear_vel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  EXPECT_EQ(regulated_dynamic_window_max_linear_vel, 0.1);
  EXPECT_EQ(regulated_dynamic_window_min_linear_vel, 0.1);

  dynamic_window_max_linear_vel = 0.1, dynamic_window_min_linear_vel = -0.2;

  std::tie(regulated_dynamic_window_max_linear_vel,
    regulated_dynamic_window_min_linear_vel) = applyRegulationToDynamicWindow(
    regulated_linear_vel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  EXPECT_EQ(regulated_dynamic_window_max_linear_vel, 0.0);
  EXPECT_EQ(regulated_dynamic_window_min_linear_vel, -0.2);

  dynamic_window_max_linear_vel = -0.2, dynamic_window_min_linear_vel = -0.3;

  std::tie(regulated_dynamic_window_max_linear_vel,
    regulated_dynamic_window_min_linear_vel) = applyRegulationToDynamicWindow(
    regulated_linear_vel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  EXPECT_EQ(regulated_dynamic_window_max_linear_vel, -0.2);
  EXPECT_EQ(regulated_dynamic_window_min_linear_vel, -0.3);

  dynamic_window_max_linear_vel = -0.2, dynamic_window_min_linear_vel = -0.4;

  std::tie(regulated_dynamic_window_max_linear_vel,
    regulated_dynamic_window_min_linear_vel) = applyRegulationToDynamicWindow(
    regulated_linear_vel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  EXPECT_EQ(regulated_dynamic_window_max_linear_vel, -0.2);
  EXPECT_EQ(regulated_dynamic_window_min_linear_vel, -0.3);

  dynamic_window_max_linear_vel = -0.4, dynamic_window_min_linear_vel = -0.5;

  std::tie(regulated_dynamic_window_max_linear_vel,
    regulated_dynamic_window_min_linear_vel) = applyRegulationToDynamicWindow(
    regulated_linear_vel,
    dynamic_window_max_linear_vel,
    dynamic_window_min_linear_vel);

  EXPECT_EQ(regulated_dynamic_window_max_linear_vel, -0.4);
  EXPECT_EQ(regulated_dynamic_window_min_linear_vel, -0.4);
}

TEST(DynamicWindowPurePursuitTest, computeOptimalVelocityWithinDynamicWindow)
{
  double dynamic_window_max_linear_vel = 0.3, dynamic_window_min_linear_vel = 0.1;
  double dynamic_window_max_angular_vel = 0.1, dynamic_window_min_angular_vel = -0.1;
  double curvature = 0.0;
  double sign = 1.0;
  double optimal_linear_vel;
  double optimal_angular_vel;

  std::tie(optimal_linear_vel,
    optimal_angular_vel) = computeOptimalVelocityWithinDynamicWindow(
    dynamic_window_max_linear_vel, dynamic_window_min_linear_vel, dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel, curvature, sign);

  EXPECT_EQ(optimal_linear_vel, 0.3);
  EXPECT_EQ(optimal_angular_vel, 0.0);


  dynamic_window_max_linear_vel = 0.3, dynamic_window_min_linear_vel = 0.1;
  dynamic_window_max_angular_vel = 0.3, dynamic_window_min_angular_vel = 0.1;

  std::tie(optimal_linear_vel,
    optimal_angular_vel) = computeOptimalVelocityWithinDynamicWindow(
    dynamic_window_max_linear_vel, dynamic_window_min_linear_vel, dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel, curvature, sign);

  EXPECT_EQ(optimal_linear_vel, 0.3);
  EXPECT_EQ(optimal_angular_vel, 0.1);

  curvature = 1.0;
  dynamic_window_max_linear_vel = 0.2, dynamic_window_min_linear_vel = 0.0;
  dynamic_window_max_angular_vel = 0.3, dynamic_window_min_angular_vel = 0.1;

  std::tie(optimal_linear_vel,
    optimal_angular_vel) = computeOptimalVelocityWithinDynamicWindow(
    dynamic_window_max_linear_vel, dynamic_window_min_linear_vel, dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel, curvature, sign);

  EXPECT_EQ(optimal_linear_vel, 0.2);
  EXPECT_EQ(optimal_angular_vel, 0.2);

  dynamic_window_max_linear_vel = 0.4, dynamic_window_min_linear_vel = 0.3;
  dynamic_window_max_angular_vel = 0.2, dynamic_window_min_angular_vel = 0.1;

  std::tie(optimal_linear_vel,
    optimal_angular_vel) = computeOptimalVelocityWithinDynamicWindow(
    dynamic_window_max_linear_vel, dynamic_window_min_linear_vel, dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel, curvature, sign);

  EXPECT_EQ(optimal_linear_vel, 0.3);
  EXPECT_EQ(optimal_angular_vel, 0.2);

  sign = -1.0;
  curvature = 0.0;
  dynamic_window_max_linear_vel = 0.3, dynamic_window_min_linear_vel = 0.1;
  dynamic_window_max_angular_vel = 0.1, dynamic_window_min_angular_vel = -0.1;

  std::tie(optimal_linear_vel,
    optimal_angular_vel) = computeOptimalVelocityWithinDynamicWindow(
    dynamic_window_max_linear_vel, dynamic_window_min_linear_vel, dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel, curvature, sign);

  EXPECT_EQ(optimal_linear_vel, 0.1);
  EXPECT_EQ(optimal_angular_vel, 0.0);

  dynamic_window_max_linear_vel = 0.3, dynamic_window_min_linear_vel = 0.1;
  dynamic_window_max_angular_vel = 0.3, dynamic_window_min_angular_vel = 0.1;

  std::tie(optimal_linear_vel,
    optimal_angular_vel) = computeOptimalVelocityWithinDynamicWindow(
    dynamic_window_max_linear_vel, dynamic_window_min_linear_vel, dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel, curvature, sign);

  EXPECT_EQ(optimal_linear_vel, 0.1);
  EXPECT_EQ(optimal_angular_vel, 0.1);

  curvature = 1.0;
  dynamic_window_max_linear_vel = 0.2, dynamic_window_min_linear_vel = 0.0;
  dynamic_window_max_angular_vel = 0.3, dynamic_window_min_angular_vel = 0.1;

  std::tie(optimal_linear_vel,
    optimal_angular_vel) = computeOptimalVelocityWithinDynamicWindow(
    dynamic_window_max_linear_vel, dynamic_window_min_linear_vel, dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel, curvature, sign);

  EXPECT_EQ(optimal_linear_vel, 0.1);
  EXPECT_EQ(optimal_angular_vel, 0.1);

  dynamic_window_max_linear_vel = 0.4, dynamic_window_min_linear_vel = 0.3;
  dynamic_window_max_angular_vel = 0.2, dynamic_window_min_angular_vel = 0.1;

  std::tie(optimal_linear_vel,
    optimal_angular_vel) = computeOptimalVelocityWithinDynamicWindow(
    dynamic_window_max_linear_vel, dynamic_window_min_linear_vel, dynamic_window_max_angular_vel,
    dynamic_window_min_angular_vel, curvature, sign);

  EXPECT_EQ(optimal_linear_vel, 0.3);
  EXPECT_EQ(optimal_angular_vel, 0.2);
}
