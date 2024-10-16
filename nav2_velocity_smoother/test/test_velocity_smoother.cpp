// Copyright (c) 2022 Samsung Research
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_velocity_smoother/velocity_smoother.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/twist_subscriber.hpp"

using namespace std::chrono_literals;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class VelSmootherShim : public nav2_velocity_smoother::VelocitySmoother
{
public:
  VelSmootherShim()
  : VelocitySmoother() {}
  nav2_util::CallbackReturn configure(const rclcpp_lifecycle::State & state)
  {
    return this->on_configure(state);
  }
  void activate(const rclcpp_lifecycle::State & state) {this->on_activate(state);}
  void deactivate(const rclcpp_lifecycle::State & state) {this->on_deactivate(state);}
  void cleanup(const rclcpp_lifecycle::State & state) {this->on_cleanup(state);}
  void shutdown(const rclcpp_lifecycle::State & state) {this->on_shutdown(state);}

  bool isOdomSmoother() {return odom_smoother_ ? true : false;}
  bool hasCommandMsg() {return last_command_time_.nanoseconds() != 0;}
  geometry_msgs::msg::TwistStamped::SharedPtr lastCommandMsg() {return command_;}

  void sendCommandMsg(geometry_msgs::msg::Twist::SharedPtr msg) {inputCommandCallback(msg);}
};

TEST(VelocitySmootherTest, openLoopTestTimer)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  std::vector<double> deadbands{0.2, 0.0, 0.0};
  smoother->declare_parameter("scale_velocities", rclcpp::ParameterValue(true));
  smoother->set_parameter(rclcpp::Parameter("scale_velocities", true));
  smoother->declare_parameter("deadband_velocity", rclcpp::ParameterValue(deadbands));
  smoother->set_parameter(rclcpp::Parameter("deadband_velocity", deadbands));
  rclcpp_lifecycle::State state;
  smoother->configure(state);
  smoother->activate(state);

  std::vector<double> linear_vels;
  auto subscription = nav2_util::TwistSubscriber(
    smoother,
    "cmd_vel_smoothed",
    1,
    [&](geometry_msgs::msg::Twist::SharedPtr msg) {
      linear_vels.push_back(msg->linear.x);
    }, [&](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      linear_vels.push_back(msg->twist.linear.x);
    });

  // Send a velocity command
  auto cmd = std::make_shared<geometry_msgs::msg::Twist>();
  cmd->linear.x = 1.0;  // Max is 0.5, so should threshold
  smoother->sendCommandMsg(cmd);

  // Process velocity smoothing and send updated odometry based on commands
  auto start = smoother->now();
  while (smoother->now() - start < 1.5s) {
    rclcpp::spin_some(smoother->get_node_base_interface());
  }

  // Sanity check we have the approximately right number of messages for the timespan and timeout
  EXPECT_GT(linear_vels.size(), 19u);
  EXPECT_LT(linear_vels.size(), 30u);

  // Should have last command be a stop since we timed out the command stream
  EXPECT_EQ(linear_vels.back(), 0.0);

  // From deadband, first few should be 0 until above 0.2
  for (unsigned int i = 0; i != linear_vels.size(); i++) {
    if (linear_vels[i] != 0) {
      EXPECT_GT(linear_vels[i], 0.2);
      break;
    }
  }

  // Process to make sure stops at limit in velocity,
  // doesn't exceed acceleration
  for (unsigned int i = 0; i != linear_vels.size(); i++) {
    EXPECT_TRUE(linear_vels[i] <= 0.5);
  }
}

TEST(VelocitySmootherTest, approxClosedLoopTestTimer)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  smoother->declare_parameter("feedback", rclcpp::ParameterValue(std::string("CLOSED_LOOP")));
  smoother->set_parameter(rclcpp::Parameter("feedback", std::string("CLOSED_LOOP")));
  rclcpp_lifecycle::State state;
  smoother->configure(state);
  smoother->activate(state);

  std::vector<double> linear_vels;
  auto subscription = nav2_util::TwistSubscriber(
    smoother,
    "cmd_vel_smoothed",
    1,
    [&](geometry_msgs::msg::Twist::SharedPtr msg) {
      linear_vels.push_back(msg->linear.x);
    }, [&](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      linear_vels.push_back(msg->twist.linear.x);
    });

  auto odom_pub = smoother->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  odom_pub->on_activate();
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  // Fill buffer with 0 twisted-commands
  for (unsigned int i = 0; i != 30; i++) {
    odom_msg.header.stamp = smoother->now() + rclcpp::Duration::from_seconds(i * 0.01);
    odom_pub->publish(odom_msg);
  }

  // Send a velocity command
  auto cmd = std::make_shared<geometry_msgs::msg::Twist>();
  cmd->linear.x = 1.0;  // Max is 0.5, so should threshold
  smoother->sendCommandMsg(cmd);

  // Process velocity smoothing and send updated odometry based on commands
  auto start = smoother->now();
  while (smoother->now() - start < 1.5s) {
    odom_msg.header.stamp = smoother->now();
    if (linear_vels.size() > 0) {
      odom_msg.twist.twist.linear.x = linear_vels.back();
    }
    odom_pub->publish(odom_msg);
    rclcpp::spin_some(smoother->get_node_base_interface());
  }

  // Sanity check we have the approximately right number of messages for the timespan and timeout
  EXPECT_GT(linear_vels.size(), 19u);
  EXPECT_LT(linear_vels.size(), 30u);

  // Should have last command be a stop since we timed out the command stream
  EXPECT_EQ(linear_vels.back(), 0.0);

  // Process to make sure stops at limit in velocity,
  // doesn't exceed acceleration
  for (unsigned int i = 0; i != linear_vels.size(); i++) {
    if (i > 0) {
      double diff = linear_vels[i] - linear_vels[i - 1];
      EXPECT_LT(diff, 0.126);  // default accel of 0.5 / 20 hz = 0.125
    }

    EXPECT_TRUE(linear_vels[i] <= 0.5);
  }
}

TEST(VelocitySmootherTest, testfindEtaConstraint)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);

  double accel = 0.1;    // dv 0.005
  double decel = -1.0;  // dv 0.05

  // In range
  // Constant positive
  EXPECT_EQ(smoother->findEtaConstraint(1.0, 1.0, accel, decel), -1);
  // Constant negative
  EXPECT_EQ(smoother->findEtaConstraint(-1.0, -1.0, accel, decel), -1);
  // Positive To Positive Accel
  EXPECT_EQ(smoother->findEtaConstraint(0.5, 0.504, accel, decel), -1);
  // Positive To Positive Decel
  EXPECT_EQ(smoother->findEtaConstraint(0.5, 0.46, accel, decel), -1);
  // 0 To Positive Accel
  EXPECT_EQ(smoother->findEtaConstraint(0.0, 0.004, accel, decel), -1);
  // Positive To 0 Decel
  EXPECT_EQ(smoother->findEtaConstraint(0.04, 0.0, accel, decel), -1);
  // Negative To Negative Accel
  EXPECT_EQ(smoother->findEtaConstraint(-0.5, -0.504, accel, decel), -1);
  // Negative To Negative Decel
  EXPECT_EQ(smoother->findEtaConstraint(-0.5, -0.46, accel, decel), -1);
  // 0 To Negative Accel
  EXPECT_EQ(smoother->findEtaConstraint(0.0, -0.004, accel, decel), -1);
  // Negative To 0 Decel
  EXPECT_EQ(smoother->findEtaConstraint(-0.04, 0.0, accel, decel), -1);
  // Negative to Positive
  EXPECT_EQ(smoother->findEtaConstraint(-0.02, 0.02, accel, decel), -1);
  // Positive to Negative
  EXPECT_EQ(smoother->findEtaConstraint(0.02, -0.02, accel, decel), -1);

  // Faster than limit
  // Positive To Positive Accel
  EXPECT_EQ(smoother->findEtaConstraint(0.5, 1.5, accel, decel), 0.005);
  // Positive To Positive Decel
  EXPECT_EQ(smoother->findEtaConstraint(1.5, 0.5, accel, decel), 0.05);
  // 0 To Positive Accel
  EXPECT_EQ(smoother->findEtaConstraint(0.0, 1.0, accel, decel), 0.005);
  // Positive To 0 Decel
  EXPECT_EQ(smoother->findEtaConstraint(1.0, 0.0, accel, decel), 0.05);
  // Negative To Negative Accel
  EXPECT_EQ(smoother->findEtaConstraint(-0.5, -1.5, accel, decel), 0.005);
  // Negative To Negative Decel
  EXPECT_EQ(smoother->findEtaConstraint(-1.5, -0.5, accel, decel), 0.05);
  // 0 To Negative Accel
  EXPECT_EQ(smoother->findEtaConstraint(0.0, -1.0, accel, decel), 0.005);
  // Negative To 0 Decel
  EXPECT_EQ(smoother->findEtaConstraint(-1.0, 0.0, accel, decel), 0.05);
  // Negative to Positive
  EXPECT_EQ(smoother->findEtaConstraint(-0.2, 0.8, accel, decel), 0.05);
  // Positive to Negative
  EXPECT_EQ(smoother->findEtaConstraint(0.2, -0.8, accel, decel), 0.05);
}

TEST(VelocitySmootherTest, testapplyConstraints)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;

  // Apply examples from testfindEtaConstraint
  // In range, so no eta or acceleration limit impact
  EXPECT_EQ(smoother->applyConstraints(1.0, 1.0, 1.5, -2.0, no_eta), 1.0);
  EXPECT_EQ(smoother->applyConstraints(0.5, 0.55, 1.5, -2.0, no_eta), 0.55);
  EXPECT_EQ(smoother->applyConstraints(0.5, 0.45, 1.5, -2.0, no_eta), 0.45);
  // Too high, without eta
  EXPECT_NEAR(smoother->applyConstraints(1.0, 2.0, 1.5, -2.0, no_eta), 1.075, 0.01);
  // Too high, with eta applied on its own axis
  EXPECT_NEAR(smoother->applyConstraints(1.0, 2.0, 1.5, -2.0, 0.075), 1.075, 0.01);
  // On another virtual axis that is OK
  EXPECT_NEAR(smoother->applyConstraints(0.5, 0.55, 1.5, -2.0, 0.075), 0.503, 0.01);

  // In a more realistic situation, applied to angular
  EXPECT_NEAR(smoother->applyConstraints(0.8, 1.0, 3.2, -3.2, 0.75), 1.075, 0.95);
}

TEST(VelocitySmootherTest, testapplyConstraintsPositiveToPositiveAccel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;
  double accel = 0.1;
  double decel = -1.0;
  double dv_accel = accel / 20.0;
  double init_vel, target_vel, v_curr;
  uint steps_to_target;

  init_vel = 1.0;
  target_vel = 2.0;
  steps_to_target = abs(target_vel - init_vel) / dv_accel;

  v_curr = init_vel;
  for (size_t i = 1; i < steps_to_target + 1; i++) {
    v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
    EXPECT_NEAR(v_curr, init_vel + i * dv_accel, 0.001);
  }
  EXPECT_NEAR(v_curr, target_vel, 0.001);
  v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
  EXPECT_NEAR(v_curr, target_vel, 0.001);
}

TEST(VelocitySmootherTest, testapplyConstraintsZeroToPositiveAccel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;
  double accel = 0.1;
  double decel = -1.0;
  double dv_accel = accel / 20.0;
  double init_vel, target_vel, v_curr;
  uint steps_to_target;

  init_vel = 0.0;
  target_vel = 2.0;
  steps_to_target = abs(target_vel - init_vel) / dv_accel;

  v_curr = init_vel;
  for (size_t i = 1; i < steps_to_target + 1; i++) {
    v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
    EXPECT_NEAR(v_curr, init_vel + i * dv_accel, 0.001);
  }
  EXPECT_NEAR(v_curr, target_vel, 0.001);
  v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
  EXPECT_NEAR(v_curr, target_vel, 0.001);
}

TEST(VelocitySmootherTest, testapplyConstraintsNegativeToPositiveDecelAccel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;
  double accel = 0.1;
  double decel = -1.0;
  double dv_accel = accel / 20.0;
  double dv_decel = -decel / 20.0;
  double init_vel, target_vel, v_curr;
  uint steps_below_zero, steps_above_zero;

  init_vel = -1.0;
  target_vel = 2.0;
  steps_below_zero = -init_vel / dv_decel;
  steps_above_zero = target_vel / dv_accel;

  v_curr = init_vel;
  for (size_t i = 1; i < steps_below_zero + steps_above_zero + 1; i++) {
    v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
    if (v_curr > 0) {
      EXPECT_NEAR(v_curr, (i - steps_below_zero) * dv_accel, 0.001);
    } else {
      EXPECT_NEAR(v_curr, init_vel + i * dv_decel, 0.001);
    }
  }
  EXPECT_NEAR(v_curr, target_vel, 0.001);
  v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
  EXPECT_NEAR(v_curr, target_vel, 0.001);
}

TEST(VelocitySmootherTest, testapplyConstraintsNegativeToNegativeAccel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;
  double accel = 0.1;
  double decel = -1.0;
  double dv_accel = accel / 20.0;
  double init_vel, target_vel, v_curr;
  uint steps_to_target;

  init_vel = -1.0;
  target_vel = -2.0;
  steps_to_target = abs(target_vel - init_vel) / dv_accel;

  v_curr = init_vel;
  for (size_t i = 1; i < steps_to_target + 1; i++) {
    v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
    EXPECT_NEAR(v_curr, init_vel - i * dv_accel, 0.001);
  }
  EXPECT_NEAR(v_curr, target_vel, 0.001);
  v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
  EXPECT_NEAR(v_curr, target_vel, 0.001);
}

TEST(VelocitySmootherTest, testapplyConstraintsZeroToNegativeAccel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;
  double accel = 0.1;
  double decel = -1.0;
  double dv_accel = accel / 20.0;
  double init_vel, target_vel, v_curr;
  uint steps_to_target;

  init_vel = 0.0;
  target_vel = -2.0;
  steps_to_target = abs(target_vel - init_vel) / dv_accel;

  v_curr = init_vel;
  for (size_t i = 1; i < steps_to_target + 1; i++) {
    v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
    EXPECT_NEAR(v_curr, init_vel - i * dv_accel, 0.001);
  }
  EXPECT_NEAR(v_curr, target_vel, 0.001);
  v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
  EXPECT_NEAR(v_curr, target_vel, 0.001);
}

TEST(VelocitySmootherTest, testapplyConstraintsPositiveToNegativeDecelAccel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;
  double accel = 0.1;
  double decel = -1.0;
  double dv_accel = accel / 20.0;
  double dv_decel = -decel / 20.0;
  double init_vel, target_vel, v_curr;
  uint steps_below_zero, steps_above_zero;

  init_vel = 1.0;
  target_vel = -2.0;
  steps_above_zero = init_vel / dv_decel;
  steps_below_zero = -target_vel / dv_accel;

  v_curr = init_vel;
  for (size_t i = 1; i < steps_below_zero + steps_above_zero + 1; i++) {
    v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
    if (v_curr < 0) {
      EXPECT_NEAR(v_curr, -static_cast<int>(i - steps_above_zero) * dv_accel, 0.001);
    } else {
      EXPECT_NEAR(v_curr, init_vel - i * dv_decel, 0.001);
    }
  }
  EXPECT_NEAR(v_curr, target_vel, 0.001);
  v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
  EXPECT_NEAR(v_curr, target_vel, 0.001);
}

TEST(VelocitySmootherTest, testapplyConstraintsPositiveToPositiveDecel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;

  // Test asymetric accel/decel use cases
  double accel = 0.1;
  double decel = -1.0;
  double dv_decel = -decel / 20.0;
  double init_vel, target_vel, v_curr;
  uint steps_to_target;

  init_vel = 2.0;
  target_vel = 1.0;
  steps_to_target = abs(target_vel - init_vel) / dv_decel;

  v_curr = init_vel;
  for (size_t i = 1; i < steps_to_target + 1; i++) {
    v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
    EXPECT_NEAR(v_curr, init_vel - i * dv_decel, 0.001);
  }
  EXPECT_NEAR(v_curr, target_vel, 0.001);
  v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
  EXPECT_NEAR(v_curr, target_vel, 0.001);
}

TEST(VelocitySmootherTest, testapplyConstraintsPositiveToZeroDecel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;
  double accel = 0.1;
  double decel = -1.0;
  double dv_decel = -decel / 20.0;
  double init_vel, target_vel, v_curr;
  uint steps_to_target;

  init_vel = 2.0;
  target_vel = 0.0;
  steps_to_target = abs(target_vel - init_vel) / dv_decel;

  v_curr = init_vel;
  for (size_t i = 1; i < steps_to_target + 1; i++) {
    v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
    EXPECT_NEAR(v_curr, init_vel - i * dv_decel, 0.001);
  }
  EXPECT_NEAR(v_curr, target_vel, 0.001);
  v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
  EXPECT_NEAR(v_curr, target_vel, 0.001);
}

TEST(VelocitySmootherTest, testapplyConstraintsNegativeToNegativeDecel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;
  double accel = 0.1;
  double decel = -1.0;
  double dv_decel = -decel / 20.0;
  double init_vel, target_vel, v_curr;
  uint steps_to_target;

  init_vel = -2.0;
  target_vel = -1.0;
  steps_to_target = abs(target_vel - init_vel) / dv_decel;

  v_curr = init_vel;
  for (size_t i = 1; i < steps_to_target + 1; i++) {
    v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
    EXPECT_NEAR(v_curr, init_vel + i * dv_decel, 0.001);
  }
  EXPECT_NEAR(v_curr, target_vel, 0.001);
  v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
  EXPECT_NEAR(v_curr, target_vel, 0.001);
}

TEST(VelocitySmootherTest, testapplyConstraintsNegativeToZeroDecel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  // default frequency is 20.0
  smoother->configure(state);
  double no_eta = 1.0;
  double accel = 0.1;
  double decel = -1.0;
  double dv_decel = -decel / 20.0;
  double init_vel, target_vel, v_curr;
  uint steps_to_target;

  init_vel = -2.0;
  target_vel = 0.0;
  steps_to_target = abs(target_vel - init_vel) / dv_decel;

  v_curr = init_vel;
  for (size_t i = 1; i < steps_to_target + 1; i++) {
    v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
    EXPECT_NEAR(v_curr, init_vel + i * dv_decel, 0.001);
  }
  EXPECT_NEAR(v_curr, target_vel, 0.001);
  v_curr = smoother->applyConstraints(v_curr, target_vel, accel, decel, no_eta);
  EXPECT_NEAR(v_curr, target_vel, 0.001);
}

TEST(VelocitySmootherTest, testCommandCallback)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  smoother->configure(state);
  smoother->activate(state);

  auto pub = smoother->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  pub->on_activate();
  auto msg = std::make_unique<geometry_msgs::msg::Twist>();
  msg->linear.x = 100.0;
  pub->publish(std::move(msg));
  rclcpp::spin_some(smoother->get_node_base_interface());

  EXPECT_TRUE(smoother->hasCommandMsg());
  EXPECT_EQ(smoother->lastCommandMsg()->twist.linear.x, 100.0);
}

TEST(VelocitySmootherTest, testClosedLoopSub)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  smoother->declare_parameter("feedback", rclcpp::ParameterValue(std::string("OPEN_LOOP")));
  smoother->set_parameter(rclcpp::Parameter("feedback", std::string("CLOSED_LOOP")));
  rclcpp_lifecycle::State state;
  smoother->configure(state);
  EXPECT_TRUE(smoother->isOdomSmoother());
}

TEST(VelocitySmootherTest, testInvalidParams)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  std::vector<double> max_vels{0.0, 0.0};  // invalid size
  smoother->declare_parameter("max_velocity", rclcpp::ParameterValue(max_vels));
  rclcpp_lifecycle::State state;
  EXPECT_EQ(smoother->configure(state), nav2_util::CallbackReturn::FAILURE);

  smoother->set_parameter(rclcpp::Parameter("feedback", std::string("LAWLS")));
  EXPECT_EQ(smoother->configure(state), nav2_util::CallbackReturn::FAILURE);
}

TEST(VelocitySmootherTest, testInvalidParamsAccelDecel)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();

  std::vector<double> bad_test_accel{-10.0, -10.0, -10.0};
  std::vector<double> bad_test_decel{10.0, 10.0, 10.0};
  std::vector<double> bad_test_min_vel{10.0, 10.0, 10.0};
  std::vector<double> bad_test_max_vel{-10.0, -10.0, -10.0};

  smoother->declare_parameter("max_velocity", rclcpp::ParameterValue(bad_test_max_vel));
  smoother->declare_parameter("min_velocity", rclcpp::ParameterValue(bad_test_min_vel));
  rclcpp_lifecycle::State state;
  EXPECT_EQ(smoother->configure(state), nav2_util::CallbackReturn::FAILURE);

  smoother->set_parameter(rclcpp::Parameter("max_accel", rclcpp::ParameterValue(bad_test_accel)));
  EXPECT_EQ(smoother->configure(state), nav2_util::CallbackReturn::FAILURE);

  smoother->set_parameter(rclcpp::Parameter("max_decel", rclcpp::ParameterValue(bad_test_decel)));
  EXPECT_EQ(smoother->configure(state), nav2_util::CallbackReturn::FAILURE);
}

TEST(VelocitySmootherTest, testDynamicParameter)
{
  auto smoother =
    std::make_shared<VelSmootherShim>();
  rclcpp_lifecycle::State state;
  smoother->configure(state);
  smoother->activate(state);
  EXPECT_FALSE(smoother->isOdomSmoother());

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    smoother->get_node_base_interface(), smoother->get_node_topics_interface(),
    smoother->get_node_graph_interface(),
    smoother->get_node_services_interface());

  std::vector<double> max_vel{10.0, 10.0, 10.0};
  std::vector<double> min_vel{0.0, 0.0, 0.0};
  std::vector<double> max_accel{10.0, 10.0, 10.0};
  std::vector<double> min_accel{0.0, 0.0, 0.0};
  std::vector<double> deadband{0.0, 0.0, 0.0};
  std::vector<double> bad_test{0.0, 0.0};
  std::vector<double> bad_test_accel{-10.0, -10.0, -10.0};
  std::vector<double> bad_test_decel{10.0, 10.0, 10.0};

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("smoothing_frequency", 100.0),
      rclcpp::Parameter("feedback", std::string("CLOSED_LOOP")),
      rclcpp::Parameter("scale_velocities", true),
      rclcpp::Parameter("max_velocity", max_vel),
      rclcpp::Parameter("min_velocity", min_vel),
      rclcpp::Parameter("max_accel", max_accel),
      rclcpp::Parameter("max_decel", min_accel),
      rclcpp::Parameter("odom_topic", std::string("TEST")),
      rclcpp::Parameter("odom_duration", 2.0),
      rclcpp::Parameter("velocity_timeout", 4.0),
      rclcpp::Parameter("deadband_velocity", deadband)});

  rclcpp::spin_until_future_complete(
    smoother->get_node_base_interface(),
    results);

  EXPECT_EQ(smoother->get_parameter("smoothing_frequency").as_double(), 100.0);
  EXPECT_EQ(smoother->get_parameter("feedback").as_string(), std::string("CLOSED_LOOP"));
  EXPECT_EQ(smoother->get_parameter("scale_velocities").as_bool(), true);
  EXPECT_EQ(smoother->get_parameter("max_velocity").as_double_array(), max_vel);
  EXPECT_EQ(smoother->get_parameter("min_velocity").as_double_array(), min_vel);
  EXPECT_EQ(smoother->get_parameter("max_accel").as_double_array(), max_accel);
  EXPECT_EQ(smoother->get_parameter("max_decel").as_double_array(), min_accel);
  EXPECT_EQ(smoother->get_parameter("odom_topic").as_string(), std::string("TEST"));
  EXPECT_EQ(smoother->get_parameter("odom_duration").as_double(), 2.0);
  EXPECT_EQ(smoother->get_parameter("velocity_timeout").as_double(), 4.0);
  EXPECT_EQ(smoother->get_parameter("deadband_velocity").as_double_array(), deadband);

  // Test reverting
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("feedback", std::string("OPEN_LOOP"))});
  rclcpp::spin_until_future_complete(
    smoother->get_node_base_interface(), results);
  EXPECT_EQ(smoother->get_parameter("feedback").as_string(), std::string("OPEN_LOOP"));

  // Test invalid change
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("feedback", std::string("LAWLS"))});
  rclcpp::spin_until_future_complete(smoother->get_node_base_interface(), results);
  EXPECT_FALSE(results.get().successful);

  // Test invalid size
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("max_velocity", bad_test)});
  rclcpp::spin_until_future_complete(smoother->get_node_base_interface(), results);
  EXPECT_FALSE(results.get().successful);

  // Test invalid accel / decel
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("max_accel", bad_test_accel)});
  rclcpp::spin_until_future_complete(smoother->get_node_base_interface(), results);
  EXPECT_FALSE(results.get().successful);
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("max_decel", bad_test_decel)});
  rclcpp::spin_until_future_complete(smoother->get_node_base_interface(), results);
  EXPECT_FALSE(results.get().successful);

  // test full state after major changes
  smoother->deactivate(state);
  smoother->cleanup(state);
  smoother->shutdown(state);
  smoother.reset();
}
