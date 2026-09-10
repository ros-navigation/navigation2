// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

// Tests motion models

using namespace mppi;  // NOLINT

/**
 * @brief Drive a control sequence through the model's per-step constraint, mirroring the loop in
 *        Optimizer::applyControlSequenceConstraints() with a single time step duration
 */
static void constrainSequence(
  MotionModel & model, models::ControlSequence & control_sequence,
  const geometry_msgs::msg::Twist & initial_speed,
  const models::ControlConstraints & constraints, const float dt)
{
  models::Control last{
    static_cast<float>(initial_speed.linear.x),
    static_cast<float>(initial_speed.linear.y),
    static_cast<float>(initial_speed.angular.z)};
  const models::Control min_deltas{dt * constraints.ax_min, dt * constraints.ay_min,
    -dt * constraints.az_max};
  const models::Control max_deltas{dt * constraints.ax_max, dt * constraints.ay_max,
    dt * constraints.az_max};

  for (unsigned int i = 0; i != control_sequence.vx.size(); i++) {
    models::Control curr{control_sequence.vx(i), control_sequence.vy(i), control_sequence.wz(i)};
    model.constrainVelocityStep(curr, last, min_deltas, max_deltas);
    control_sequence.vx(i) = curr.vx;
    control_sequence.vy(i) = curr.vy;
    control_sequence.wz(i) = curr.wz;
    last = curr;
  }
}

/**
 * @brief Constrain a control sequence starting from rest, with acceleration limits wide enough
 *        that only the velocity envelope binds, so each time step can be checked on its own
 */
static void constrainWithFreeAcceleration(
  MotionModel & model, models::ControlSequence & control_sequence,
  const float vx_max, const float vx_min, const float vy_max, const float wz_max)
{
  const models::ControlConstraints constraints{vx_max, vx_min, vy_max, wz_max,
    1e3f, -1e3f, -1e3f, 1e3f, 1e3f};
  model.setConstraints(constraints, 0.1f, 0.0f, 0.0f, 0.0f, false);
  constrainSequence(model, control_sequence, geometry_msgs::msg::Twist(), constraints, 0.1f);
}

TEST(MotionModelTests, DiffDriveTest)
{
  models::ControlSequence control_sequence;
  models::State state;
  int batches = 1000;
  int timesteps = 50;
  control_sequence.reset(timesteps);  // populates with zeros
  state.reset(batches, timesteps);  // populates with zeros
  std::unique_ptr<DiffDriveMotionModel> model =
    std::make_unique<DiffDriveMotionModel>();

  // Check that predict properly populates the trajectory velocities with the control velocities
  state.cvx = 10 * Eigen::ArrayXXf::Ones(batches, timesteps);
  state.cvy = 5 * Eigen::ArrayXXf::Ones(batches, timesteps);
  state.cwz = 1 * Eigen::ArrayXXf::Ones(batches, timesteps);

  // Manually set state index 0 from initial conditions which would be the speed of the robot
  state.vx.col(0) = 10;
  state.wz.col(0) = 1;

  model->predict(state);

  EXPECT_TRUE(state.vx.isApprox(state.cvx));
  EXPECT_TRUE(state.vy.isApprox(Eigen::ArrayXXf::Zero(batches, timesteps)));  // non-holonomic
  EXPECT_TRUE(state.wz.isApprox(state.cwz));

  // The base implementation bounds each axis by its own velocity and acceleration limits
  const models::ControlConstraints constraints{0.5f, -0.35f, 0.3f, 1.9f,
    1.0f, -1.0f, -1.0f, 1.0f, 2.0f};
  model->setConstraints(constraints, 0.1f, 0.0f, 0.0f, 0.0f, false);

  control_sequence.vx.setConstant(5.0f);
  control_sequence.wz.setConstant(5.0f);
  constrainSequence(
    *model, control_sequence, geometry_msgs::msg::Twist(), constraints, 0.1f);

  // One controller period of acceleration from rest, then ramping to the per-axis limits
  EXPECT_NEAR(control_sequence.vx(0), 0.1f * constraints.ax_max, 1e-6);
  EXPECT_NEAR(control_sequence.wz(0), 0.1f * constraints.az_max, 1e-6);
  EXPECT_NEAR(control_sequence.vx(timesteps - 1), constraints.vx_max, 1e-6);
  EXPECT_NEAR(control_sequence.wz(timesteps - 1), constraints.wz, 1e-6);
  EXPECT_TRUE((control_sequence.vy == 0.0f).all());  // non-holonomic

  // Check that Diff Drive is properly non-holonomic
  EXPECT_EQ(model->isHolonomic(), false);

  // Check it cleanly destructs
  model.reset();
}

TEST(MotionModelTests, OmniTest)
{
  models::ControlSequence control_sequence;
  models::State state;
  int batches = 1000;
  int timesteps = 50;
  control_sequence.reset(timesteps);  // populates with zeros
  state.reset(batches, timesteps);  // populates with zeros
  std::unique_ptr<OmniMotionModel> model =
    std::make_unique<OmniMotionModel>();

  // Check that predict properly populates the trajectory velocities with the control velocities
  state.cvx = 10 * Eigen::ArrayXXf::Ones(batches, timesteps);
  state.cvy = 5 * Eigen::ArrayXXf::Ones(batches, timesteps);
  state.cwz = 1 * Eigen::ArrayXXf::Ones(batches, timesteps);

  // Manually set state index 0 from initial conditions which would be the speed of the robot
  state.vx.col(0) = 10;
  state.vy.col(0) = 5;
  state.wz.col(0) = 1;

  model->predict(state);

  EXPECT_TRUE(state.vx.isApprox(state.cvx));
  EXPECT_TRUE(state.vy.isApprox(state.cvy));  // holonomic
  EXPECT_TRUE(state.wz.isApprox(state.cwz));

  // Check that Omni Drive is properly holonomic
  EXPECT_EQ(model->isHolonomic(), true);

  // Check it cleanly destructs
  model.reset();
}

TEST(MotionModelTests, OmniEllipticalConstraintsTest)
{
  models::ControlSequence control_sequence;
  control_sequence.reset(6);  // populates with zeros
  auto model = std::make_unique<OmniMotionModel>();

  // Velocities inside the ellipse, and exactly on either semi-axis, are left untouched.
  // The last two are outside the ellipse, thus pulled onto the ellipse.
  control_sequence.vx << 0.25f, 0.5f, 0.0f, -0.2f, 0.5f, -0.35f;
  control_sequence.vy << 0.15f, 0.0f, -0.3f, -0.1f, 0.3f, 0.3f;

  control_sequence.wz.setLinSpaced(6, -1.0f, 1.0f);
  const Eigen::ArrayXf wz_initial = control_sequence.wz;

  constrainWithFreeAcceleration(*model, control_sequence, 0.5f, -0.35f, 0.3f, 1.9f);

  EXPECT_NEAR(control_sequence.vx(0), 0.25f, 1e-6);
  EXPECT_NEAR(control_sequence.vy(0), 0.15f, 1e-6);
  EXPECT_NEAR(control_sequence.vx(1), 0.5f, 1e-6);
  EXPECT_NEAR(control_sequence.vy(1), 0.0f, 1e-6);
  EXPECT_NEAR(control_sequence.vx(2), 0.0f, 1e-6);
  EXPECT_NEAR(control_sequence.vy(2), -0.3f, 1e-6);
  EXPECT_NEAR(control_sequence.vx(3), -0.2f, 1e-6);
  EXPECT_NEAR(control_sequence.vy(3), -0.1f, 1e-6);

  // Scaled radially by 1/sqrt(2), so the direction of travel is preserved
  EXPECT_NEAR(control_sequence.vx(4), 0.5f / std::sqrt(2.0f), 1e-6);
  EXPECT_NEAR(control_sequence.vy(4), 0.3f / std::sqrt(2.0f), 1e-6);
  EXPECT_NEAR(control_sequence.vx(5), -0.35f / std::sqrt(2.0f), 1e-6);
  EXPECT_NEAR(control_sequence.vy(5), 0.3f / std::sqrt(2.0f), 1e-6);

  // The projected corners now sit on the ellipse, and below both per-axis limits
  for (unsigned int i = 4; i != 6; i++) {
    const float vx_limit = control_sequence.vx(i) >= 0.0f ? 0.5f : 0.35f;
    const float radius = std::hypot(control_sequence.vx(i) / vx_limit,
        control_sequence.vy(i) / 0.3f);
    EXPECT_NEAR(radius, 1.0f, 1e-5);
    EXPECT_LT(std::fabs(control_sequence.vx(i)), vx_limit);
    EXPECT_LT(std::fabs(control_sequence.vy(i)), 0.3f);
  }

  // Angular velocity is not touched by the holonomic envelope
  EXPECT_TRUE(control_sequence.wz.isApprox(wz_initial));

  // Check it cleanly destructs
  model.reset();
}

TEST(MotionModelTests, OmniZeroLimitConstraintsTest)
{
  // An axis whose limit is zero cannot be commanded at all, while the other axis must still
  // reach its own limit. The box clamp handles this before the ellipse is ever evaluated.
  models::ControlSequence control_sequence;
  control_sequence.reset(3);  // populates with zeros
  auto model = std::make_unique<OmniMotionModel>();

  // vy_max = 0: no lateral motion
  control_sequence.vx << 0.4f, -0.3f, 0.5f;
  control_sequence.vy << 0.2f, -0.2f, 0.3f;
  constrainWithFreeAcceleration(*model, control_sequence, 0.5f, -0.35f, 0.0f, 1.9f);

  EXPECT_TRUE((control_sequence.vy == 0.0f).all());
  EXPECT_NEAR(control_sequence.vx(0), 0.4f, 1e-6);
  EXPECT_NEAR(control_sequence.vx(1), -0.3f, 1e-6);
  EXPECT_NEAR(control_sequence.vx(2), 0.5f, 1e-6);

  // vx_min = 0: no reversing, while forward motion stays bound by the ellipse
  control_sequence.vx << 0.4f, -0.3f, 0.5f;
  control_sequence.vy << 0.0f, 0.0f, 0.3f;
  constrainWithFreeAcceleration(*model, control_sequence, 0.5f, 0.0f, 0.3f, 1.9f);

  EXPECT_NEAR(control_sequence.vx(0), 0.4f, 1e-6);
  EXPECT_NEAR(control_sequence.vx(1), 0.0f, 1e-6);
  EXPECT_NEAR(control_sequence.vx(2), 0.5f / std::sqrt(2.0f), 1e-6);
  EXPECT_NEAR(control_sequence.vy(2), 0.3f / std::sqrt(2.0f), 1e-6);

  // Check it cleanly destructs
  model.reset();
}

TEST(MotionModelTests, OmniPerAxisLimitsTest)
{
  // With use_velocity_ellipse_scaling false, each axis is bounded on its own
  models::ControlSequence control_sequence;
  control_sequence.reset(2);  // populates with zeros
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  auto model = std::make_unique<OmniMotionModel>();

  node->declare_parameter(name + ".omni.use_velocity_ellipse_scaling", false);
  model->initialize(&param_handler, name + ".omni");
  EXPECT_FALSE(model->useVelocityEllipseScaling());

  // The rectangle's corner passes through untouched
  control_sequence.vx << 0.5f, -0.35f;
  control_sequence.vy << 0.3f, 0.3f;
  constrainWithFreeAcceleration(*model, control_sequence, 0.5f, -0.35f, 0.3f, 1.9f);

  EXPECT_NEAR(control_sequence.vx(0), 0.5f, 1e-6);
  EXPECT_NEAR(control_sequence.vy(0), 0.3f, 1e-6);
  EXPECT_NEAR(control_sequence.vx(1), -0.35f, 1e-6);
  EXPECT_NEAR(control_sequence.vy(1), 0.3f, 1e-6);

  // Check it cleanly destructs
  model.reset();
}

TEST(MotionModelTests, AckermannTurningRadiusTest)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  auto model = std::make_unique<AckermannMotionModel>();
  // Initialize the plugin: parameters live under "test.ackermann"
  model->initialize(&param_handler, name + ".ackermann");

  EXPECT_NEAR(model->getMinTurningRadius(), 0.2, 1e-6);
  EXPECT_FALSE(model->isHolonomic());
  const float r = model->getMinTurningRadius();

  // More curvature than the turning radius allows, driving forwards and in reverse, turning
  // either way. Rotation keeps its sign but is bounded by |vx| / min_turning_r.
  const unsigned int timesteps = 20;
  for (const float vx_demand : {1.0f, -1.0f}) {
    for (const float wz_demand : {20.0f, -20.0f}) {
      models::ControlSequence control_sequence;
      control_sequence.reset(timesteps);  // populates with zeros
      control_sequence.vx.setConstant(vx_demand);
      control_sequence.wz.setConstant(wz_demand);
      constrainWithFreeAcceleration(*model, control_sequence, 1.0f, -1.0f, 0.0f, 100.0f);

      for (unsigned int i = 0; i != timesteps; i++) {
        EXPECT_EQ(std::signbit(control_sequence.wz(i)), std::signbit(wz_demand));
        EXPECT_LE(
          std::fabs(control_sequence.wz(i)), std::fabs(control_sequence.vx(i)) / r + 1e-5f);
      }

      // vx is free to reach its own limit, and wz saturates on the turning radius bound
      EXPECT_NEAR(control_sequence.vx(timesteps - 1), vx_demand, 1e-6);
      EXPECT_NEAR(
        control_sequence.wz(timesteps - 1), std::copysign(std::fabs(vx_demand) / r, wz_demand),
        1e-5);
    }
  }

  // Braking while turning tightens the turning radius bound as vx falls, and wz follows it down
  // even though that is faster than az_max: wz is vx times curvature, not an actuated axis.
  models::ControlSequence control_sequence;
  control_sequence.reset(2);  // populates with zeros
  control_sequence.vx.setConstant(0.5f);
  control_sequence.wz.setConstant(4.0f);

  const models::ControlConstraints constraints{1.0f, -1.0f, 0.0f, 100.0f,
    10.0f, -10.0f, -10.0f, 10.0f, 1.0f};
  model->setConstraints(constraints, 0.1f, 0.0f, 0.0f, 0.0f, false);

  geometry_msgs::msg::Twist speed;
  speed.linear.x = 1.0;
  speed.angular.z = 4.0;
  constrainSequence(*model, control_sequence, speed, constraints, 0.1f);

  // vx reaches its demand, and wz lands exactly on the radius bound that vx now allows
  EXPECT_NEAR(control_sequence.vx(0), 0.5f, 1e-5);
  EXPECT_NEAR(control_sequence.wz(0), std::fabs(control_sequence.vx(0)) / r, 1e-5);

  // That step took wz well past what az_max alone would permit, which is the intended trade:
  // the radius bound wins because wz is not independently actuated on this vehicle
  EXPECT_GT(std::fabs(control_sequence.wz(0) - 4.0f), 0.1f * constraints.az_max);

  // Braking through a standstill while turning must not deadlock: wz is free to follow vx down
  control_sequence.reset(20);
  control_sequence.vx.setConstant(-0.35f);
  control_sequence.wz.setConstant(1.0f);
  speed.linear.x = 0.2;
  speed.angular.z = 1.0;  // exactly on the radius bound, r * wz == vx
  const models::ControlConstraints brake_constraints{0.5f, -0.35f, 0.0f, 1.9f,
    3.0f, -3.0f, -3.0f, 3.0f, 3.5f};
  model->setConstraints(brake_constraints, 0.05f, 0.0f, 0.0f, 0.0f, false);
  constrainSequence(*model, control_sequence, speed, brake_constraints, 0.05f);

  EXPECT_LT(control_sequence.vx(0), 0.2f);          // it actually brakes
  EXPECT_NEAR(control_sequence.vx(19), -0.35f, 1e-5);  // and reaches the requested reverse speed
  for (unsigned int i = 0; i != 20; i++) {
    EXPECT_LE(std::fabs(control_sequence.wz(i)), std::fabs(control_sequence.vx(i)) / r + 1e-5f);
  }

  // A turning radius of zero is not validated anywhere, and neither is calling this before
  // initialize(), which would divide by zero. Check the horizon stays finite.
  auto zero_radius = std::make_unique<AckermannMotionModel>();
  EXPECT_NEAR(zero_radius->getMinTurningRadius(), 0.0f, 1e-9);

  control_sequence.reset(2);
  control_sequence.vx.setZero();
  control_sequence.wz.setConstant(50.0f);
  constrainWithFreeAcceleration(*zero_radius, control_sequence, 1.0f, -1.0f, 0.0f, 2.0f);

  EXPECT_TRUE(control_sequence.wz.isFinite().all());
  EXPECT_TRUE((control_sequence.wz == 0.0f).all());  // stopped, so no rotation

  // Check it cleanly destructs
  model.reset();
}

static models::ControlConstraints unboundedConstraints()
{
  return {100, -100, 100, 100, 1000, -1000, -1000, 1000, 1000};
}

TEST(MotionModelTests, DelayReplayAllAxes)
{
  // Omni model exercises all three axes (vx, vy, wz) in a single test.
  // delay_vx = 0.10s -> offset 2; delay_wz = 0.15s -> offset 3.
  models::State state;
  state.reset(8, 20);

  OmniMotionModel model;

  // Set model_dt, model_delay_vx, model_delay_vy, model_delay_wz = 0.05f, 0.10f, 0.10f, 0.15f;
  model.setConstraints(unboundedConstraints(), 0.05f, 0.10f, 0.10f, 0.15f, false);

  // Ring-buffer size of vx/vy/wz = 2/2/3 steps
  // After 3 pushes per axis the rings hold the most-recent 2/2/3 values.
  model.pushCommandHistory(1.0f, 2.0f, 3.0f);
  model.pushCommandHistory(5.0f, 6.0f, 7.0f);
  model.pushCommandHistory(9.0f, 8.0f, 4.0f);

  model.predict(state);

  // predict() populates starting at column 1; column 0 holds the current
  // measurement / last command set by the optimizer before predict() runs.
  EXPECT_TRUE(state.vx.col(0).isApproxToConstant(0.0f));
  EXPECT_TRUE(state.vy.col(0).isApproxToConstant(0.0f));
  EXPECT_TRUE(state.wz.col(0).isApproxToConstant(0.0f));

  EXPECT_TRUE(state.vx.col(1).isApproxToConstant(9.0f));
  EXPECT_TRUE(state.vy.col(1).isApproxToConstant(8.0f));
  EXPECT_TRUE(state.wz.col(1).isApproxToConstant(7.0f));

  // The wz buffer has a size of 3, so check the last value as well
  EXPECT_TRUE(state.wz.col(2).isApproxToConstant(4.0f));

  // Outside of the delay window are still zero
  EXPECT_TRUE(state.vx.col(2).isApproxToConstant(0.0f));
  EXPECT_TRUE(state.vy.col(2).isApproxToConstant(0.0f));
  EXPECT_TRUE(state.vx.col(3).isApproxToConstant(0.0f));
  EXPECT_TRUE(state.vy.col(3).isApproxToConstant(0.0f));
  EXPECT_TRUE(state.wz.col(3).isApproxToConstant(0.0f));
}

TEST(MotionModelTests, DelayVyIgnoredOnDiffDrive)
{
  models::State state;
  state.reset(8, 20);

  DiffDriveMotionModel model;
  model.setConstraints(unboundedConstraints(), 0.05f, 0.0f, 0.10f, 0.0f, false);
  model.pushCommandHistory(0.0f, 99.0f, 0.0f);
  model.pushCommandHistory(0.0f, 99.0f, 0.0f);

  EXPECT_NO_THROW(model.predict(state));
  EXPECT_TRUE(state.vy.isApprox(Eigen::ArrayXXf::Zero(8, 20)));
}

TEST(MotionModelTests, DelayClearCommandHistory)
{
  models::State state;
  state.reset(8, 20);

  DiffDriveMotionModel model;
  model.setConstraints(unboundedConstraints(), 0.05f, 0.10f, 0.0f, 0.15f, false);
  model.pushCommandHistory(7.0f, 0.0f, 7.0f);
  model.clearCommandHistory();
  model.predict(state);

  EXPECT_TRUE(state.vx.col(1).isApproxToConstant(0.0f));
  EXPECT_TRUE(state.wz.col(1).isApproxToConstant(0.0f));
  EXPECT_TRUE(state.wz.col(2).isApproxToConstant(0.0f));
}

TEST(MotionModelTests, DelayZeroSkipsShift)
{
  models::State state;
  state.reset(8, 20);

  // Set non-zero velocity and check that it remains unchanged after predict()
  state.vx.col(0).setConstant(2.0f);
  state.cvx.setConstant(2.0f);

  DiffDriveMotionModel model;
  model.setConstraints(unboundedConstraints(), 0.05f, 0.0f, 0.0f, 0.0f, false);
  EXPECT_NO_THROW(model.pushCommandHistory(123.0f, 0.0f, 0.0f));
  model.predict(state);

  EXPECT_TRUE(state.vx.isApproxToConstant(2.0f));
}

TEST(MotionModelTests, ClampRawControlsRewritesRawCommand)
{
  models::State state;
  state.reset(1, 3);

  // ax_max/ax_min/az_max = 1.0, so each step can only change velocity by
  // model_dt * 1.0 = 1.0, well below the 5.0 raw commands set below
  models::ControlConstraints tight_constraints{
    100.0f, -100.0f, 100.0f, 100.0f, 1.0f, -1.0f, -1.0f, 1.0f, 1.0f};

  OmniMotionModel model;
  model.setConstraints(tight_constraints, 1.0f, 0.0f, 0.0f, 0.0f, true);

  state.cvx.col(0).setConstant(5.0f);
  state.cvx.col(1).setConstant(5.0f);
  state.cwz.col(0).setConstant(5.0f);
  state.cwz.col(1).setConstant(5.0f);

  model.predict(state);

  EXPECT_TRUE(state.vx.col(1).isApproxToConstant(1.0f));
  EXPECT_TRUE(state.vx.col(2).isApproxToConstant(2.0f));
  EXPECT_TRUE(state.wz.col(1).isApproxToConstant(1.0f));
  EXPECT_TRUE(state.wz.col(2).isApproxToConstant(2.0f));

  // clamp_raw_controls=true applies acceleration limits on the raw commands as well
  EXPECT_TRUE(state.cvx.col(0).isApproxToConstant(1.0f));
  EXPECT_TRUE(state.cvx.col(1).isApproxToConstant(2.0f));
  EXPECT_TRUE(state.cwz.col(0).isApproxToConstant(1.0f));
  EXPECT_TRUE(state.cwz.col(1).isApproxToConstant(2.0f));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
