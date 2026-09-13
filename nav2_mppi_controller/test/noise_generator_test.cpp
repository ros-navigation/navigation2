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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_mppi_controller/tools/noise_generator.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"

// Tests noise generator object

using namespace mppi;  // NOLINT

TEST(NoiseGeneratorTest, NoiseGeneratorLifecycle)
{
  // Tests shuts down internal thread cleanly
  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;

  auto node = std::make_shared<nav2::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  std::string name = "test";
  ParametersHandler handler(node, name);

  generator.initialize(settings, false, "test_name", &handler);
  generator.reset(settings, false);
  generator.shutdown();
}

TEST(NoiseGeneratorTest, NoiseGeneratorMain)
{
  // Tests shuts down internal thread cleanly
  auto node = std::make_shared<nav2::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(true));
  std::string name = "test";
  ParametersHandler handler(node, name);
  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;
  settings.sampling_std.vx = 0.1;
  settings.sampling_std.vy = 0.1;
  settings.sampling_std.wz = 0.1;

  // Populate a potential control sequence
  mppi::models::ControlSequence control_sequence;
  control_sequence.reset(25);
  for (unsigned int i = 0; i != control_sequence.vx.rows(); i++) {
    control_sequence.vx(i) = i;
    control_sequence.vy(i) = i;
    control_sequence.wz(i) = i;
  }

  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);

  // Request an update with no noise yet generated, should result in identical outputs
  generator.initialize(settings, false, "test_name", &handler);
  generator.reset(settings, false);  // sets initial sizing and zeros out noises
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  generator.setNoisedControls(state, control_sequence);

  // save initial state
  auto initial_cvx_0 = state.cvx(0);
  auto initial_cvy_0 = state.cvy(0);
  auto initial_cwz_0 = state.cwz(0);
  auto initial_cvx_9 = state.cvx(0, 9);
  auto initial_cvy_9 = state.cvy(0, 9);
  auto initial_cwz_9 = state.cwz(0, 9);

  EXPECT_NE(state.cvx(0), 0);
  EXPECT_EQ(state.cvy(0), 0);  // Not populated in non-holonomic
  EXPECT_NE(state.cwz(0), 0);
  EXPECT_NE(state.cvx(0, 9), 9);
  EXPECT_EQ(state.cvy(0, 9), 9);  // Not populated in non-holonomic
  EXPECT_NE(state.cwz(0, 9), 9);

  EXPECT_NEAR(state.cvx(0), 0, 0.3);
  EXPECT_NEAR(state.cwz(0), 0, 0.3);
  EXPECT_NEAR(state.cvx(0, 9), 9, 0.3);
  EXPECT_NEAR(state.cwz(0, 9), 9, 0.3);

  // Request an update with noise requested
  generator.generateNextNoises();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  generator.setNoisedControls(state, control_sequence);

  // Ensure the state has changed after generating new noises
  EXPECT_NE(state.cvx(0), initial_cvx_0);
  EXPECT_EQ(state.cvy(0), initial_cvy_0);  // Not populated in non-holonomic
  EXPECT_NE(state.cwz(0), initial_cwz_0);
  EXPECT_NE(state.cvx(0, 9), initial_cvx_9);
  EXPECT_EQ(state.cvy(0, 9), initial_cvy_9);  // Not populated in non-holonomic
  EXPECT_NE(state.cwz(0, 9), initial_cwz_9);


  // Test holonomic setting
  generator.reset(settings, true);  // Now holonomically
  generator.generateNextNoises();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  generator.setNoisedControls(state, control_sequence);
  EXPECT_NE(state.cvx(0), 0);
  EXPECT_NE(state.cvy(0), 0);  // Now populated in non-holonomic
  EXPECT_NE(state.cwz(0), 0);
  EXPECT_NE(state.cvx(0, 9), 9);
  EXPECT_NE(state.cvy(0, 9), 9);  // Now populated in non-holonomic
  EXPECT_NE(state.cwz(0, 9), 9);

  EXPECT_NEAR(state.cvx(0), 0, 0.3);
  EXPECT_NEAR(state.cvy(0), 0, 0.3);
  EXPECT_NEAR(state.cwz(0), 0, 0.3);
  EXPECT_NEAR(state.cvx(0, 9), 9, 0.3);
  EXPECT_NEAR(state.cvy(0, 9), 9, 0.3);
  EXPECT_NEAR(state.cwz(0, 9), 9, 0.3);

  generator.shutdown();
}

TEST(NoiseGeneratorTest, NoiseGeneratorMainNoRegenerate)
{
  // This time with no regeneration of noises
  auto node = std::make_shared<nav2::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  std::string name = "test";
  ParametersHandler handler(node, name);
  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;
  settings.sampling_std.vx = 0.1;
  settings.sampling_std.vy = 0.1;
  settings.sampling_std.wz = 0.1;

  // Populate a potential control sequence
  mppi::models::ControlSequence control_sequence;
  control_sequence.reset(25);
  for (unsigned int i = 0; i != control_sequence.vx.rows(); i++) {
    control_sequence.vx(i) = i;
    control_sequence.vy(i) = i;
    control_sequence.wz(i) = i;
  }

  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);

  // Request an update with no noise yet generated, should result in identical outputs
  generator.initialize(settings, false, "test_name", &handler);
  generator.reset(settings, false);  // sets initial sizing and zeros out noises
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  generator.setNoisedControls(state, control_sequence);

  // save initial state
  auto initial_cvx_0 = state.cvx(0);
  auto initial_cvy_0 = state.cvy(0);
  auto initial_cwz_0 = state.cwz(0);
  auto initial_cvx_9 = state.cvx(0, 9);
  auto initial_cvy_9 = state.cvy(0, 9);
  auto initial_cwz_9 = state.cwz(0, 9);

  EXPECT_NE(state.cvx(0), 0);
  EXPECT_EQ(state.cvy(0), 0);  // Not populated in non-holonomic
  EXPECT_NE(state.cwz(0), 0);
  EXPECT_NE(state.cvx(0, 9), 9);
  EXPECT_EQ(state.cvy(0, 9), 9);  // Not populated in non-holonomic
  EXPECT_NE(state.cwz(0, 9), 9);

  EXPECT_NEAR(state.cvx(0), 0, 0.3);
  EXPECT_NEAR(state.cwz(0), 0, 0.3);
  EXPECT_NEAR(state.cvx(0, 9), 9, 0.3);
  EXPECT_NEAR(state.cwz(0, 9), 9, 0.3);

  // this doesn't work if regenerate_noises is false
  generator.generateNextNoises();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  generator.setNoisedControls(state, control_sequence);

  // Ensure the state has changed after generating new noises
  EXPECT_EQ(state.cvx(0), initial_cvx_0);
  EXPECT_EQ(state.cvy(0), initial_cvy_0);  // Not populated in non-holonomic
  EXPECT_EQ(state.cwz(0), initial_cwz_0);
  EXPECT_EQ(state.cvx(0, 9), initial_cvx_9);
  EXPECT_EQ(state.cvy(0, 9), initial_cvy_9);  // Not populated in non-holonomic
  EXPECT_EQ(state.cwz(0, 9), initial_cwz_9);

  generator.shutdown();
}

TEST(NoiseGeneratorTest, AdaptiveStdsDisabled)
{
  // With every decay disabled, the adaptive deviations must mirror the static ones
  // no matter how fast the robot is moving
  auto node = std::make_shared<nav2::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  std::string name = "test";
  ParametersHandler handler(node, name);
  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;
  settings.sampling_std.vx = 0.2;
  settings.sampling_std.vy = 0.3;
  settings.sampling_std.wz = 0.4;
  settings.advanced_constraints.vx_std_decay_strength = -1.0;
  settings.advanced_constraints.vy_std_decay_strength = -1.0;
  settings.advanced_constraints.wz_std_decay_strength = -1.0;

  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);

  generator.initialize(settings, true, "test_name", &handler);
  generator.reset(settings, true);

  // Standstill
  generator.computeAdaptiveStds(state);
  EXPECT_NEAR(generator.getVxStdAdaptive(), 0.2, 1e-6);
  EXPECT_NEAR(generator.getVyStdAdaptive(), 0.3, 1e-6);
  EXPECT_NEAR(generator.getWzStdAdaptive(), 0.4, 1e-6);

  // Moving fast
  state.speed.linear.x = 1.5;
  state.speed.linear.y = 1.5;
  generator.computeAdaptiveStds(state);
  EXPECT_NEAR(generator.getVxStdAdaptive(), 0.2, 1e-6);
  EXPECT_NEAR(generator.getVyStdAdaptive(), 0.3, 1e-6);
  EXPECT_NEAR(generator.getWzStdAdaptive(), 0.4, 1e-6);

  generator.shutdown();
}

TEST(NoiseGeneratorTest, AdaptiveStdsPerAxisDecay)
{
  // Each linear axis decays on its own measured speed, wz decays on the linear speed magnitude
  auto node = std::make_shared<nav2::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  std::string name = "test";
  ParametersHandler handler(node, name);
  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;
  settings.sampling_std.vx = 0.6;
  settings.sampling_std.vy = 0.5;
  settings.sampling_std.wz = 0.4;
  settings.advanced_constraints.vx_std_decay_to = 0.2;
  settings.advanced_constraints.vx_std_decay_strength = 3.0;
  settings.advanced_constraints.vy_std_decay_to = 0.1;
  settings.advanced_constraints.vy_std_decay_strength = 3.0;
  settings.advanced_constraints.wz_std_decay_to = 0.05;
  settings.advanced_constraints.wz_std_decay_strength = 3.0;

  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);

  generator.initialize(settings, true, "test_name", &handler);
  generator.reset(settings, true);

  EXPECT_TRUE(generator.validateVxStdDecayConstraints());
  EXPECT_TRUE(generator.validateVyStdDecayConstraints());
  EXPECT_TRUE(generator.validateWzStdDecayConstraints());

  // At standstill every deviation equals its configured (boosted) value
  generator.computeAdaptiveStds(state);
  EXPECT_NEAR(generator.getVxStdAdaptive(), 0.6, 1e-6);
  EXPECT_NEAR(generator.getVyStdAdaptive(), 0.5, 1e-6);
  EXPECT_NEAR(generator.getWzStdAdaptive(), 0.4, 1e-6);

  // Moving on x only: vx decays towards its target, vy is untouched, wz decays too
  state.speed.linear.x = 10.0;
  state.speed.linear.y = 0.0;
  generator.computeAdaptiveStds(state);
  EXPECT_NEAR(generator.getVxStdAdaptive(), 0.2, 1e-4);
  EXPECT_NEAR(generator.getVyStdAdaptive(), 0.5, 1e-6);
  EXPECT_NEAR(generator.getWzStdAdaptive(), 0.05, 1e-4);

  // Moving on y only: vy decays towards its target, vx is untouched
  state.speed.linear.x = 0.0;
  state.speed.linear.y = 10.0;
  generator.computeAdaptiveStds(state);
  EXPECT_NEAR(generator.getVxStdAdaptive(), 0.6, 1e-6);
  EXPECT_NEAR(generator.getVyStdAdaptive(), 0.1, 1e-4);

  // The decay is monotonic, a slower robot always samples a wider spread
  state.speed.linear.x = 0.3;
  state.speed.linear.y = 0.0;
  generator.computeAdaptiveStds(state);
  const float std_at_slow_speed = generator.getVxStdAdaptive();
  state.speed.linear.x = 0.9;
  generator.computeAdaptiveStds(state);
  const float std_at_high_speed = generator.getVxStdAdaptive();
  EXPECT_GT(std_at_slow_speed, std_at_high_speed);
  EXPECT_LT(std_at_slow_speed, 0.6);
  EXPECT_GT(std_at_high_speed, 0.2);

  generator.shutdown();
}

TEST(NoiseGeneratorTest, AdaptiveStdsNonHolonomicAndInvalidBounds)
{
  // vy never adapts on a non holonomic base, and out of bounds targets fall back
  // to the static value
  auto node = std::make_shared<nav2::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  std::string name = "test";
  ParametersHandler handler(node, name);
  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;
  settings.sampling_std.vx = 0.6;
  settings.sampling_std.vy = 0.5;
  settings.sampling_std.wz = 0.4;
  // decay_to above the configured std, invalid
  settings.advanced_constraints.vx_std_decay_to = 0.9;
  settings.advanced_constraints.vx_std_decay_strength = 3.0;
  // valid, but the base below is not holonomic
  settings.advanced_constraints.vy_std_decay_to = 0.1;
  settings.advanced_constraints.vy_std_decay_strength = 3.0;
  // negative decay_to, invalid
  settings.advanced_constraints.wz_std_decay_to = -0.1;
  settings.advanced_constraints.wz_std_decay_strength = 3.0;

  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);
  state.speed.linear.x = 1.0;
  state.speed.linear.y = 1.0;

  generator.initialize(settings, false, "test_name", &handler);
  generator.reset(settings, false);

  EXPECT_FALSE(generator.validateVxStdDecayConstraints());
  EXPECT_TRUE(generator.validateVyStdDecayConstraints());
  EXPECT_FALSE(generator.validateWzStdDecayConstraints());

  generator.computeAdaptiveStds(state);
  EXPECT_NEAR(generator.getVxStdAdaptive(), 0.6, 1e-6);
  EXPECT_NEAR(generator.getVyStdAdaptive(), 0.5, 1e-6);
  EXPECT_NEAR(generator.getWzStdAdaptive(), 0.4, 1e-6);

  generator.shutdown();
}

TEST(NoiseGeneratorTest, AdaptiveStdsZeroDecayTargetRejected)
{
  // A decay target of 0 would drive the deviation to zero and blow up the gamma control cost
  // weight, so it is treated as an invalid configuration and the decay is skipped
  auto node = std::make_shared<nav2::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  std::string name = "test";
  ParametersHandler handler(node, name);
  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;
  settings.sampling_std.vx = 0.6;
  settings.sampling_std.vy = 0.5;
  settings.sampling_std.wz = 0.4;
  settings.advanced_constraints.vx_std_decay_to = 0.0;
  settings.advanced_constraints.vx_std_decay_strength = 3.0;
  settings.advanced_constraints.vy_std_decay_to = 0.0;
  settings.advanced_constraints.vy_std_decay_strength = 3.0;
  settings.advanced_constraints.wz_std_decay_to = 0.0;
  settings.advanced_constraints.wz_std_decay_strength = 3.0;

  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);

  generator.initialize(settings, true, "test_name", &handler);
  generator.reset(settings, true);

  EXPECT_FALSE(generator.validateVxStdDecayConstraints());
  EXPECT_FALSE(generator.validateVyStdDecayConstraints());
  EXPECT_FALSE(generator.validateWzStdDecayConstraints());

  // Even at a speed high enough to underflow the exponential, every deviation stays at its
  // configured value instead of collapsing to zero
  state.speed.linear.x = 50.0;
  state.speed.linear.y = 50.0;
  generator.computeAdaptiveStds(state);
  EXPECT_NEAR(generator.getVxStdAdaptive(), 0.6, 1e-6);
  EXPECT_NEAR(generator.getVyStdAdaptive(), 0.5, 1e-6);
  EXPECT_NEAR(generator.getWzStdAdaptive(), 0.4, 1e-6);

  generator.shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
