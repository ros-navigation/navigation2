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
#include <cmath>
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

static double highFrequencyEnergyRatio(const Eigen::ArrayXXf & values)
{
  double high_frequency_energy = 0.0;
  double total_energy = 0.0;
  const int time_steps = values.cols();
  const double two_pi = 2.0 * std::acos(-1.0);

  for (int row = 0; row < values.rows(); ++row) {
    const double mean = values.row(row).mean();
    for (int frequency_index = 1; frequency_index <= time_steps / 2; ++frequency_index) {
      double real = 0.0;
      double imaginary = 0.0;
      for (int time_index = 0; time_index < time_steps; ++time_index) {
        const double angle = two_pi *
          static_cast<double>(frequency_index) *
          static_cast<double>(time_index) /
          static_cast<double>(time_steps);
        const double value = static_cast<double>(values(row, time_index)) - mean;
        real += value * std::cos(angle);
        imaginary -= value * std::sin(angle);
      }

      const double energy = real * real + imaginary * imaginary;
      total_energy += energy;
      if (frequency_index >= time_steps / 4) {
        high_frequency_energy += energy;
      }
    }
  }

  return high_frequency_energy / total_energy;
}

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

TEST(NoiseGeneratorTest, ColoredNoiseDisabledDoesNotRequireOptionalParams)
{
  auto node = std::make_shared<nav2::LifecycleNode>("colored_noise_disabled_node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  node->declare_parameter("test_name.colored_noise.enabled", rclcpp::ParameterValue(false));
  std::string name = "test";
  ParametersHandler handler(node, name);

  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 20;
  settings.time_steps = 10;
  settings.sampling_std.vx = 0.1;
  settings.sampling_std.vy = 0.1;
  settings.sampling_std.wz = 0.1;

  EXPECT_NO_THROW(generator.initialize(settings, false, "test_name", &handler));
  EXPECT_NO_THROW(generator.reset(settings, false));
  generator.shutdown();
}

TEST(NoiseGeneratorTest, ColoredNoiseHasLowerHighFrequencyEnergyThanWhiteNoise)
{
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 400;
  settings.time_steps = 56;
  settings.sampling_std.vx = 0.4;
  settings.sampling_std.vy = 0.4;
  settings.sampling_std.wz = 0.4;

  auto sample_noise = [&](bool use_colored_noise) {
      auto node = std::make_shared<nav2::LifecycleNode>(
        use_colored_noise ? "colored_noise_node" : "white_noise_node");
      node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
      node->declare_parameter(
        "test_name.colored_noise.enabled",
        rclcpp::ParameterValue(use_colored_noise));
      if (use_colored_noise) {
        node->declare_parameter("test_name.colored_noise.exponent", rclcpp::ParameterValue(2.0));
        node->declare_parameter("test_name.colored_noise.offset_t", rclcpp::ParameterValue(1));
        node->declare_parameter(
          "test_name.colored_noise.offset_decay_rate",
          rclcpp::ParameterValue(0.97));
        node->declare_parameter("test_name.colored_noise.fmin", rclcpp::ParameterValue(0.0));
      }

      std::string name = "test";
      ParametersHandler handler(node, name);
      NoiseGenerator generator;
      generator.initialize(settings, false, "test_name", &handler);
      generator.reset(settings, false);

      mppi::models::ControlSequence control_sequence;
      control_sequence.reset(settings.time_steps);
      mppi::models::State state;
      state.reset(settings.batch_size, settings.time_steps);
      generator.setNoisedControls(state, control_sequence);
      generator.shutdown();
      return state.cvx;
    };

  const auto white_noise = sample_noise(false);
  const auto colored_noise = sample_noise(true);

  EXPECT_LT(highFrequencyEnergyRatio(colored_noise), highFrequencyEnergyRatio(white_noise) * 0.25);
}

TEST(NoiseGeneratorTest, ColoredNoiseInvalidParametersAreClamped)
{
  auto node = std::make_shared<nav2::LifecycleNode>("colored_noise_clamp_node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  node->declare_parameter("test_name.colored_noise.enabled", rclcpp::ParameterValue(true));
  node->declare_parameter("test_name.colored_noise.exponent", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("test_name.colored_noise.offset_t", rclcpp::ParameterValue(-4));
  node->declare_parameter("test_name.colored_noise.offset_decay_rate", rclcpp::ParameterValue(1.4));
  node->declare_parameter("test_name.colored_noise.fmin", rclcpp::ParameterValue(0.8));
  std::string name = "test";
  ParametersHandler handler(node, name);

  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 20;
  settings.time_steps = 10;
  settings.sampling_std.vx = 0.1;
  settings.sampling_std.vy = 0.1;
  settings.sampling_std.wz = 0.1;

  generator.initialize(settings, false, "test_name", &handler);
  generator.reset(settings, false);

  mppi::models::ControlSequence control_sequence;
  control_sequence.reset(settings.time_steps);
  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);
  EXPECT_NO_THROW(generator.setNoisedControls(state, control_sequence));
  EXPECT_TRUE(state.cvx.isFinite().all());
  EXPECT_TRUE(state.cwz.isFinite().all());
  generator.shutdown();
}

TEST(NoiseGeneratorTest, ColoredNoiseZeroStdProducesZeroNoise)
{
  auto node = std::make_shared<nav2::LifecycleNode>("colored_noise_zero_std_node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  node->declare_parameter("test_name.colored_noise.enabled", rclcpp::ParameterValue(true));
  std::string name = "test";
  ParametersHandler handler(node, name);

  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 20;
  settings.time_steps = 10;
  settings.sampling_std.vx = 0.0;
  settings.sampling_std.vy = 0.0;
  settings.sampling_std.wz = 0.0;

  generator.initialize(settings, false, "test_name", &handler);
  generator.reset(settings, false);

  mppi::models::ControlSequence control_sequence;
  control_sequence.reset(settings.time_steps);
  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);
  generator.setNoisedControls(state, control_sequence);

  EXPECT_TRUE(state.cvx.isApproxToConstant(0.0f));
  EXPECT_TRUE(state.cwz.isApproxToConstant(0.0f));
  generator.shutdown();
}

TEST(NoiseGeneratorTest, ColoredNoiseGeneratesHolonomicYAxis)
{
  auto node = std::make_shared<nav2::LifecycleNode>("colored_noise_holonomic_node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  node->declare_parameter("test_name.colored_noise.enabled", rclcpp::ParameterValue(true));
  std::string name = "test";
  ParametersHandler handler(node, name);

  NoiseGenerator generator;
  mppi::models::OptimizerSettings settings;
  settings.batch_size = 20;
  settings.time_steps = 10;
  settings.sampling_std.vx = 0.1;
  settings.sampling_std.vy = 0.1;
  settings.sampling_std.wz = 0.1;

  generator.initialize(settings, true, "test_name", &handler);
  generator.reset(settings, true);

  mppi::models::ControlSequence control_sequence;
  control_sequence.reset(settings.time_steps);
  mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);
  generator.setNoisedControls(state, control_sequence);

  EXPECT_TRUE(state.cvy.isFinite().all());
  EXPECT_GT(state.cvy.abs().maxCoeff(), 0.0f);
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
