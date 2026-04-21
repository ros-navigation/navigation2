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
#include <limits>
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

// AI-generated contribution marker:
// LP-MPPI-focused helpers and tests below were added with AI assistance.
class NoiseGeneratorTester : public NoiseGenerator
{
public:
  void setFilterInputsForTest(float model_dt, double cutoff_hz, int order)
  {
    settings_.model_dt = model_dt;
    filter_cutoff_frequency_ = cutoff_hz;
    filter_order_ = order;
  }

  bool designLowPassForTest()
  {
    return designButterworthLowPass();
  }

  double getConfiguredCutoff() const
  {
    return filter_cutoff_frequency_;
  }

  bool isFilterConfigured() const
  {
    return lpf_configured_;
  }
};

double meanAbsTemporalDifference(const Eigen::ArrayXXf & data)
{
  if (data.cols() < 2) {
    return 0.0;
  }

  double total = 0.0;
  size_t count = 0;
  for (int row = 0; row < data.rows(); ++row) {
    for (int col = 1; col < data.cols(); ++col) {
      total += std::abs(data(row, col) - data(row, col - 1));
      ++count;
    }
  }

  return count > 0 ? total / static_cast<double>(count) : 0.0;
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

TEST(NoiseGeneratorTest, LowPassFilterSmoothsPerturbations)
{
  auto filtered_node = std::make_shared<nav2::LifecycleNode>("filtered_node");
  filtered_node->declare_parameter("filtered_ns.regenerate_noises", rclcpp::ParameterValue(false));
  filtered_node->declare_parameter("filtered_ns.use_low_pass_filter", rclcpp::ParameterValue(true));
  filtered_node->declare_parameter(
    "filtered_ns.filter_cutoff_frequency", rclcpp::ParameterValue(0.8));
  filtered_node->declare_parameter("filtered_ns.filter_order", rclcpp::ParameterValue(2));
  std::string filtered_name = "filtered";
  ParametersHandler filtered_handler(filtered_node, filtered_name);

  auto raw_node = std::make_shared<nav2::LifecycleNode>("raw_node");
  raw_node->declare_parameter("raw_ns.regenerate_noises", rclcpp::ParameterValue(false));
  raw_node->declare_parameter("raw_ns.use_low_pass_filter", rclcpp::ParameterValue(false));
  std::string raw_name = "raw";
  ParametersHandler raw_handler(raw_node, raw_name);

  mppi::models::OptimizerSettings settings;
  settings.batch_size = 128;
  settings.time_steps = 80;
  settings.model_dt = 0.05;
  settings.sampling_std.vx = 0.25;
  settings.sampling_std.vy = 0.25;
  settings.sampling_std.wz = 0.25;

  mppi::models::ControlSequence control_sequence;
  control_sequence.reset(settings.time_steps);
  mppi::models::State filtered_state, raw_state;
  filtered_state.reset(settings.batch_size, settings.time_steps);
  raw_state.reset(settings.batch_size, settings.time_steps);

  NoiseGenerator filtered_generator;
  filtered_generator.initialize(settings, false, "filtered_ns", &filtered_handler);
  filtered_generator.reset(settings, false);
  filtered_generator.setNoisedControls(filtered_state, control_sequence);
  filtered_generator.shutdown();

  NoiseGenerator raw_generator;
  raw_generator.initialize(settings, false, "raw_ns", &raw_handler);
  raw_generator.reset(settings, false);
  raw_generator.setNoisedControls(raw_state, control_sequence);
  raw_generator.shutdown();

  const double filtered_vx_diff = meanAbsTemporalDifference(filtered_state.cvx);
  const double raw_vx_diff = meanAbsTemporalDifference(raw_state.cvx);
  const double filtered_wz_diff = meanAbsTemporalDifference(filtered_state.cwz);
  const double raw_wz_diff = meanAbsTemporalDifference(raw_state.cwz);

  EXPECT_LT(filtered_vx_diff, raw_vx_diff);
  EXPECT_LT(filtered_wz_diff, raw_wz_diff);

  // Repeat in holonomic mode to verify vy smoothing path.
  filtered_state.reset(settings.batch_size, settings.time_steps);
  raw_state.reset(settings.batch_size, settings.time_steps);

  NoiseGenerator filtered_holo_generator;
  filtered_holo_generator.initialize(settings, true, "filtered_ns", &filtered_handler);
  filtered_holo_generator.reset(settings, true);
  filtered_holo_generator.setNoisedControls(filtered_state, control_sequence);
  filtered_holo_generator.shutdown();

  NoiseGenerator raw_holo_generator;
  raw_holo_generator.initialize(settings, true, "raw_ns", &raw_handler);
  raw_holo_generator.reset(settings, true);
  raw_holo_generator.setNoisedControls(raw_state, control_sequence);
  raw_holo_generator.shutdown();

  const double filtered_vy_diff = meanAbsTemporalDifference(filtered_state.cvy);
  const double raw_vy_diff = meanAbsTemporalDifference(raw_state.cvy);

  EXPECT_LT(filtered_vy_diff, raw_vy_diff);
}

TEST(NoiseGeneratorTest, CutoffClampedToNyquist)
{
  auto node = std::make_shared<nav2::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  node->declare_parameter("test_name.use_low_pass_filter", rclcpp::ParameterValue(true));
  node->declare_parameter("test_name.filter_cutoff_frequency", rclcpp::ParameterValue(1000.0));
  node->declare_parameter("test_name.filter_order", rclcpp::ParameterValue(2));
  std::string name = "test";
  ParametersHandler handler(node, name);

  mppi::models::OptimizerSettings settings;
  settings.batch_size = 64;
  settings.time_steps = 32;
  settings.model_dt = 0.1;
  settings.sampling_std.vx = 0.2;
  settings.sampling_std.vy = 0.2;
  settings.sampling_std.wz = 0.2;

  NoiseGeneratorTester generator;
  generator.initialize(settings, false, "test_name", &handler);

  const double nyquist = 1.0 / (2.0 * settings.model_dt);
  EXPECT_TRUE(generator.isFilterConfigured());
  EXPECT_GT(generator.getConfiguredCutoff(), 0.0);
  EXPECT_LT(generator.getConfiguredCutoff(), nyquist);

  generator.shutdown();
}

TEST(NoiseGeneratorTest, LowPassFilterDisabledWithNonPositiveModelDt)
{
  auto node = std::make_shared<nav2::LifecycleNode>("node_invalid_dt");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  node->declare_parameter("test_name.use_low_pass_filter", rclcpp::ParameterValue(true));
  node->declare_parameter("test_name.filter_cutoff_frequency", rclcpp::ParameterValue(1.0));
  node->declare_parameter("test_name.filter_order", rclcpp::ParameterValue(2));
  std::string name = "test";
  ParametersHandler handler(node, name);

  mppi::models::OptimizerSettings settings;
  settings.batch_size = 16;
  settings.time_steps = 8;
  settings.model_dt = 0.0;
  settings.sampling_std.vx = 0.2;
  settings.sampling_std.vy = 0.2;
  settings.sampling_std.wz = 0.2;

  NoiseGeneratorTester generator;
  generator.initialize(settings, false, "test_name", &handler);

  EXPECT_FALSE(generator.isFilterConfigured());
  generator.shutdown();
}

TEST(NoiseGeneratorTest, LowPassFilterDisabledWithNonPositiveCutoff)
{
  auto node = std::make_shared<nav2::LifecycleNode>("node_invalid_cutoff");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  node->declare_parameter("test_name.use_low_pass_filter", rclcpp::ParameterValue(true));
  node->declare_parameter("test_name.filter_cutoff_frequency", rclcpp::ParameterValue(0.0));
  node->declare_parameter("test_name.filter_order", rclcpp::ParameterValue(2));
  std::string name = "test";
  ParametersHandler handler(node, name);

  mppi::models::OptimizerSettings settings;
  settings.batch_size = 16;
  settings.time_steps = 8;
  settings.model_dt = 0.1;
  settings.sampling_std.vx = 0.2;
  settings.sampling_std.vy = 0.2;
  settings.sampling_std.wz = 0.2;

  NoiseGeneratorTester generator;
  generator.initialize(settings, false, "test_name", &handler);

  EXPECT_FALSE(generator.isFilterConfigured());
  generator.shutdown();
}

TEST(NoiseGeneratorTest, LowPassFilterDisabledWithInvalidOrder)
{
  auto node = std::make_shared<nav2::LifecycleNode>("node_invalid_order");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  node->declare_parameter("test_name.use_low_pass_filter", rclcpp::ParameterValue(true));
  node->declare_parameter("test_name.filter_cutoff_frequency", rclcpp::ParameterValue(1.0));
  node->declare_parameter("test_name.filter_order", rclcpp::ParameterValue(0));
  std::string name = "test";
  ParametersHandler handler(node, name);

  mppi::models::OptimizerSettings settings;
  settings.batch_size = 16;
  settings.time_steps = 8;
  settings.model_dt = 0.1;
  settings.sampling_std.vx = 0.2;
  settings.sampling_std.vy = 0.2;
  settings.sampling_std.wz = 0.2;

  NoiseGeneratorTester generator;
  generator.initialize(settings, false, "test_name", &handler);

  EXPECT_FALSE(generator.isFilterConfigured());
  generator.shutdown();
}

TEST(NoiseGeneratorTest, LowPassFilterDisabledWhenDesignFails)
{
  auto node = std::make_shared<nav2::LifecycleNode>("node_design_fail");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  node->declare_parameter("test_name.use_low_pass_filter", rclcpp::ParameterValue(true));
  node->declare_parameter("test_name.filter_cutoff_frequency", rclcpp::ParameterValue(1.0));
  node->declare_parameter("test_name.filter_order", rclcpp::ParameterValue(2));
  std::string name = "test";
  ParametersHandler handler(node, name);

  mppi::models::OptimizerSettings settings;
  settings.batch_size = 16;
  settings.time_steps = 8;
  settings.model_dt = std::numeric_limits<float>::quiet_NaN();
  settings.sampling_std.vx = 0.2;
  settings.sampling_std.vy = 0.2;
  settings.sampling_std.wz = 0.2;

  NoiseGeneratorTester generator;
  generator.initialize(settings, false, "test_name", &handler);

  EXPECT_FALSE(generator.isFilterConfigured());
  generator.shutdown();
}

TEST(NoiseGeneratorTest, DesignRejectsInvalidOrderInput)
{
  NoiseGeneratorTester generator;
  generator.setFilterInputsForTest(0.1f, 1.0, 0);
  EXPECT_FALSE(generator.designLowPassForTest());
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
