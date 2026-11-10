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

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/utilities.hpp"
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

TEST(NoiseGeneratorTest, ResetUsesUpdatedSamplingStd)
{
  for (const bool regenerate : {false, true}) {
    SCOPED_TRACE(regenerate);
    auto node = std::make_shared<nav2::LifecycleNode>("node");
    node->declare_parameter("test.regenerate_noises", rclcpp::ParameterValue(regenerate));
    std::string name = "test";
    ParametersHandler handler(node, name);
    NoiseGenerator generator;
    models::OptimizerSettings settings;
    settings.batch_size = 1000;
    settings.time_steps = 20;
    settings.sampling_std.vx = 0.2f;
    settings.sampling_std.vy = 0.3f;
    settings.sampling_std.wz = 0.4f;

    models::ControlSequence control_sequence;
    control_sequence.reset(settings.time_steps);
    models::State state;
    state.reset(settings.batch_size, settings.time_steps);

    auto wait_for_noise = [&]() {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        do {
          generator.setNoisedControls(state, control_sequence);
          if (!state.cwz.isZero()) {
            return true;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (std::chrono::steady_clock::now() < deadline);
        return false;
      };

    generator.initialize(settings, true, name, &handler);
    generator.reset(settings, true);
    EXPECT_TRUE(wait_for_noise());

    settings.sampling_std.vx = 0.05f;
    settings.sampling_std.vy = 0.07f;
    settings.sampling_std.wz = 0.09f;
    generator.reset(settings, true);
    EXPECT_TRUE(wait_for_noise());
    generator.shutdown();

    // Zero controls expose the new noise scale directly, with 20,000 samples per axis.
    EXPECT_NEAR(std::sqrt(state.cvx.square().mean()), settings.sampling_std.vx, 0.005f);
    EXPECT_NEAR(std::sqrt(state.cvy.square().mean()), settings.sampling_std.vy, 0.007f);
    EXPECT_NEAR(std::sqrt(state.cwz.square().mean()), settings.sampling_std.wz, 0.009f);
  }
}

TEST(NoiseGeneratorTest, ShutdownWithRegeneratedNoisesDoesNotHang)
{
  // shutdown() must wake the noise thread even if it is just about to wait on the condition
  // variable, so repeat short lifecycles many times to hit that window.
  auto node = std::make_shared<nav2::LifecycleNode>("node");
  node->declare_parameter("test.regenerate_noises", rclcpp::ParameterValue(true));
  std::string name = "test";
  ParametersHandler handler(node, name);
  models::OptimizerSettings settings;
  settings.batch_size = 32;
  settings.time_steps = 8;
  settings.sampling_std.vx = 0.2f;
  settings.sampling_std.vy = 0.2f;
  settings.sampling_std.wz = 0.4f;

  models::ControlSequence control_sequence;
  control_sequence.reset(settings.time_steps);
  models::State state;
  state.reset(settings.batch_size, settings.time_steps);

  for (int i = 0; i < 4000; ++i) {
    auto generator = std::make_shared<NoiseGenerator>();
    generator->initialize(settings, false, name, &handler);
    // shutdown() runs on a helper thread that is already started and is called 0-63 us after
    // the last generateNextNoises(), sweeping the moment the noise thread goes back to wait.
    auto go = std::make_shared<std::atomic<bool>>(false);
    const auto delay = std::chrono::microseconds(i % 64);
    // A hung shutdown must fail the test, not block it: on failure the future is leaked
    // instead of being destroyed, as its destructor would wait for shutdown() to return.
    auto shutdown = std::make_unique<std::future<void>>(
      std::async(
        std::launch::async, [generator, go, delay]() {
          while (!go->load()) {
            std::this_thread::yield();
          }
          const auto start = std::chrono::steady_clock::now();
          while (std::chrono::steady_clock::now() - start < delay) {
          }
          generator->shutdown();
        }));
    for (int cycle = 0; cycle < 3; ++cycle) {
      generator->setNoisedControls(state, control_sequence);
      generator->generateNextNoises();
    }
    go->store(true);
    if (shutdown->wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      shutdown.release();
      FAIL() << "NoiseGenerator::shutdown() did not return in lifecycle " << i;
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
