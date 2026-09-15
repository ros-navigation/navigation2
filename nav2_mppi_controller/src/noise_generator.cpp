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

#include "nav2_mppi_controller/tools/noise_generator.hpp"

#include <cmath>
#include <memory>
#include <mutex>

namespace mppi
{

/**
  * @brief Checks whether a decay is enabled and configured within its bounds
  * @param strength Strength of the decay function, values <= 0 disable the decay
  * @param decay_to Deviation approached while speed goes to infinity
  * @param base_std Deviation used at standstill
  * @return true if the decay is enabled and decay_to lies within (0, base_std]
  */
bool shouldApplyStdDecay(float strength, float decay_to, float base_std);

/**
  * @brief Exponentially decays a sampling deviation towards decay_to as speed grows
  * <pre>f(x) = (base_std - decay_to) * e^(-strength * speed) + decay_to</pre>
  * @param base_std Deviation used at standstill
  * @param decay_to Deviation approached while speed goes to infinity
  * @param strength Strength of the decay function
  * @param speed Speed the decay is driven by, must be non negative
  * @return Decayed deviation
  */
float applyStdDecay(float base_std, float decay_to, float strength, float speed);

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

  vx_std_adaptive_ = settings.sampling_std.vx;
  vy_std_adaptive_ = settings.sampling_std.vy;
  wz_std_adaptive_ = settings.sampling_std.wz;

  ndistribution_vx_ = std::normal_distribution(0.0f, settings_.sampling_std.vx);
  ndistribution_vy_ = std::normal_distribution(0.0f, settings_.sampling_std.vy);
  ndistribution_wz_ = std::normal_distribution(0.0f, settings_.sampling_std.wz);

  auto getParam = param_handler->getParamGetter(name);
  getParam(regenerate_noises_, "regenerate_noises", false);

  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::shutdown()
{
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::computeAdaptiveStds(const models::State & state)
{
  const auto & s = settings_;
  const auto & c = s.advanced_constraints;

  const auto vx = std::fabs(static_cast<float>(state.speed.linear.x));
  const auto vy = std::fabs(static_cast<float>(state.speed.linear.y));

  // vx decays on the measured vx of the robot
  if (shouldApplyStdDecay(c.vx_std_decay_strength, c.vx_std_decay_to, s.sampling_std.vx)) {
    vx_std_adaptive_ =
      applyStdDecay(s.sampling_std.vx, c.vx_std_decay_to, c.vx_std_decay_strength, vx);
  } else {
    vx_std_adaptive_ = s.sampling_std.vx;
  }

  // vy decays on the measured vy of the robot and only matters on a holonomic base
  if (is_holonomic_ &&
    shouldApplyStdDecay(c.vy_std_decay_strength, c.vy_std_decay_to, s.sampling_std.vy))
  {
    vy_std_adaptive_ =
      applyStdDecay(s.sampling_std.vy, c.vy_std_decay_to, c.vy_std_decay_strength, vy);
  } else {
    vy_std_adaptive_ = s.sampling_std.vy;
  }

  // wz decays on the linear speed magnitude of the robot
  if (shouldApplyStdDecay(c.wz_std_decay_strength, c.wz_std_decay_to, s.sampling_std.wz)) {
    const float linear_speed = is_holonomic_ ? hypotf(vx, vy) : vx;
    wz_std_adaptive_ = applyStdDecay(
      s.sampling_std.wz, c.wz_std_decay_to, c.wz_std_decay_strength, linear_speed);
  } else {
    wz_std_adaptive_ = s.sampling_std.wz;
  }

  // Check if there's any change on adaptive std's and re-create relevant distribution if any.
  // Note that a refreshed distribution only reaches the sampled noises on the next
  // generateNoisedControls() call, which requires the regenerate_noises parameter to be true.
  if (ndistribution_vx_.stddev() != vx_std_adaptive_) {
    ndistribution_vx_ = std::normal_distribution(0.0f, vx_std_adaptive_);
  }
  if (ndistribution_vy_.stddev() != vy_std_adaptive_) {
    ndistribution_vy_ = std::normal_distribution(0.0f, vy_std_adaptive_);
  }
  if (ndistribution_wz_.stddev() != wz_std_adaptive_) {
    ndistribution_wz_ = std::normal_distribution(0.0f, wz_std_adaptive_);
  }
}

float applyStdDecay(float base_std, float decay_to, float strength, float speed)
{
  return (base_std - decay_to) * std::exp(-1.0f * strength * speed) + decay_to;
}

bool shouldApplyStdDecay(float strength, float decay_to, float base_std)
{
  // Values <= 0 disable the decay
  if (strength <= 0.0f) {
    return false;
  }

  // Out of bounds targets are ignored rather than applied with an asymptote that breaks the cost.
  if (decay_to <= 0.0f || decay_to > base_std) {
    return false;
  }
  return true;
}

bool NoiseGenerator::validateVxStdDecayConstraints() const
{
  const models::AdvancedConstraints & c = settings_.advanced_constraints;
  // Assume valid if the vx decay is disabled
  if (c.vx_std_decay_strength <= 0.0f) {
    return true;  // valid
  }

  return shouldApplyStdDecay(
    c.vx_std_decay_strength, c.vx_std_decay_to, settings_.sampling_std.vx);
}

bool NoiseGenerator::validateVyStdDecayConstraints() const
{
  const models::AdvancedConstraints & c = settings_.advanced_constraints;
  // Assume valid if the vy decay is disabled
  if (c.vy_std_decay_strength <= 0.0f) {
    return true;  // valid
  }

  return shouldApplyStdDecay(
    c.vy_std_decay_strength, c.vy_std_decay_to, settings_.sampling_std.vy);
}

bool NoiseGenerator::validateWzStdDecayConstraints() const
{
  const models::AdvancedConstraints & c = settings_.advanced_constraints;
  // Assume valid if angular decay is disabled
  if (c.wz_std_decay_strength <= 0.0f) {
    return true;  // valid
  }

  return shouldApplyStdDecay(
    c.wz_std_decay_strength, c.wz_std_decay_to, settings_.sampling_std.wz);
}

float NoiseGenerator::getVxStdAdaptive() const
{
  return vx_std_adaptive_;
}

float NoiseGenerator::getVyStdAdaptive() const
{
  return vy_std_adaptive_;
}

float NoiseGenerator::getWzStdAdaptive() const
{
  return wz_std_adaptive_;
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  computeAdaptiveStds(state);

  state.cvx = noises_vx_.rowwise() + control_sequence.vx.transpose();
  state.cvy = noises_vy_.rowwise() + control_sequence.vy.transpose();
  state.cwz = noises_wz_.rowwise() + control_sequence.wz.transpose();
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    // reset initial adaptive values to parameterized values
    vx_std_adaptive_ = settings_.sampling_std.vx;
    vy_std_adaptive_ = settings_.sampling_std.vy;
    wz_std_adaptive_ = settings_.sampling_std.wz;

    noises_vx_.setZero(settings_.batch_size, settings_.time_steps);
    noises_vy_.setZero(settings_.batch_size, settings_.time_steps);
    noises_wz_.setZero(settings_.batch_size, settings_.time_steps);

    // Validate decay function, print warning message if decay_to is out of bounds
    if (!validateWzStdDecayConstraints()) {
      // RCLCPP_WARN_STREAM(
      //   *logger_, "wz_std_decay_to must be between 0 and wz_std. wz: "
      //               << std::to_string(settings_.constraints.wz) << ", wz_std_decay_to: "
      //               << std::to_string(settings_.advanced_constraints.wz_std_decay_to));
    }
    ready_ = true;
  }

  if (regenerate_noises_) {
    noise_cond_.notify_all();
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::noiseThread()
{
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() {return ready_;});
    ready_ = false;
    generateNoisedControls();
  } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;
  noises_vx_ = Eigen::ArrayXXf::NullaryExpr(
    s.batch_size, s.time_steps, [&]() {return ndistribution_vx_(generator_);});
  noises_wz_ = Eigen::ArrayXXf::NullaryExpr(
    s.batch_size, s.time_steps, [&]() {return ndistribution_wz_(generator_);});
  if (is_holonomic_) {
    noises_vy_ = Eigen::ArrayXXf::NullaryExpr(
      s.batch_size, s.time_steps, [&]() {return ndistribution_vy_(generator_);});
  }
}

}  // namespace mppi
