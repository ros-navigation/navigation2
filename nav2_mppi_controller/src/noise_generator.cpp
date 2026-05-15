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

#include <unsupported/Eigen/FFT>

#include <algorithm>
#include <cmath>
#include <complex>
#include <memory>
#include <mutex>
#include <vector>

namespace mppi
{

void ColoredNoiseGenerator::configure(
  bool enabled, float exponent, int offset_t, float offset_decay_rate,
  float fmin, const rclcpp::Logger & logger)
{
  enabled_ = enabled;
  if (!enabled_) {
    return;
  }

  if (exponent < 0.0f) {
    RCLCPP_WARN(
      logger,
      "colored_noise.exponent must be >= 0.0. Clamping %.3f to 0.0.",
      exponent);
    exponent = 0.0f;
  }

  if (offset_t < 0) {
    RCLCPP_WARN(
      logger,
      "colored_noise.offset_t must be >= 0. Clamping %d to 0.",
      offset_t);
    offset_t = 0;
  }

  if (offset_decay_rate < 0.0f || offset_decay_rate > 1.0f) {
    const float clamped = std::clamp(offset_decay_rate, 0.0f, 1.0f);
    RCLCPP_WARN(
      logger,
      "colored_noise.offset_decay_rate must be in [0.0, 1.0]. Clamping %.3f to %.3f.",
      offset_decay_rate, clamped);
    offset_decay_rate = clamped;
  }

  if (fmin < 0.0f || fmin > 0.5f) {
    const float clamped = std::clamp(fmin, 0.0f, 0.5f);
    RCLCPP_WARN(
      logger,
      "colored_noise.fmin must be in [0.0, 0.5]. Clamping %.3f to %.3f.",
      fmin, clamped);
    fmin = clamped;
  }

  exponent_ = exponent;
  offset_t_ = offset_t;
  offset_decay_rate_ = offset_decay_rate;
  fmin_ = fmin;
}

bool ColoredNoiseGenerator::isEnabled() const
{
  return enabled_;
}

void ColoredNoiseGenerator::generate(
  Eigen::ArrayXXf & noise, float sigma, std::default_random_engine & generator) const
{
  const int batch_size = noise.rows();
  const int time_steps = noise.cols();
  if (time_steps <= 0 || batch_size <= 0 || sigma <= 0.0f) {
    noise.setZero(batch_size, time_steps);
    return;
  }

  const int sample_time_steps = 2 * time_steps;
  const int frequency_count = sample_time_steps / 2 + 1;
  const float cutoff_frequency =
    std::max(fmin_, 1.0f / static_cast<float>(sample_time_steps));

  Eigen::ArrayXf frequency_scale(frequency_count);
  int smaller_frequency_count = 0;
  bool lower_frequency_bins_filled = false;
  for (int i = 0; i < frequency_count; ++i) {
    const float frequency = static_cast<float>(i) / static_cast<float>(sample_time_steps);
    if (frequency < cutoff_frequency) {
      ++smaller_frequency_count;
      continue;
    }

    if (!lower_frequency_bins_filled) {
      for (int j = 0; j < smaller_frequency_count; ++j) {
        frequency_scale(j) = std::pow(frequency, -exponent_ / 2.0f);
      }
      lower_frequency_bins_filled = true;
    }
    frequency_scale(i) = std::pow(frequency, -exponent_ / 2.0f);
  }
  double scale_square_sum = 0.0;
  for (int i = 1; i < frequency_count - 1; ++i) {
    scale_square_sum += static_cast<double>(frequency_scale(i)) *
      static_cast<double>(frequency_scale(i));
  }

  const float final_frequency_factor =
    (1.0f + static_cast<float>(sample_time_steps % 2)) / 2.0f;
  const double final_frequency_scale =
    static_cast<double>(frequency_scale(frequency_count - 1) * final_frequency_factor);
  scale_square_sum += final_frequency_scale * final_frequency_scale;

  const double unit_std =
    2.0 * std::sqrt(scale_square_sum) / static_cast<double>(sample_time_steps);

  const float unit_scale = 1.0f / static_cast<float>(unit_std);
  const int cropped_offset_t = std::min(offset_t_, sample_time_steps - 1);
  Eigen::FFT<float> fft;
  std::vector<std::complex<float>> frequency_domain(sample_time_steps);
  std::vector<std::complex<float>> time_domain;
  std::normal_distribution<float> normal_distribution(0.0f, 1.0f);

  for (int row = 0; row < batch_size; ++row) {
    std::fill(
      frequency_domain.begin(), frequency_domain.end(),
      std::complex<float>(0.0f, 0.0f));

    frequency_domain[0] = std::complex<float>(
      normal_distribution(generator) * frequency_scale(0) * std::sqrt(2.0f),
      0.0f);

    for (int frequency_index = 1; frequency_index < frequency_count - 1; ++frequency_index) {
      const std::complex<float> sample(
        normal_distribution(generator) * frequency_scale(frequency_index),
        normal_distribution(generator) * frequency_scale(frequency_index));
      frequency_domain[frequency_index] = sample;
      frequency_domain[sample_time_steps - frequency_index] = std::conj(sample);
    }

    frequency_domain[frequency_count - 1] = std::complex<float>(
      normal_distribution(generator) * frequency_scale(frequency_count - 1) * std::sqrt(2.0f),
      0.0f);
    fft.inv(time_domain, frequency_domain);

    const float offset_value = time_domain[cropped_offset_t].real() * unit_scale;
    float offset_decay = 1.0f;
    for (int time_index = 0; time_index < time_steps; ++time_index) {
      noise(row, time_index) =
        sigma * (time_domain[time_index].real() * unit_scale - offset_value * offset_decay);
      offset_decay *= offset_decay_rate_;
    }
  }
}

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;
  generator_ = std::default_random_engine();

  ndistribution_vx_ = std::normal_distribution(0.0f, settings_.sampling_std.vx);
  ndistribution_vy_ = std::normal_distribution(0.0f, settings_.sampling_std.vy);
  ndistribution_wz_ = std::normal_distribution(0.0f, settings_.sampling_std.wz);

  auto getParam = param_handler->getParamGetter(name);
  getParam(regenerate_noises_, "regenerate_noises", false);
  bool colored_noise_enabled = false;
  getParam(
    colored_noise_enabled, "colored_noise.enabled", false,
    ParameterType::Static);
  if (colored_noise_enabled) {
    double exponent = 2.0;
    int offset_t = 1;
    double offset_decay_rate = 0.97;
    double fmin = 0.0;
    getParam(exponent, "colored_noise.exponent", 2.0, ParameterType::Static);
    getParam(offset_t, "colored_noise.offset_t", 1, ParameterType::Static);
    getParam(
      offset_decay_rate, "colored_noise.offset_decay_rate", 0.97,
      ParameterType::Static);
    getParam(fmin, "colored_noise.fmin", 0.0, ParameterType::Static);
    colored_noise_generator_.configure(
      true, static_cast<float>(exponent), offset_t,
      static_cast<float>(offset_decay_rate), static_cast<float>(fmin), logger_);
  } else {
    colored_noise_generator_.configure(false, 2.0f, 1, 0.97f, 0.0f, logger_);
  }

  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::shutdown()
{
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    active_ = false;
    ready_ = true;
  }
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

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);
  if (regenerate_noises_) {
    noise_cond_.wait(guard, [this]() {return !ready_ || !active_;});
  }

  state.cvx = noises_vx_.rowwise() + control_sequence.vx.transpose();
  state.cvy = noises_vy_.rowwise() + control_sequence.vy.transpose();
  state.cwz = noises_wz_.rowwise() + control_sequence.wz.transpose();
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    settings_ = settings;
    is_holonomic_ = is_holonomic;
    ndistribution_vx_ = std::normal_distribution(0.0f, settings_.sampling_std.vx);
    ndistribution_vy_ = std::normal_distribution(0.0f, settings_.sampling_std.vy);
    ndistribution_wz_ = std::normal_distribution(0.0f, settings_.sampling_std.wz);

    // Recompute the noises on reset, initialization, and fallback.
    noises_vx_.setZero(settings_.batch_size, settings_.time_steps);
    noises_vy_.setZero(settings_.batch_size, settings_.time_steps);
    noises_wz_.setZero(settings_.batch_size, settings_.time_steps);
    ready_ = false;
    generateNoisedControls();
  }

  noise_cond_.notify_all();
}

void NoiseGenerator::noiseThread()
{
  while (true) {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() {return ready_ || !active_;});
    if (!active_) {
      break;
    }
    ready_ = false;
    generateNoisedControls();
    guard.unlock();
    noise_cond_.notify_all();
  }
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;
  if (colored_noise_generator_.isEnabled()) {
    noises_vx_.resize(s.batch_size, s.time_steps);
    noises_wz_.resize(s.batch_size, s.time_steps);
    colored_noise_generator_.generate(noises_vx_, s.sampling_std.vx, generator_);
    colored_noise_generator_.generate(noises_wz_, s.sampling_std.wz, generator_);
    if (is_holonomic_) {
      noises_vy_.resize(s.batch_size, s.time_steps);
      colored_noise_generator_.generate(noises_vy_, s.sampling_std.vy, generator_);
    }
    return;
  }

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
