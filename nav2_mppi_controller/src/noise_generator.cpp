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

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <vector>

namespace mppi
{

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

  ndistribution_vx_ = std::normal_distribution(0.0f, settings_.sampling_std.vx);
  ndistribution_vy_ = std::normal_distribution(0.0f, settings_.sampling_std.vy);
  ndistribution_wz_ = std::normal_distribution(0.0f, settings_.sampling_std.wz);

  auto getParam = param_handler->getParamGetter(name);
  getParam(regenerate_noises_, "regenerate_noises", false);
  getParam(use_low_pass_filter_, "use_low_pass_filter", false, ParameterType::Static);
  getParam(
    filter_cutoff_frequency_, "filter_cutoff_frequency", 2.0,
    ParameterType::Static);
  getParam(filter_order_, "filter_order", 2, ParameterType::Static);

  configureLowPassFilter();

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

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  state.cvx = noises_vx_.rowwise() + control_sequence.vx.transpose();
  state.cvy = noises_vy_.rowwise() + control_sequence.vy.transpose();
  state.cwz = noises_wz_.rowwise() + control_sequence.wz.transpose();
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  ndistribution_vx_ = std::normal_distribution(0.0f, settings_.sampling_std.vx);
  ndistribution_vy_ = std::normal_distribution(0.0f, settings_.sampling_std.vy);
  ndistribution_wz_ = std::normal_distribution(0.0f, settings_.sampling_std.wz);
  configureLowPassFilter();

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noises_vx_.setZero(settings_.batch_size, settings_.time_steps);
    noises_vy_.setZero(settings_.batch_size, settings_.time_steps);
    noises_wz_.setZero(settings_.batch_size, settings_.time_steps);
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

  if (lpf_configured_) {
    applyLowPassFilter(noises_vx_);
    applyLowPassFilter(noises_wz_);
    if (is_holonomic_) {
      applyLowPassFilter(noises_vy_);
    }
  }
}

void NoiseGenerator::configureLowPassFilter()
{
  lpf_configured_ = false;
  lpf_a_.clear();
  lpf_b_.clear();

  if (!use_low_pass_filter_) {
    return;
  }

  if (settings_.model_dt <= 0.0f) {
    RCLCPP_WARN(
      logger_,
      "Low-pass filter disabled: model_dt must be > 0, got %.6f",
      settings_.model_dt);
    return;
  }

  const double fs = 1.0 / static_cast<double>(settings_.model_dt);
  const double nyquist = 0.5 * fs;

  if (filter_cutoff_frequency_ <= 0.0) {
    RCLCPP_WARN(
      logger_,
      "Low-pass filter disabled: filter_cutoff_frequency must be > 0, got %.6f",
      filter_cutoff_frequency_);
    return;
  }

  if (filter_order_ < 1) {
    RCLCPP_WARN(
      logger_,
      "Low-pass filter disabled: filter_order must be >= 1, got %d",
      filter_order_);
    return;
  }

  if (filter_cutoff_frequency_ >= nyquist) {
    const double clamped = std::max(
      std::nextafter(nyquist, 0.0),
      nyquist * 0.99);
    RCLCPP_WARN(
      logger_,
      "filter_cutoff_frequency %.6f Hz is >= Nyquist %.6f Hz. Clamping to %.6f Hz.",
      filter_cutoff_frequency_, nyquist, clamped);
    filter_cutoff_frequency_ = clamped;
  }

  if (!designButterworthLowPass()) {
    RCLCPP_WARN(logger_, "Low-pass filter disabled: failed to design coefficients.");
    return;
  }

  lpf_configured_ = true;
}

bool NoiseGenerator::designButterworthLowPass()
{
  const int order_i = filter_order_;
  if (order_i < 1) {
    return false;
  }
  const size_t order = static_cast<size_t>(order_i);

  const double fs = 1.0 / static_cast<double>(settings_.model_dt);
  const double pi = std::acos(-1.0);
  const double omega_c = 2.0 * fs * std::tan(pi * filter_cutoff_frequency_ / fs);

  if (!std::isfinite(omega_c) || omega_c <= 0.0) {
    return false;
  }

  std::vector<std::complex<double>> z_poles;
  z_poles.reserve(order);

  for (int k = 0; k < order_i; ++k) {
    const double theta = pi * (2.0 * static_cast<double>(k) + 1.0 + order_i) /
      (2.0 * static_cast<double>(order_i));
    const std::complex<double> s_pole = omega_c * std::exp(std::complex<double>(0.0, theta));
    const std::complex<double> z_pole = (2.0 * fs + s_pole) / (2.0 * fs - s_pole);
    z_poles.push_back(z_pole);
  }

  // Denominator A(z^-1) = Π (1 - z_pole * z^-1)
  std::vector<std::complex<double>> a_poly(1, std::complex<double>(1.0, 0.0));
  for (const auto & pole : z_poles) {
    std::vector<std::complex<double>> next(a_poly.size() + 1, std::complex<double>(0.0, 0.0));
    for (size_t i = 0; i < a_poly.size(); ++i) {
      next[i] += a_poly[i];
      next[i + 1] -= a_poly[i] * pole;
    }
    a_poly = std::move(next);
  }

  lpf_a_.resize(a_poly.size());
  for (size_t i = 0; i < a_poly.size(); ++i) {
    lpf_a_[i] = a_poly[i].real();
  }

  // Numerator B(z^-1) = K * (1 + z^-1)^order
  std::vector<double> b_poly(1, 1.0);
  b_poly.resize(order + 1, 0.0);
  for (int i = 1; i <= order_i; ++i) {
    for (int j = i; j > 0; --j) {
      b_poly[static_cast<size_t>(j)] += b_poly[static_cast<size_t>(j - 1)];
    }
  }

  const double sum_a = std::accumulate(lpf_a_.begin(), lpf_a_.end(), 0.0);
  const double sum_b = std::accumulate(b_poly.begin(), b_poly.end(), 0.0);
  if (std::abs(sum_b) < std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const double dc_gain = sum_a / sum_b;
  lpf_b_.resize(b_poly.size());
  for (size_t i = 0; i < b_poly.size(); ++i) {
    lpf_b_[i] = b_poly[i] * dc_gain;
  }

  if (std::abs(lpf_a_.front()) < std::numeric_limits<double>::epsilon()) {
    return false;
  }

  // Normalize by a0 for direct-form implementation.
  const double a0 = lpf_a_.front();
  for (auto & a : lpf_a_) {
    a /= a0;
  }
  for (auto & b : lpf_b_) {
    b /= a0;
  }

  return true;
}

void NoiseGenerator::applyLowPassFilter(Eigen::ArrayXXf & signal) const
{
  if (!lpf_configured_) {
    return;
  }

  const size_t order = static_cast<size_t>(filter_order_);
  std::vector<double> x_hist(order + 1, 0.0);
  std::vector<double> y_hist(order + 1, 0.0);

  for (int row = 0; row < signal.rows(); ++row) {
    std::fill(x_hist.begin(), x_hist.end(), 0.0);
    std::fill(y_hist.begin(), y_hist.end(), 0.0);

    for (int col = 0; col < signal.cols(); ++col) {
      for (size_t idx = order; idx > 0; --idx) {
        x_hist[idx] = x_hist[idx - 1];
        y_hist[idx] = y_hist[idx - 1];
      }

      x_hist[0] = static_cast<double>(signal(row, col));

      double y = 0.0;
      for (size_t idx = 0; idx <= order; ++idx) {
        y += lpf_b_[idx] * x_hist[idx];
      }
      for (size_t idx = 1; idx <= order; ++idx) {
        y -= lpf_a_[idx] * y_hist[idx];
      }

      y_hist[0] = y;
      signal(row, col) = static_cast<float>(y);
    }
  }
}

}  // namespace mppi
