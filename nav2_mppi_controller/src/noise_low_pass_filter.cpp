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

#include "nav2_mppi_controller/tools/noise_low_pass_filter.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <numeric>

namespace mppi
{

void NoiseLowPassFilter::configure(
  bool active, float model_dt, double cutoff_frequency, int order,
  const rclcpp::Logger & logger)
{
  reset();
  requested_active_ = active;

  if (!active) {
    return;
  }

  requested_cutoff_frequency_ = cutoff_frequency;
  model_dt_ = model_dt;
  order_ = order;

  if (model_dt_ <= 0.0f) {
    RCLCPP_WARN(
      logger,
      "Low-pass filter disabled: model_dt must be > 0, got %.6f",
      model_dt_);
    return;
  }

  const double fs = 1.0 / static_cast<double>(model_dt_);
  const double nyquist = 0.5 * fs;
  if (!std::isfinite(nyquist) || nyquist <= 0.0) {
    RCLCPP_WARN(
      logger,
      "Low-pass filter disabled: Nyquist frequency must be > 0, got %.6f",
      nyquist);
    return;
  }

  if (requested_cutoff_frequency_ <= 0.0) {
    RCLCPP_WARN(
      logger,
      "Low-pass filter disabled: cutoff_frequency must be > 0, got %.6f",
      requested_cutoff_frequency_);
    return;
  }

  if (order_ < 1) {
    RCLCPP_WARN(
      logger,
      "Low-pass filter disabled: order must be >= 1, got %d",
      order_);
    return;
  }

  effective_cutoff_frequency_ = requested_cutoff_frequency_;
  if (effective_cutoff_frequency_ >= nyquist) {
    effective_cutoff_frequency_ = std::nextafter(nyquist, 0.0);
    RCLCPP_WARN(
      logger,
      "Low-pass filter cutoff_frequency %.6f Hz is >= Nyquist %.6f Hz. "
      "Clamping effective cutoff to %.6f Hz.",
      requested_cutoff_frequency_, nyquist, effective_cutoff_frequency_);
  }

  if (!design(model_dt_)) {
    reset();
    RCLCPP_WARN(logger, "Low-pass filter disabled: failed to design coefficients.");
    return;
  }

  active_ = true;
}

void NoiseLowPassFilter::resetDt(float model_dt, const rclcpp::Logger & logger)
{
  const bool requested_active = requested_active_;
  const double requested_cutoff_frequency = requested_cutoff_frequency_;
  const int order = order_;

  configure(requested_active, model_dt, requested_cutoff_frequency, order, logger);
}

bool NoiseLowPassFilter::isActive() const
{
  return active_;
}

double NoiseLowPassFilter::getRequestedCutoffFrequency() const
{
  return requested_cutoff_frequency_;
}

double NoiseLowPassFilter::getEffectiveCutoffFrequency() const
{
  return effective_cutoff_frequency_;
}

int NoiseLowPassFilter::getOrder() const
{
  return order_;
}

int NoiseLowPassFilter::getWarmupSteps(unsigned int time_steps) const
{
  if (!active_ || model_dt_ <= 0.0f || effective_cutoff_frequency_ <= 0.0) {
    return 0;
  }

  const double sample_rate = 1.0 / static_cast<double>(model_dt_);
  const int cutoff_response_steps = static_cast<int>(
    std::ceil(4.0 * sample_rate / effective_cutoff_frequency_));
  const int order_response_steps = std::max(1, order_) * 8;
  const int max_warmup_steps = std::max(1, static_cast<int>(time_steps) * 2);

  return std::clamp(
    std::max(cutoff_response_steps, order_response_steps), 1, max_warmup_steps);
}

void NoiseLowPassFilter::filter(Eigen::ArrayXXf & signal) const
{
  if (!active_) {
    return;
  }

  const size_t order = static_cast<size_t>(order_);
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
        y += b_[idx] * x_hist[idx];
      }
      for (size_t idx = 1; idx <= order; ++idx) {
        y -= a_[idx] * y_hist[idx];
      }

      y_hist[0] = y;
      signal(row, col) = static_cast<float>(y);
    }
  }
}

void NoiseLowPassFilter::reset()
{
  requested_active_ = false;
  active_ = false;
  model_dt_ = 0.0f;
  requested_cutoff_frequency_ = 0.0;
  effective_cutoff_frequency_ = 0.0;
  order_ = 0;
  a_.clear();
  b_.clear();
}

bool NoiseLowPassFilter::design(float model_dt)
{
  const size_t order = static_cast<size_t>(order_);

  const double fs = 1.0 / static_cast<double>(model_dt);
  const double pi = std::acos(-1.0);
  const double omega_c = 2.0 * fs * std::tan(pi * effective_cutoff_frequency_ / fs);

  if (!std::isfinite(omega_c) || omega_c <= 0.0) {
    return false;
  }

  std::vector<std::complex<double>> z_poles;
  z_poles.reserve(order);

  for (int k = 0; k < order_; ++k) {
    const double theta = pi * (2.0 * static_cast<double>(k) + 1.0 + order_) /
      (2.0 * static_cast<double>(order_));
    const std::complex<double> s_pole = omega_c * std::exp(std::complex<double>(0.0, theta));
    const std::complex<double> z_pole = (2.0 * fs + s_pole) / (2.0 * fs - s_pole);
    z_poles.push_back(z_pole);
  }

  // Denominator A(z^-1) = Pi (1 - z_pole * z^-1)
  std::vector<std::complex<double>> a_poly(1, std::complex<double>(1.0, 0.0));
  for (const auto & pole : z_poles) {
    std::vector<std::complex<double>> next(a_poly.size() + 1, std::complex<double>(0.0, 0.0));
    for (size_t i = 0; i < a_poly.size(); ++i) {
      next[i] += a_poly[i];
      next[i + 1] -= a_poly[i] * pole;
    }
    a_poly = std::move(next);
  }

  a_.resize(a_poly.size());
  for (size_t i = 0; i < a_poly.size(); ++i) {
    a_[i] = a_poly[i].real();
  }

  // Numerator B(z^-1) = K * (1 + z^-1)^order
  std::vector<double> b_poly(1, 1.0);
  b_poly.resize(order + 1, 0.0);
  for (int i = 1; i <= order_; ++i) {
    for (int j = i; j > 0; --j) {
      b_poly[static_cast<size_t>(j)] += b_poly[static_cast<size_t>(j - 1)];
    }
  }

  const double sum_a = std::accumulate(a_.begin(), a_.end(), 0.0);
  const double sum_b = std::accumulate(b_poly.begin(), b_poly.end(), 0.0);
  if (std::abs(sum_b) < std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const double dc_gain = sum_a / sum_b;
  b_.resize(b_poly.size());
  for (size_t i = 0; i < b_poly.size(); ++i) {
    b_[i] = b_poly[i] * dc_gain;
  }

  if (a_.empty() || std::abs(a_.front()) < std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const double a0 = a_.front();
  for (auto & a : a_) {
    a /= a0;
  }
  for (auto & b : b_) {
    b /= a0;
  }

  return true;
}

}  // namespace mppi
