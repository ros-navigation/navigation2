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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__NOISE_LOW_PASS_FILTER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__NOISE_LOW_PASS_FILTER_HPP_

#include <Eigen/Dense>

#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace mppi
{

/**
 * @class mppi::NoiseLowPassFilter
 * @brief Butterworth low-pass filter for LP-MPPI sampled perturbations
 */
class NoiseLowPassFilter
{
public:
  /**
   * @brief Configure the low-pass filter
   * @param active Whether filtering is requested
   * @param model_dt MPPI model timestep in seconds
   * @param cutoff_frequency Requested cutoff frequency in Hz
   * @param order Requested Butterworth filter order
   * @param logger Logger for validation warnings
   */
  void configure(
    bool active, float model_dt, double cutoff_frequency, int order,
    const rclcpp::Logger & logger);

  /**
   * @brief Recompute filter coefficients with a new MPPI model timestep
   * @param model_dt MPPI model timestep in seconds
   * @param logger Logger for validation warnings
   */
  void resetDt(float model_dt, const rclcpp::Logger & logger);

  /**
   * @brief Check whether the filter is configured and active
   * @return True when configured successfully
   */
  bool isActive() const;

  /**
   * @brief Get the requested cutoff frequency
   * @return Requested cutoff frequency in Hz
   */
  double getRequestedCutoffFrequency() const;

  /**
   * @brief Get the effective cutoff frequency used by the filter design
   * @return Effective cutoff frequency in Hz
   */
  double getEffectiveCutoffFrequency() const;

  /**
   * @brief Get the configured filter order
   * @return Filter order
   */
  int getOrder() const;

  /**
   * @brief Get warm-up samples to generate before cropping the filtered sequence
   * @param time_steps MPPI prediction horizon samples
   * @return Number of warm-up samples
   */
  int getWarmupSteps(unsigned int time_steps) const;

  /**
   * @brief Apply the filter over the time axis for each sampled trajectory row
   * @param signal Matrix in shape [batch_size, time_steps]
   */
  void filter(Eigen::ArrayXXf & signal) const;

private:
  void reset();
  bool design(float model_dt);

  bool requested_active_{false};
  bool active_{false};
  float model_dt_{0.0f};
  double requested_cutoff_frequency_{0.0};
  double effective_cutoff_frequency_{0.0};
  int order_{0};
  std::vector<double> a_;
  std::vector<double> b_;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__NOISE_LOW_PASS_FILTER_HPP_
