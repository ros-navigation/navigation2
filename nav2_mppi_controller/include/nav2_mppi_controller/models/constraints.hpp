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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_

namespace mppi::models
{

/**
 * @struct mppi::models::AxisResponse
 * @brief Response of the robot's velocity on an axis to a command: commands within the deadband
 * have no effect, otherwise are scaled by the gain, take effect after the delay, and the
 * velocity then responds with a first-order lag
 */
struct AxisResponse
{
  float delay{0.0f};
  float time_constant{0.0f};
  float gain{1.0f};
  float deadband{0.0f};

  /**
   * @brief Whether commands are achieved immediately after the delay
   */
  bool isIdeal() const
  {
    return time_constant <= 0.0f && gain == 1.0f && deadband <= 0.0f;
  }
};

/**
 * @struct mppi::models::ActuationModel
 * @brief Response of the robot to commands on each axis
 */
struct ActuationModel
{
  AxisResponse vx;
  AxisResponse vy;
  AxisResponse wz;
  // Whether the first control of a sequence is sent, rather than pinned to the current velocity
  bool first_control_sent{true};
};

/**
 * @struct mppi::models::ControlConstraints
 * @brief Constraints on control
 */
struct ControlConstraints
{
  float vx_max;
  float vx_min;
  float vy;
  float wz;
  float ax_max;
  float ax_min;
  float ay_min;
  float ay_max;
  float az_max;
};

/**
 * @struct mppi::models::SamplingStd
 * @brief Noise parameters for sampling trajectories
 */
struct SamplingStd
{
  float vx;
  float vy;
  float wz;
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
