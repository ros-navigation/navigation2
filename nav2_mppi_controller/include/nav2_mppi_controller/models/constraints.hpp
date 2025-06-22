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

#include <memory>
#include <string>
#include <stdexcept>

namespace mppi::models
{

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

struct AdvancedConstraints
{
  /**
   * @brief Defines the strength of the reduction function
   * Allows dynamic modification of wz_std (angular deviation) based on linear velocity of the robot.
   * When a robot with high inertia (e.g. 500kg) is moving fast and if wz_std is above 0.3, oscillation behavior can be observed. Lowering wz_std stabilizes the robot but then the maneuvers take more time.
   * Dynamically reducing wz_std as vx, vy increase (speed of the robot) solves both problems.
   * Suggested values to start with: wz_std = 0.4, wz_std_decay_to = 0.05, wz_std_decay_strength = 3.0
   * The following is used as the decay function
   * <pre>f(x) = (wz_std - wz_std_decay_to) * e^(-wz_std_decay_strength * v_linear) + wz_std_decay_to</pre>
   * <a href="https://www.wolframalpha.com/input?i=plot+%5B%28a-b%29+*+e%5E%28-c+*+x%29+%2B+b%5D+with+a%3D0.5%2C+b%3D0.05%2C+c%3D3">Playground</a>
   * Default: -1.0 (disabled)
   */
  float wz_std_decay_strength;

  /**
   * @brief Target wz_std value while linear speed goes to infinity.
   * Must be between 0 and wz_std.
   * Has no effect if `advanced.wz_std_decay_strength` <= 0.0
   * Default: 0.0
   */
  float wz_std_decay_to;
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

  /**
   * @brief Internal variable that holds wz_std after decay is applied.
   * If decay is disabled, SamplingStd.wz == wz_std_adaptive
  */
  std::shared_ptr<float> wz_std_adaptive;

  /**
   * @param c Reference to the advanced constraints
   * @param quiet Whether to throw reason or just return false if there's an invalid constraint config
   * @return true if constraints are valid
   * @throw std::runtime_error Throws the error reason only if quiet is false and constraint configuration is invalid
   */
  bool validateConstraints(const AdvancedConstraints & c, bool quiet)
  {
    // Assume valid if angular decay is disabled
    if (c.wz_std_decay_strength <= 0.0f) {
      return true;  // valid
    }

    if (c.wz_std_decay_to < 0.0f || c.wz_std_decay_to > wz) {
      if (quiet) {
        return false;
      }
      // else
      const std::string msg =
        "wz_std_decay_to must be between 0 and wz_std. "
        "wz: " + std::to_string(wz) + ", wz_std_decay_to: " + std::to_string(c.wz_std_decay_to);
      throw std::runtime_error(msg);
    }
    return true;
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__CONSTRAINTS_HPP_
