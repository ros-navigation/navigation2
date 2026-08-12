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

#include <Eigen/Dense>
#include <cmath>

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

/**
 * @brief Returns a scale factor, projecting any infeasible velocity vector back onto the
 *        elliptical velocity space
 *
 * The envelope is the ellipse inscribed by the per-axis limits,
 *     (vx / vx_max)² + (vy / vy_max)² <= 1     driving forward, vx >= 0
 *     (vx / vx_min)² + (vy / vy_max)² <= 1     driving in reverse, vx < 0
 *
 * @return Scale factors in (0, 1]
 */
template<typename Derived>
inline typename Derived::PlainObject getVelocityScalingFactor(
  const Eigen::ArrayBase<Derived> & vx, const Eigen::ArrayBase<Derived> & vy,
  const float vx_max, const float vx_min, const float vy_max)
{
  // Protect inverse of v_max² against division by zero
  constexpr float eps = 1e-6f;
  const float vx_min_abs = std::fabs(vx_min);
  // 1/(vx_max²) for vx >= 0, 1/(vx_min²) for vx < 0, and 1/(vy_max²) for vy
  const float inv_vx_max_sq = vx_max > eps ? 1.0f / (vx_max * vx_max) : 0.0f;
  const float inv_vx_min_sq = vx_min_abs > eps ? 1.0f / (vx_min_abs * vx_min_abs) : 0.0f;
  const float inv_vy_max_sq = vy_max > eps ? 1.0f / (vy_max * vy_max) : 0.0f;

  // Return the scale factor
  // 1/sqrt[(vx/vx_max)² + (vy/vy_max)²], vx >= 0
  // 1/sqrt[(vx/vx_min)² + (vy/vy_max)²], vx < 0
  return (
    // x-axis contribution
    (vx >= 0.0f).select(
      vx.square() * inv_vx_max_sq,
      vx.square() * inv_vx_min_sq
    )

    // y-axis contribution
    + (vy.square() * inv_vy_max_sq)

  ).max(1.0f)  // clamp to 1.0f to avoid scaling feasible velocities
         .sqrt().inverse();  // Invert the squared distance to get the final multiplier
}

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
