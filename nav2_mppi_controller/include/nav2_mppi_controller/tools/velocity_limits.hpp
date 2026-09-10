// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__VELOCITY_LIMITS_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__VELOCITY_LIMITS_HPP_

#include <algorithm>

// Scalar velocity and acceleration limit helpers.

namespace mppi::utils
{

/**
 * @brief Clamps the input between the given lower and upper bounds.
 * @param lower_bound
 * @param upper_bound
 * @return Clamped output
 */
inline float clamp(
  const float lower_bound, const float upper_bound, const float input)
{
  return std::min(upper_bound, std::max(input, lower_bound));
}

/**
 * @brief Clamp velocity by acceleration limits, whereas max is used for
 * accelerating in speed (forward or reverse) and min is used for deceleration
 * @param last_vel Previous velocity
 * @param curr_vel Current velocity to clamp
 * @param min_delta Minimum velocity change (deceleration, typically negative)
 * @param max_delta Maximum velocity change (acceleration, typically positive)
 * @return Clamped velocity
 */
inline float clampVelocityByAccel(
  const float last_vel, const float curr_vel,
  const float min_delta, const float max_delta)
{
  if (last_vel >= 0) {
    return clamp(last_vel + min_delta, last_vel + max_delta, curr_vel);
  }
  return clamp(last_vel - max_delta, last_vel - min_delta, curr_vel);
}

/**
 * @brief Fraction of a requested velocity change that the acceleration limits allow
 * @param last_vel Velocity at the previous time step
 * @param curr_vel Requested velocity
 * @param min_delta Most negative change in speed magnitude allowed this step
 * @param max_delta Most positive change in speed magnitude allowed this step
 * @return Fraction in [0, 1], or 1 when no change is requested
 */
inline float reachableFraction(
  const float last_vel, const float curr_vel,
  const float min_delta, const float max_delta)
{
  const float delta = curr_vel - last_vel;
  if (delta == 0.0f) {
    return 1.0f;
  }
  return (clampVelocityByAccel(last_vel, curr_vel, min_delta, max_delta) - last_vel) / delta;
}

}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__VELOCITY_LIMITS_HPP_
