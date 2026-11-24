// Copyright (c) 2026 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__ACCEL_BOUNDS_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__ACCEL_BOUNDS_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <utility>

namespace mppi::utils
{

/**
 * @class mppi::utils::AccelBounds
 * @brief Velocities reachable in one step: speeding up (away from zero, forward or reverse) by up
 * to max_delta and slowing down (toward zero) by up to -min_delta. When slowing down through zero
 * within the step, the rest of the step speeds up in the other direction, so from rest both
 * directions are limited by max_delta, regardless of measurement noise around zero.
 */
template<typename T>
class AccelBounds
{
public:
  /**
   * @brief Constructor
   * @param min_delta Maximum change in velocity in a step when slowing down (negative)
   * @param max_delta Maximum change in velocity in a step when speeding up (positive)
   */
  AccelBounds(const T min_delta, const T max_delta)
  : up_(max_delta), down_(-min_delta),
    // Through zero, the velocity reached is linear in the starting velocity: slowing down to zero
    // takes |v| / down_ of the step, then speeding up for the rest of it
    crossing_(down_ > T(0) && down_ != up_), slope_(down_ > T(0) ? up_ / down_ : T(0)),
    slows_faster_(down_ >= up_) {}

  /**
   * @brief Velocities reachable in one step from a velocity
   * @param v Velocity
   * @return Lowest and highest reachable velocities
   */
  std::pair<T, T> operator()(const T v) const
  {
    if (v >= T(0)) {
      T lower = v - down_;
      if (crossing_) {
        const T through_zero = slope_ * v - up_;
        lower = slows_faster_ ? std::max(lower, through_zero) : std::min(lower, through_zero);
      }
      return {lower, v + up_};
    }
    T upper = v + down_;
    if (crossing_) {
      const T through_zero = slope_ * v + up_;
      upper = slows_faster_ ? std::min(upper, through_zero) : std::max(upper, through_zero);
    }
    return {v - up_, upper};
  }

  /**
   * @brief Clamp controls to the velocities reachable in one step from the previous velocities
   * @param prev Previous velocities
   * @param control Controls to clamp
   * @param out Clamped velocities
   */
  void clamp(
    const Eigen::Ref<const Eigen::ArrayXf> & prev, const Eigen::Ref<const Eigen::ArrayXf> & control,
    Eigen::Ref<Eigen::ArrayXf> out) const
  {
    if (down_ == up_) {
      // Symmetric limits, through zero or not
      out = control.cwiseMax(prev - up_).cwiseMin(prev + up_);
    } else if (!crossing_) {
      // Unable to slow down
      out = control
        .cwiseMax((prev >= T(0)).select(prev - down_, prev - up_))
        .cwiseMin((prev < T(0)).select(prev + down_, prev + up_));
    } else {
      // With the positive and negative parts of the velocity, only one of which is non-zero,
      // each bound is the zero crossing or not, without selecting on the sign: speeding up from
      // the other part, at zero, takes min(down, up) = up when slowing down faster, else max
      const auto pos = prev.max(T(0));
      const auto neg = prev.min(T(0));
      if (slows_faster_) {
        out = control
          .cwiseMax(neg + (pos - down_).max(slope_ * pos - up_))
          .cwiseMin(pos + (neg + down_).min(slope_ * neg + up_));
      } else {
        out = control
          .cwiseMax(neg + (pos - down_).min(slope_ * pos - up_))
          .cwiseMin(pos + (neg + down_).max(slope_ * neg + up_));
      }
    }
  }

private:
  T up_;
  T down_;
  bool crossing_;
  T slope_;
  bool slows_faster_;
};

}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__ACCEL_BOUNDS_HPP_
