// Copyright (c) 2025 Fumiya Ohnishi
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

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__DYNAMIC_WINDOW_PURE_PURSUIT_FUNCTIONS_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__DYNAMIC_WINDOW_PURE_PURSUIT_FUNCTIONS_HPP_

#include <string>
#include <vector>
#include <algorithm>
#include <tuple>
#include <utility>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_regulated_pure_pursuit_controller
{

namespace dynamic_window_pure_pursuit
{

struct DynamicWindowBounds
{
  double max_linear_vel;
  double min_linear_vel;
  double max_angular_vel;
  double min_angular_vel;
};

/**
 * @brief Compute the dynamic window (feasible velocity bounds) based on the current speed and the given velocity and acceleration constraints.
 * @param current_speed     Current linear and angular velocity of the robot
 * @param max_linear_vel    Maximum allowable linear velocity
 * @param min_linear_vel    Minimum allowable linear velocity
 * @param max_angular_vel   Maximum allowable angular velocity
 * @param min_angular_vel   Minimum allowable angular velocity
 * @param max_linear_accel  Maximum allowable linear acceleration
 * @param max_linear_decel  Maximum allowable linear deceleration
 * @param max_angular_accel Maximum allowable angular acceleration
 * @param max_angular_decel Maximum allowable angular deceleration
 * @param dt                Control duration
 * @return                  Computed dynamic window's velocity bounds
 */
inline DynamicWindowBounds computeDynamicWindow(
  const geometry_msgs::msg::Twist & current_speed,
  const double & max_linear_vel,
  const double & min_linear_vel,
  const double & max_angular_vel,
  const double & min_angular_vel,
  const double & max_linear_accel,
  const double & max_linear_decel,
  const double & max_angular_accel,
  const double & max_angular_decel,
  const double & dt
)
{
  DynamicWindowBounds dynamic_window;
  constexpr double Eps = 1e-3;

  // function to compute dynamic window for a single dimension
  auto compute_window = [&](const double & current_vel, const double & max_vel,
    const double & min_vel, const double & max_accel, const double & max_decel)
    {
      double candidate_max_vel = 0.0;
      double candidate_min_vel = 0.0;

      if (current_vel > Eps) {
      // if the current velocity is positive, acceleration means an increase in speed
        candidate_max_vel = current_vel + max_accel * dt;
        candidate_min_vel = current_vel - max_decel * dt;
      } else if (current_vel < -Eps) {
      // if the current velocity is negative, acceleration means a decrease in speed
        candidate_max_vel = current_vel + max_decel * dt;
        candidate_min_vel = current_vel - max_accel * dt;
      } else {
      // if the current velocity is zero, allow acceleration in both directions.
        candidate_max_vel = current_vel + max_accel * dt;
        candidate_min_vel = current_vel - max_accel * dt;
      }

    // clip to max/min velocity limits
      double dynamic_window_max_vel = std::min(candidate_max_vel, max_vel);
      double dynamic_window_min_vel = std::max(candidate_min_vel, min_vel);
      return std::make_tuple(dynamic_window_max_vel, dynamic_window_min_vel);
    };

  // linear velocity
  std::tie(dynamic_window.max_linear_vel,
        dynamic_window.min_linear_vel) = compute_window(current_speed.linear.x,
                 max_linear_vel, min_linear_vel,
                 max_linear_accel, max_linear_decel);

  // angular velocity
  std::tie(dynamic_window.max_angular_vel,
        dynamic_window.min_angular_vel) = compute_window(current_speed.angular.z,
                 max_angular_vel, min_angular_vel,
                 max_angular_accel, max_angular_decel);

  return dynamic_window;
}

/**
 * @brief                        Apply regulated linear velocity to the dynamic window
 * @param regulated_linear_vel   Regulated linear velocity
 * @param dynamic_window         Dynamic window to be regulated
 */
inline void applyRegulationToDynamicWindow(
  const double & regulated_linear_vel,
  DynamicWindowBounds & dynamic_window)
{
  double regulated_dynamic_window_max_linear_vel;
  double regulated_dynamic_window_min_linear_vel;

  // Extract the portion of the dynamic window that lies within the range [0, regulated_linear_vel]
  if (regulated_linear_vel >= 0.0) {
    regulated_dynamic_window_max_linear_vel = std::min(
      dynamic_window.max_linear_vel, regulated_linear_vel);
    regulated_dynamic_window_min_linear_vel = std::max(
      dynamic_window.min_linear_vel, 0.0);
  } else {
    regulated_dynamic_window_max_linear_vel = std::min(
      dynamic_window.max_linear_vel, 0.0);
    regulated_dynamic_window_min_linear_vel = std::max(
      dynamic_window.min_linear_vel, regulated_linear_vel);
  }

  if (regulated_dynamic_window_max_linear_vel < regulated_dynamic_window_min_linear_vel) {
    // No valid portion of the dynamic window remains after applying the regulation
    if (regulated_dynamic_window_min_linear_vel > 0.0) {
      // If the dynamic window is entirely in the positive range,
      // collapse both bounds to dynamic_window_min_linear_vel
      regulated_dynamic_window_max_linear_vel = regulated_dynamic_window_min_linear_vel;
    } else {
      // If the dynamic window is entirely in the negative range,
      // collapse both bounds to dynamic_window_max_linear_vel
      regulated_dynamic_window_min_linear_vel = regulated_dynamic_window_max_linear_vel;
    }
  }

  dynamic_window.max_linear_vel = regulated_dynamic_window_max_linear_vel;
  dynamic_window.min_linear_vel = regulated_dynamic_window_min_linear_vel;
}


/**
 * @brief                Compute the optimal velocity to follow the path within the dynamic window
 * @param dynamic_window Dynamic window defining feasible velocity bounds
 * @param curvature      Curvature of the path to follow
 * @param sign           Velocity sign (forward or backward)
 * @return               Optimal linear and angular velocity
 */
inline std::tuple<double, double> computeOptimalVelocityWithinDynamicWindow(
  const DynamicWindowBounds & dynamic_window,
  const double & curvature,
  const double & sign
)
{
  double optimal_linear_vel;
  double optimal_angular_vel;

  // consider linear_vel - angular_vel space (horizontal and vertical axes respectively)
  // Select the closest point to the line
  // angular_vel = curvature * linear_vel within the dynamic window.
  // If multiple points are equally close, select the one with the largest linear_vel.

  // When curvature == 0, the line is angular_vel = 0
  if (abs(curvature) < 1e-3) {
    // linear velocity
    if (sign >= 0.0) {
      // If moving forward, select the max linear vel
      optimal_linear_vel = dynamic_window.max_linear_vel;
    } else {
      // If moving backward, select the min linear vel
      optimal_linear_vel = dynamic_window.min_linear_vel;
    }

    // angular velocity
    // If the line angular_vel = 0 intersects the dynamic window,angular_vel = 0.0
    if (dynamic_window.min_angular_vel <= 0.0 && 0.0 <= dynamic_window.max_angular_vel) {
      optimal_angular_vel = 0.0;
    } else {
      // If not, select angular vel within dynamic window closest to 0
      if (std::abs(dynamic_window.min_angular_vel) <= std::abs(dynamic_window.max_angular_vel)) {
        optimal_angular_vel = dynamic_window.min_angular_vel;
      } else {
        optimal_angular_vel = dynamic_window.max_angular_vel;
      }
    }
    return std::make_tuple(optimal_linear_vel, optimal_angular_vel);
  }

  // When the dynamic window and the line angular_vel = curvature * linear_vel intersect,
  // select the intersection point that yields the highest linear velocity.

  // List the four candidate intersection points
  std::pair<double, double> candidates[] = {
    {dynamic_window.min_linear_vel, curvature * dynamic_window.min_linear_vel},
    {dynamic_window.max_linear_vel, curvature * dynamic_window.max_linear_vel},
    {dynamic_window.min_angular_vel / curvature, dynamic_window.min_angular_vel},
    {dynamic_window.max_angular_vel / curvature, dynamic_window.max_angular_vel}
  };

  double best_linear_vel = -std::numeric_limits<double>::infinity() * sign;
  double best_angular_vel = 0.0;

  for (auto [linear_vel, angular_vel] : candidates) {
    // Check whether the candidate lies within the dynamic window
    if (linear_vel >= dynamic_window.min_linear_vel &&
      linear_vel <= dynamic_window.max_linear_vel &&
      angular_vel >= dynamic_window.min_angular_vel &&
      angular_vel <= dynamic_window.max_angular_vel)
    {
      // Select the candidate with the largest linear velocity (considering moving direction)
      if (linear_vel * sign > best_linear_vel * sign) {
        best_linear_vel = linear_vel;
        best_angular_vel = angular_vel;
      }
    }
  }

  // If best_linear_vel was updated, it means that a valid intersection exists
  if (best_linear_vel != -std::numeric_limits<double>::infinity() * sign) {
    optimal_linear_vel = best_linear_vel;
    optimal_angular_vel = best_angular_vel;
    return std::make_tuple(optimal_linear_vel, optimal_angular_vel);
  }

  // When the dynamic window and the line angular_vel = curvature * linear_vel have no intersection,
  // select the point within the dynamic window that is closest to the line.

  // Because the dynamic window is a convex region,
  // the closest point must be one of its four corners.
  const std::array<std::array<double, 2>, 4> corners = {{
    {dynamic_window.min_linear_vel, dynamic_window.min_angular_vel},
    {dynamic_window.min_linear_vel, dynamic_window.max_angular_vel},
    {dynamic_window.max_linear_vel, dynamic_window.min_angular_vel},
    {dynamic_window.max_linear_vel, dynamic_window.max_angular_vel}
  }};

  // Compute the distance from a point (linear_vel, angular_vel)
  // to the line angular_vel = curvature * linear_vel
  const double denom = std::sqrt(curvature * curvature + 1.0);
  auto compute_dist = [&](const std::array<double, 2> & corner) -> double {
      return std::abs(curvature * corner[0] - corner[1]) / denom;
    };

  double closest_dist = std::numeric_limits<double>::infinity();
  best_linear_vel = -std::numeric_limits<double>::infinity() * sign;
  best_angular_vel = 0.0;

  for (const auto & corner : corners) {
    const double dist = compute_dist(corner);
    // Update if this corner is closer to the line,
    // or equally close but has a larger linear velocity (considering moving direction)
    if (dist < closest_dist ||
      (std::abs(dist - closest_dist) <= 1e-3 && corner[0] * sign > best_linear_vel * sign))
    {
      closest_dist = dist;
      best_linear_vel = corner[0];
      best_angular_vel = corner[1];
    }
  }

  optimal_linear_vel = best_linear_vel;
  optimal_angular_vel = best_angular_vel;

  return std::make_tuple(optimal_linear_vel, optimal_angular_vel);
}

}  // namespace dynamic_window_pure_pursuit

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__DYNAMIC_WINDOW_PURE_PURSUIT_FUNCTIONS_HPP_
