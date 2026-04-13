// Copyright (c) 2025 Nav2 Contributors
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

#ifndef CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_MATH_HPP_
#define CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"

namespace nav2_geometric_planners
{

/**
 * @brief Convert a ROS time stamp to seconds since epoch.
 * @param stamp ROS time stamp.
 * @return Time in seconds.
 */
inline double stampToSec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

/**
 * @brief Check that a ROS time stamp is non-zero.
 * @param stamp ROS time stamp.
 * @return True if the stamp carries a meaningful time value.
 */
inline bool isValidStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec != 0 || stamp.nanosec != 0;
}

/**
 * @brief Natural cubic spline in one dimension.
 *
 * Interpolates through the data points (t_[i], y_[i]) with natural boundary
 * conditions (second derivative = 0 at both ends).  The piecewise polynomial
 * on [t_[i], t_[i+1]] is:
 *
 *   S_i(t) = a_[i] + b_[i]*(t-t_[i]) + c_[i]*(t-t_[i])^2 + d_[i]*(t-t_[i])^3
 */
struct CubicSpline1D
{
  std::vector<double> t_;  ///< Parameter knots (strictly increasing).
  std::vector<double> a_;  ///< Constant coefficients (= y values).
  std::vector<double> b_;  ///< Linear coefficients.
  std::vector<double> c_;  ///< Quadratic coefficients.
  std::vector<double> d_;  ///< Cubic coefficients.

  /**
   * @brief Build the spline from parameter–value pairs.
   *
   * @param t Strictly increasing parameter values (timestamps).
   * @param y Corresponding function values (x or y coordinates).
   * @throws std::invalid_argument if fewer than 2 points are given.
   */
  void build(const std::vector<double> & t, const std::vector<double> & y)
  {
    const int n = static_cast<int>(t.size());
    if (n < 2) {
      throw std::invalid_argument("CubicSpline1D::build requires at least 2 points");
    }

    t_ = t;
    a_ = y;

    // Step sizes between knots.
    std::vector<double> h(n - 1);
    for (int i = 0; i < n - 1; ++i) {h[i] = t[i + 1] - t[i];}

    // Solve tridiagonal system for second derivatives M[i].
    // Natural BCs: M[0] = M[n-1] = 0.
    std::vector<double> M(n, 0.0);
    solveTridiagonal(h, y, n, M);

    // Compute spline coefficients from M.
    b_.resize(n - 1);
    c_.resize(n - 1);
    d_.resize(n - 1);
    for (int i = 0; i < n - 1; ++i) {
      a_[i] = y[i];  // already set above, explicit for clarity
      b_[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (2.0 * M[i] + M[i + 1]) / 6.0;
      c_[i] = M[i] / 2.0;
      d_[i] = (M[i + 1] - M[i]) / (6.0 * h[i]);
    }
  }

  /**
   * @brief Evaluate the spline at parameter t_eval.
   *
   * Clamps t_eval to [t_.front(), t_.back()].
   *
   * @param t_eval Query parameter.
   * @return Interpolated value.
   */
  double evaluate(double t_eval) const
  {
    t_eval = std::max(t_.front(), std::min(t_eval, t_.back()));

    // Find the interval containing t_eval.
    const int n = static_cast<int>(t_.size());
    int idx = static_cast<int>(
      std::lower_bound(t_.begin(), t_.end(), t_eval) - t_.begin()) - 1;
    idx = std::max(0, std::min(idx, n - 2));

    const double dt = t_eval - t_[idx];
    return a_[idx] + b_[idx] * dt + c_[idx] * dt * dt + d_[idx] * dt * dt * dt;
  }

private:
  /**
   * @brief Solve the natural-boundary tridiagonal system for M[].
   * @param h  Step sizes (length n-1).
   * @param y  Function values (length n).
   * @param n  Number of data points.
   * @param M  Output: second derivatives (length n, M[0]=M[n-1]=0).
   */
  void solveTridiagonal(
    const std::vector<double> & h,
    const std::vector<double> & y,
    int n,
    std::vector<double> & M) const
  {
    // Interior equations: h[i-1]*M[i-1] + 2*(h[i-1]+h[i])*M[i] + h[i]*M[i+1] = rhs[i]
    std::vector<double> diag(n - 2), upper(n - 3), lower(n - 3), rhs(n - 2);

    for (int i = 1; i < n - 1; ++i) {
      diag[i - 1] = 2.0 * (h[i - 1] + h[i]);
      rhs[i - 1] = 6.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    }
    for (int i = 0; i < n - 3; ++i) {
      upper[i] = h[i + 1];
      lower[i] = h[i + 1];
    }

    // Thomas algorithm (forward sweep).
    for (int i = 1; i < n - 2; ++i) {
      const double w = lower[i - 1] / diag[i - 1];
      diag[i] -= w * upper[i - 1];
      rhs[i]  -= w * rhs[i - 1];
    }

    // Back substitution.
    std::vector<double> interior(n - 2);
    interior[n - 3] = rhs[n - 3] / diag[n - 3];
    for (int i = n - 4; i >= 0; --i) {
      interior[i] = (rhs[i] - upper[i] * interior[i + 1]) / diag[i];
    }

    M[0] = 0.0;
    M[n - 1] = 0.0;
    for (int i = 1; i < n - 1; ++i) {M[i] = interior[i - 1];}
  }
};

}  // namespace nav2_geometric_planners

#endif  // CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_MATH_HPP_
