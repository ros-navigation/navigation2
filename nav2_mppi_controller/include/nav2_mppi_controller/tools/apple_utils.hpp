// Copyright (c) 2026 Dhruv Patel
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__APPLE_UTILS_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__APPLE_UTILS_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi::utils
{
/**
 * @brief Manual cumsum for 1D tensors (X, Y integration)
 */
template<typename E>
xt::xtensor<float, 1> manual_cumsum_1d(const E & expression)
{
  auto shape = expression.shape();
  xt::xtensor<float, 1> result = xt::zeros<float>(shape);
  float sum = 0.0f;
  for (size_t i = 0; i < shape[0]; ++i) {
    sum += static_cast<float>(expression(i));
    result(i) = sum;
  }
  return result;
}

/**
 * @brief Manual cumsum for 2D tensors (Trajectory integration)
 */
template<typename E>
xt::xtensor<float, 2> manual_cumsum_2d(const E & expression, int axis)
{
  auto shape = expression.shape();
  xt::xtensor<float, 2> result = xt::zeros<float>(shape);
  if (axis == 1) {
    for (size_t i = 0; i < shape[0]; ++i) {
      float row_sum = 0.0f;
      for (size_t j = 0; j < shape[1]; ++j) {
        row_sum += static_cast<float>(expression(i, j));
        result(i, j) = row_sum;
      }
    }
  } else {
    for (size_t j = 0; j < shape[1]; ++j) {
      float col_sum = 0.0f;
      for (size_t i = 0; i < shape[0]; ++i) {
        col_sum += static_cast<float>(expression(i, j));
        result(i, j) = col_sum;
      }
    }
  }
  return result;
}
}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__APPLE_UTILS_HPP_