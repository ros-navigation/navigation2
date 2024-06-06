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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_

#include <Eigen/Dense>

namespace mppi::models
{

/**
 * @struct mppi::models::Path
 * @brief Path represented as a tensor
 */
struct Path
{
  Eigen::VectorXf x;
  Eigen::VectorXf y;
  Eigen::VectorXf yaws;

  /**
    * @brief Reset path data
    */
  void reset(unsigned int size)
  {
    x = Eigen::VectorXf::Zero(size);
    y = Eigen::VectorXf::Zero(size);
    yaws = Eigen::VectorXf::Zero(size);
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__PATH_HPP_
