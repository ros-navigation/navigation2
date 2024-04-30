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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_

// xtensor creates warnings that needs to be ignored as we are building with -Werror
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#pragma GCC diagnostic pop

namespace mppi::models
{

/**
 * @class mppi::models::Trajectories
 * @brief Candidate Trajectories
 */
struct Trajectories
{
  xt::xtensor<float, 2> x;
  xt::xtensor<float, 2> y;
  xt::xtensor<float, 2> yaws;

  /**
    * @brief Reset state data
    */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    x = xt::zeros<float>({batch_size, time_steps});
    y = xt::zeros<float>({batch_size, time_steps});
    yaws = xt::zeros<float>({batch_size, time_steps});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__TRAJECTORIES_HPP_
