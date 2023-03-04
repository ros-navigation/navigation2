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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__CONTROL_SEQUENCE_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__CONTROL_SEQUENCE_HPP_

#include <xtensor/xtensor.hpp>

namespace mppi::models
{

/**
 * @struct mppi::models::Control
 * @brief A set of controls
 */
struct Control
{
  float vx, vy, wz;
};

/**
 * @struct mppi::models::ControlSequence
 * @brief A control sequence over time (e.g. trajectory)
 */
struct ControlSequence
{
  xt::xtensor<float, 1> vx;
  xt::xtensor<float, 1> vy;
  xt::xtensor<float, 1> wz;

  void reset(unsigned int time_steps)
  {
    vx = xt::zeros<float>({time_steps});
    vy = xt::zeros<float>({time_steps});
    wz = xt::zeros<float>({time_steps});
  }
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__CONTROL_SEQUENCE_HPP_
