// Copyright (c) 2024 Enway GmbH, Adi Vardi
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__DIRECTION_CHANGE_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__DIRECTION_CHANGE_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::DirectionChangeCritic
 * @brief Critic objective function for penalizing changes in driving direction.
 */
class DirectionChangeCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to changing driving direction
   *
   * @param costs [out] add goal angle cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float threshold_to_consider_{0};
  bool enforce_path_inversion_{false};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__DIRECTION_CHANGE_CRITIC_HPP_
