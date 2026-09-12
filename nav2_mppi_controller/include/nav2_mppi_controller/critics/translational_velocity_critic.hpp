// Copyright (c) 2026 Duatic AG
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__TRANSLATIONAL_VELOCITY_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__TRANSLATIONAL_VELOCITY_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::TranslationalVelocityCritic
 * @brief Critic objective function penalizing combined translational velocities that exceed the
 *        ellipse spanned by the per-axis velocity limits
 *
 * The per-axis limits bound vx and vy independently, so a diagonal command may reach a combined
 * speed of sqrt(vx_max² + vy_max²) while a straight one is capped at vx_max. A critic incentivizing
 * the highest translational speed therefore favors driving diagonally. Bounding the combination by
 * the ellipse through the per-axis limits removes that incentive. Only holonomic motion models are
 * scored, as the others have no vy to combine.
 */
class TranslationalVelocityCritic : public CriticFunction
{
public:
  /**
   * @brief Initialize critic
   */
  void initialize() override;

  /**
   * @brief Evaluate cost related to translational velocities outside the velocity ellipse
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float vx_max_{0};
  float vx_min_{0};
  float vy_max_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__TRANSLATIONAL_VELOCITY_CRITIC_HPP_
