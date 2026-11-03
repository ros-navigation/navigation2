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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__OMNI_CONSTRAINT_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__OMNI_CONSTRAINT_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::OmniConstraintCritic
 * @brief Critic objective function for enforcing velocity limits on omnidirectional robots.
 *
 * Bounding omnidirectional robot velocities per axis individually makes critics rewarding progress
 * incentivize diagonal driving, since sqrt(vx_max² + vy_max²) exceeds vx_max and vy_max
 * individually. This critic bounds vx and vy by the ellipse spanned by the controller's vx_max,
 * vx_min and vy_max instead. This penalizes translational speeds above the individual axis limits.
 *
 * For mecanum drivetrains use this critic together with the MecanumCritic, which enforces the wheel speed limits.
 *
 * The cost is the speed the sample gives up when scaled back onto the ellipse.
 */
class OmniConstraintCritic : public CriticFunction
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

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__OMNI_CONSTRAINT_CRITIC_HPP_
