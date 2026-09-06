// Copyright (c) 2026 Rem3000
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
//
// AI disclosure: this file was drafted with AI assistance (Claude) and reviewed by the author.

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__AXIS_ALIGN_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__AXIS_ALIGN_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::AxisAlignCritic
 * @brief Critic objective function for holonomic platforms that penalizes diagonal motion,
 * i.e. commanding vx and vy at the same time.
 *
 * Mecanum bases only drive two of their four wheels when translating at 45 degrees, which makes
 * diagonal motion slip-prone and poorly tracked on real hardware, while pure forward/backward or
 * pure lateral motion drives all four wheels. This critic scores each trajectory by how far its
 * body-frame velocity is from either axis. It is inactive for non-holonomic motion models.
 */
class AxisAlignCritic : public CriticFunction
{
public:
  /**
   * @brief Initialize critic
   */
  void initialize() override;

  /**
   * @brief Evaluate cost related to diagonal (simultaneous vx / vy) motion
   *
   * @param data Critic data to use in scoring; cost values are added to data.costs
   */
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float threshold_to_consider_{0};
  bool normalize_{true};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__AXIS_ALIGN_CRITIC_HPP_
