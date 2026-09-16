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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__MECANUM_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__MECANUM_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::MecanumCritic
 * @brief Critic objective function penalizing velocity combinations a mecanum drivetrain cannot
 *        deliver, its wheels being shared between the two translational axes and rotation
 *
 * A mecanum drivetrain produces vx, vy and wz from the same four wheels: with rollers at 45 degrees
 * a wheel turns at (vx ± vy ± (lx + ly) * wz) / r, so a wheel saturates once
 * |vx| / vx_max + |vy| / vy_max + |wz| / wz_max reaches 1, each per-axis limit being what that axis
 * could reach on its own. The achievable set is therefore a simplex rather than the box the limits
 * describe: a diagonal command is slower than a straight one, and rotating costs translational
 * speed. Only holonomic motion models are scored, as the others have no vy to combine.
 *
 * The cost is the speed the sample has to give up to fit: the translational speed plus the rotation
 * converted to the same units by its moment arm lx + ly, times the fraction of the demand the
 * wheels cannot meet. Rotation is charged like translation, so a sample that is infeasible through
 * rotation alone is scored like any other.
 */
class MecanumCritic : public CriticFunction
{
public:
  /**
   * @brief Initialize critic
   */
  void initialize() override;

  /**
   * @brief Evaluate cost related to velocities the mecanum wheels cannot deliver
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
  float wz_max_{0};
  // sum_of_robot_center_projection_on_X_Y_axis, named as the ros2_control mecanum controller
  // names it
  float center_projection_{0};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__MECANUM_CRITIC_HPP_
