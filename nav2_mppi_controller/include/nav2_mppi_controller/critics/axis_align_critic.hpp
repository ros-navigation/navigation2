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
 *
 * The normalized score is not an arbitrary heuristic: for a mecanum base it is exactly the wheel
 * speed imbalance whenever wz is zero. With 45 degree rollers, wheel radius r, half-wheelbase lx
 * and half-track ly, the inverse kinematics of the four wheels are
 *
 *     r * w_fl = vx - vy - (lx + ly) * wz       r * w_fr = vx + vy + (lx + ly) * wz
 *     r * w_rl = vx + vy - (lx + ly) * wz       r * w_rr = vx - vy + (lx + ly) * wz
 *
 * At wz = 0 the four wheel speeds collapse onto two magnitudes, |vx + vy| / r and |vx - vy| / r,
 * whose larger and smaller values are (|vx| + |vy|) / r and abs(|vx| - |vy|) / r. Their
 * normalized imbalance is therefore
 *
 *     (max - min) / (max + min) = 2 * min(|vx|, |vy|) / (2 * max(|vx|, |vy|))
 *                               = min(|vx|, |vy|) / max(|vx|, |vy|)
 *
 * which is the ratio scored below. The geometry cancels, so the score depends on neither r nor
 * lx, ly: it is 0 when all four wheels turn at the same speed and 1 at 45 degrees, where two of
 * them are commanded to a standstill and the entire traction demand falls on the other two. That
 * loss of traction margin, rather than kinematic infeasibility, is what the critic prices in.
 * A non-zero wz breaks the pairing above, so the identity is stated for pure translation; the
 * critic scores the translational part only and leaves rotation to the other critics. The
 * identity is pinned by the AxisAlignCriticWheelSpeedImbalance test in test/critics_tests.cpp.
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
