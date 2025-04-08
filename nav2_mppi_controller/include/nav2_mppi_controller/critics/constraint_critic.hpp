// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for enforcing feasible constraints
 */
class ConstraintCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;

  /**
   * @brief Apply acceleration constraints to the cost
   *
   * @param min_accel Minimum acceleration, should be negative
   * @param max_accel Maximum acceleration, must be positive
   * @param init_value Initial value of the state
   * @param velocities Velocities to apply constraints to
   * @param model_dt Model time step
   * @param costs [out] add reference cost values to this tensor
   */
  void applyAccelerationConstraints(
    const float min_accel,
    const float max_accel,
    const float init_value,
    const Eigen::ArrayXXf & velocities,
    const float model_dt,
    Eigen::ArrayXf & costs);

  float getMaxVelConstraint() {return max_vel_;}
  float getMinVelConstraint() {return min_vel_;}
  float getAxMinConstraint() {return ax_min_;}
  float getAxMaxConstraint() {return ax_max_;}

protected:
  unsigned int power_{0};
  float weight_{0};
  float min_vel_;
  float max_vel_;
  float ax_min_;
  float ax_max_;
  float ay_min_;
  float ay_max_;
  bool use_acc_{false};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
