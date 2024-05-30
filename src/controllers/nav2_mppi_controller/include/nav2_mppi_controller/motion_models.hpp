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

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <cstdint>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"

// xtensor creates warnings that needs to be ignored as we are building with -Werror
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xmath.hpp>
#include <xtensor/xmasked_view.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xnoalias.hpp>
#pragma GCC diagnostic pop

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State & state)
  {
    // Previously completed via tensor views, but found to be 10x slower
    // using namespace xt::placeholders;  // NOLINT
    // xt::noalias(xt::view(state.vx, xt::all(), xt::range(1, _))) =
    //   xt::noalias(xt::view(state.cvx, xt::all(), xt::range(0, -1)));
    // xt::noalias(xt::view(state.wz, xt::all(), xt::range(1, _))) =
    //   xt::noalias(xt::view(state.cwz, xt::all(), xt::range(0, -1)));
    // if (isHolonomic()) {
    //   xt::noalias(xt::view(state.vy, xt::all(), xt::range(1, _))) =
    //     xt::noalias(xt::view(state.cvy, xt::all(), xt::range(0, -1)));
    // }

    const bool is_holo = isHolonomic();
    for (unsigned int i = 0; i != state.vx.shape(0); i++) {
      for (unsigned int j = 1; j != state.vx.shape(1); j++) {
        state.vx(i, j) = state.cvx(i, j - 1);
        state.wz(i, j) = state.cwz(i, j - 1);
        if (is_holo) {
          state.vy(i, j) = state.cvy(i, j - 1);
        }
      }
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::AckermannMotionModel
    */
  explicit AckermannMotionModel(ParametersHandler * param_handler)
  {
    auto getParam = param_handler->getParamGetter("AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;
    auto & wz = control_sequence.wz;

    auto view = xt::masked_view(wz, xt::fabs(vx) / xt::fabs(wz) < min_turning_r_);
    view = xt::sign(wz) * vx / min_turning_r_;
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {return min_turning_r_;}

private:
  float min_turning_r_{0};
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
