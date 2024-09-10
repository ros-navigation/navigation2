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
#include <string>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"

#include <Eigen/Dense>

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
    * @brief Initialize motion model on bringup and set required variables
    * @param control_constraints Constraints on control
    * @param model_dt duration of a time step
    */
  void initialize(const models::ControlConstraints & control_constraints, float model_dt)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
  }

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State & state)
  {
    const bool is_holo = isHolonomic();
    float && max_delta_vx = model_dt_ * control_constraints_.ax_max;
    float && min_delta_vx = model_dt_ * control_constraints_.ax_min;
    float && max_delta_vy = model_dt_ * control_constraints_.ay_max;
    float && max_delta_wz = model_dt_ * control_constraints_.az_max;

    unsigned int n_rows = state.vx.rows();
    unsigned int n_cols = state.vx.cols();

    // Default layout in eigen is column-major, hence accessing elements in column-major fashion to utilize L1 cache as much as possible
    for (unsigned int i = 1; i != n_cols; i++) {
      for (unsigned int j = 0; j != n_rows; j++) {
        float & vx_last = state.vx(j, i - 1);
        float & cvx_curr = state.cvx(j, i - 1);
        cvx_curr = std::min(vx_last + max_delta_vx, std::max(cvx_curr, vx_last + min_delta_vx));
        state.vx(j, i) = cvx_curr;
        
        float & wz_last = state.wz(j, i - 1);
        float & cwz_curr = state.cwz(j, i - 1);
        cwz_curr = std::min(wz_last + max_delta_wz, std::max(cwz_curr, wz_last - max_delta_wz));
        state.wz(j, i) = cwz_curr;

        if (is_holo) {
          float & vy_last = state.vy(j, i - 1);
          float & cvy_curr = state.cvy(j, i - 1);
          cvy_curr = std::min(vy_last + max_delta_vy, std::max(cvy_curr, vy_last - max_delta_vy));
          state.vy(j, i) = cvy_curr;       
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

protected:
  float model_dt_{0.0};
  models::ControlConstraints control_constraints_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f};
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
  explicit AckermannMotionModel(ParametersHandler * param_handler, const std::string & name)
  {
    auto getParam = param_handler->getParamGetter(name + ".AckermannConstraints");
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
    const auto vx_ptr = control_sequence.vx.data();
    auto wz_ptr = control_sequence.wz.data();
    int steps = control_sequence.vx.size();
    for(int i = 0; i < steps; i++)
    {
      float && wz_constrained = fabs(*(vx_ptr + i) / min_turning_r_);
      float & wz_curr = *(wz_ptr + i);
      wz_curr = std::min(wz_constrained, std::max(wz_curr, -1 * wz_constrained));
    }
    // Taking more time compared to for loop
    //wz = ((vx.abs() / wz.abs() < min_turning_r_).select(wz, wz.sign() * vx / min_turning_r_)).eval();

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
