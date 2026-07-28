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

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <Eigen/Dense>

#include <cstdint>
#include <string>
#include <algorithm>
#include <vector>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract pluginlib class for modeling a vehicle
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
   * @brief Initialize motion model on bringup.
   * @param param_handler Pointer to the shared parameters handler
   * @param plugin_name   Namespaced name of this plugin instance
   */
  virtual void initialize(
    ParametersHandler * /*param_handler*/,
    const std::string & /*plugin_name*/)
  {}

  /**
    * @brief Initialize motion model on bringup and set required variables
    * @param control_constraints Constraints on control
    * @param model_dt duration of a time step
    */
  void setConstraints(
    const models::ControlConstraints & control_constraints, float model_dt,
    float model_delay_vx, float model_delay_vy, float model_delay_wz, bool clamp_raw_controls)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
    model_delay_vx_ = model_delay_vx;
    model_delay_vy_ = model_delay_vy;
    model_delay_wz_ = model_delay_wz;
    clamp_raw_controls_ = clamp_raw_controls;

    cmd_history_vx_.resize(offsetSteps(model_delay_vx_), 0.0f);
    cmd_history_vy_.resize(offsetSteps(model_delay_vy_), 0.0f);
    cmd_history_wz_.resize(offsetSteps(model_delay_wz_), 0.0f);
  }

  /**
    * @brief Push the most recently published command to the per-axis history
    *        ring buffers. Called once per controller cycle from the optimizer.
    */
  void pushCommandHistory(float vx, float vy, float wz)
  {
    pushOne(cmd_history_vx_, vx);
    pushOne(cmd_history_vy_, vy);
    pushOne(cmd_history_wz_, wz);
  }

  /**
    * @brief Zero the ring buffers
    */
  void clearCommandHistory()
  {
    std::fill(cmd_history_vx_.begin(), cmd_history_vx_.end(), 0.0f);
    std::fill(cmd_history_vy_.begin(), cmd_history_vy_.end(), 0.0f);
    std::fill(cmd_history_wz_.begin(), cmd_history_wz_.end(), 0.0f);
  }

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State & state)
  {
    const bool is_holo = isHolonomic();
    float max_delta_vx = model_dt_ * control_constraints_.ax_max;
    float min_delta_vx = model_dt_ * control_constraints_.ax_min;
    float max_delta_vy = model_dt_ * control_constraints_.ay_max;
    float min_delta_vy = model_dt_ * control_constraints_.ay_min;
    float max_delta_wz = model_dt_ * control_constraints_.az_max;
    unsigned int n_cols = state.vx.cols();

    // Set dynamic limits to the platform velocities from the raw controls sampling
    for (unsigned int i = 1; i < n_cols; i++) {
      auto lower_bound_vx = (state.vx.col(i - 1) >
        0).select(
        state.vx.col(i - 1) + min_delta_vx,
        state.vx.col(i - 1) - max_delta_vx);
      auto upper_bound_vx = (state.vx.col(i - 1) >
        0).select(
        state.vx.col(i - 1) + max_delta_vx,
        state.vx.col(i - 1) - min_delta_vx);
      state.vx.col(i) = state.cvx.col(i - 1)
        .cwiseMax(lower_bound_vx)
        .cwiseMin(upper_bound_vx);
      if (clamp_raw_controls_) {
        state.cvx.col(i - 1) = state.vx.col(i);
      }

      state.wz.col(i) = state.cwz.col(i - 1)
        .cwiseMax(state.wz.col(i - 1) - max_delta_wz)
        .cwiseMin(state.wz.col(i - 1) + max_delta_wz);
      if (clamp_raw_controls_) {
        state.cwz.col(i - 1) = state.wz.col(i);
      }

      if (is_holo) {
        auto lower_bound_vy = (state.vy.col(i - 1) >
          0).select(
          state.vy.col(i - 1) + min_delta_vy,
          state.vy.col(i - 1) - max_delta_vy);
        auto upper_bound_vy = (state.vy.col(i - 1) >
          0).select(
          state.vy.col(i - 1) + max_delta_vy,
          state.vy.col(i - 1) - min_delta_vy);
        state.vy.col(i) = state.cvy.col(i - 1)
          .cwiseMax(lower_bound_vy)
          .cwiseMin(upper_bound_vy);
        if (clamp_raw_controls_) {
          state.cvy.col(i - 1) = state.vy.col(i);
        }
      }
    }

    const unsigned int offset_vx = std::floor((model_delay_vx_ / model_dt_) + 0.5);
    const unsigned int offset_vy = std::floor((model_delay_vy_ / model_dt_) + 0.5);
    const unsigned int offset_wz = std::floor((model_delay_wz_ / model_dt_) + 0.5);

    if (offset_vx > 0u || offset_wz > 0u || (is_holo && offset_vy > 0u)) {
      applyDelayShift(state, is_holo, offset_vx, offset_vy, offset_wz);
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() const = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}

protected:
  /**
    * @brief Apply the per-axis input-delay shift to velocity rollout.
    *
    * For j in [1, offset) — the delay window — fill `dst` with history[j]
    */
  void applyDelayShift(
    models::State & state, bool is_holo,
    unsigned int offset_vx, unsigned int offset_vy, unsigned int offset_wz) const
  {
    auto shift = [](Eigen::ArrayXXf & velocities, unsigned int offset,
      const std::vector<float> & history) {
        const unsigned int cols = static_cast<unsigned int>(velocities.cols());
        if (offset == 0u || cols == 0u) {
          return;
        }

        // Shift cols in-place right-to-left by offset
        for (unsigned int k = (offset < cols) ? cols - offset : 0u; k > 0; --k) {
          velocities.col(offset + k - 1) = velocities.col(k);
        }

        // Fill delay window cols with the in-flight commands from history.
        const unsigned int end = std::min(offset, cols);
        for (unsigned int j = 1; j < end; ++j) {
          velocities.col(j).setConstant(history[j]);
        }
      };

    shift(state.vx, offset_vx, cmd_history_vx_);
    shift(state.wz, offset_wz, cmd_history_wz_);

    if (is_holo) {
      shift(state.vy, offset_vy, cmd_history_vy_);
    }
  }

  /**
    * @brief Convert a delay in seconds to an offset in number of rollout steps, rounding to the nearest step.
    */
  std::size_t offsetSteps(float delay) const
  {
    if (delay <= 0.0f || model_dt_ <= 0.0f) {
      return 0u;
    }
    return static_cast<std::size_t>(std::floor(delay / model_dt_ + 0.5f));
  }

  /**
    * @brief Push a value to the back of a ring buffer and rotate the elements.
    */
  static void pushOne(std::vector<float> & buf, float v)
  {
    if (buf.empty()) {return;}
    std::rotate(buf.begin(), buf.begin() + 1, buf.end());
    buf.back() = v;
  }

  float model_dt_{0.0};
  float model_delay_vx_{0.0};
  float model_delay_vy_{0.0};
  float model_delay_wz_{0.0};
  bool clamp_raw_controls_{false};

  // Per-axis ring buffer of recently published commands
  std::vector<float> cmd_history_vx_;
  std::vector<float> cmd_history_vy_;
  std::vector<float> cmd_history_wz_;

  models::ControlConstraints control_constraints_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f};
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
  AckermannMotionModel() = default;

  /**
   * @brief Initialize motion model.
   * @param param_handler Pointer to the shared parameters handler
   * @param plugin_name   Namespaced name of this plugin instance
   */
  void initialize(
    ParametersHandler * param_handler,
    const std::string & plugin_name) override
  {
    auto getParam = param_handler->getParamGetter(plugin_name);
    getParam(min_turning_r_, "min_turning_r", 0.2f);
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() const override
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    const auto wz_constrained = control_sequence.vx.abs() / min_turning_r_;
    control_sequence.wz = control_sequence.wz
      .max((-wz_constrained))
      .min(wz_constrained);
  }
  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() const {return min_turning_r_;}

private:
  float min_turning_r_{0.0f};
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
  bool isHolonomic() const override
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
  bool isHolonomic() const override
  {
    return true;
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
