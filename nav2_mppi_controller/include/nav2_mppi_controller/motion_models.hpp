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

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract pluginlib base class for vehicle motion models used in MPPI optimization.
 *
 * Motion models define how a robot's velocity state evolves over time and enforce
 * kinematic constraints (e.g. minimum turning radius for Ackermann drives).
 *
 * To implement a custom motion model, inherit from this class, implement all pure
 * virtual methods, and register it with pluginlib using PLUGINLIB_EXPORT_CLASS.
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
   *
   * Called once after plugin instantiation. The @p plugin_name is the namespace
   * under which this model's parameters live in the ROS parameter server
   * (e.g. "FollowPath.ackermann").  Derived classes should call this base
   * implementation and then read their own parameters via @p param_handler.
   *
   * @param param_handler Pointer to the shared parameters handler
   * @param plugin_name   Namespaced name of this plugin instance
   */
  virtual void initialize(
    ParametersHandler * param_handler,
    const std::string & plugin_name)
  {
    (void)param_handler;
    (void)plugin_name;
  }

  /**
   * @brief Set the current kinematic constraints and model time step.
   *
   * Called every time the optimizer resets (including on speed-limit changes).
   *
   * @param control_constraints Active kinematic/dynamic limits
   * @param model_dt            Duration of one prediction time step (seconds)
   */
  void setConstraints(
    const models::ControlConstraints & control_constraints,
    float model_dt)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
  }

  /**
   * @brief Propagate noised control velocities forward through the state.
   *
   * The default implementation handles acceleration clamping for vx, wz, and
   * (for holonomic models) vy.  Override to provide custom dynamics.
   *
   * @param state State containing sampled control velocities (cvx/cvy/cwz) to
   *              propagate into vehicle velocities (vx/vy/wz)
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

    for (unsigned int i = 1; i < n_cols; i++) {
      auto lower_bound_vx = (state.vx.col(i - 1) >
        0).select(
        state.vx.col(i - 1) + min_delta_vx,
        state.vx.col(i - 1) - max_delta_vx);
      auto upper_bound_vx = (state.vx.col(i - 1) >
        0).select(
        state.vx.col(i - 1) + max_delta_vx,
        state.vx.col(i - 1) - min_delta_vx);

      state.cvx.col(i - 1) = state.cvx.col(i - 1)
        .cwiseMax(lower_bound_vx)
        .cwiseMin(upper_bound_vx);
      state.vx.col(i) = state.cvx.col(i - 1);

      state.cwz.col(i - 1) = state.cwz.col(i - 1)
        .cwiseMax(state.wz.col(i - 1) - max_delta_wz)
        .cwiseMin(state.wz.col(i - 1) + max_delta_wz);
      state.wz.col(i) = state.cwz.col(i - 1);

      if (is_holo) {
        auto lower_bound_vy = (state.vy.col(i - 1) >
          0).select(
          state.vy.col(i - 1) + min_delta_vy,
          state.vy.col(i - 1) - max_delta_vy);
        auto upper_bound_vy = (state.vy.col(i - 1) >
          0).select(
          state.vy.col(i - 1) + max_delta_vy,
          state.vy.col(i - 1) - min_delta_vy);
        state.cvy.col(i - 1) = state.cvy.col(i - 1)
          .cwiseMax(lower_bound_vy)
          .cwiseMin(upper_bound_vy);
        state.vy.col(i) = state.cvy.col(i - 1);
      }
    }
  }

  /**
   * @brief Whether the motion model uses the lateral (Y) velocity axis.
   * @return true if holonomic (e.g. omnidirectional), false otherwise
   */
  virtual bool isHolonomic() const = 0;

  /**
   * @brief Whether this model imposes a minimum turning radius constraint.
   *
   * Override and return @c true in models that restrict the ratio |vx|/|wz|
   * (e.g. Ackermann steering).  Used by ConstraintCritic to apply the
   * appropriate cost term without requiring a dynamic_cast.
   *
   * @return true if the model has a minimum turning radius constraint
   */
  virtual bool hasConstrainedTurningRadius() const {return false;}

  /**
   * @brief Get the minimum turning radius for models that constrain it.
   *
   * Only meaningful when hasConstrainedTurningRadius() returns @c true.
   *
   * @return Minimum turning radius in metres (default 0)
   */
  virtual float getMinTurningRadius() const {return 0.0f;}

  /**
   * @brief Apply hard kinematic constraints to the optimal control sequence.
   *
   * Called once per optimisation cycle on the winning control sequence.
   * The default implementation is a no-op.
   *
   * @param control_sequence Control sequence to clamp / modify in place
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}

protected:
  float model_dt_{0.0f};
  models::ControlConstraints control_constraints_{
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann steering motion model plugin.
 *
 * Reads the following parameter from its plugin namespace:
 *   - @c min_turning_r  (float, default 0.2 m) — minimum turning radius
 *
 * Example YAML configuration:
 * @code{yaml}
 *   motion_model: "ackermann"
 *   ackermann:
 *     plugin: "mppi::AckermannMotionModel"
 *     min_turning_r: 0.5
 * @endcode
 */
class AckermannMotionModel : public MotionModel
{
public:
  AckermannMotionModel() = default;

  /**
   * @brief Initialize the Ackermann model, reading min_turning_r from parameters.
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
   * @brief Whether the motion model is holonomic.
   * @return false — Ackermann is non-holonomic
   */
  bool isHolonomic() const override {return false;}

  /**
   * @brief Whether this model imposes a minimum turning radius constraint.
   * @return true
   */
  bool hasConstrainedTurningRadius() const override {return true;}

  
  /**
   * @brief Apply the minimum turning radius constraint to the control sequence.
   *
   * Clamps |wz| so that |vx| / |wz| >= min_turning_r_ for every time step.
   *
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
   * @brief Get the minimum turning radius.
   * @return Minimum turning radius in metres
   */
  float getMinTurningRadius() const override {return min_turning_r_;}

private:
  float min_turning_r_{0.0f};
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model plugin.
 *
 * Non-holonomic model with no additional kinematic constraints beyond the
 * standard velocity and acceleration limits.
 *
 * Example YAML configuration:
 * @code{yaml}
 *   motion_model: "diff_drive"
 *   diff_drive:
 *     plugin: "mppi::DiffDriveMotionModel"
 * @endcode
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic.
   * @return false — differential drive is non-holonomic
   */
  bool isHolonomic() const override {return false;}
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional (holonomic) motion model plugin.
 *
 * Supports independent control of vx, vy, and wz.
 *
 * Example YAML configuration:
 * @code{yaml}
 *   motion_model: "omni"
 *   omni:
 *     plugin: "mppi::OmniMotionModel"
 * @endcode
 */
class OmniMotionModel : public MotionModel
{
public:
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic.
   * @return true — omnidirectional drive is holonomic
   */
  bool isHolonomic() const override {return true;}
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
