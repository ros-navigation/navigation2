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

#include <cmath>
#include <cstdint>
#include <limits>
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
    * @param actuation Response of the robot to commands
    * @param clamp_raw_controls Whether to clamp the sampled controls to the feasible velocities
    */
  void setConstraints(
    const models::ControlConstraints & control_constraints, float model_dt,
    const models::ActuationModel & actuation, bool clamp_raw_controls)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
    actuation_ = actuation;
    clamp_raw_controls_ = clamp_raw_controls;

    cmd_history_vx_.resize(offsetSteps(actuation_.vx.delay), 0.0f);
    cmd_history_vy_.resize(offsetSteps(actuation_.vy.delay), 0.0f);
    cmd_history_wz_.resize(offsetSteps(actuation_.wz.delay), 0.0f);
  }

  /**
    * @brief Initialize motion model on bringup and set required variables
    * @param control_constraints Constraints on control
    * @param model_dt duration of a time step
    * @param model_delay_vx Delay (s) of the robot's response to vx commands
    * @param model_delay_vy Delay (s) of the robot's response to vy commands
    * @param model_delay_wz Delay (s) of the robot's response to wz commands
    * @param clamp_raw_controls Whether to clamp the sampled controls to the feasible velocities
    */
  void setConstraints(
    const models::ControlConstraints & control_constraints, float model_dt,
    float model_delay_vx, float model_delay_vy, float model_delay_wz, bool clamp_raw_controls)
  {
    models::ActuationModel actuation;
    actuation.vx.delay = model_delay_vx;
    actuation.vy.delay = model_delay_vy;
    actuation.wz.delay = model_delay_wz;
    setConstraints(control_constraints, model_dt, actuation, clamp_raw_controls);
  }

  /**
    * @brief Push the most recently published command to the per-axis history
    *        ring buffers, as one model_dt step.
    */
  void pushCommandHistory(float vx, float vy, float wz)
  {
    pushOne(cmd_history_vx_, vx);
    pushOne(cmd_history_vy_, vy);
    pushOne(cmd_history_wz_, wz);
  }

  /**
    * @brief Push the most recently published command to the per-axis history ring buffers.
    *        Called once per controller cycle from the optimizer. Each buffer element holds the
    *        last command published within a model_dt step, so that the history is replayed with
    *        the correct timing when the controller runs faster than model_dt.
    * @param period Time (s) since the previous command was published
    */
  void pushCommandHistory(float vx, float vy, float wz, float period)
  {
    if (!std::isfinite(history_step_elapsed_) || model_dt_ <= 0.0f) {
      history_step_elapsed_ = 0.0;
      pushCommandHistory(vx, vy, wz);
      return;
    }

    history_step_elapsed_ += period;
    const int64_t steps = std::lround(history_step_elapsed_ / model_dt_);
    if (steps == 0) {
      // Still within the current step, so this command supersedes the last
      overwriteNewest(cmd_history_vx_, vx);
      overwriteNewest(cmd_history_vy_, vy);
      overwriteNewest(cmd_history_wz_, wz);
      return;
    }

    // The last command was held through any skipped steps
    const int64_t max_steps = static_cast<int64_t>(std::max(
        {cmd_history_vx_.size(), cmd_history_vy_.size(), cmd_history_wz_.size()}));
    for (int64_t i = 1; i < std::min(steps, max_steps + 1); ++i) {
      holdNewest(cmd_history_vx_);
      holdNewest(cmd_history_vy_);
      holdNewest(cmd_history_wz_);
    }
    pushCommandHistory(vx, vy, wz);
    history_step_elapsed_ -= steps * model_dt_;
  }

  /**
    * @brief Zero the ring buffers
    */
  void clearCommandHistory()
  {
    std::fill(cmd_history_vx_.begin(), cmd_history_vx_.end(), 0.0f);
    std::fill(cmd_history_vy_.begin(), cmd_history_vy_.end(), 0.0f);
    std::fill(cmd_history_wz_.begin(), cmd_history_wz_.end(), 0.0f);
    history_step_elapsed_ = std::numeric_limits<double>::quiet_NaN();
  }

  /**
    * @brief Get the duration per axis over which commands already sent are modeled as in flight
    *        when a new command is sent, due to delay
    * @return Duration (s) per axis
    */
  models::Control getInFlightDuration() const
  {
    return {
      model_dt_ * inFlightSteps(cmd_history_vx_),
      isHolonomic() ? model_dt_ * inFlightSteps(cmd_history_vy_) : 0.0f,
      model_dt_ * inFlightSteps(cmd_history_wz_)};
  }

  /**
    * @brief Predict the velocity when the next command takes effect, by replaying the
    *        in-flight commands from the current velocity
    * @param velocity Current velocity
    * @return Velocity after the in-flight commands
    */
  models::Control propagateInFlight(const models::Control & velocity)
  {
    const unsigned int delay_vx = inFlightSteps(cmd_history_vx_);
    const unsigned int delay_vy = isHolonomic() ? inFlightSteps(cmd_history_vy_) : 0u;
    const unsigned int delay_wz = inFlightSteps(cmd_history_wz_);

    models::State state;
    state.reset(1u, std::max({delay_vx, delay_vy, delay_wz}) + 1u);
    state.vx(0, 0) = velocity.vx;
    state.vy(0, 0) = velocity.vy;
    state.wz(0, 0) = velocity.wz;
    predict(state);
    return {state.vx(0, delay_vx), state.vy(0, delay_vy), state.wz(0, delay_wz)};
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
    const auto n_rows = state.vx.rows();

    // Velocity limits by acceleration from the last step, which differ when slowing down
    auto limit_vx = [&](auto && vel, const auto & last_vel, const auto & cmd) {
        vel = cmd
          .cwiseMax((last_vel > 0).select(last_vel + min_delta_vx, last_vel - max_delta_vx))
          .cwiseMin((last_vel > 0).select(last_vel + max_delta_vx, last_vel - min_delta_vx));
      };
    auto limit_vy = [&](auto && vel, const auto & last_vel, const auto & cmd) {
        vel = cmd
          .cwiseMax((last_vel > 0).select(last_vel + min_delta_vy, last_vel - max_delta_vy))
          .cwiseMin((last_vel > 0).select(last_vel + max_delta_vy, last_vel - min_delta_vy));
      };
    auto limit_wz = [&](auto && vel, const auto & last_vel, const auto & cmd) {
        vel = cmd.cwiseMax(last_vel - max_delta_wz).cwiseMin(last_vel + max_delta_wz);
      };

    // Commands sent before this one are still in flight over the first steps due to the delay,
    // so the sampled controls take effect after them, continuing from their velocity. When the
    // first control isn't sent, being pinned to the current velocity, the first sent control
    // follows those in flight so that it takes effect after exactly the delay.
    const auto n_controls = state.cvx.cols();
    auto step = [&](unsigned int i, Eigen::ArrayXXf & vel, Eigen::ArrayXXf & controls,
      const std::vector<float> & history, auto && limit) {
        const unsigned int delay = inFlightSteps(history);
        if (i - 1 < delay) {
          limit(vel.col(i), vel.col(i - 1), Eigen::ArrayXf::Constant(n_rows, history[i]));
          return;
        }
        const unsigned int skip = (delay > 0u && !actuation_.first_control_sent) ? 1u : 0u;
        const auto c = std::min<Eigen::Index>(i - 1 - delay + skip, n_controls - 1);
        limit(vel.col(i), vel.col(i - 1), controls.col(c));
        if (clamp_raw_controls_) {
          controls.col(c) = vel.col(i);
        }
      };

    // Set dynamic limits to the platform velocities from the raw controls sampling
    for (unsigned int i = 1; i < n_cols; i++) {
      step(i, state.vx, state.cvx, cmd_history_vx_, limit_vx);
      step(i, state.wz, state.cwz, cmd_history_wz_, limit_wz);
      if (is_holo) {
        step(i, state.vy, state.cvy, cmd_history_vy_, limit_vy);
      }
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
    * @brief Number of rollout steps over which commands already sent are still in flight. A
    *        delay of a single step is the latency inherent to the rollouts, so is not replayed.
    */
  static unsigned int inFlightSteps(const std::vector<float> & history)
  {
    return history.size() > 1u ? static_cast<unsigned int>(history.size() - 1u) : 0u;
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

  /**
    * @brief Replace the newest value of a ring buffer.
    */
  static void overwriteNewest(std::vector<float> & buf, float v)
  {
    if (!buf.empty()) {
      buf.back() = v;
    }
  }

  /**
    * @brief Push a repeat of the newest value of a ring buffer.
    */
  static void holdNewest(std::vector<float> & buf)
  {
    if (!buf.empty()) {
      pushOne(buf, buf.back());
    }
  }

  float model_dt_{0.0};
  models::ActuationModel actuation_;
  bool clamp_raw_controls_{false};
  // Time (s) since the model_dt step of the newest command history element began
  double history_step_elapsed_{std::numeric_limits<double>::quiet_NaN()};

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
