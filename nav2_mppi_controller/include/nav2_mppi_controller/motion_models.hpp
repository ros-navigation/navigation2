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
   * @brief Configure the motion model on bringup: the parameters shared by all motion models,
   *        including the robot's response delay, then the plugin's own via initialize()
   * @param param_handler Pointer to the shared parameters handler
   * @param parent_name   Namespace of the controller
   * @param plugin_name   Namespaced name of this plugin instance
   */
  void on_configure(
    ParametersHandler * param_handler, const std::string & parent_name,
    const std::string & plugin_name)
  {
    auto getParentParam = param_handler->getParamGetter(parent_name);
    getParentParam(delay_vx_, "model_delay_vx", 0.0f);
    getParentParam(delay_vy_, "model_delay_vy", 0.0f);
    getParentParam(delay_wz_, "model_delay_wz", 0.0f);
    initialize(param_handler, plugin_name);
  }

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
    * @brief Set the constraints and time step the motion model predicts with
    * @param control_constraints Constraints on control
    * @param model_dt duration of a time step
    * @param clamp_raw_controls Whether to clamp the raw controls to the predicted velocities
    */
  void setConstraints(
    const models::ControlConstraints & control_constraints, float model_dt,
    bool clamp_raw_controls)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
    clamp_raw_controls_ = clamp_raw_controls;

    // Keep the commands sent when the history size is unchanged, e.g. across parameter updates,
    // as resizing the ring buffer in place would misorder it
    resizeHistory(history_vx_, offsetSteps(delay_vx_));
    resizeHistory(history_vy_, offsetSteps(delay_vy_));
    resizeHistory(history_wz_, offsetSteps(delay_wz_));
  }

  /**
    * @brief Inform the model of the command sent to the robot, once per control cycle
    * @param cmd Command sent
    */
  void onCommandSent(const geometry_msgs::msg::Twist & cmd)
  {
    pushOne(history_vx_, static_cast<float>(cmd.linear.x));
    pushOne(history_vy_, static_cast<float>(cmd.linear.y));
    pushOne(history_wz_, static_cast<float>(cmd.angular.z));
  }

  /**
    * @brief Forget the commands sent, when the robot's motion is no longer a continuation of them
    */
  void reset()
  {
    std::fill(history_vx_.begin(), history_vx_.end(), 0.0f);
    std::fill(history_vy_.begin(), history_vy_.end(), 0.0f);
    std::fill(history_wz_.begin(), history_wz_.end(), 0.0f);
  }

  /**
    * @brief Get the velocity the next control continues from, which the control sequence must be
    *        feasible from. Commands still in flight take effect first, so on those axes it is the
    *        last command sent rather than the robot's current velocity.
    * @param speed Current robot velocity
    * @return Velocity to continue from
    */
  geometry_msgs::msg::Twist getControlStart(const geometry_msgs::msg::Twist & speed) const
  {
    geometry_msgs::msg::Twist start = speed;
    if (history_vx_.size() > 1u) {
      start.linear.x = history_vx_.back();
    }
    if (isHolonomic() && history_vy_.size() > 1u) {
      start.linear.y = history_vy_.back();
    }
    if (history_wz_.size() > 1u) {
      start.angular.z = history_wz_.back();
    }
    return start;
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
      clampByAccel(
        state.vx.col(i - 1), state.cvx.col(i - 1), min_delta_vx, max_delta_vx, state.vx.col(i));
      if (clamp_raw_controls_) {
        state.cvx.col(i - 1) = state.vx.col(i);
      }

      clampByAccel(
        state.wz.col(i - 1), state.cwz.col(i - 1), -max_delta_wz, max_delta_wz, state.wz.col(i));
      if (clamp_raw_controls_) {
        state.cwz.col(i - 1) = state.wz.col(i);
      }

      if (is_holo) {
        clampByAccel(
          state.vy.col(i - 1), state.cvy.col(i - 1), min_delta_vy, max_delta_vy, state.vy.col(i));
        if (clamp_raw_controls_) {
          state.cvy.col(i - 1) = state.vy.col(i);
        }
      }
    }

    // The commands still in flight play out before the sampled controls take effect
    applyDelayShift(state.vx, history_vx_);
    applyDelayShift(state.wz, history_wz_);
    if (is_holo) {
      applyDelayShift(state.vy, history_vy_);
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
   * @brief Clamp controls to the velocities reachable in a step from the previous velocities:
   *        speeding up (away from zero) by up to max_delta and slowing down by up to -min_delta.
   *        Slowing down through zero speeds up in the other direction for the rest of the step,
   *        so from rest both directions are limited by max_delta.
   */
  static void clampByAccel(
    const Eigen::Ref<const Eigen::ArrayXf> & prev, const Eigen::Ref<const Eigen::ArrayXf> & control,
    const float min_delta, const float max_delta, Eigen::Ref<Eigen::ArrayXf> out)
  {
    const float up = max_delta, down = -min_delta;
    if (down == up) {
      // Symmetric limits, through zero or not
      out = control.cwiseMax(prev - up).cwiseMin(prev + up);
    } else if (down <= 0.0f) {
      // Unable to slow down
      out = control
        .cwiseMax((prev >= 0.0f).select(prev - down, prev - up))
        .cwiseMin((prev < 0.0f).select(prev + down, prev + up));
    } else {
      // Of the positive and negative parts of the velocity, only one is non-zero: slowing down
      // to zero takes |v| / down of the step, then speeding up at up for the rest of it
      const float slope = up / down;
      const auto pos = prev.max(0.0f);
      const auto neg = prev.min(0.0f);
      if (down > up) {
        out = control
          .cwiseMax(neg + (pos - down).max(slope * pos - up))
          .cwiseMin(pos + (neg + down).min(slope * neg + up));
      } else {
        out = control
          .cwiseMax(neg + (pos - down).min(slope * pos - up))
          .cwiseMin(pos + (neg + down).max(slope * neg + up));
      }
    }
  }

  /**
    * @brief Apply an axis' input delay to its velocity rollout: the delay, in steps, is the size of
    *        its history of commands sent within it. For j in [1, offset), the delay window, the
    *        velocities are the commands in flight, history[j]. A delay of a single model_dt is the
    *        latency inherent to the trajectories, so leaves them unchanged.
    * @param velocities Velocities of the axis
    * @param history Commands sent within the axis' delay, oldest first
    */
  static void applyDelayShift(Eigen::ArrayXXf & velocities, const std::vector<float> & history)
  {
    const unsigned int offset = static_cast<unsigned int>(history.size());
    const unsigned int cols = static_cast<unsigned int>(velocities.cols());
    if (offset <= 1u || cols == 0u) {
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
    * @brief Resize a ring buffer, zeroing it only when its size changes
    */
  static void resizeHistory(std::vector<float> & buf, std::size_t size)
  {
    if (buf.size() != size) {
      buf.assign(size, 0.0f);
    }
  }

  float model_dt_{0.0};
  bool clamp_raw_controls_{false};

  // The robot's response delay per axis and the commands sent within it, still in flight
  float delay_vx_{0.0f};
  float delay_vy_{0.0f};
  float delay_wz_{0.0f};
  std::vector<float> history_vx_;
  std::vector<float> history_vy_;
  std::vector<float> history_wz_;

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
