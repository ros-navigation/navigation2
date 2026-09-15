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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_

#include <Eigen/Dense>

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <random>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_mppi_controller/models/optimizer_settings.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"

namespace mppi
{

/**
 * @class mppi::NoiseGenerator
 * @brief Generates noise trajectories from optimal trajectory
 */
class NoiseGenerator
{
public:
  /**
    * @brief Constructor for mppi::NoiseGenerator
    */
  NoiseGenerator() = default;

  /**
   * @brief Initialize noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   * @param name Namespace for configs
   * @param param_handler Get parameters util
   */
  void initialize(
    mppi::models::OptimizerSettings & settings,
    bool is_holonomic, const std::string & name, ParametersHandler * param_handler);

  /**
   * @brief Shutdown noise generator thread
   */
  void shutdown();

  /**
   * @brief Signal to the noise thread the controller is ready to generate a new
   * noised control for the next iteration
   */
  void generateNextNoises();

  /**
   * @brief set noised control_sequence to state controls
   * @return noises vx, vy, wz
   */
  void setNoisedControls(models::State & state, const models::ControlSequence & control_sequence);

  /**
   * Computes adaptive values of the SamplingStd parameters and updates adaptive counterparts
   * See also *_std_decay_strength, *_std_decay_to parameters for more information on how
   * vx, vy, wz => *_std_adaptive are computed.
   * @param state Current state of the robot
   */
  void computeAdaptiveStds(const models::State & state);

  /**
   * Validates the vx decay constraints and returns true if constraints are valid
   * @return true if constraints are valid
   */
  bool validateVxStdDecayConstraints() const;

  /**
   * Validates the vy decay constraints and returns true if constraints are valid
   * @return true if constraints are valid
   */
  bool validateVyStdDecayConstraints() const;

  /**
   * Validates the wz decay constraints and returns true if constraints are valid
   * @return true if constraints are valid
   */
  bool validateWzStdDecayConstraints() const;

  float getVxStdAdaptive() const;

  float getVyStdAdaptive() const;

  float getWzStdAdaptive() const;

  /**
   * @brief Reset noise generator with settings and model types
   * @param settings Settings of controller
   * @param is_holonomic If base is holonomic
   */
  void reset(mppi::models::OptimizerSettings & settings, bool is_holonomic);

protected:
  /**
   * @brief Thread to execute noise generation process
   */
  void noiseThread();

  /**
   * @brief Generate random controls by gaussian noise with mean in
   * control_sequence_
   *
   * @return tensor of shape [ batch_size_, time_steps_, 2]
   * where 2 stands for v, w
   */
  void generateNoisedControls();

  Eigen::ArrayXXf noises_vx_;
  Eigen::ArrayXXf noises_vy_;
  Eigen::ArrayXXf noises_wz_;

  // mt19937_64 should perform 3x faster than default_random_engine
  std::mt19937_64 generator_;
  std::normal_distribution<float> ndistribution_vx_;
  std::normal_distribution<float> ndistribution_wz_;
  std::normal_distribution<float> ndistribution_vy_;

  mppi::models::OptimizerSettings settings_;
  bool is_holonomic_;

  std::thread noise_thread_;
  std::condition_variable noise_cond_;
  std::mutex noise_lock_;
  bool active_{false}, ready_{false}, regenerate_noises_{false};

  /**
   * @brief Internal variables that hold the sampling deviations after decay is applied.
   * If a decay is disabled, the adaptive value equals its SamplingStd counterpart.
  */
  float vx_std_adaptive_{0.0f};
  float vy_std_adaptive_{0.0f};
  float wz_std_adaptive_{0.0f};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
