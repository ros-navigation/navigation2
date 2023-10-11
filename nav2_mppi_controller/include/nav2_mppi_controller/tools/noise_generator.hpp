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

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

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

  xt::xtensor<float, 2> noises_vx_;
  xt::xtensor<float, 2> noises_vy_;
  xt::xtensor<float, 2> noises_wz_;

  mppi::models::OptimizerSettings settings_;
  bool is_holonomic_;

  std::thread noise_thread_;
  std::condition_variable noise_cond_;
  std::mutex noise_lock_;
  bool active_{false}, ready_{false}, regenerate_noises_{false};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
