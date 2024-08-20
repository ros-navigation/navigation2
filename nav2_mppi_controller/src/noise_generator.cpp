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

#include "nav2_mppi_controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>

namespace mppi
{

std::normal_distribution<float> NoiseGenerator::ndistribution_vx_;
std::normal_distribution<float> NoiseGenerator::ndistribution_vy_;
std::normal_distribution<float> NoiseGenerator::ndistribution_wz_;
std::default_random_engine NoiseGenerator::generator_;

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

  ndistribution_vx_ = std::normal_distribution(0.0f, settings_.sampling_std.vx);
  ndistribution_vy_ = std::normal_distribution(0.0f, settings_.sampling_std.vy);
  ndistribution_wz_ = std::normal_distribution(0.0f, settings_.sampling_std.wz);

  auto getParam = param_handler->getParamGetter(name);
  getParam(regenerate_noises_, "regenerate_noises", false);

  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::shutdown()
{
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  state.cvx = noises_vx_.rowwise() + control_sequence.vx.transpose();
  state.cvy = noises_vy_.rowwise() + control_sequence.vy.transpose();
  state.cwz = noises_wz_.rowwise() + control_sequence.wz.transpose();
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noises_vx_ = Eigen::ArrayXXf::Zero(settings_.batch_size, settings_.time_steps);
    noises_vy_ = Eigen::ArrayXXf::Zero(settings_.batch_size, settings_.time_steps);
    noises_wz_ = Eigen::ArrayXXf::Zero(settings_.batch_size, settings_.time_steps);
    ready_ = true;
  }

  if (regenerate_noises_) {
    noise_cond_.notify_all();
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::noiseThread()
{
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() {return ready_;});
    ready_ = false;
    generateNoisedControls();
  } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;
  noises_vx_ = Eigen::ArrayXXf::NullaryExpr(s.batch_size, s.time_steps, [&] () {return ndistribution_vx_(generator_);});
  noises_wz_ = Eigen::ArrayXXf::NullaryExpr(s.batch_size, s.time_steps, [&] () {return ndistribution_wz_(generator_);});
  if(is_holonomic_) {
    noises_vy_ = Eigen::ArrayXXf::NullaryExpr(s.batch_size, s.time_steps, [&] () {return ndistribution_vy_(generator_);});
  }
}

}  // namespace mppi
