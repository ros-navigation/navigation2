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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace mppi::models
{

/**
 * @struct mppi::models::State
 * @brief State information: velocities, controls, poses, speed
 */
struct State
{
  Eigen::ArrayXXf vx;
  Eigen::ArrayXXf vy;
  Eigen::ArrayXXf wz;

  Eigen::ArrayXXf cvx;
  Eigen::ArrayXXf cvy;
  Eigen::ArrayXXf cwz;

  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;

  /**
   * @brief Internal variable that holds wz_std after decay is applied.
   * If decay is disabled, SamplingStd.wz == wz_std_adaptive
   */
  float wz_std_adaptive;

  /**
    * @brief Reset state data
    */
  void reset(unsigned int batch_size, unsigned int time_steps, float wz_std)
  {
    vx.setZero(batch_size, time_steps);
    vy.setZero(batch_size, time_steps);
    wz.setZero(batch_size, time_steps);

    cvx.setZero(batch_size, time_steps);
    cvy.setZero(batch_size, time_steps);
    cwz.setZero(batch_size, time_steps);

    wz_std_adaptive = wz_std;  // reset initial adaptive value to parameterized value
  }
};
}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_
