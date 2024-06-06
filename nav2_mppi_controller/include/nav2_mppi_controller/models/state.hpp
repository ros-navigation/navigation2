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
  Eigen::MatrixXf vx;
  Eigen::MatrixXf vy;
  Eigen::MatrixXf wz;

  Eigen::MatrixXf cvx;
  Eigen::MatrixXf cvy;
  Eigen::MatrixXf cwz;

  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Twist speed;

  /**
    * @brief Reset state data
    */
  void reset(unsigned int batch_size, unsigned int time_steps)
  {
    vx = Eigen::MatrixXf::Zero(batch_size, time_steps);
    vy = Eigen::MatrixXf::Zero(batch_size, time_steps);
    wz = Eigen::MatrixXf::Zero(batch_size, time_steps);

    cvx = Eigen::MatrixXf::Zero(batch_size, time_steps);
    cvy = Eigen::MatrixXf::Zero(batch_size, time_steps);
    cwz = Eigen::MatrixXf::Zero(batch_size, time_steps);
  }
};
}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__STATE_HPP_
