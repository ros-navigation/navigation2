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

#ifndef NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
#define NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_

#include <memory>
#include <vector>

// xtensor creates warnings that needs to be ignored as we are building with -Werror
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xtensor.hpp>
#pragma GCC diagnostic pop

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/motion_models.hpp"


namespace mppi
{

/**
 * @struct mppi::CriticData
 * @brief Data to pass to critics for scoring, including state, trajectories,
 * pruned path, global goal, costs, and important parameters to share
 */
struct CriticData
{
  const models::State & state;
  const models::Trajectories & trajectories;
  const models::Path & path;
  const geometry_msgs::msg::Pose & goal;

  xt::xtensor<float, 1> & costs;
  float & model_dt;

  bool fail_flag;
  nav2_core::GoalChecker * goal_checker;
  std::shared_ptr<MotionModel> motion_model;
  std::optional<std::vector<bool>> path_pts_valid;
  std::optional<size_t> furthest_reached_path_point;
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_DATA_HPP_
