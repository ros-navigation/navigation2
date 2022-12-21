// Copyright (c) 2022 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.


#ifndef NAV2_WS_SRC_NAVIGATION2_NAV2_PLANNER_INCLUDE_NAV2_PLANNER_PATH_VALIDATOR_HPP_
#define NAV2_WS_SRC_NAVIGATION2_NAV2_PLANNER_INCLUDE_NAV2_PLANNER_PATH_VALIDATOR_HPP_

#include "nav2_msgs/msg/path_with_cost.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_planner
{
using Path = nav2_msgs::msg::PathWithCost;
using Pose = geometry_msgs::msg::PoseStamped;
using PathIndex = unsigned int;
using Point = geometry_msgs::msg::Point;
using Costs = std::vector<float>;

class [[maybe_unused]] PathValidator
{
public:
  PathValidator() = default;
  /**
   * @brief check if the path is valid
   * \param path the path to check
   * \param cost_change_z_score
   * \param consider_cost_change true, change in cost in considered
   * \return true path is valid
   */
  bool check(const Path & path, float cost_change_z_score, bool consider_cost_change);

  /**
   * @brief determine if the path is lethal
   * @param path the path to check
   * @return true, if path is lethal
   */
  bool isThisALethal(const Path & path);

  /**
   * @brief find the path index closest to the robot
   * @param path the path to evalutate
   * \param pose the current pose of the robotic platform
   * \return PathIndex, the index of the pose closest to the robot
   */
  PathIndex truncate(const Path &path, const Pose &pose);

  bool isCostChangeSignifant(const Costs &original_costs,
                             const Costs &current_costs,
                             float z_score);
};
}

#endif //NAV2_WS_SRC_NAVIGATION2_NAV2_PLANNER_INCLUDE_NAV2_PLANNER_PATH_VALIDATOR_HPP_
