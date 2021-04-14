// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER__UTILS_HPP_
#define NAV2_SMAC_PLANNER__UTILS_HPP_

#include <vector>
#include "Eigen/Core"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/utils.h"

namespace nav2_smac_planner
{

/**
* @brief Remove hooking at end of paths
* @param path Path to remove hooking from
*/
inline void removeHook(std::vector<Eigen::Vector2d> & path)
{
  // Removes the end "hooking" since goal is locked in place
  Eigen::Vector2d interpolated_second_to_last_point;
  interpolated_second_to_last_point = (path.end()[-3] + path.end()[-1]) / 2.0;
  if (
    squaredDistance(path.end()[-2], path.end()[-1]) >
    squaredDistance(interpolated_second_to_last_point, path.end()[-1]))
  {
    path.end()[-2] = interpolated_second_to_last_point;
  }
}

/**
* @brief Create an Eigen Vector2D of world poses from continuous map coords
* @param mx float of map X coordinate
* @param my float of map Y coordinate
* @param costmap Costmap pointer
* @return Eigen::Vector2d eigen vector of the generated path
*/
inline Eigen::Vector2d getWorldCoords(
  const float & mx, const float & my, const nav2_costmap_2d::Costmap2D * costmap)
{
  float world_x =
    static_cast<float>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
  float world_y =
    static_cast<float>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
  return Eigen::Vector2d(world_x, world_y);
}

/**
* @brief Create quaternion from A* coord bins
* @param theta continuous bin coordinates angle
* @return quaternion orientation in map frame
*/
inline geometry_msgs::msg::Quaternion getWorldOrientation(const float & theta, const float & bin_size)
{
  // theta is in continuous bin coordinates, must convert to world orientation
  tf2::Quaternion q;
  q.setEuler(0.0, 0.0, theta * static_cast<double>(bin_size));
  return tf2::toMsg(q);
}

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__UTILS_HPP_
