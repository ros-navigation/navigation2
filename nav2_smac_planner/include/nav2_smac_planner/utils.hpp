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
#include <memory>

#include "Eigen/Core"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"

namespace nav2_smac_planner
{

/**
* @brief Create an Eigen Vector2D of world poses from continuous map coords
* @param mx float of map X coordinate
* @param my float of map Y coordinate
* @param costmap Costmap pointer
* @return Eigen::Vector2d eigen vector of the generated path
*/
inline geometry_msgs::msg::Pose getWorldCoords(
  const float & mx, const float & my, const nav2_costmap_2d::Costmap2D * costmap)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x =
    static_cast<float>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
  msg.position.y =
    static_cast<float>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
  return msg;
}

/**
* @brief Create quaternion from A* coord bins
* @param theta continuous bin coordinates angle
* @return quaternion orientation in map frame
*/
inline geometry_msgs::msg::Quaternion getWorldOrientation(
  const float & theta,
  const float & bin_size)
{
  // theta is in continuous bin coordinates, must convert to world orientation
  tf2::Quaternion q;
  q.setEuler(0.0, 0.0, theta * static_cast<double>(bin_size));
  return tf2::toMsg(q);
}

/**
* @brief Find the min cost of the inflation decay function for which the robot MAY be
* in collision in any orientation
* @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
* @return double circumscribed cost, any higher than this and need to do full footprint collision checking
* since some element of the robot could be in collision
*/
inline double findCircumscribedCost(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  double result = -1.0;
  bool inflation_layer_found = false;
  std::vector<std::shared_ptr<nav2_costmap_2d::Layer>>::iterator layer;

  // check if the costmap has an inflation layer
  for (layer = costmap->getLayeredCostmap()->getPlugins()->begin();
    layer != costmap->getLayeredCostmap()->getPlugins()->end();
    ++layer)
  {
    std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation_layer =
      std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
    if (!inflation_layer) {
      continue;
    }

    inflation_layer_found = true;
    double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
    double resolution = costmap->getCostmap()->getResolution();
    result = static_cast<double>(inflation_layer->computeCost(circum_radius / resolution));
  }

  if (!inflation_layer_found) {
    RCLCPP_WARN(
      rclcpp::get_logger("computeCircumscribedCost"),
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times!");
  }

  return result;
}

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__UTILS_HPP_
