// Copyright (c) 2022 Samsung Research America
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

#ifndef NAV2_CONTROLLER__PATH_HANDLER_HPP_
#define NAV2_CONTROLLER__PATH_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace nav2_controller
{

/**
 * @class nav2_controller::PathHandler
 * @brief Handles input paths to transform them to local frames required
 */
class PathHandler
{
public:
  /**
   * @brief Constructor for nav2_controller::PathHandler
   */
  PathHandler(
    double transform_tolerance,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Destrructor for nav2_controller::PathHandler
   */
  ~PathHandler() = default;

  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @param max_robot_pose_search_dist Distance to search for matching nearest path point
   * @param reject_unit_path If true, fail if path has only one pose
   * @return Path in new frame
   */
  nav_msgs::msg::Path pruneGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    double max_robot_pose_search_dist, bool reject_unit_path = false);

  void setPlan(const nav_msgs::msg::Path & path) {global_plan_ = path;}

  nav_msgs::msg::Path getPlan() {return global_plan_;}

protected:
  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PATH_HANDLER_HPP_
