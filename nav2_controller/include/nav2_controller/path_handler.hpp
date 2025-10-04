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
#include "nav2_controller/parameter_handler.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_controller
{
using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;

/**
 * @class nav2_controller::PathHandler
 * @brief Handles input paths to prune them within the local costmap
 */
class PathHandler
{
public:
  /**
   * @brief Constructor for nav2_controller::PathHandler
   */
  PathHandler(
    Parameters * params,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Destructor for nav2_controller::PathHandler
   */
  ~PathHandler() = default;

  /**
   * @brief Prunes global plan, bounded around the robot's position and within the local costmap
   * @param pose pose to transform
   * @return Path after pruned
   */
  nav_msgs::msg::Path pruneGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
    * @brief Set new reference plan
    * @param Path Path to use
    */
  void setPlan(const nav_msgs::msg::Path & path);

  /**
    * @brief Get reference plan
    * @return Path
    */
  nav_msgs::msg::Path getPlan() {return global_plan_;}

  /**
    * @brief Check if the robot pose is within the set inversion tolerances
    * @param robot_pose Robot's current pose to check
    * @return bool If the robot pose is within the set inversion tolerances
    */
  bool isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose);

  /**
    * @brief Prune a path to only interesting portions
    * @param plan Plan to prune
    * @param end Final path iterator
    */
  void prunePlan(nav_msgs::msg::Path & plan, const PathIterator end);

  /**
   * @brief Get the global goal pose transformed to the local frame
   * @param stamp Time to get the goal pose at
   * @return Transformed goal pose
   */
  geometry_msgs::msg::PoseStamped getTransformedGoal(const builtin_interfaces::msg::Time & stamp);

protected:
  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path global_plan_up_to_inversion_;
  unsigned int inversion_locale_{0u};
  Parameters * params_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PATH_HANDLER_HPP_
