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
 * @brief Handles input paths to transform them to local frames required
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
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
   * Points ineligible to be selected as a lookahead point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @param max_robot_pose_search_dist Distance to search for matching nearest path point
   * @param reject_unit_path If true, fail if path has only one pose
   * @return Path in new frame
   */
  nav_msgs::msg::Path pruneGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose);

  void setPlan(const nav_msgs::msg::Path & path);

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

protected:
  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

  /**
   * @brief Find the iterator of the first pose at which there is an inversion on the path,
   * @param path to check for inversion
   * @return the first point after the inversion found in the path
   */
  inline unsigned int findFirstPathInversion(nav_msgs::msg::Path & path)
  {
    // At least 3 poses for a possible inversion
    if (path.poses.size() < 3) {
      return path.poses.size();
    }

    // Iterating through the path to determine the position of the path inversion
    for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
      // We have two vectors for the dot product OA and AB. Determining the vectors.
      float oa_x = path.poses[idx].pose.position.x -
        path.poses[idx - 1].pose.position.x;
      float oa_y = path.poses[idx].pose.position.y -
        path.poses[idx - 1].pose.position.y;
      float ab_x = path.poses[idx + 1].pose.position.x -
        path.poses[idx].pose.position.x;
      float ab_y = path.poses[idx + 1].pose.position.y -
        path.poses[idx].pose.position.y;

      // Checking for the existence of cusp, in the path, using the dot product.
      float dot_product = (oa_x * ab_x) + (oa_y * ab_y);
      if (dot_product < 0.0f) {
        return idx + 1;
      }

      if (
        (hypot(oa_x, oa_y) == 0.0 &&
        path.poses[idx - 1].pose.orientation !=
        path.poses[idx].pose.orientation)
        ||
        (hypot(ab_x, ab_y) == 0.0 &&
        path.poses[idx].pose.orientation !=
        path.poses[idx + 1].pose.orientation))
      {
        // returning the distance since the points overlap
        // but are not simply duplicate points (e.g. in place rotation)
        return idx + 1;
      }
    }

    return path.poses.size();
  }

  /**
   * @brief Find and remove poses after the first inversion in the path
   * @param path to check for inversion
   * @return The location of the inversion, return 0 if none exist
   */
  inline unsigned int removePosesAfterFirstInversion(nav_msgs::msg::Path & path)
  {
    nav_msgs::msg::Path cropped_path = path;
    const unsigned int first_after_inversion = findFirstPathInversion(cropped_path);
    if (first_after_inversion == path.poses.size()) {
      return 0u;
    }

    cropped_path.poses.erase(
      cropped_path.poses.begin() + first_after_inversion, cropped_path.poses.end());
    path = cropped_path;
    return first_after_inversion;
  }

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path global_plan_up_to_inversion_;
  unsigned int inversion_locale_{0u};
  Parameters * params_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PATH_HANDLER_HPP_
