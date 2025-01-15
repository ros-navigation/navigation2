// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Dexory
// Copyright (c) 2023 Open Navigation LLC
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

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_

#include <vector>
#include <utility>
#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_core/controller_exceptions.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;
using PathRange = std::pair<PathIterator, PathIterator>;

/**
 * @class mppi::PathHandler
 * @brief Manager of incoming reference paths for transformation and processing
 */

class PathHandler
{
public:
  /**
    * @brief Constructor for mppi::PathHandler
    */
  PathHandler() = default;

  /**
    * @brief Destructor for mppi::PathHandler
    */
  ~PathHandler() = default;

  /**
    * @brief Initialize path handler on bringup
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param tf TF buffer for transformations
    * @param dynamic_parameter_handler Parameter handler object
    */
  void initialize(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>,
    std::shared_ptr<tf2_ros::Buffer>, ParametersHandler *);

  /**
    * @brief Set new reference path
    * @param Plan Path to use
    */
  void setPath(const nav_msgs::msg::Path & plan);

  /**
    * @brief Get reference path
    * @return Path
    */
  nav_msgs::msg::Path & getPath();

  /**
   * @brief transform global plan to local applying constraints,
   * then prune global plan
   * @param robot_pose Pose of robot
   * @return global plan in local frame
   */
  nav_msgs::msg::Path transformPath(const geometry_msgs::msg::PoseStamped & robot_pose);

  /**
   * @brief Get the global goal pose transformed to the local frame
   * @param stamp Time to get the goal pose at
   * @return Transformed goal pose
   */
  geometry_msgs::msg::PoseStamped getTransformedGoal(const builtin_interfaces::msg::Time & stamp);

protected:
  /**
    * @brief Transform a pose to another frame
    * @param frame Frame to transform to
    * @param in_pose Input pose
    * @param out_pose Output pose
    * @return Bool if successful
    */
  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
    * @brief Get largest dimension of costmap (radially)
    * @return Max distance from center of costmap to edge
    */
  double getMaxCostmapDist();

  /**
    * @brief Transform a pose to the global reference frame
    * @param pose Current pose
    * @return output poose in global reference frame
    */
  geometry_msgs::msg::PoseStamped
  transformToGlobalPlanFrame(const geometry_msgs::msg::PoseStamped & pose);

  /**
    * @brief Get global plan within window of the local costmap size
    * @param global_pose Robot pose
    * @return plan transformed in the costmap frame and iterator to the first pose of the global
    * plan (for pruning)
    */
  std::pair<nav_msgs::msg::Path, PathIterator> getGlobalPlanConsideringBoundsInCostmapFrame(
    const geometry_msgs::msg::PoseStamped & global_pose);

  /**
    * @brief Prune a path to only interesting portions
    * @param plan Plan to prune
    * @param end Final path iterator
    */
  void prunePlan(nav_msgs::msg::Path & plan, const PathIterator end);

  /**
    * @brief Check if the robot pose is within the set inversion tolerances
    * @param robot_pose Robot's current pose to check
    * @return bool If the robot pose is within the set inversion tolerances
    */
  bool isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose);

  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  ParametersHandler * parameters_handler_;

  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path global_plan_up_to_inversion_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};

  double max_robot_pose_search_dist_{0};
  double prune_distance_{0};
  double transform_tolerance_{0};
  float inversion_xy_tolerance_{0.2};
  float inversion_yaw_tolerance{0.4};
  bool enforce_path_inversion_{false};
  unsigned int inversion_locale_{0u};
};
}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
