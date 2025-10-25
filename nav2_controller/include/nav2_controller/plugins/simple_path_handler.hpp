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

#ifndef NAV2_CONTROLLER__PLUGINS__SIMPLE_PATH_HANDLER_HPP_
#define NAV2_CONTROLLER__PLUGINS__SIMPLE_PATH_HANDLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <utility>
#include "nav2_core/path_handler.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

namespace nav2_controller
{
using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;
using PathSegment = std::pair<PathIterator, PathIterator>;
/**
* @class SimplePathHandler
* @brief This plugin manages the global plan by clipping it to the local
* segment, typically bounded by the local costmap size
* and transforming the resulting path into the odom frame.
*/

class SimplePathHandler : public nav2_core::PathHandler
{
public:
  void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const rclcpp::Logger & logger,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    std::shared_ptr<tf2_ros::Buffer> tf) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose) override;
  geometry_msgs::msg::PoseStamped getTransformedGoal(
    const builtin_interfaces::msg::Time & stamp) override;

protected:
  /**
    * @brief Transform a pose to the global reference frame
    * @param pose Current pose
    * @return output poose in global reference frame
    */
  geometry_msgs::msg::PoseStamped transformToGlobalPlanFrame(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
    * @brief Finds the start and end iterators defining the segment of the global plan
    * that should be used for local control.
    * @param global_pose Robot's current pose in map frame
    * @return PathSegment A pair of iterators representing the [start, end) range of the local plan segment.
    */
  PathSegment findPlanSegmentIterators(const geometry_msgs::msg::PoseStamped & global_pose);

   /**
    * @brief Transforms a predefined segment of the global plan into the odom frame.
    * @param global_pose Robot's current pose
    * @param closest_point Iterator to the starting pose of the path segment.
    * @param pruned_plan_end Iterator to the ending pose of the path segment.
    * @return nav_msgs::msg::Path The transformed local plan segment in the odom frame.
    */
  nav_msgs::msg::Path transformLocalPlan(
    const geometry_msgs::msg::PoseStamped & global_pose,
    const PathIterator & closest_point,
    const PathIterator & pruned_plan_end);

  /**
   * Get the greatest extent of the costmap in meters from the center.
   * @return max of distance from center in meters to edge of costmap
   */
  double getCostmapMaxExtent() const;

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

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rclcpp::Logger logger_ {rclcpp::get_logger("ControllerServer")};
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path global_plan_up_to_inversion_;
  unsigned int inversion_locale_{0u};
  bool interpolate_curvature_after_goal_, enforce_path_inversion_;
  double max_robot_pose_search_dist_, transform_tolerance_, prune_distance_;
  float inversion_xy_tolerance_, inversion_yaw_tolerance_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__SIMPLE_PATH_HANDLER_HPP_
