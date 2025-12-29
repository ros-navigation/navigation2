// Copyright (c) 2025 Maurice Alexander Purnawan
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

#ifndef NAV2_CORE__PATH_HANDLER_HPP_
#define NAV2_CORE__PATH_HANDLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_core
{
using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;
using PathSegment = std::pair<PathIterator, PathIterator>;

/**
 * @class PathHandler
 * @brief Function-object for handling the path from Planner Server
 *
 * This class defines the plugin interface used by the Controller Server to manage the
 * path received from the Planner Server. Its primary responsibilities are pruning
 * path segments the robot has already traversed and transforming the remaining,
 * relevant portion of the path into the costmap's global or base frame.
 */
class PathHandler
{
public:
  typedef std::shared_ptr<nav2_core::PathHandler> Ptr;

  virtual ~PathHandler() {}

  /**
   * @brief Initialize parameters
   * @param parent Lifecycle node pointer
   * @param logger Node logging interface
   * @param plugin_name Name of the plugin
   * @param costmap_ros Costmap2DROS object
   * @param tf Shared ptr of TF2 buffer
   */
  virtual void initialize(
    const nav2::LifecycleNode::WeakPtr & parent,
    const rclcpp::Logger & logger,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    std::shared_ptr<tf2_ros::Buffer> tf) = 0;

  /**
   * @brief Set new reference plan
   * @param Path Path to use
   */
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief Determines the portion of the global plan to be used for local control.
   * This function locates the start and end iterators of the global plan segment
   * that is relevant for controller computation based on the robot's current pose and local costmap.
   * @param pose Robot pose in odom frame
   * @return PathSegment A pair of iterators defining the start and end of the
   *         selected plan segment.
   */
  virtual PathSegment findPlanSegment(
    const geometry_msgs::msg::PoseStamped & pose) = 0;

  /**
    * @brief Transforms a predefined segment of the global plan into the costmap global frame.
    * @param closest_point Iterator to the starting pose of the path segment.
    * @param pruned_plan_end Iterator to the ending pose of the path segment.
    * @return nav_msgs::msg::Path The transformed local plan segment in the costmap global frame.
    */
  virtual nav_msgs::msg::Path transformLocalPlan(
    const PathIterator & closest_point,
    const PathIterator & pruned_plan_end) = 0;

  /**
   * @brief Get the global goal pose transformed to the costmap global frame
   * @param stamp Time to get the goal pose at
   * @return Transformed goal pose
   */
  virtual geometry_msgs::msg::PoseStamped getTransformedGoal(
    const builtin_interfaces::msg::Time & stamp) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__PATH_HANDLER_HPP_
