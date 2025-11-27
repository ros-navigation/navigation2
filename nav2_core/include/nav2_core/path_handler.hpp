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

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_core
{

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
   * @brief Initialize any parameters from the NodeHandle
   * @param parent Node pointer for grabbing parameters
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
   * @brief transform global plan to local applying constraints,
   * then prune global plan
   * @param pose pose to transform
   * @return Path after pruned
   */
  virtual nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose) = 0;

  /**
   * @brief Get the global goal pose transformed to the desired frame
   * @param stamp Time to get the goal pose at
   * @return Transformed goal pose
   */
  virtual geometry_msgs::msg::PoseStamped getTransformedGoal(
    const builtin_interfaces::msg::Time & stamp) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__PATH_HANDLER_HPP_
