// Copyright (c) 2019 Samsung Research America
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

#ifndef NAV2_CORE_GLOBAL_PLANNER_H_
#define NAV2_CORE_GLOBAL_PLANNER_H_

#include <string>
#include "rclcpp/rclcpp.h"
#include "nav2_costmap_2d/nav2_costmap_2d.hpp"
#include "tf2_ros/Buffer.h"
#include "nav2_msgs/msg/path.h"
#include "geometry_msgs/msg/pose_stamped.h"

namespace nav2_core
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib
 */
class GlobalPlanner
{
public:
  /**
   * @brief Virtual destructor
   */
  virtual ~GlobalPlanner() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
  virtual void configure(const rclcpp::LifecycleNode * parent,
    const std::string & name, tf2_ros::Buffer * tf,
    nav2_costmap_2d::Costmap2DROS * costmap_ros) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method to shutdown planner and any threads involved in execution.
   */
  virtual void shutdown() = 0;

  /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return      The sequence of poses to get from start to goal, if any
   */
  virtual nav2_msgs::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE_GLOBAL_PLANNER_H_
