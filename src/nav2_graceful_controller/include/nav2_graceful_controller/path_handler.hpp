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

#ifndef NAV2_GRACEFUL_CONTROLLER__PATH_HANDLER_HPP_
#define NAV2_GRACEFUL_CONTROLLER__PATH_HANDLER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_graceful_controller
{

/**
 * @class nav2_graceful_controller::PathHandler
 * @brief Handles input paths to transform them to local frames required
 */
class PathHandler
{
public:
  /**
   * @brief Constructor for nav2_graceful_controller::PathHandler
   */
  PathHandler(
    tf2::Duration transform_tolerance,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Destructor for nav2_graceful_controller::PathHandler
   */
  ~PathHandler() = default;

  /**
   * @brief Transforms global plan into same frame as pose and clips poses ineligible for motionTarget
   * Points ineligible to be selected as a motion target point if they are any of the following:
   * - Outside the local_costmap (collision avoidance cannot be assured)
   * @param pose pose to transform
   * @param max_robot_pose_search_dist Distance to search for matching nearest path point
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    double max_robot_pose_search_dist);

  /**
   * @brief Sets the global plan
   *
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path);

  /**
   * @brief Gets the global plan
   *
   * @return The global plan
   */
  nav_msgs::msg::Path getPlan() {return global_plan_;}

protected:
  rclcpp::Duration transform_tolerance_{0, 0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  rclcpp::Logger logger_ {rclcpp::get_logger("GracefulPathHandler")};
};

}  // namespace nav2_graceful_controller

#endif  // NAV2_GRACEFUL_CONTROLLER__PATH_HANDLER_HPP_
