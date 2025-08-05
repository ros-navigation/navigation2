// Copyright (c) 2025, Angsa Robotics GmbH
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

#ifndef NAV2_MPPI_CONTROLLER__COLLISION_CHECKER_HPP_
#define NAV2_MPPI_CONTROLLER__COLLISION_CHECKER_HPP_

#include <memory>
#include <vector>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace nav2_mppi_controller
{

/**
 * @struct CollisionResult
 * @brief Result of collision checking
 */
struct CollisionResult
{
  bool in_collision;
  std::vector<float> center_cost;
  std::vector<float> footprint_cost;
};

/**
 * @class nav2_mppi_controller::MPPICollisionChecker
 * @brief A costmap grid collision checker
 */
class MPPICollisionChecker
  : public nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
{
public:
  /**
   * @brief A constructor for nav2_mppi_controller::MPPICollisionChecker
   * @param costmap The costmap to collision check against
   * @param node Node to extract clock and logger from
   */
  MPPICollisionChecker(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief A constructor for nav2_mppi_controller::MPPICollisionChecker
   * for use when irregular bin intervals are appropriate
   * @param costmap The costmap to collision check against
   * @param angles The vector of possible angle bins to precompute for
   * orientations for to speed up collision checking, in radians
   */
  // MPPICollisionChecker(
  //   nav2_costmap_2d::Costmap2D * costmap,
  //   std::vector<float> & angles);

  /**
   * @brief Set the footprint to use with collision checker
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius collision checking
   * @param inflation_layer_name Optional name of inflation layer for cost calculation
   */
  void setFootprint(
    const nav2_costmap_2d::Footprint & footprint,
    const bool & radius,
    const std::string & inflation_layer_name = "");

  /**
   * @brief Check if in collision with costmap and footprint at poses (batch processing)
   * @param x Vector of X coordinates of poses to check against
   * @param y Vector of Y coordinates of poses to check against
   * @param yaw Vector of yaw angles of poses to check against in radians
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return CollisionResult struct with collision status and vectors of center cost, footprint cost, and area cost
   */
  CollisionResult inCollision(
    const std::vector<float> & x,
    const std::vector<float> & y,
    const std::vector<float> & yaw,
    const bool & traverse_unknown);

  /**
   * @brief Get costmap ros object for inflation layer params
   * @return Costmap ros
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getCostmapROS() {return costmap_ros_;}

  /**
   * @brief Check if value outside the range
   * @param max Maximum value of the range
   * @param value the value to check if it is within the range
   * @return boolean if in range or not
   */
  bool outsideRange(const unsigned int & max, const float & value) const;

  /**
   * @brief Create convex hull from a set of points
   * @param points Input points to create convex hull from
   * @return Convex hull as a footprint
   */
  nav2_costmap_2d::Footprint createConvexHull(
    const std::vector<geometry_msgs::msg::Point> & points);

  /**
   * @brief Find the circumscribed cost for collision checking optimization
   * @param inflation_layer_name Optional name of inflation layer
   * @return The circumscribed cost value
   */
  float findCircumscribedCost(const std::string & inflation_layer_name = "");

private:
  /**
   * @brief Transform footprint to given orientation
   * @param footprint The base footprint to transform
   * @param yaw The yaw angle to transform to
   * @return Transformed footprint
   */
  nav2_costmap_2d::Footprint transformFootprint(
    const nav2_costmap_2d::Footprint & footprint,
    float yaw) const;

protected:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Footprint unoriented_footprint_;
  bool footprint_is_radius_{false};
  float possible_collision_cost_{-1};
  float circumscribed_radius_{-1.0f};
  float circumscribed_cost_{-1.0f};
  rclcpp::Logger logger_{rclcpp::get_logger("MPPICollisionChecker")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace nav2_mppi_controller

#endif  // NAV2_MPPI_CONTROLLER__COLLISION_CHECKER_HPP_
