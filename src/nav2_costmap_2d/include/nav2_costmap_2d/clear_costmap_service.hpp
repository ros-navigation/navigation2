// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_COSTMAP_2D__CLEAR_COSTMAP_SERVICE_HPP_
#define NAV2_COSTMAP_2D__CLEAR_COSTMAP_SERVICE_HPP_

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_costmap_2d
{

class Costmap2DROS;

/**
 * @class ClearCostmapService
 * @brief Exposes services to clear costmap objects in inclusive/exclusive regions or completely
 */
class ClearCostmapService
{
public:
  /**
   * @brief A constructor
   */
  ClearCostmapService(const nav2_util::LifecycleNode::WeakPtr & parent, Costmap2DROS & costmap);

  /**
   * @brief A constructor
   */
  ClearCostmapService() = delete;

  /**
   * @brief Clears the region outside of a user-specified area reverting to the static map
   */
  void clearRegion(double reset_distance, bool invert);

  /**
   * @brief Clears all layers
   */
  void clearEntirely();

private:
  // The Logger object for logging
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};

  // The costmap to clear
  Costmap2DROS & costmap_;

  // Clearing parameters
  unsigned char reset_value_;

  // Server for clearing the costmap
  rclcpp::Service<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedPtr clear_except_service_;
  /**
   * @brief Callback to clear costmap except in a given region
   */
  void clearExceptRegionCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapExceptRegion::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapExceptRegion::Response> response);

  rclcpp::Service<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_service_;
  /**
   * @brief Callback to clear costmap in a given region
   */
  void clearAroundRobotCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Response> response);

  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_service_;
  /**
   * @brief Callback to clear costmap
   */
  void clearEntireCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response> response);

  /**
   * @brief  Function used to clear a given costmap layer
   */
  void clearLayerRegion(
    std::shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance,
    bool invert);

  /**
   * @brief Get the robot's position in the costmap using the master costmap
   */
  bool getPosition(double & x, double & y) const;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__CLEAR_COSTMAP_SERVICE_HPP_
