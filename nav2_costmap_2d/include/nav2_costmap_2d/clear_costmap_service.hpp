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

class ClearCostmapService
{
public:
  ClearCostmapService(nav2_util::LifecycleNode::SharedPtr node, Costmap2DROS & costmap);

  ClearCostmapService() = delete;

  // Clears the region outside of a user-specified area reverting to the static map
  void clearExceptRegion(double reset_distance = 3.0);

  // Clears within a window around the robot
  void clearAroundRobot(double window_size_x, double window_size_y);

  // Clears all layers
  void clearEntirely();

private:
  // The ROS node to use for getting parameters, creating the service and logging
  nav2_util::LifecycleNode::SharedPtr node_;

  // The costmap to clear
  Costmap2DROS & costmap_;

  // Clearing parameters
  unsigned char reset_value_;
  std::vector<std::string> clearable_layers_;

  // Server for clearing the costmap
  rclcpp::Service<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedPtr clear_except_service_;
  void clearExceptRegionCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapExceptRegion::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapExceptRegion::Response> response);

  rclcpp::Service<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_service_;
  void clearAroundRobotCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Response> response);

  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_service_;
  void clearEntireCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response> response);

  void clearLayerExceptRegion(
    std::shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance);

  bool isClearable(const std::string & layer_name) const;

  bool getPosition(double & x, double & y) const;

  std::string getLayerName(const Layer & layer) const;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__CLEAR_COSTMAP_SERVICE_HPP_
