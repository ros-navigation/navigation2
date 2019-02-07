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

#ifndef NAV2_COSTMAP_2D__COSTMAP_CLEANER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_CLEANER_HPP_

#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clean_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

namespace nav2_costmap_2d
{

class Costmap2DROS;

// Clears the region outside of a user-specified area
// Currently reverting to the static map
class CostmapCleaner
{
public:
  CostmapCleaner(rclcpp::Node::SharedPtr & node, Costmap2DROS * costmap);

  CostmapCleaner() = delete;

  void clean();

private:
  // The ROS node to use for getting parameters, creating the service and logging
  rclcpp::Node::SharedPtr node_;

  // The costmap to clear
  Costmap2DROS * costmap_;

  // Cleaning parameters
  double reset_distance_;
  std::vector<std::string> cleanable_layers_;

  // Server for cleaning the costmap
  rclcpp::Service<nav2_msgs::srv::CleanCostmap>::SharedPtr server_;
  void cleanCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::CleanCostmap::Request> request,
    const std::shared_ptr<nav2_msgs::srv::CleanCostmap::Response> response);

  void cleanLayer(std::shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y);

  bool getPose(double & x, double & y) const;

  std::string getLayerName(const Layer & layer) const;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_CLEANER_HPP_
