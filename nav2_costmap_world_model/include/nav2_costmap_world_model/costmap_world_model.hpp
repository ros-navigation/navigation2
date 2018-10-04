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

#ifndef NAV2_COSTMAP_WORLD_MODEL__COSTMAP_WORLD_MODEL_HPP_
#define NAV2_COSTMAP_WORLD_MODEL__COSTMAP_WORLD_MODEL_HPP_

#include <string>
#include <vector>
#include <memory>
#include "costmap_2d/costmap_2d_ros.h"
#include "costmap_2d/inflation_layer.h"
#include "costmap_2d/static_layer.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_tasks/map_service_client.hpp"

namespace nav2_costmap_world_model
{

class CostmapWorldModel : public rclcpp::Node
{
public:
  explicit CostmapWorldModel(const std::string & name);
  CostmapWorldModel();

  void addStaticLayer();
  void addInflationLayer();

private:
  void costmap_callback(
      const std::shared_ptr<rmw_request_id_t>/*request_header*/,
      const std::shared_ptr<nav2_world_model_msgs::srv::GetCostmap::Request>/*request*/,
      const std::shared_ptr<nav2_world_model_msgs::srv::GetCostmap::Response> response);

  // Server for providing a costmap
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr costmapServer_;

  // TODO(orduno): Define a server for scoring trajectories
  // rclcpp::Service<nav2_msgs::srv::ScoreTrajectory>::SharedPtr scoringServer_;

  // TODO(orduno): Define a task for handling trajectory scoring
  // std::unique_ptr<ScoreTrajectoryClient> scorer;

  costmap_2d::LayeredCostmap * layered_costmap_;
  tf2_ros::Buffer * tf_;
};

}  // namespace nav2_costmap_world_model

#endif  // NAV2_COSTMAP_WORLD_MODEL__COSTMAP_WORLD_MODEL_HPP_
