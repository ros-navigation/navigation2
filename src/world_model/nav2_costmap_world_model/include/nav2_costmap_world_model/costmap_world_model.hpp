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
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_libs_msgs/msg/costmap.hpp"
#include "nav2_world_model_msgs/srv/get_costmap.hpp"
#include "nav2_tasks/map_service_client.hpp"

namespace nav2_costmap_world_model
{

class CostmapWorldModel : public rclcpp::Node
{
public:
  explicit CostmapWorldModel(const std::string & name);
  CostmapWorldModel();

private:
  // Server for providing a costmap
  rclcpp::Service<nav2_world_model_msgs::srv::GetCostmap>::SharedPtr costmapServer_;

  // TODO(orduno): Define a server for scoring trajectories
  // rclcpp::Service<nav2_world_model_msgs::srv::ScoreTrajectory>::SharedPtr scoringServer_;

  // TODO(orduno): Define a client for getting the static map
  // rclcpp::Client<nav2_world_model_msgs::srv::GetMap>::SharedPtr mapClient_;

  // TODO(orduno): Alternatively, obtain from a latched topic
  // rclcpp::Subscription<nav2_world_model_msgs::OccupancyGrid>::SharedPtr mapSub_;

  // TODO(orduno): Define a task for handling trajectory scoring
  // std::unique_ptr<ScoreTrajectoryClient> scorer;

  // TODO(orduno): std::unique_ptr<LayeredCostmap> layeredCostmap_;
  std::unique_ptr<nav2_util::Costmap> costmap_;

  nav2_tasks::MapServiceClient map_client_;
};

}  // namespace nav2_costmap_world_model

#endif  // NAV2_COSTMAP_WORLD_MODEL__COSTMAP_WORLD_MODEL_HPP_
