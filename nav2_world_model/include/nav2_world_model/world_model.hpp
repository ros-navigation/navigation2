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

#ifndef NAV2_WORLD_MODEL__WORLD_MODEL_HPP_
#define NAV2_WORLD_MODEL__WORLD_MODEL_HPP_

#include <string>
#include <vector>
#include <memory>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "tf2_ros/transform_listener.h"

namespace nav2_world_model
{

class WorldModel : public rclcpp::Node
{
public:
  WorldModel(rclcpp::executor::Executor & executor, const std::string & name);
  explicit WorldModel(rclcpp::executor::Executor & executor);

private:
  void costmap_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request> request,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response);

  // Server for providing a costmap
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr costmapServer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__WORLD_MODEL_HPP_
