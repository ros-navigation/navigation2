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
#include "nav2_costmap_2d/inflation_layer.h"
#include "nav2_costmap_2d/layered_costmap.h"
#include "nav2_costmap_2d/static_layer.h"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "tf2_ros/transform_listener.h"

namespace nav2_costmap_world_model
{

class CostmapWorldModel : public rclcpp::Node
{
public:
  explicit CostmapWorldModel(const std::string & name);
  CostmapWorldModel();

  template<class LayerT>
  void addLayer(std::string layer_name)
  {
    auto layer = std::make_shared<LayerT>();
    layered_costmap_->addPlugin(layer);
    layer->initialize(layered_costmap_, layer_name, tf_, node_);
  }

  void setFootprint(double length, double width);

private:
  void costmap_callback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request>/*request*/,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response);

  // Server for providing a costmap
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr costmapServer_;
  nav2_costmap_2d::LayeredCostmap * layered_costmap_;
  tf2_ros::Buffer * tf_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace nav2_costmap_world_model

#endif  // NAV2_COSTMAP_WORLD_MODEL__COSTMAP_WORLD_MODEL_HPP_
