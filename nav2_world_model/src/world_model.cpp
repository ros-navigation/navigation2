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

#include "nav2_world_model/world_model.hpp"

#include <iostream>
#include <memory>

namespace nav2_world_model
{

WorldModel::WorldModel()
: nav2_lifecycle::LifecycleNode("world_model")
{
  RCLCPP_INFO(get_logger(), "Creating");

  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("world_model_global_costmap");
  costmap_thread_ = std::make_unique<std::thread>([](rclcpp_lifecycle::LifecycleNode::SharedPtr node) {rclcpp::spin(node->get_node_base_interface());}, costmap_ros_);
}

WorldModel::~WorldModel()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  costmap_thread_->join();
}

nav2_lifecycle::CallbackReturn
WorldModel::onConfigure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "onConfigure");

  costmap_ros_->onConfigure(state);

  // Create a service that will use the callback function to handle requests.
  costmap_service_ = create_service<nav2_msgs::srv::GetCostmap>("GetCostmap",
      std::bind(&WorldModel::costmap_service_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
WorldModel::onActivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "onActivate");

  costmap_ros_->onActivate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
WorldModel::onDeactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "onDeactivate");

  costmap_ros_->onDeactivate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
WorldModel::onCleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "onCleanup");

  costmap_ros_->onCleanup(state);
  costmap_service_.reset();

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

void
WorldModel::costmap_service_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request>/*request*/,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received costmap service request");

  // TODO(bpwilcox): Grab correct orientation information
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.0);

  // TODO: 
  nav2_costmap_2d::Costmap2D * costmap_ = costmap_ros_->getCostmap();

  auto size_x = costmap_->getSizeInCellsX();
  auto size_y = costmap_->getSizeInCellsY();
  auto data_length = size_x * size_y;
  unsigned char * data = costmap_->getCharMap();

  response->map.header.stamp = now();
  response->map.header.frame_id = "map";
  response->map.metadata.size_x = size_x;
  response->map.metadata.size_y = size_y;
  response->map.metadata.resolution = costmap_->getResolution();
  response->map.metadata.layer = "Master";
  response->map.metadata.map_load_time = now();
  response->map.metadata.update_time = now();
  response->map.metadata.origin.position.x = costmap_->getOriginX();
  response->map.metadata.origin.position.y = costmap_->getOriginY();
  response->map.metadata.origin.position.z = 0.0;
  response->map.metadata.origin.orientation = tf2::toMsg(quaternion);
  response->map.data.resize(data_length);
  response->map.data.assign(data, data + data_length);
}

}  // namespace nav2_world_model
