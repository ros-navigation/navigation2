// Copyright (c) 2020 Samsung Research Russia
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

// TODO(AlexeyMerzlyakov): This dummy info publisher should be removed
// after Semantic Map Server having the same functionality will be developed.

#include "nav2_map_server/costmap_filter_info_server.hpp"

#include <string>
#include <memory>
#include <utility>

namespace nav2_map_server
{

CostmapFilterInfoServer::CostmapFilterInfoServer(const rclcpp::NodeOptions & options)
: nav2::LifecycleNode("costmap_filter_info_server", "", options)
{
  declare_parameter("filter_info_topic", "costmap_filter_info");
  declare_parameter("type", 0);
  declare_parameter("mask_topic", "filter_mask");
  declare_parameter("base", 0.0);
  declare_parameter("multiplier", 1.0);
}

CostmapFilterInfoServer::~CostmapFilterInfoServer()
{
}

nav2::CallbackReturn
CostmapFilterInfoServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  std::string filter_info_topic = get_parameter("filter_info_topic").as_string();

  publisher_ = this->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic, nav2::qos::LatchedPublisherQoS());

  msg_ = nav2_msgs::msg::CostmapFilterInfo();
  msg_.header.frame_id = "";
  msg_.header.stamp = now();
  msg_.type = get_parameter("type").as_int();
  msg_.filter_mask_topic = get_parameter("mask_topic").as_string();
  msg_.base = static_cast<float>(get_parameter("base").as_double());
  msg_.multiplier = static_cast<float>(get_parameter("multiplier").as_double());

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
CostmapFilterInfoServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  publisher_->on_activate();
  auto costmap_filter_info = std::make_unique<nav2_msgs::msg::CostmapFilterInfo>(msg_);
  publisher_->publish(std::move(costmap_filter_info));

  // create bond connection
  createBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
CostmapFilterInfoServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  publisher_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
CostmapFilterInfoServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  publisher_.reset();

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
CostmapFilterInfoServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2::CallbackReturn::SUCCESS;
}

}  // namespace nav2_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_map_server::CostmapFilterInfoServer)
