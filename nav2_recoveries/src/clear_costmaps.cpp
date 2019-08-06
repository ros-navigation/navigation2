// Copyright (c) 2019 Samsung Research America
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

#include <string>
#include <memory>
#include <future>

#include "nav2_recoveries/clear_costmaps.hpp"

namespace nav2_recoveries
{

ClearCostmaps::ClearCostmaps(rclcpp::Node::SharedPtr & node, const std::string & srv_name)
: node_(node)
{
  last_clear_time_ = std::chrono::system_clock::now();

  // reasonable defaults
  service_names_.push_back("/local_costmap/clear_entirely_local_costmap");
  service_names_.push_back("/global_costmap/clear_entirely_global_costmap");
  int service_timeout_in_s = 1;
  int frequency_timeout_in_s = 1;

  // get specifics from parameter server
  node_->get_parameter("clear_costmap_service_names", service_names_);
  node_->get_parameter("service_timeout", service_timeout_in_s);
  node_->get_parameter("frequency_timeout_in_s", frequency_timeout_in_s);
  service_timeout_in_s_ = std::chrono::seconds(service_timeout_in_s);
  frequency_timeout_in_s_ = std::chrono::seconds(frequency_timeout_in_s);

  // create our clients and server
  for (uint i = 0; i != service_names_.size(); i++) {
    if (i == 0) {
      services_string_list_ = service_names_.at(i);
    } else {
      services_string_list_ += ", " + service_names_.at(i);
    }
    costmap_clearing_services_.push_back(
      node_->create_client<ClearEntireCostmap>(service_names_.at(i)));
    costmap_clearing_services_.back()->wait_for_service(std::chrono::seconds(3));
  }

  bt_clear_costmap_servicer_ = node_->create_service<ClearEntireCostmap>(srv_name,
      std::bind(&ClearCostmaps::onCall, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(node_->get_logger(), "Registered potential costmaps to clear: %s",
    services_string_list_.c_str());
}

ClearCostmaps::~ClearCostmaps()
{
}

void ClearCostmaps::onCall(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ClearEntireCostmap::Request>/*request*/,
  const std::shared_ptr<ClearEntireCostmap::Response>/*response*/)
{
  RCLCPP_INFO(node_->get_logger(), "Clearing costmaps: %s...",
    services_string_list_.c_str());

  last_clear_time_ = std::chrono::system_clock::now();

  if (std::chrono::system_clock::now() - last_clear_time_ >
    frequency_timeout_in_s_)
  {
    RCLCPP_WARN(node_->get_logger(), "Cannot clear costmaps within %0.2fs "
      "of each call.", frequency_timeout_in_s_);
    return;
  }

  std::string failed_costmap_clear = "";

  for (uint i = 0; i != service_names_.size(); i++) {
    auto & current_service = costmap_clearing_services_.at(i);
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    auto future_result = current_service->async_send_request(request);

    std::future_status status = future_result.wait_for(service_timeout_in_s_);
    if (status == std::future_status::deferred) {
      failed_costmap_clear += std::string(current_service->get_service_name()) + " ";
    }
  }

  if (!failed_costmap_clear.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Failed to clear at least one costmap: %s.",
      failed_costmap_clear.c_str());
  }
}

}  // namespace nav2_recoveries
