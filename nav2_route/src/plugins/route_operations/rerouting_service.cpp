// Copyright (c) 2025, Open Navigation LLC
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


#include <memory>
#include <string>

#include "nav2_route/plugins/route_operations/rerouting_service.hpp"

namespace nav2_route
{

void ReroutingService::configure(
  const nav2_util::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/* costmap_subscriber */,
  const std::string & name)
{
  RCLCPP_INFO(node->get_logger(), "Configuring Rerouting service operation.");
  name_ = name;
  logger_ = node->get_logger();
  reroute_.store(false);
  service_ = node->create_service<std_srvs::srv::Trigger>(
    std::string(node->get_name()) + "/" + getName() + "/reroute",
    std::bind(&ReroutingService::serviceCb, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void ReroutingService::serviceCb(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(logger_, "A reroute has been requested!");
  reroute_.store(true);
  response->success = true;
}

OperationResult ReroutingService::perform(
  NodePtr /*node*/,
  EdgePtr /*edge_entered*/,
  EdgePtr /*edge_exited*/,
  const Route & /*route*/,
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/,
  const Metadata * /*mdata*/)
{
  OperationResult result;
  if (reroute_.load()) {
    reroute_.store(false);
    result.reroute = true;
    return result;
  }

  return result;
}

}  // namespace nav2_route

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_route::ReroutingService, nav2_route::RouteOperation)
