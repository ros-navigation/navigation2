// Copyright (c) 2024 Jatin Patil
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

#ifndef NAV2_RVIZ_PLUGINS__COSTMAP_COST_TOOL_HPP_
#define NAV2_RVIZ_PLUGINS__COSTMAP_COST_TOOL_HPP_

#include <memory>

#include <nav2_msgs/srv/get_costs.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rclcpp/rclcpp.hpp>
#include "nav2_util/service_client.hpp"

namespace nav2_rviz_plugins
{
class CostmapCostTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  CostmapCostTool();
  virtual ~CostmapCostTool();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

  void callCostService(float x, float y);

  void handleLocalCostResponse(rclcpp::Client<nav2_msgs::srv::GetCosts>::SharedFuture);
  void handleGlobalCostResponse(rclcpp::Client<nav2_msgs::srv::GetCosts>::SharedFuture);

private Q_SLOTS:

private:
  std::shared_ptr<nav2_util::ServiceClient<nav2_msgs::srv::GetCosts>> local_cost_client_;
  std::shared_ptr<nav2_util::ServiceClient<nav2_msgs::srv::GetCosts>> global_cost_client_;
  // The Node pointer that we need to keep alive for the duration of this plugin.
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;

  QCursor std_cursor_;
  QCursor hit_cursor_;
  rviz_common::properties::BoolProperty * auto_deactivate_property_;
  rviz_common::properties::QosProfileProperty * qos_profile_property_;

  rclcpp::QoS qos_profile_;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__COSTMAP_COST_TOOL_HPP_
