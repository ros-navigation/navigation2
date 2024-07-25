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

#include <nav2_msgs/srv/get_cost.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_default_plugins/tools/point/point_tool.hpp>
#include <rclcpp/rclcpp.hpp>

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

  void handleLocalCostResponse(rclcpp::Client<nav2_msgs::srv::GetCost>::SharedFuture);
  void handleGlobalCostResponse(rclcpp::Client<nav2_msgs::srv::GetCost>::SharedFuture);

private Q_SLOTS:
  void updateAutoDeactivate();

private:
  rclcpp::Client<nav2_msgs::srv::GetCost>::SharedPtr local_cost_client_;
  rclcpp::Client<nav2_msgs::srv::GetCost>::SharedPtr global_cost_client_;
  rclcpp::Node::SharedPtr node_;

  QCursor std_cursor_;
  QCursor hit_cursor_;
  rviz_common::properties::BoolProperty * auto_deactivate_property_;
  rviz_common::properties::QosProfileProperty * qos_profile_property_;

  rclcpp::QoS qos_profile_;
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__COSTMAP_COST_TOOL_HPP_
