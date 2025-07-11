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

#include "nav2_rviz_plugins/costmap_cost_tool.hpp"
#include <rviz_common/viewport_mouse_event.hpp>
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/bool_property.hpp"

namespace nav2_rviz_plugins
{
CostmapCostTool::CostmapCostTool()
: qos_profile_(5)
{
  shortcut_key_ = 'm';

  auto_deactivate_property_ = new rviz_common::properties::BoolProperty(
    "Single click", true,
    "Switch away from this tool after one click.",
    getPropertyContainer(), SLOT(updateAutoDeactivate()), this);
}

CostmapCostTool::~CostmapCostTool() {}

void CostmapCostTool::onInitialize()
{
  hit_cursor_ = cursor_;
  std_cursor_ = rviz_common::getDefaultCursor();

  setName("Costmap Cost");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/PointStamped.png"));

  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  local_cost_client_ =
    node_->create_client<nav2_msgs::srv::GetCost>("local_costmap/get_cost_local_costmap");
  global_cost_client_ =
    node_->create_client<nav2_msgs::srv::GetCost>("global_costmap/get_cost_global_costmap");
}

void CostmapCostTool::activate() {}
void CostmapCostTool::deactivate() {}

int CostmapCostTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;

  Ogre::Vector3 position;
  bool success = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, position);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (success) {
    std::ostringstream s;
    s << "<b>Left-Click:</b> Select this point.";
    s.precision(3);
    s << " [" << position.x << "," << position.y << "," << position.z << "]";
    setStatus(s.str().c_str());

    if (event.leftUp()) {
      auto point = rviz_common::pointOgreToMsg(position);

      callCostService(point.x, point.y);

      if (auto_deactivate_property_->getBool()) {
        flags |= Finished;
      }
    }
  } else {
    setStatus("Move over an object to select the target point.");
  }
  return flags;
}

void CostmapCostTool::callCostService(float x, float y)
{
  // Create request for local costmap
  auto request = std::make_shared<nav2_msgs::srv::GetCost::Request>();
  request->x = x;
  request->y = y;

  // Call local costmap service
  if (local_cost_client_->wait_for_service(std::chrono::seconds(1))) {
    local_cost_client_->async_send_request(request,
      std::bind(&CostmapCostTool::handleLocalCostResponse, this, std::placeholders::_1));
  }

  // Call global costmap service
  if (global_cost_client_->wait_for_service(std::chrono::seconds(1))) {
    global_cost_client_->async_send_request(request,
      std::bind(&CostmapCostTool::handleGlobalCostResponse, this, std::placeholders::_1));
  }
}

void CostmapCostTool::handleLocalCostResponse(
  rclcpp::Client<nav2_msgs::srv::GetCost>::SharedFuture future)
{
  auto response = future.get();
  if (response->cost != -1) {
    RCLCPP_INFO(node_->get_logger(), "Local costmap cost: %.1f", response->cost);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get local costmap cost");
  }
}

void CostmapCostTool::handleGlobalCostResponse(
  rclcpp::Client<nav2_msgs::srv::GetCost>::SharedFuture future)
{
  auto response = future.get();
  if (response->cost != -1) {
    RCLCPP_INFO(node_->get_logger(), "Global costmap cost: %.1f", response->cost);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get global costmap cost");
  }
}
}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::CostmapCostTool, rviz_common::Tool)
