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

  node_ptr_ = context_->getRosNodeAbstraction().lock();
  if (node_ptr_ == nullptr) {
    // The node no longer exists, so just don't initialize
    RCLCPP_ERROR(
      rclcpp::get_logger("costmap_cost_tool"),
      "Underlying ROS node no longer exists, initialization failed");
    return;
  }
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

  node->declare_parameter("service_introspection_mode", "disabled");
  std::string service_introspection_mode;
  node->get_parameter("service_introspection_mode", service_introspection_mode);
  local_cost_client_ =
    std::make_shared<nav2_util::ServiceClient<nav2_msgs::srv::GetCosts>>(
      "/local_costmap/get_cost_local_costmap",
      service_introspection_mode,
      node);
  global_cost_client_ =
    std::make_shared<nav2_util::ServiceClient<nav2_msgs::srv::GetCosts>>(
      "/global_costmap/get_cost_global_costmap",
      service_introspection_mode,
      node);
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
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  // Create request for local costmap
  auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = context_->getFixedFrame().toStdString();
  pose.header.stamp = node->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  request->poses.push_back(pose);
  request->use_footprint = false;

  // Call local costmap service
  auto local_cost_response_ = local_cost_client_->invoke_shared(request).get();
  RCLCPP_INFO(node->get_logger(), "Local costmap cost: %.1f", local_cost_response_->costs[0]);

  auto global_cost_response_ = global_cost_client_->invoke_shared(request).get();
  RCLCPP_INFO(node->get_logger(), "Global costmap cost: %.1f", global_cost_response_->costs[0]);
}
}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::CostmapCostTool, rviz_common::Tool)
