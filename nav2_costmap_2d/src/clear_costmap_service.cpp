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

#include <vector>
#include <string>
#include <algorithm>
#include <memory>

#include "nav2_costmap_2d/clear_costmap_service.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_costmap_2d
{

using std::vector;
using std::string;
using std::shared_ptr;
using std::any_of;
using ClearExceptRegion = nav2_msgs::srv::ClearCostmapExceptRegion;
using ClearAroundRobot = nav2_msgs::srv::ClearCostmapAroundRobot;
using ClearAroundPose = nav2_msgs::srv::ClearCostmapAroundPose;
using ClearEntirely = nav2_msgs::srv::ClearEntireCostmap;

ClearCostmapService::ClearCostmapService(
  const nav2::LifecycleNode::WeakPtr & parent,
  Costmap2DROS & costmap)
: costmap_(costmap)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  reset_value_ = costmap_.getCostmap()->getDefaultValue();

  clear_except_service_ = node->create_service<ClearExceptRegion>(
    std::string("clear_except_") + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearExceptRegionCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  clear_around_service_ = node->create_service<ClearAroundRobot>(
    std::string("clear_around_") + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearAroundRobotCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  clear_around_pose_service_ = node->create_service<ClearAroundPose>(
    std::string("clear_around_pose_") + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearAroundPoseCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  clear_entire_service_ = node->create_service<ClearEntirely>(
    std::string("clear_entirely_") + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearEntireCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

ClearCostmapService::~ClearCostmapService()
{
  // make sure services shutdown.
  clear_except_service_.reset();
  clear_around_service_.reset();
  clear_around_pose_service_.reset();
  clear_entire_service_.reset();
}

void ClearCostmapService::clearExceptRegionCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearExceptRegion::Request> request,
  const shared_ptr<ClearExceptRegion::Response> response)
{
  RCLCPP_INFO(
    logger_, "%s",
    ("Received request to clear except a region the " + costmap_.getName()).c_str());

  response->success = clearRegion(request->reset_distance, true, request->plugins);
}

void ClearCostmapService::clearAroundRobotCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearAroundRobot::Request> request,
  const shared_ptr<ClearAroundRobot::Response> response)
{
  response->success = clearRegion(request->reset_distance, false, request->plugins);
}

void ClearCostmapService::clearAroundPoseCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearAroundPose::Request> request,
  const shared_ptr<ClearAroundPose::Response> response)
{
  RCLCPP_INFO(
    logger_, "%s",
    ("Received request to clear around pose for " + costmap_.getName()).c_str());

  response->success = clearAroundPose(request->pose, request->reset_distance, request->plugins);
}

void ClearCostmapService::clearEntireCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ClearEntirely::Request> request,
  const std::shared_ptr<ClearEntirely::Response> response)
{
  RCLCPP_INFO(
    logger_, "%s",
    ("Received request to clear entirely the " + costmap_.getName()).c_str());

  response->success = clearEntirely(request->plugins);
}

bool ClearCostmapService::clearAroundPose(
  const geometry_msgs::msg::PoseStamped & pose,
  const double reset_distance,
  const std::vector<std::string> & plugins)
{
  double x, y;

  // Transform pose to costmap frame if necessary
  geometry_msgs::msg::PoseStamped global_pose;
  try {
    if (pose.header.frame_id == costmap_.getGlobalFrameID()) {
      global_pose = pose;
    } else {
      costmap_.getTfBuffer()->transform(pose, global_pose, costmap_.getGlobalFrameID());
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger_,
      "Cannot clear map around pose because pose cannot be transformed to costmap frame: %s",
      ex.what());
    return false;
  }

  x = global_pose.pose.position.x;
  y = global_pose.pose.position.y;

  auto layers = costmap_.getLayeredCostmap()->getPlugins();
  
  if (!plugins.empty()) {
    return validateAndClearPlugins(
      plugins, layers,
      [this, x, y, reset_distance](std::shared_ptr<CostmapLayer> & layer) {
        clearLayerRegion(layer, x, y, reset_distance, false);
      },
      "clear costmap around pose");
  } else {
    // Clear all clearable layers (default behavior)
    bool any_plugin_cleared = false;
    for (auto & layer : *layers) {
      if (layer->isClearable()) {
        auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
        clearLayerRegion(costmap_layer, x, y, reset_distance, false);
        any_plugin_cleared = true;
      }
    }
    
    return any_plugin_cleared;
  }
}

bool ClearCostmapService::clearRegion(
  const double reset_distance, bool invert,
  const std::vector<std::string> & plugins)
{
  double x, y;

  if (!getPosition(x, y)) {
    RCLCPP_ERROR(
      logger_, "%s",
      "Cannot clear map because robot pose cannot be retrieved.");
    return false;
  }

  auto layers = costmap_.getLayeredCostmap()->getPlugins();
  
  if (!plugins.empty()) {
    return validateAndClearPlugins(
      plugins, layers,
      [this, x, y, reset_distance, invert](std::shared_ptr<CostmapLayer> & layer) {
        clearLayerRegion(layer, x, y, reset_distance, invert);
      },
      "clear costmap region");
  } else {
    // Clear all clearable layers (default behavior)
    bool any_plugin_cleared = false;
    for (auto & layer : *layers) {
      if (layer->isClearable()) {
        auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
        clearLayerRegion(costmap_layer, x, y, reset_distance, invert);
        any_plugin_cleared = true;
      }
    }
    
    return any_plugin_cleared;
  }
}

void ClearCostmapService::clearLayerRegion(
  shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance,
  bool invert)
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  double start_point_x = pose_x - reset_distance / 2;
  double start_point_y = pose_y - reset_distance / 2;
  double end_point_x = start_point_x + reset_distance;
  double end_point_y = start_point_y + reset_distance;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

  costmap->clearArea(start_x, start_y, end_x, end_y, invert);

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
}

bool ClearCostmapService::clearEntirely(const std::vector<std::string> & plugins)
{
  if (plugins.empty()) {
    // Default behavior: clear all layers
    std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getCostmap()->getMutex()));
    RCLCPP_INFO(logger_, "Clearing all layers in costmap: %s", costmap_.getName().c_str());
    costmap_.resetLayers();
    return true;
  } else {
    // Clear only specified plugins
    std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getCostmap()->getMutex()));
    auto layers = costmap_.getLayeredCostmap()->getPlugins();
    
    bool result = validateAndClearPlugins(
      plugins, layers,
      [this](std::shared_ptr<CostmapLayer> & layer) {
        RCLCPP_INFO(logger_, "Clearing entire layer: %s", layer->getName().c_str());
        layer->resetMap(0, 0, layer->getSizeInCellsX(), layer->getSizeInCellsY());
      },
      "clear costmap entirely");
    
    if (result) {
      RCLCPP_INFO(logger_, "Resetting master costmap after plugin clearing");
      costmap_.getCostmap()->resetMap(0, 0,
        costmap_.getCostmap()->getSizeInCellsX(),
        costmap_.getCostmap()->getSizeInCellsY());
    }
    
    return result;
  }
}

void ClearCostmapService::validateAndCategorizePlugins(
  const std::vector<std::string> & requested_plugins,
  const std::vector<std::shared_ptr<Layer>> * layers,
  std::vector<std::string> & valid_plugins,
  std::vector<std::string> & invalid_plugins) const
{
  valid_plugins.clear();
  invalid_plugins.clear();
  
  for (const auto & requested_plugin : requested_plugins) {
    bool found = false;
    bool clearable = false;
    
    for (auto & layer : *layers) {
      if (layer->getName() == requested_plugin) {
        found = true;
        clearable = layer->isClearable();
        break;
      }
    }
    
    if (!found) {
      invalid_plugins.push_back(requested_plugin + " (not found)");
    } else if (!clearable) {
      invalid_plugins.push_back(requested_plugin + " (not clearable)");
    } else {
      valid_plugins.push_back(requested_plugin);
    }
  }
}

bool ClearCostmapService::validateAndClearPlugins(
  const std::vector<std::string> & plugins,
  const std::vector<std::shared_ptr<Layer>> * layers,
  std::function<void(std::shared_ptr<CostmapLayer> &)> clear_callback,
  const std::string & operation_name) const
{
  std::vector<std::string> invalid_plugins;
  std::vector<std::string> valid_plugins;
  
  // Validate all requested plugins
  validateAndCategorizePlugins(plugins, layers, valid_plugins, invalid_plugins);
  
  // Log validation errors
  if (!invalid_plugins.empty()) {
    std::string error_msg = "Invalid plugin(s) requested for clearing: ";
    for (size_t i = 0; i < invalid_plugins.size(); ++i) {
      error_msg += invalid_plugins[i];
      if (i < invalid_plugins.size() - 1) {
        error_msg += ", ";
      }
    }
    RCLCPP_ERROR(logger_, "%s", error_msg.c_str());
  }
  
  // Clear all valid plugins using the provided callback
  bool any_plugin_cleared = false;
  for (auto & layer : *layers) {
    if (std::find(valid_plugins.begin(), valid_plugins.end(), 
                  layer->getName()) != valid_plugins.end()) {
      auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
      clear_callback(costmap_layer);
      any_plugin_cleared = true;
    }
  }
  
  // Return failure if any requested plugin was invalid
  if (!invalid_plugins.empty()) {
    RCLCPP_ERROR(logger_, 
      "Failed to %s: %zu invalid plugin(s) out of %zu requested",
      operation_name.c_str(), invalid_plugins.size(), plugins.size());
    return false;
  }
  
  if (!any_plugin_cleared) {
    RCLCPP_ERROR(logger_, "No plugins were cleared in %s", operation_name.c_str());
    return false;
  }
  
  return true;
}

bool ClearCostmapService::shouldClearLayer(
  const std::shared_ptr<Layer> & layer,
  const std::vector<std::string> & plugins) const
{
  // If no specific plugins requested, use default behavior (clear all clearable layers)
  if (plugins.empty()) {
    return layer->isClearable();
  }

  // If specific plugins requested, check if this layer is in the list AND clearable
  bool is_in_plugin_list = std::find(plugins.begin(), plugins.end(),
    layer->getName()) != plugins.end();
  if (is_in_plugin_list && !layer->isClearable()) {
    RCLCPP_WARN(
      logger_,
      "Plugin '%s' was requested to be cleared but is not clearable. Skipping.",
      layer->getName().c_str());
    return false;
  }

  return is_in_plugin_list && layer->isClearable();
}

bool ClearCostmapService::getPosition(double & x, double & y) const
{
  geometry_msgs::msg::PoseStamped pose;
  if (!costmap_.getRobotPose(pose)) {
    return false;
  }

  x = pose.pose.position.x;
  y = pose.pose.position.y;

  return true;
}

}  // namespace nav2_costmap_2d
