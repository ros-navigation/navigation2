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
  const shared_ptr<ClearExceptRegion::Response>/*response*/)
{
  RCLCPP_INFO(
    logger_, "%s",
    ("Received request to clear except a region the " + costmap_.getName()).c_str());

  clearRegion(request->reset_distance, true, request->plugins);
}

void ClearCostmapService::clearAroundRobotCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearAroundRobot::Request> request,
  const shared_ptr<ClearAroundRobot::Response>/*response*/)
{
  clearRegion(request->reset_distance, false, request->plugins);
}

void ClearCostmapService::clearAroundPoseCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearAroundPose::Request> request,
  const shared_ptr<ClearAroundPose::Response>/*response*/)
{
  RCLCPP_INFO(
    logger_, "%s",
    ("Received request to clear around pose for " + costmap_.getName()).c_str());

  clearAroundPose(request->pose, request->reset_distance, request->plugins);
}

void ClearCostmapService::clearEntireCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ClearEntirely::Request> request,
  const std::shared_ptr<ClearEntirely::Response>/*response*/)
{
  RCLCPP_INFO(
    logger_, "%s",
    ("Received request to clear entirely the " + costmap_.getName()).c_str());

  clearEntirely(request->plugins);
}

void ClearCostmapService::clearAroundPose(
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
    return;
  }

  x = global_pose.pose.position.x;
  y = global_pose.pose.position.y;

  auto layers = costmap_.getLayeredCostmap()->getPlugins();

  for (auto & layer : *layers) {
    if (shouldClearLayer(layer, plugins)) {
      auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
      clearLayerRegion(costmap_layer, x, y, reset_distance, false);
    }
  }
}

void ClearCostmapService::clearRegion(
  const double reset_distance, bool invert,
  const std::vector<std::string> & plugins)
{
  double x, y;

  if (!getPosition(x, y)) {
    RCLCPP_ERROR(
      logger_, "%s",
      "Cannot clear map because robot pose cannot be retrieved.");
    return;
  }

  auto layers = costmap_.getLayeredCostmap()->getPlugins();

  for (auto & layer : *layers) {
    if (shouldClearLayer(layer, plugins)) {
      auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
      clearLayerRegion(costmap_layer, x, y, reset_distance, invert);
    }
  }

  // AlexeyMerzlyakov: No need to clear layer region for costmap filters
  // as they are always supposed to be not clearable.
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

void ClearCostmapService::clearEntirely(const std::vector<std::string> & plugins)
{
  if (plugins.empty()) {
    // Default behavior: clear all layers
    std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getCostmap()->getMutex()));
    RCLCPP_INFO(logger_, "Clearing all layers in costmap: %s", costmap_.getName().c_str());
    costmap_.resetLayers();
  } else {
    // Clear only specified plugins
    auto layers = costmap_.getLayeredCostmap()->getPlugins();
    for (auto & layer : *layers) {
      if (shouldClearLayer(layer, plugins)) {
        if (layer->isClearable()) {
          RCLCPP_INFO(logger_, "Clearing entire layer: %s", layer->getName().c_str());
          auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
          std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_layer->getMutex()));
          costmap_layer->resetMap(0, 0, costmap_layer->getSizeInCellsX(),
            costmap_layer->getSizeInCellsY());
        } else {
          RCLCPP_WARN(
            logger_,
            "Layer '%s' is not clearable, skipping.",
            layer->getName().c_str());
        }
      }
    }
  }
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
