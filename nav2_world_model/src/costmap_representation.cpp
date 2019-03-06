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

#include <string>
#include <sstream>
#include <memory>
#include <vector>
#include <algorithm>

#include "nav2_world_model/costmap_representation.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/angleutils.hpp"

namespace nav2_world_model
{

using nav2_costmap_2d::MapLocation;
using nav2_util::Point2D;

CostmapRepresentation::CostmapRepresentation(
  const std::string name,
  rclcpp::Node::SharedPtr & node,
  rclcpp::executor::Executor & executor,
  std::string frame_id)
: WorldRepresentation(name, node, frame_id),
  region_visualizer_(node_, frame_id_)
{
  tf_node_ = rclcpp::Node::make_shared(name + "_tf_Node");
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(tf_node_->get_clock()),
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, tf_node_, true),
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name_, *tfBuffer_);
  costmap_ = costmap_ros_->getCostmap();
  executor.add_node(costmap_ros_);
}

GetCostmap::Response
CostmapRepresentation::getCostmap(const GetCostmap::Request & /*request*/)
{
  GetCostmap::Response response;
  response.map.metadata.size_x = costmap_->getSizeInCellsX();
  response.map.metadata.size_y = costmap_->getSizeInCellsY();
  response.map.metadata.resolution = costmap_->getResolution();
  response.map.metadata.layer = "Master";
  response.map.metadata.map_load_time = costmap_ros_->now();
  response.map.metadata.update_time = costmap_ros_->now();

  tf2::Quaternion quaternion;
  // TODO(bpwilcox): Grab correct orientation information
  quaternion.setRPY(0.0, 0.0, 0.0);  // set roll, pitch, yaw
  response.map.metadata.origin.position.x = costmap_->getOriginX();
  response.map.metadata.origin.position.y = costmap_->getOriginY();
  response.map.metadata.origin.position.z = 0.0;
  response.map.metadata.origin.orientation = tf2::toMsg(quaternion);

  response.map.header.stamp = costmap_ros_->now();
  response.map.header.frame_id = frame_id_;

  unsigned char * data = costmap_->getCharMap();
  auto data_length = response.map.metadata.size_x * response.map.metadata.size_y;
  response.map.data.resize(data_length);
  response.map.data.assign(data, data + data_length);

  return response;
}

ProcessRegion::Response
CostmapRepresentation::confirmFreeSpace(const ProcessRegion::Request & request)
{
  ProcessRegion::Response response;
  response.result = checkIfFree(request);
  return response;
}

bool CostmapRepresentation::checkIfFree(const ProcessRegion::Request & request)
{
  // Define the vertices of a rectangle
  std::vector<MapLocation> rectangle;
  if (!generateRectangleVertices(request, rectangle)) {
    RCLCPP_ERROR(node_->get_logger(),
      "Could not generate rectangle vertices. Checking if region is free failed");
    return false;
  }

  // Get all the cell locations inside the region
  std::vector<MapLocation> polygon_cells;
  costmap_->convexFillCells(rectangle, polygon_cells);

  if (polygon_cells.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
      "Could not generate rectangle cells. Checking if region is free failed");
    return false;
  }

  // Check if there is at least one cell not free and send to visualizer for display
  bool allFree = true;
  for (const auto & cell : polygon_cells) {
    double wx, wy;
    costmap_->mapToWorld(cell.x, cell.y, wx, wy);

    if (isFree(cell)) {
      region_visualizer_.addFreeCell(wx, wy, 0.0);
    } else {
      allFree = false;
      region_visualizer_.addOccupiedCell(wx, wy, 0.0);
    }
  }
  region_visualizer_.publish(costmap_->getResolution());

  return allFree;
}

bool CostmapRepresentation::generateRectangleVertices(
  const ProcessRegion::Request & request, std::vector<MapLocation> & map_locations) const
{
  // Define the vertices in world frame
  double left = std::max(request.region.corner.y, request.region.opposite_corner.y);
  double right = std::min(request.region.corner.y, request.region.opposite_corner.y);
  double up = std::max(request.region.corner.x, request.region.opposite_corner.x);
  double down = std::min(request.region.corner.x, request.region.opposite_corner.x);

  std::vector<Point2D> vertices = {
    Point2D{left, down}, Point2D{left, up}, Point2D{right, up}, Point2D{right, down}};

  // Convert the vertices to map coordinates
  for (const auto & point : vertices) {
    if (!addToMapLocations(map_locations, point)) {
      RCLCPP_ERROR(node_->get_logger(), "Point not added.");
      return false;
    }
  }

  return true;
}

bool CostmapRepresentation::addToMapLocations(
  std::vector<MapLocation> & locations, const Point2D & point) const
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(point.x, point.y, mx, my)) {
    std::ostringstream oss;
    oss << point;
    RCLCPP_ERROR(node_->get_logger(),
      "Conversion from world to map frame failed for " + oss.str());
    return false;
  }

  locations.push_back(MapLocation{mx, my});
  return true;
}

bool CostmapRepresentation::isFree(const MapLocation & location) const
{
  return costmap_->getCost(location.x, location.y) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

}  // namespace nav2_world_model
