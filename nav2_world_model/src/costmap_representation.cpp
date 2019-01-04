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

#include "nav2_world_model/costmap_representation.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_world_model
{

using nav2_costmap_2d::MapLocation;
using nav2_util::Point;

CostmapRepresentation::CostmapRepresentation(
  const std::string name,
  rclcpp::Node::SharedPtr & node,
  rclcpp::executor::Executor & executor,
  rclcpp::Clock::SharedPtr & clock)
: WorldRepresentation(name, node),
  clock_(clock),
  tfBuffer_(clock_),
  tfListener_(tfBuffer_),
  region_visualizer_(node_)
{
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name_, tfBuffer_);
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
  response.map.header.frame_id = "map";

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
  response.was_successful = checkIfFree(request);
  return response;
}

ProcessRegion::Response
CostmapRepresentation::clearArea(const ProcessRegion::Request & /*request*/)
{
  // TODO(orduno)
  ProcessRegion::Response response;
  response.was_successful = false;
  return response;
}

bool CostmapRepresentation::checkIfFree(const ProcessRegion::Request & request)
{
  std::vector<MapLocation> polygon_cells;

  // Get all the cell locations inside the region
  costmap_->convexFillCells(generateRectangleVertices(request), polygon_cells);

  // Check if there's at least one cell not free
  bool allFree = true;
  for (const auto & cell : polygon_cells) {
    if (!isFree(cell)) {
      allFree = false;
    }
  }

  region_visualizer_.publish(costmap_->getResolution());

  return allFree;
}

std::vector<MapLocation> CostmapRepresentation::generateRectangleVertices(
  const ProcessRegion::Request & request) const
{
   // Define the coordinates
  double top = request.reference.y + request.height / 2 - request.offset.y;
  double down = request.reference.y - request.height / 2 - request.offset.y;
  double right = request.reference.x + request.width / 2 - request.offset.x;
  double left = request.reference.x - request.width / 2 - request.offset.x;

  // Add the vertices
  std::vector<Point> points = {
    Point{left, down}, Point{left, top}, Point{right, top}, Point{right, down}};

  // TODO(orduno) X,y axis seem to be flipped between rviz and gazebo
  const double rvizToGazeboOffset = M_PI/2;

  // Rotate the vertices
  for (auto & point : points) {
    point.rotateAroundPoint(
      request.rotation + rvizToGazeboOffset,
      Point{request.reference.x, request.reference.y});
  }

  // Convert to map coordinates
  std::vector<MapLocation> vertices;

  for (const auto & point : points) {
    addVertex(vertices, point);
  }

  return vertices;
}

void CostmapRepresentation::addVertex(
  std::vector<MapLocation> & vertices, const Point & vertex) const
{
  unsigned int mx, my;
  costmap_->worldToMap(vertex.x, vertex.y, mx, my);
  vertices.push_back(MapLocation{mx, my});
}

bool CostmapRepresentation::isFree(const MapLocation & location)
{
  bool isFree = (costmap_->getCost(location.x, location.y)
    < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  double wx, wy;
  costmap_->mapToWorld(location.x, location.y, wx, wy);

  if (isFree) {
    region_visualizer_.addFreeCell(wx, wy, 0.0);
  }  else {
    region_visualizer_.addOccupiedCell(wx, wy, 0.0);
  }

  return isFree;
}

}  // namespace nav2_world_model
