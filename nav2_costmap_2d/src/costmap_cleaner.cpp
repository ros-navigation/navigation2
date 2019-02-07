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

#include "nav2_costmap_2d/costmap_cleaner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_costmap_2d
{

using std::vector;
using std::string;
using std::shared_ptr;
using std::any_of;

CostmapCleaner::CostmapCleaner(rclcpp::Node::SharedPtr & node, Costmap2DROS * costmap)
: node_(node), costmap_(costmap)
{
  node_->get_parameter_or<double>("reset_distance", reset_distance_, 3.0);

  node_->get_parameter_or_set("cleanable_layers", cleanable_layers_, {"obstacle_layer"});

  server_ = node_->create_service<nav2_msgs::srv::CleanCostmap>("Clean" + costmap->getName(),
      std::bind(&CostmapCleaner::cleanCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void CostmapCleaner::cleanCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<nav2_msgs::srv::CleanCostmap::Request>/*request*/,
  const shared_ptr<nav2_msgs::srv::CleanCostmap::Response>/*response*/)
{
  RCLCPP_INFO(node_->get_logger(), "Received request to clean " + costmap_->getName());

  if (costmap_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "Costmap is undefined. Doing nothing.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Will proceed with cleaning the costmap");

  clean();
}

void CostmapCleaner::clean()
{
  double x, y;

  if (!getPose(x, y)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot clean map because robot pose cannot be retrieved.");
    return;
  }

  auto layers = costmap_->getLayeredCostmap()->getPlugins();

  for (auto & layer : *layers) {
    auto name = getLayerName(*layer);

    if (any_of(
        begin(cleanable_layers_), end(cleanable_layers_), [&name](auto l) {return l == name;}))
    {
      auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
      cleanLayer(costmap_layer, x, y);
    }
  }
}

void CostmapCleaner::cleanLayer(shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y)
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  double start_point_x = pose_x - reset_distance_ / 2;
  double start_point_y = pose_y - reset_distance_ / 2;
  double end_point_x = start_point_x + reset_distance_;
  double end_point_y = start_point_y + reset_distance_;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

  unsigned char * grid = costmap->getCharMap();

  for (int x = 0; x < static_cast<int>(costmap->getSizeInCellsX()); x++) {
    bool isOutXrange = x<start_x && x> end_x;

    for (int y = 0; y < static_cast<int>(costmap->getSizeInCellsY()); y++) {
      bool isOutYrange = y<start_y && y> end_y;

      if (isOutXrange && isOutYrange) {
        int index = costmap->getIndex(x, y);

        if (grid[index] != NO_INFORMATION) {
          grid[index] = NO_INFORMATION;
        }
      }
    }
  }

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
}

bool CostmapCleaner::getPose(double & x, double & y) const
{
  geometry_msgs::msg::PoseStamped pose;
  if (!costmap_->getRobotPose(pose)) {
    return false;
  }

  x = pose.pose.position.x;
  y = pose.pose.position.y;

  return true;
}

string CostmapCleaner::getLayerName(const Layer & layer) const
{
  string name = layer.getName();

  int slash = name.rfind('/');

  if (slash != std::string::npos) {
    name = name.substr(slash + 1);
  }

  return name;
}

}  // namespace nav2_costmap_2d
