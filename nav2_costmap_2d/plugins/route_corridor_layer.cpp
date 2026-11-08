// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_costmap_2d/route_corridor_layer.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include "pluginlib/class_list_macros.hpp"
#include "nlohmann/json.hpp"

namespace nav2_costmap_2d
{
void RouteCorridorLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RouteCorridorLayer: failed to lock node");
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("graph_filepath", rclcpp::ParameterValue(std::string("")));
  declareParameter("map_topic", rclcpp::ParameterValue("~/" + name_ + "/map"));
  declareParameter("max_deviation", rclcpp::ParameterValue(1.0));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".max_deviation", max_deviation_);
  if (!std::isfinite(max_deviation_) || max_deviation_ <= 0.0) {
    throw std::invalid_argument("RouteCorridorLayer: width must be positive and finite");
  }
  declareParameter("recovery_cost", rclcpp::ParameterValue(200));
  int recovery_cost = 200;
  node->get_parameter(name_ + ".recovery_cost", recovery_cost);
  if (recovery_cost < 1 || recovery_cost >= INSCRIBED_INFLATED_OBSTACLE) {
    throw std::invalid_argument("RouteCorridorLayer: recovery_cost must be in [1, 252]");
  }
  recovery_cost_ = static_cast<unsigned char>(recovery_cost);
  current_ = !enabled_;
  std::string map_topic;
  node->get_parameter(name_ + ".map_topic", map_topic);
  mask_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).transient_local().reliable());
  mask_pub_->on_activate();
  std::string graph_filepath;
  node->get_parameter(name_ + ".graph_filepath", graph_filepath);
  if (enabled_) {
    if (graph_filepath.empty()) {
      throw std::invalid_argument("RouteCorridorLayer: graph_filepath is required");
    }
    loadGraph(graph_filepath);
  }
}

void RouteCorridorLayer::loadGraph(const std::string & filepath)
{
  // Load every edge, including intermediate vertices, rather than a selected route.
  std::vector<Segment> segments;
  try {
    std::ifstream input(filepath);
    if (!input) {
      throw std::runtime_error("cannot open file");
    }
    const auto graph = nlohmann::json::parse(input);
    if (graph.at("type") != "FeatureCollection" || !graph.at("features").is_array()) {
      throw std::runtime_error("expected a GeoJSON FeatureCollection");
    }
    auto append_line = [&segments](const nlohmann::json & line) {
        if (!line.is_array() || line.size() < 2) {
          throw std::runtime_error("edge must contain at least two coordinates");
        }
        auto point = [](const nlohmann::json & coordinate) {
            if (!coordinate.is_array() || coordinate.size() < 2 ||
              !coordinate.at(0).is_number() || !coordinate.at(1).is_number())
            {
              throw std::runtime_error("expected numeric XY coordinates");
            }
            geometry_msgs::msg::Point result;
            result.x = coordinate.at(0).get<double>();
            result.y = coordinate.at(1).get<double>();
            if (!std::isfinite(result.x) || !std::isfinite(result.y)) {
              throw std::runtime_error("non-finite edge coordinates");
            }
            return result;
          };
        auto previous = point(line.at(0));
        for (size_t i = 1; i < line.size(); ++i) {
          auto next = point(line.at(i));
          Segment edge;
          edge.start = previous;
          edge.end = next;
          segments.push_back(edge);
          previous = next;
        }
      };
    for (const auto & feature : graph.at("features")) {
      const auto & geometry = feature.at("geometry");
      const auto type = geometry.at("type").get<std::string>();
      if (type == "LineString") {
        append_line(geometry.at("coordinates"));
      } else if (type == "MultiLineString") {
        const auto & lines = geometry.at("coordinates");
        if (!lines.is_array() || lines.empty()) {
          throw std::runtime_error("empty or invalid MultiLineString");
        }
        for (const auto & line : lines) {
          append_line(line);
        }
      } else if (type != "Point") {
        throw std::runtime_error("unsupported graph geometry: " + type);
      }
    }
    if (segments.empty()) {
      throw std::runtime_error("graph contains no edges");
    }
  } catch (const std::exception & error) {
    throw std::runtime_error("RouteCorridorLayer: failed to load " + filepath + ": " +
        error.what());
  }
  edges_ = std::move(segments);
  RCLCPP_INFO(logger_, "Loaded %zu corridor segments from %s",
    edges_.size(), filepath.c_str());
}

void RouteCorridorLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  pose_valid_ = std::isfinite(robot_x) && std::isfinite(robot_y);
  const auto * map = layered_costmap_->getCostmap();
  *min_x = std::min(*min_x, map->getOriginX());
  *min_y = std::min(*min_y, map->getOriginY());
  *max_x = std::max(*max_x, map->getOriginX() + map->getSizeInMetersX());
  *max_y = std::max(*max_y, map->getOriginY() + map->getSizeInMetersY());
}

void RouteCorridorLayer::buildMask(const Costmap2D & master)
{
  mask_.header.frame_id = layered_costmap_->getGlobalFrameID();
  mask_.header.stamp = clock_->now();
  mask_.info.map_load_time = mask_.header.stamp;
  mask_.info.resolution = master.getResolution();
  mask_.info.width = master.getSizeInCellsX();
  mask_.info.height = master.getSizeInCellsY();
  mask_.info.origin.position.x = master.getOriginX();
  mask_.info.origin.position.y = master.getOriginY();
  mask_.info.origin.orientation.w = 1.0;
  mask_.data.assign(static_cast<size_t>(mask_.info.width) * mask_.info.height, 100);
  for (unsigned int y = 0; y < mask_.info.height; ++y) {
    for (unsigned int x = 0; x < mask_.info.width; ++x) {
      double wx, wy;
      master.mapToWorld(x, y, wx, wy);
      for (const auto & edge : edges_) {
        const double dx = edge.end.x - edge.start.x;
        const double dy = edge.end.y - edge.start.y;
        const double length_sq = dx * dx + dy * dy;
        const double t = length_sq > 0.0 ? std::clamp(
          ((wx - edge.start.x) * dx + (wy - edge.start.y) * dy) / length_sq,
          0.0, 1.0) : 0.0;
        if (std::hypot(wx - edge.start.x - t * dx, wy - edge.start.y - t * dy) <=
          max_deviation_)
        {
          mask_.data[y * mask_.info.width + x] = 0;
          break;
        }
      }
    }
  }
  built_ = true;
  current_ = true;
  edges_.clear();  // Updates only sample the fixed mask.
  mask_pub_->publish(mask_);
  RCLCPP_INFO(logger_, "Built fixed startup corridor mask");
}

bool RouteCorridorLayer::insideCorridor(double wx, double wy) const
{
  if (!std::isfinite(wx) || !std::isfinite(wy)) {
    return false;
  }
  const double mx = std::floor((wx - mask_.info.origin.position.x) / mask_.info.resolution);
  const double my = std::floor((wy - mask_.info.origin.position.y) / mask_.info.resolution);
  return mx >= 0 && my >= 0 && mx < mask_.info.width && my < mask_.info.height &&
         mask_.data[static_cast<size_t>(my) * mask_.info.width + static_cast<size_t>(mx)] == 0;
}

void RouteCorridorLayer::updateCosts(
  Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }
  if (!built_) {
    buildMask(master);
  }
  const unsigned char outside_cost = pose_valid_ && !insideCorridor(robot_x_, robot_y_) ?
    recovery_cost_ : LETHAL_OBSTACLE;
  for (int y = std::max(0, min_j);
    y < std::min(max_j, static_cast < int > (master.getSizeInCellsY()));
    ++y)
  {
    for (int x = std::max(0, min_i);
      x < std::min(max_i, static_cast < int > (master.getSizeInCellsX()));
      ++x)
    {
      double wx, wy;
      master.mapToWorld(x, y, wx, wy);
      if (!insideCorridor(wx, wy)) {
        // Hard restriction includes unknown space. Recovery preserves all source costs.
        master.setCost(x, y, outside_cost == LETHAL_OBSTACLE ? LETHAL_OBSTACLE :
          std::max(master.getCost(x, y), outside_cost));
      }
    }
  }
}
}  // namespace nav2_costmap_2d

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::RouteCorridorLayer, nav2_costmap_2d::Layer)
