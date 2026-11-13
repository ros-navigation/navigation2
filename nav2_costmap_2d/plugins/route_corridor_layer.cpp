// Copyright (c) 2026 Ye_X
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

#include "nav2_costmap_2d/route_corridor_layer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include "pluginlib/class_list_macros.hpp"
#include "nlohmann/json.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::RouteCorridorLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace nav2_costmap_2d
{

// Convert a world coordinate to a cell index.
int world_to_cell(double world, double origin, double resolution, int limit)
{
  int cell = std::clamp(std::ceil((world - origin) / resolution - 0.5), 0.0, static_cast<double>(limit));
  return cell;
}

void RouteCorridorLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RouteCorridorLayer: failed to lock node");
  }
  // Parameters
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
  // Load every edge from the GeoJSON file, splitting intermediate vertices into segments.
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

void RouteCorridorLayer::updateBounds(// The bound is the entire costmap
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
  const auto build_start = std::chrono::steady_clock::now();
  mask_.header.frame_id = layered_costmap_->getGlobalFrameID();
  mask_.header.stamp = clock_->now();
  mask_.info.map_load_time = mask_.header.stamp;
  mask_.info.resolution = master.getResolution();
  mask_.info.width = master.getSizeInCellsX();
  mask_.info.height = master.getSizeInCellsY();
  mask_.info.origin.position.x = master.getOriginX();
  mask_.info.origin.position.y = master.getOriginY();
  mask_.info.origin.orientation.w = 1.0;
  const int width = static_cast<int>(mask_.info.width);
  const int height = static_cast<int>(mask_.info.height);
  const double resolution = master.getResolution();
  const double origin_x = master.getOriginX();
  const double origin_y = master.getOriginY();
  const size_t cell_count = static_cast<size_t>(width) * height;
  mask_.data.assign(cell_count, 0); 
  corridor_cells_.assign(cell_count, false); // a boolean mask of corridor cells. For each route segment check only nearby cells
  std::vector<size_t> corridor_indices;
  for (const auto & edge : edges_) {
    const double dx = edge.end.x - edge.start.x;
    const double dy = edge.end.y - edge.start.y;
    const double length_sq = dx * dx + dy * dy;
    const int min_y = world_to_cell(
      std::min(edge.start.y, edge.end.y) - max_deviation_, origin_y, resolution, height);
    const int max_y = world_to_cell(
      std::max(edge.start.y, edge.end.y) + max_deviation_, origin_y, resolution, height);
    
      // Iterate only the regin near the edges to save compute. Mark corridor cells and cache their indices for later boundary expansion.
      for (int y = min_y; y < height && y <= max_y; ++y) {
      const double wy = origin_y + (y + 0.5) * resolution; // Center of the cell in world coordinates.
      double t_min = 0.0;
      double t_max = 1.0;
      if (dy != 0.0) {
        const double t1 = (wy - max_deviation_ - edge.start.y) / dy;
        const double t2 = (wy + max_deviation_ - edge.start.y) / dy;
        t_min = std::clamp(std::min(t1, t2), 0.0, 1.0);
        t_max = std::clamp(std::max(t1, t2), 0.0, 1.0);
      }
      const double x1 = edge.start.x + t_min * dx;
      const double x2 = edge.start.x + t_max * dx;
      const int min_x = world_to_cell(
        std::min(x1, x2) - max_deviation_, origin_x, resolution, width);
      const int max_x = world_to_cell(
        std::max(x1, x2) + max_deviation_, origin_x, resolution, width);
      for (int x = min_x; x < width && x <= max_x; ++x) {
        const size_t index = static_cast<size_t>(y) * width + x;
        if (corridor_cells_[index]) {
          continue;
        }
        const double wx = origin_x + (x + 0.5) * resolution;
        const double t = length_sq > 0.0 ? std::clamp(
          ((wx - edge.start.x) * dx + (wy - edge.start.y) * dy) / length_sq,
          0.0, 1.0) : 0.0;
        if (std::hypot(wx - edge.start.x - t * dx, wy - edge.start.y - t * dy) <=
          max_deviation_)
        {
          corridor_cells_[index] = true;
          corridor_indices.push_back(index);
        }
      }
    }
  }
  // Expand only from corridor cells and retain the exterior three-pixel band.
  constexpr int boundary_width = 3;
  boundary_indices_.clear();
  for (const size_t index : corridor_indices) {
    const int x = static_cast<int>(index % width);
    const int y = static_cast<int>(index / width);
    for (int dy = -boundary_width; dy <= boundary_width; ++dy) {
      for (int dx = -boundary_width; dx <= boundary_width; ++dx) {
        const int nx = x + dx;
        const int ny = y + dy;
        if (dx * dx + dy * dy > boundary_width * boundary_width ||
          nx < 0 || ny < 0 || nx >= width || ny >= height)
        {
          continue;
        }
        const size_t neighbor = static_cast<size_t>(ny) * width + nx;
        if (!corridor_cells_[neighbor] && mask_.data[neighbor] == 0) {
          mask_.data[neighbor] = 100;
          boundary_indices_.push_back(neighbor);
        }
      }
    }
  }
  built_ = true;
  current_ = true;
  edges_.clear();  // Updates only sample the fixed mask.
  // Measure construction time.
  const double build_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - build_start).count();
  mask_pub_->publish(mask_);
  RCLCPP_INFO(
    logger_, "Built fixed startup corridor mask in %.3f ms (%u x %u cells, "
    "%zu corridor cells, %zu boundary cells; excludes publishing)",
    build_ms, mask_.info.width, mask_.info.height,
    corridor_indices.size(), boundary_indices_.size());
}

bool RouteCorridorLayer::insideCorridor(double wx, double wy) const
{
  if (!std::isfinite(wx) || !std::isfinite(wy)) {
    return false;
  }
  const double mx = std::floor((wx - mask_.info.origin.position.x) / mask_.info.resolution);
  const double my = std::floor((wy - mask_.info.origin.position.y) / mask_.info.resolution);
  return mx >= 0 && my >= 0 && mx < mask_.info.width && my < mask_.info.height &&
         corridor_cells_[static_cast<size_t>(my) * mask_.info.width + static_cast<size_t>(mx)];
}

void RouteCorridorLayer::updateCosts(
  Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
{
  // Apply costs along the three-pixel exterior corridor boundary.
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }
  if (!built_) {
    buildMask(master);
  }
  const unsigned char outside_cost = pose_valid_ && !insideCorridor(robot_x_, robot_y_) ?
    recovery_cost_ : LETHAL_OBSTACLE;
  // Visit only cached boundary cells. Rasterize their world extents into the current
  // costmap so rolling origins and differing resolutions still sample the fixed mask.
  const double resolution = master.getResolution();
  for (const size_t index : boundary_indices_) {
    const double wx = mask_.info.origin.position.x +
      (index % mask_.info.width) * mask_.info.resolution;
    const double wy = mask_.info.origin.position.y +
      (index / mask_.info.width) * mask_.info.resolution;
    const int x_begin = std::max(min_i, world_to_cell(
      wx, master.getOriginX(), resolution, master.getSizeInCellsX()));
    const int x_end = std::min(max_i, world_to_cell(
      wx + mask_.info.resolution, master.getOriginX(), resolution, master.getSizeInCellsX()));
    const int y_begin = std::max(min_j, world_to_cell(
      wy, master.getOriginY(), resolution, master.getSizeInCellsY()));
    const int y_end = std::min(max_j, world_to_cell(
      wy + mask_.info.resolution, master.getOriginY(), resolution, master.getSizeInCellsY()));
    for (int y = y_begin; y < y_end; ++y) {
      for (int x = x_begin; x < x_end; ++x) {
        // Hard restriction includes unknown space. Recovery preserves all source costs.
        master.setCost(x, y, outside_cost == LETHAL_OBSTACLE ? LETHAL_OBSTACLE :
          std::max(master.getCost(x, y), outside_cost));
      }
    }
  }
}
}  // namespace nav2_costmap_2d
