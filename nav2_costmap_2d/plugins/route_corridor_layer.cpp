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
#include <stdexcept>
#include <string>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/robot_utils.hpp"

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

// Convert a cell index to a world coordinate (center of the cell).
double cell_to_world(int cell, double origin, double resolution)
{
  return origin + (cell + 0.5) * resolution;
}

void RouteCorridorLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RouteCorridorLayer: failed to lock node");
  }

  // Declare parameters
  enabled_ =                      node->declare_or_get_parameter(name_ + ".enabled", true);
  const auto path_topic =         node->declare_or_get_parameter<std::string>(
                                    name_ + ".path_topic", "/plan");
  const auto map_topic =          node->declare_or_get_parameter<std::string>(
                                    name_ + ".map_topic", "~/" + name_ + "/map");
  max_deviation_ =                node->declare_or_get_parameter(name_ + ".max_deviation", 1.0);
  const auto recovery_cost =      node->declare_or_get_parameter(name_ + ".recovery_cost", 200);
  const auto goal_status_topics = node->declare_or_get_parameter(
                                    name_ + ".goal_status_topics",
                                    std::vector<std::string>{
                                      "navigate_to_pose/_action/status", "navigate_through_poses/_action/status"});

  if (!std::isfinite(max_deviation_) || max_deviation_ <= 0.0) {
    throw std::invalid_argument("RouteCorridorLayer: width must be positive and finite");
  }
  if (recovery_cost < 1 || recovery_cost >= INSCRIBED_INFLATED_OBSTACLE) {
    throw std::invalid_argument("RouteCorridorLayer: recovery_cost must be in [1, 252]");
  }
  recovery_cost_ = static_cast<unsigned char>(recovery_cost);
  current_ = true;

  // Create corridor map publisher
  mask_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).transient_local().reliable());
  mask_pub_->on_activate();

  // Path subscriber and pathCallback
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    joinWithParentNamespace(path_topic),
    std::bind(&RouteCorridorLayer::pathCallback, this, std::placeholders::_1),
    rclcpp::QoS(1).reliable(), callback_group_);

  // goal status subscriber and goalStatusCallback: when goal is reached or aborted
  // clear the corridor and reset costmaps
  for (const auto & topic : goal_status_topics) {
    goal_status_subs_.push_back(node->create_subscription<action_msgs::msg::GoalStatusArray>(
        joinWithParentNamespace(topic),
        std::bind(&RouteCorridorLayer::goalStatusCallback, this, std::placeholders::_1),
        rclcpp::QoS(1).reliable().transient_local(), callback_group_));
  }
}

void RouteCorridorLayer::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
  std::vector<Segment> new_segments;
  geometry_msgs::msg::Point previous;
  RCLCPP_INFO(
  logger_, "Corridor received path: %zu poses, frame='%s'",
  path->poses.size(), path->header.frame_id.c_str());
  const auto & frame = layered_costmap_->getGlobalFrameID();
  for (size_t i = 0; i < path->poses.size(); ++i) {
    auto pose = path->poses[i];
    if (pose.header.frame_id.empty()) {
      pose.header = path->header;
    }
    if (pose.header.frame_id.empty() ||
      !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y))
    {
      RCLCPP_WARN(logger_, "Ignoring corridor path with missing frame or invalid coordinates");
      return;
    }
    if (pose.header.frame_id != frame) {
      geometry_msgs::msg::PoseStamped transformed;
      if (!nav2_util::transformPoseInTargetFrame(pose, transformed, *tf_, frame)) {
        RCLCPP_WARN(logger_, "Unable to transform corridor path to %s", frame.c_str());
        return;
      }
      pose = transformed;
    }
    const auto & point = pose.pose.position;
    if (i > 0) {
      new_segments.push_back({previous, point});
    }
    previous = point;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_finished_) {
    return;
  }
  path_segments_ = std::move(new_segments); //incoming path replaces the previous path
  built_ = false;
}

void RouteCorridorLayer::goalStatusCallback(
  const action_msgs::msg::GoalStatusArray::SharedPtr status)
{
  if (status->status_list.empty()) {
    return;
  }
  const auto stamp = [](const action_msgs::msg::GoalStatus & goal) {
      return rclcpp::Time(goal.goal_info.stamp).nanoseconds();
    };
  const auto newest = std::max_element(
    status->status_list.begin(), status->status_list.end(),
    [&stamp](const auto & a, const auto & b) {return stamp(a) < stamp(b);});
  std::lock_guard<std::mutex> lock(mutex_);
  const int64_t goal_stamp = stamp(*newest);
  if (goal_stamp < latest_goal_stamp_) {
    return;
  }
  const bool new_goal = latest_goal_stamp_ < 0 || newest->goal_info.goal_id.uuid != latest_goal_id_;
  if (new_goal) {
    latest_goal_id_ = newest->goal_info.goal_id.uuid;
    latest_goal_stamp_ = goal_stamp;
    goal_finished_ = false;
  }
  using action_msgs::msg::GoalStatus;
  if (goal_finished_ ||
    (newest->status != GoalStatus::STATUS_SUCCEEDED &&
    newest->status != GoalStatus::STATUS_ABORTED))
  {
    return;
  }
  goal_finished_ = true;
  goal_clear_pending_ = true;
  path_segments_.clear();
  boundary_indices_.clear();
  std::fill(corridor_cells_.begin(), corridor_cells_.end(), false);
  std::fill(mask_.data.begin(), mask_.data.end(), 0);
  built_ = false;
  if (!mask_.data.empty()) {
    mask_.header.stamp = clock_->now();
    mask_.info.map_load_time = mask_.header.stamp;
    mask_pub_->publish(mask_);
  }
  RCLCPP_INFO(
    logger_, "Navigation goal %s: cleared corridor mask; costmap reset queued",
    newest->status == GoalStatus::STATUS_SUCCEEDED ? "succeeded" : "aborted");
}

// Clear costmaps
void RouteCorridorLayer::clearCostmaps(Costmap2D & master)
{
  // The caller runs in the costmap update thread with the costmap locked.
  master.resetMap(0, 0, master.getSizeInCellsX(), master.getSizeInCellsY());
  for (const auto & plugin : *layered_costmap_->getPlugins()) {
    if (plugin.get() != this) {
      plugin->reset();
    }
  }
  for (const auto & filter : *layered_costmap_->getFilters()) {
    if (filter.get() != this) {
      filter->reset();
    }
  }
  goal_clear_pending_ = false;
}

void RouteCorridorLayer::updateCosts(
  Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
{
  // Assign high cost to the corridor
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }
  if (goal_clear_pending_) {
    clearCostmaps(master);
  }
  if (!built_ || mask_.info.width != master.getSizeInCellsX() ||
    mask_.info.height != master.getSizeInCellsY() ||
    mask_.info.resolution != static_cast<float>(master.getResolution()) ||
    mask_.info.origin.position.x != master.getOriginX() ||
    mask_.info.origin.position.y != master.getOriginY())
  {
    buildMask(master);
  }
  const unsigned char outside_cost = pose_valid_ && !insideCorridor(robot_x_, robot_y_) ?
    recovery_cost_ : LETHAL_OBSTACLE;
  // The mask is rebuilt whenever the master geometry changes, so indices match.
  for (const size_t index : boundary_indices_) {
    const int x = static_cast<int>(index % mask_.info.width);
    const int y = static_cast<int>(index / mask_.info.width);
    if (x >= min_i && x < max_i && y >= min_j && y < max_j) {
      // Hard restriction includes unknown space. Recovery preserves all source costs.
      master.setCost(x, y, outside_cost == LETHAL_OBSTACLE ? LETHAL_OBSTACLE :
        std::max(master.getCost(x, y), outside_cost));
    }
  }
}

void RouteCorridorLayer::buildMask(Costmap2D & master)
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
  corridor_cells_.assign(cell_count, false); // a boolean mask of corridor cells.
  std::vector<size_t> corridor_indices;
  // Fewer than two segments: publish a clear mask and reset costmaps.
  // boundary_indices_.clear();
  // if (path_segments_.size() < 2) {
  //   built_ = true;

  //   path_segments_.clear();
  //   mask_pub_->publish(mask_);
  //   RCLCPP_INFO(
  //     logger_, "Cleared corridor mask (%u x %u cells; excludes publishing)",
  //     mask_.info.width, mask_.info.height);
  //   clearCostmaps(master);
  //   return;
  // }
  for (const auto & segment : path_segments_) {
    const double dx = segment.end.x - segment.start.x;
    const double dy = segment.end.y - segment.start.y;
    const double length_sq = dx * dx + dy * dy;
    const int min_y = world_to_cell(
      std::min(segment.start.y, segment.end.y) - max_deviation_, origin_y, resolution, height);
    const int max_y = world_to_cell(
      std::max(segment.start.y, segment.end.y) + max_deviation_, origin_y, resolution, height);

    // Iterate only the regin near the segments to save compute. Mark corridor cells and cache their indices for later boundary expansion.
    for (int y = min_y; y < height && y <= max_y; ++y) {
      const double wy = cell_to_world(y, origin_y, resolution); // Center of the cell in world coordinates.
      double t_min = 0.0; 
      double t_max = 1.0;
      if (dy != 0.0) {
        const double t1 = (wy - max_deviation_ - segment.start.y) / dy;  
        const double t2 = (wy + max_deviation_ - segment.start.y) / dy;
        t_min = std::clamp(std::min(t1, t2), 0.0, 1.0); 
        t_max = std::clamp(std::max(t1, t2), 0.0, 1.0);
      }
      const double x1 = segment.start.x + t_min * dx;
      const double x2 = segment.start.x + t_max * dx;
      const int min_x = world_to_cell(
        std::min(x1, x2) - max_deviation_, origin_x, resolution, width);
      const int max_x = world_to_cell(
        std::max(x1, x2) + max_deviation_, origin_x, resolution, width);
      for (int x = min_x; x < width && x <= max_x; ++x) {
        const size_t index = static_cast<size_t>(y) * width + x;
        if (corridor_cells_[index]) {
          continue;
        }
        const double wx = cell_to_world(x, origin_x, resolution);
        const double t = length_sq > 0.0 ? std::clamp(
          ((wx - segment.start.x) * dx + (wy - segment.start.y) * dy) / length_sq,
          0.0, 1.0) : 0.0;
        if (std::hypot(wx - segment.start.x - t * dx, wy - segment.start.y - t * dy) <=
          max_deviation_)
        {
          corridor_cells_[index] = true;
          corridor_indices.push_back(index);
        }
      }
    }
  }
  // Expand from corridor cells for 3 pixels.
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
  // path_segments_.clear();  
  // Measure construction time.
  const double build_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - build_start).count();
  mask_pub_->publish(mask_);
  RCLCPP_INFO(
    logger_, "Built corridor mask in %.3f ms (%u x %u cells, "
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

// Update the bounds to include the entire master costmap
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
}  // namespace nav2_costmap_2d
