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

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace nav2_costmap_2d
{

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

  default_value_ = FREE_SPACE;
  matchSize();

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

void RouteCorridorLayer::matchSize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  CostmapLayer::matchSize();
  built_ = false;
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
  std::lock_guard<Costmap2D::mutex_t> grid_lock(*getMutex());
  resetMaps();
  built_ = false;
  RCLCPP_INFO(
    logger_, "Navigation goal %s: cleared corridor mask; costmap reset queued",
    newest->status == GoalStatus::STATUS_SUCCEEDED ? "succeeded" : "aborted");
}

// Clear costmaps
void RouteCorridorLayer::resetCorridorStates(Costmap2D & master)
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
  
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return;
  }
  if (goal_clear_pending_) {
    resetCorridorStates(master);
  }
  std::lock_guard<Costmap2D::mutex_t> grid_lock(*getMutex());
  if (!built_ || getSizeInCellsX() != master.getSizeInCellsX() ||
    getSizeInCellsY() != master.getSizeInCellsY() ||
    getResolution() != master.getResolution() ||
    getOriginX() != master.getOriginX() ||
    getOriginY() != master.getOriginY())
  {
    buildMask(master);
  }
  const unsigned char outside_cost = pose_valid_ && !insideCorridor(robot_x_, robot_y_) ? // Assign high cost to the corridor
    recovery_cost_ : LETHAL_OBSTACLE;
  // The mask is rebuilt whenever the master geometry changes, so indices match.
  for (const size_t index : boundary_indices_) {
    const int x = static_cast<int>(index % getSizeInCellsX());
    const int y = static_cast<int>(index / getSizeInCellsX());
    costmap_[index] = outside_cost;
    if (x >= min_i && x < max_i && y >= min_j && y < max_j) {
      
      master.setCost(x, y, outside_cost == LETHAL_OBSTACLE ? LETHAL_OBSTACLE :
        std::max(master.getCost(x, y), outside_cost));  //Preserve the original cost outside the corridor
    }
  }
}

void RouteCorridorLayer::buildMask(Costmap2D & master)
{
  const auto build_start = std::chrono::steady_clock::now();
  resizeMap(
    master.getSizeInCellsX(), master.getSizeInCellsY(), master.getResolution(),
    master.getOriginX(), master.getOriginY());
  const int width = static_cast<int>(getSizeInCellsX());
  const int height = static_cast<int>(getSizeInCellsY());
  const size_t cell_count = static_cast<size_t>(width) * height;
  corridor_cells_.assign(cell_count, false); // a boolean mask of corridor cells.
  std::vector<size_t> corridor_indices;
  // Fewer than two segments: publish a clear mask and reset costmaps.
  // boundary_indices_.clear();
  // if (path_segments_.size() < 2) {
  //   built_ = true;

  //   path_segments_.clear();
  //   RCLCPP_INFO(
  //     logger_, "Cleared corridor mask (%u x %u cells; excludes publishing)",
  //     getSizeInCellsX(), getSizeInCellsY());
  //   resetCorridorStates(master);
  //   return;
  // }
  for (const auto & segment : path_segments_) {
    const double dx = segment.end.x - segment.start.x;
    const double dy = segment.end.y - segment.start.y;
    const double length_sq = dx * dx + dy * dy;
    int unused_x, min_y, max_y;
    master.worldToMapEnforceBounds(
      master.getOriginX(), std::min(segment.start.y, segment.end.y) - max_deviation_,
      unused_x, min_y);
    master.worldToMapEnforceBounds(
      master.getOriginX(), std::max(segment.start.y, segment.end.y) + max_deviation_,
      unused_x, max_y);

    // Iterate only the regin near the segments to save compute. Mark corridor cells and cache their indices for later boundary expansion.
    for (int y = min_y; y < height && y <= max_y; ++y) {
      double wx, wy;
      master.mapToWorld(0, y, wx, wy); // Center of the cell in world coordinates.
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
      int min_x, max_x, unused_y;
      master.worldToMapEnforceBounds(
        std::min(x1, x2) - max_deviation_, wy, min_x, unused_y);
      master.worldToMapEnforceBounds(
        std::max(x1, x2) + max_deviation_, wy, max_x, unused_y);
      for (int x = min_x; x < width && x <= max_x; ++x) {
        const size_t index = static_cast<size_t>(y) * width + x;
        if (corridor_cells_[index]) {
          continue;
        }
        master.mapToWorld(x, y, wx, wy);
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
        if (!corridor_cells_[neighbor] && costmap_[neighbor] == FREE_SPACE) {
          costmap_[neighbor] = LETHAL_OBSTACLE;
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
  RCLCPP_INFO(
    logger_, "Built corridor mask in %.3f ms (%u x %u cells, "
    "%zu corridor cells, %zu boundary cells; excludes publishing)",
    build_ms, getSizeInCellsX(), getSizeInCellsY(),
    corridor_indices.size(), boundary_indices_.size());
}

bool RouteCorridorLayer::insideCorridor(double wx, double wy) const
{
  if (!std::isfinite(wx) || !std::isfinite(wy)) {
    return false;
  }
  unsigned int mx, my;
  const auto * costmap = layered_costmap_->getCostmap();
  if (!costmap->worldToMap(wx, wy, mx, my)) {
    return false;
  }
  const auto index = static_cast<std::size_t>(my) * getSizeInCellsX() + mx;
  return corridor_cells_[index];
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

// Reset clears path segments, empty both masks
void RouteCorridorLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  path_segments_.clear();
  built_ = false;
  // Not sure if needed, needs testing
  boundary_indices_.clear();
  std::fill(corridor_cells_.begin(), corridor_cells_.end(), false);
  std::lock_guard<Costmap2D::mutex_t> grid_lock(*getMutex());
  resetMaps();
}
}  // namespace nav2_costmap_2d
