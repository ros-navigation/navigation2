/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Marc Blöchlinger
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Marc Blöchlinger
 *         Eitan Marder-Eppstein (Original author)
 *         David V. Lu!! (Original author)
 *********************************************************************/
#include "nav2_costmap_2d/asymmetric_inflation_layer.hpp"

#include <limits>
#include <vector>
#include <algorithm>
#include <utility>
#include <unordered_map>
#include <cstdint>
#include <cmath>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "nav2_ros_common/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::AsymmetricInflationLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

AsymmetricInflationLayer::AsymmetricInflationLayer()
: cost_scaling_factor_left_(0),
  cost_scaling_factor_right_(0),
  goal_distance_threshold_(0),
  cache_length_(0)
{
}

void
AsymmetricInflationLayer::onInitialize()
{
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);
    inflation_radius_ = node->declare_or_get_parameter(
      name_ + "." + "inflation_radius", 2.0);
    inflate_unknown_ = node->declare_or_get_parameter(name_ + "." + "inflate_unknown", false);
    inflate_around_unknown_ = node->declare_or_get_parameter(
      name_ + "." + "inflate_around_unknown", false);
    num_threads_ = node->declare_or_get_parameter(
      name_ + "." + "num_threads", -1);
    cost_scaling_factor_left_ = node->declare_or_get_parameter(
      name_ + "." + "cost_scaling_factor_left", 4.0);
    cost_scaling_factor_right_ = node->declare_or_get_parameter(
      name_ + "." + "cost_scaling_factor_right", 4.0);
    plan_topic_ = node->declare_or_get_parameter<std::string>(
      name_ + "." + "plan_topic", "plan");
    goal_distance_threshold_ = node->declare_or_get_parameter(
      name_ + "." + "goal_distance_threshold", 1.5);

    // Apply the same bound checks as dynamic reconfigure, so bad YAML values fail
    // loudly at startup instead of silently producing bad costmaps.
    if (inflation_radius_ <= 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: inflation_radius must be > 0");
    }
    if (cost_scaling_factor_left_ <= 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: cost_scaling_factor_left must be > 0");
    }
    if (cost_scaling_factor_right_ <= 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: cost_scaling_factor_right must be > 0");
    }
    if (goal_distance_threshold_ < 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: goal_distance_threshold must be >= 0");
    }
    if (num_threads_ < -1) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: num_threads must be -1 (auto) or > 0");
    }

    cost_scaling_factor_ =
      std::max(cost_scaling_factor_left_, cost_scaling_factor_right_);

    plan_topic_ = joinWithParentNamespace(plan_topic_);
    path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      plan_topic_,
      std::bind(
        &AsymmetricInflationLayer::globalPathCallback,
        this, std::placeholders::_1),
      rclcpp::QoS(1).durability_volatile());
  }

  setCurrent(true);
  seen_roi_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_reinflation_ = false;
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  matchSize();
}

void
AsymmetricInflationLayer::activate()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &AsymmetricInflationLayer::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &AsymmetricInflationLayer::updateParametersCallback,
      this, std::placeholders::_1));
}

void
AsymmetricInflationLayer::deactivate()
{
  auto node = node_.lock();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
}

void
AsymmetricInflationLayer::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    latest_global_path_ = msg;
  }
  // Path change invalidates all asymmetric costs in the costmap.
  // Force a full-map reinflation on the next update cycle.
  need_reinflation_ = true;
  setCurrent(false);
}

void
AsymmetricInflationLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  InflationLayer::matchSize();

  computeAsymmetricCaches();
}

void
AsymmetricInflationLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // Track robot pose for the goal-proximity fallback check
  current_robot_x_ = robot_x;
  current_robot_y_ = robot_y;

  InflationLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
AsymmetricInflationLayer::onFootprintChanged()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  InflationLayer::onFootprintChanged();
  computeAsymmetricCaches();
}

std::vector<std::pair<double, double>>
AsymmetricInflationLayer::extractLocalPath(
  nav2_costmap_2d::Costmap2D & master_grid)
{
  std::vector<std::pair<double, double>> local_path_pts;
  nav_msgs::msg::Path current_path;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!latest_global_path_ || latest_global_path_->poses.empty()) {
      return local_path_pts;
    }
    current_path = *latest_global_path_;
  }

  // Check if the path is already in costmap frame
  std::string global_frame = layered_costmap_->getGlobalFrameID();
  std::string path_frame = current_path.header.frame_id;
  geometry_msgs::msg::TransformStamped transform;
  bool need_transform = (global_frame != path_frame && !path_frame.empty());

  // Find the transform from path frame to costmap frame (e.g., map -> odom)
  if (need_transform) {
    try {
      transform = tf_->lookupTransform(global_frame, path_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        logger_,
        "AsymmetricInflationLayer: TF lookup failed (%s -> %s): %s. "
        "Falling back to symmetric inflation.",
        path_frame.c_str(), global_frame.c_str(), ex.what());
      return local_path_pts;
    }
  }

  // Disable asymmetry near the goal to prevent target oscillations
  geometry_msgs::msg::PoseStamped goal_pose = current_path.poses.back();
  if (need_transform) {
    tf2::doTransform(goal_pose, goal_pose, transform);
  }

  double dist_to_goal = std::hypot(
    goal_pose.pose.position.x - current_robot_x_,
    goal_pose.pose.position.y - current_robot_y_);

  if (dist_to_goal <= goal_distance_threshold_) {
    // Empty vector causes algorithm to use standard symmetry
    return local_path_pts;
  }

  // Extract local path
  for (const auto & pose : current_path.poses) {
    geometry_msgs::msg::PoseStamped transformed_pose = pose;
    if (need_transform) {
      tf2::doTransform(pose, transformed_pose, transform);
    }

    double px = transformed_pose.pose.position.x;
    double py = transformed_pose.pose.position.y;
    unsigned int mx, my;

    // Only process points inside our current local costmap window
    if (master_grid.worldToMap(px, py, mx, my)) {
      local_path_pts.push_back({px, py});
    }
  }
  return local_path_pts;
}

int8_t
AsymmetricInflationLayer::computeObstacleSide(
  double cx, double cy,
  const std::vector<size_t> & candidates,
  const std::vector<std::pair<double, double>> & local_path_pts)
{
  const double inflation_radius_sq = inflation_radius_ * inflation_radius_;

  double min_dist_sq = std::numeric_limits<double>::max();
  double best_cross = 0.0;

  // Evaluate candidate segments provided by the spatial hash.
  for (size_t p : candidates) {
    // Define segment endpoints A (start) and B (end).
    double ax = local_path_pts[p].first;
    double ay = local_path_pts[p].second;
    double bx = local_path_pts[p + 1].first;
    double by = local_path_pts[p + 1].second;

    // Skip cells outside of the segment bounding box expanded by the inflation radius.
    double min_x = std::min(ax, bx) - inflation_radius_;
    double max_x = std::max(ax, bx) + inflation_radius_;
    if (cx < min_x || cx > max_x) {continue;}

    double min_y = std::min(ay, by) - inflation_radius_;
    double max_y = std::max(ay, by) + inflation_radius_;
    if (cy < min_y || cy > max_y) {continue;}

    // Calculate distance to path segment and orientation via cross product.
    // Vectors: AB (path segment) and AC (path to cell)
    double abx = bx - ax;
    double aby = by - ay;
    double len_sq = abx * abx + aby * aby;

    double acx = cx - ax;
    double acy = cy - ay;

    double dist_sq;
    double cross;

    // Prevent division by zero for zero-length segments
    if (len_sq < 1e-10) {
      dist_sq = acx * acx + acy * acy;
      cross = 0.0;
    } else {
      // 't': Scalar projection of C onto AB, clamped to segment bounds [0, 1]
      double t = std::clamp((acx * abx + acy * aby) / len_sq, 0.0, 1.0);

      // Vector from the projected point on AB to C
      double proj_dx = acx - t * abx;
      double proj_dy = acy - t * aby;
      dist_sq = proj_dx * proj_dx + proj_dy * proj_dy;

      // 2D cross product for orientation (Positive = Left, Negative = Right)
      cross = abx * acy - aby * acx;
    }

    // Update if shortest distance so far
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_cross = cross;
    }
  }

  // Check if the cell is outside the inflation radius.
  if (min_dist_sq > inflation_radius_sq) {
    return 0;
  }

  // Return the orientation based on the cross product of the closest segment.
  if (best_cross > 0.0) {return 1;}    // Left
  if (best_cross < 0.0) {return -1;}   // Right

  return 0;  // Neutral/On the line
}

void
AsymmetricInflationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // Abort inflation if it's disabled
  if (!enabled_) {
    return;
  }

  // Fill the symmetric inflation baseline, then apply asymmetric inflation afterwards.
  InflationLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);

  // Extract path that's inside the costmap
  std::vector<std::pair<double, double>> local_path_pts = extractLocalPath(master_grid);

  // Abort if we don't have a valid path or if the scaling rates are equal (no asymmetry).
  if (!((local_path_pts.size() >= 2) &&
    (cost_scaling_factor_left_ != cost_scaling_factor_right_)))
  {
    setCurrent(true);
    return;
  }

  // Build a spatial hash to map nearby segments to a cell
  auto spatial_hash = buildPathSpatialHash(local_path_pts);

  // Seed the asymmetric BFS by classifying boundary obstacle cells
  seedAsymmetricBFS(master_grid, min_i, min_j, max_i, max_j, spatial_hash, local_path_pts);

  // Run Dial's Algorithm BFS to propagate asymmetric inflation costs
  runAsymmetricBFS(master_grid, min_i, min_j, max_i, max_j);

  setCurrent(true);
}

std::unordered_map<uint64_t, std::vector<size_t>>
AsymmetricInflationLayer::buildPathSpatialHash(
  const std::vector<std::pair<double, double>> & local_path_pts)
{
  std::unordered_map<uint64_t, std::vector<size_t>> spatial_hash;

  for (size_t p = 0; p < local_path_pts.size() - 1; ++p) {
    // Create segment AB from consecutive path points
    double ax = local_path_pts[p].first;
    double ay = local_path_pts[p].second;
    double bx = local_path_pts[p + 1].first;
    double by = local_path_pts[p + 1].second;

    // Pad the segment's bounding box by the inflation radius.
    double min_x = std::min(ax, bx) - inflation_radius_;
    double max_x = std::max(ax, bx) + inflation_radius_;
    double min_y = std::min(ay, by) - inflation_radius_;
    double max_y = std::max(ay, by) + inflation_radius_;

    // Find which buckets this padded segment touches
    int64_t min_bx = static_cast<int64_t>(std::floor(min_x / inflation_radius_));
    int64_t max_bx = static_cast<int64_t>(std::floor(max_x / inflation_radius_));
    int64_t min_by = static_cast<int64_t>(std::floor(min_y / inflation_radius_));
    int64_t max_by = static_cast<int64_t>(std::floor(max_y / inflation_radius_));

    for (int64_t b_x = min_bx; b_x <= max_bx; ++b_x) {
      for (int64_t b_y = min_by; b_y <= max_by; ++b_y) {
        // Bitwise magic to safely map 2D signed coordinates into a 1D unsigned 64-bit key
        uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(b_x)) << 32) |
          (static_cast<uint32_t>(b_y));

        spatial_hash[key].push_back(p);
      }
    }
  }

  return spatial_hash;
}

void
AsymmetricInflationLayer::seedAsymmetricBFS(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const std::unordered_map<uint64_t, std::vector<size_t>> & spatial_hash,
  const std::vector<std::pair<double, double>> & local_path_pts)
{
  unsigned char * master_array = master_grid.getCharMap();
  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());

  // Inflate update window by inflation radius.
  roi_min_i_ = std::max(0, min_i - static_cast<int>(cell_inflation_radius_));
  roi_min_j_ = std::max(0, min_j - static_cast<int>(cell_inflation_radius_));
  const int roi_max_i = std::min(size_x, max_i + static_cast<int>(cell_inflation_radius_));
  const int roi_max_j = std::min(size_y, max_j + static_cast<int>(cell_inflation_radius_));
  roi_width_ = roi_max_i - roi_min_i_;
  roi_height_ = roi_max_j - roi_min_j_;

  seen_roi_.assign(static_cast<size_t>(roi_width_) * static_cast<size_t>(roi_height_), 0u);

  // Helper function to check if a neighbor is "traversable" (i.e., open space)
  auto is_traversable = [&](int nx, int ny) {
      unsigned char n_cost = master_array[master_grid.getIndex(nx, ny)];
      if (inflate_around_unknown_) {
        return  n_cost != LETHAL_OBSTACLE && n_cost != NO_INFORMATION;
      } else {
        return  n_cost != LETHAL_OBSTACLE;
      }
    };

  // The symmetric inflation pass has already applied the correct costs to the favored side.
  // Therefore we now only need to seed the BFS with cells from the disfavored side.
  int8_t disfavored_side = (cost_scaling_factor_left_ < cost_scaling_factor_right_) ? 1 : -1;

  // Seed all obstacle boundary cells, that are nearby a path segment
  for (int j = roi_min_j_; j < roi_max_j; j++) {
    for (int i = roi_min_i_; i < roi_max_i; i++) {
      unsigned char cost = master_array[master_grid.getIndex(i, j)];

      // Early exit 1: Skip cells that aren't lethal/unknown obstacles
      if (cost != LETHAL_OBSTACLE && !(inflate_around_unknown_ && cost == NO_INFORMATION)) {
        continue;
      }

      // Check if the cell touches the absolute edges of the costmap
      bool is_on_map_edge = (i == 0 || i == size_x - 1 || j == 0 || j == size_y - 1);

      // An obstacle cell is a boundary if it's on the map edge OR touches free space.
      bool is_obstacle_boundary = is_on_map_edge ||
        is_traversable(i - 1, j) ||
        is_traversable(i + 1, j) ||
        is_traversable(i, j - 1) ||
        is_traversable(i, j + 1);

      // Early exit 2: Skip interior obstacle cells, mark as seen so we don't enqueue them
      if (!is_obstacle_boundary) {
        seen_roi_[(j - roi_min_j_) * roi_width_ + (i - roi_min_i_)] = 1u;
        continue;
      }

      // Find segments that are nearby this cell using the spatial hash
      double cx, cy;
      master_grid.mapToWorld(i, j, cx, cy);
      int64_t b_x = static_cast<int64_t>(std::floor(cx / inflation_radius_));
      int64_t b_y = static_cast<int64_t>(std::floor(cy / inflation_radius_));

      uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(b_x)) << 32) |
        static_cast<uint32_t>(b_y);

      auto candidate_segments = spatial_hash.find(key);

      // Early exit 3: Skip cells with no nearby path segments
      if (candidate_segments == spatial_hash.end()) {continue;}

      // Determine which side of the path this cell is on
      int8_t side = computeObstacleSide(cx, cy, candidate_segments->second, local_path_pts);

      // Only enqueue boundary cells on the disfavored side of the path.
      // Cells on favored side already got correctly inflated during the symmetric inflation pass.
      if (side != 0 && side == disfavored_side) {
        inflation_cells_[0].emplace_back(i, j, i, j, side);
      }
    }
  }
}

void
AsymmetricInflationLayer::runAsymmetricBFS(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  // Cells are enqueued by effective distance, combining true distance with path-side weighting.
  for (size_t current_bin = 0; current_bin < inflation_cells_.size(); ++current_bin) {
    while (!inflation_cells_[current_bin].empty()) {
      const AsymmetricCellData cell = inflation_cells_[current_bin].back();
      inflation_cells_[current_bin].pop_back();

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int src_x = cell.src_x_;
      unsigned int src_y = cell.src_y_;
      unsigned int index = master_grid.getIndex(mx, my);

      // Get index
      const int roi_x = static_cast<int>(mx) - roi_min_i_;
      const int roi_y = static_cast<int>(my) - roi_min_j_;
      const size_t roi_idx =
        static_cast<size_t>(roi_y) * static_cast<size_t>(roi_width_) +
        static_cast<size_t>(roi_x);

      // Skip cells we've already visited
      if (seen_roi_[roi_idx]) {
        continue;
      }
      seen_roi_[roi_idx] = 1u;

      // path_side is carried inline in the queue entry — no global array lookup needed.
      int8_t path_side = cell.path_side_;

      // Get effective distance to categorise the cell into a cost bin.
      // This is where the asymmetry happens: each side uses a different effective distance.
      double physical_dist = asymmetricDistanceLookup(mx, my, src_x, src_y);
      double eff_dist = getEffectiveDistance(physical_dist, path_side);
      size_t bin = static_cast<size_t>(eff_dist * kEffDistPrecision);

      unsigned char new_cost = (bin < cached_costs_.size()) ? cached_costs_[bin] : 0;
      unsigned char old_cost = master_array[index];

      // Skip cells, where the new cost is less than the old cost
      // This means we reached the Voronoi boundary and can stop expanding
      if (new_cost < old_cost) {
        continue;
      }

      // Write costs only within the originally requested window (min/max)
      if (new_cost > old_cost) {
        if (static_cast<int>(mx) >= min_i &&
          static_cast<int>(my) >= min_j &&
          static_cast<int>(mx) < max_i &&
          static_cast<int>(my) < max_j)
        {
          master_array[index] = new_cost;
        }
      }

      // Propagate to unvisited neighbors within the ROI.
      // enqueueAsymmetric() returns the lowest non-empty bin after insertion.
      // This ensures that we always expand the lowest effective distance wavefront next.
      if (roi_x > 0 && mx > 0 && !seen_roi_[roi_y * roi_width_ + (roi_x - 1)]) {
        current_bin = enqueueAsymmetric(mx - 1, my, src_x, src_y, path_side, current_bin);
      }
      if (roi_y > 0 && my > 0 && !seen_roi_[(roi_y - 1) * roi_width_ + roi_x]) {
        current_bin = enqueueAsymmetric(mx, my - 1, src_x, src_y, path_side, current_bin);
      }
      if (roi_x < roi_width_ - 1 && mx < size_x - 1 &&
        !seen_roi_[roi_y * roi_width_ + (roi_x + 1)])
      {
        current_bin = enqueueAsymmetric(mx + 1, my, src_x, src_y, path_side, current_bin);
      }
      if (roi_y < roi_height_ - 1 && my < size_y - 1 &&
        !seen_roi_[(roi_y + 1) * roi_width_ + roi_x])
      {
        current_bin = enqueueAsymmetric(mx, my + 1, src_x, src_y, path_side, current_bin);
      }
    }
  }
}

size_t
AsymmetricInflationLayer::enqueueAsymmetric(
  unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y, int8_t path_side, size_t current_bin)
{
  double physical_dist = asymmetricDistanceLookup(mx, my, src_x, src_y);

  // Stop propagating the wave at the configured inflation radius
  if (physical_dist > cell_inflation_radius_) {
    return current_bin;
  }

  // Map the effective (asymmetric) distance to a priority queue bin.
  // This allows disfavored-side obstacles to claim contested cells first,
  // shifting the Voronoi boundary toward the disfavored side.
  double eff_dist = getEffectiveDistance(physical_dist, path_side);
  size_t target_bin = static_cast<size_t>(eff_dist * kEffDistPrecision);
  inflation_cells_[target_bin].emplace_back(mx, my, src_x, src_y, path_side);

  // Return the lowest bin, to ensure we always expand the lowest effective distance wavefront.
  return std::min(target_bin, current_bin);
}

void
AsymmetricInflationLayer::computeAsymmetricCaches()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  // If the inflation radius has changed -> update the distance cache.
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    cached_cell_inflation_radius_ = cell_inflation_radius_;

    cache_length_ = cell_inflation_radius_ + 2;
    cached_distances_.resize(cache_length_ * cache_length_);
    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        cached_distances_[i * cache_length_ + j] = hypot(i, j);
      }
    }
  }

  // Recompute the cost cache
  double max_eff_dist = static_cast<double>(cell_inflation_radius_);
  size_t max_bin = static_cast<size_t>(std::ceil(max_eff_dist * kEffDistPrecision)) + 2;

  cached_costs_.resize(max_bin);
  for (size_t i = 0; i < max_bin; ++i) {
    double eff_dist = static_cast<double>(i) / kEffDistPrecision;
    cached_costs_[i] = computeCost(eff_dist);
  }

  // Reinitialize the inflation_cells buckets
  inflation_cells_.clear();
  inflation_cells_.resize(max_bin);
}

rcl_interfaces::msg::SetParametersResult
AsymmetricInflationLayer::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE &&
      param_name == name_ + ".inflation_radius")
    {
      if (parameter.as_double() <= 0.0) {
        RCLCPP_WARN(
          logger_, "inflation_radius must be > 0. Rejecting parameter update.");
        result.successful = false;
        result.reason = "inflation_radius must be > 0";
        return result;
      }
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE &&
      param_name == name_ + ".cost_scaling_factor_left")
    {
      if (parameter.as_double() <= 0.0) {
        RCLCPP_WARN(
          logger_, "cost_scaling_factor_left must be > 0. Rejecting parameter update.");
        result.successful = false;
        result.reason = "cost_scaling_factor_left must be > 0";
        return result;
      }
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE &&
      param_name == name_ + ".cost_scaling_factor_right")
    {
      if (parameter.as_double() <= 0.0) {
        RCLCPP_WARN(
          logger_, "cost_scaling_factor_right must be > 0. Rejecting parameter update.");
        result.successful = false;
        result.reason = "cost_scaling_factor_right must be > 0";
        return result;
      }
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE &&
      param_name == name_ + ".goal_distance_threshold")
    {
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
          logger_, "goal_distance_threshold must be >= 0. Rejecting parameter update.");
        result.successful = false;
        result.reason = "goal_distance_threshold must be >= 0";
        return result;
      }
      continue;
    }

    if (param_type == ParameterType::PARAMETER_INTEGER &&
      param_name == name_ + ".num_threads")
    {
      if (parameter.as_int() < -1) {
        RCLCPP_WARN(
          logger_, "num_threads must be -1 (auto) or > 0. Rejecting parameter update.");
        result.successful = false;
        result.reason = "num_threads must be -1 (auto) or > 0";
        return result;
      }
    }
  }

  return result;
}

void
AsymmetricInflationLayer::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  bool need_cache_recompute = false;
  bool side_scaling_changed = false;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + ".inflation_radius" &&
        inflation_radius_ != parameter.as_double())
      {
        inflation_radius_ = parameter.as_double();
        need_reinflation_ = true;
        need_cache_recompute = true;
        setCurrent(false);
      } else if (param_name == name_ + ".cost_scaling_factor_left" &&  // NOLINT
        cost_scaling_factor_left_ != parameter.as_double())
      {
        cost_scaling_factor_left_ = parameter.as_double();
        side_scaling_changed = true;
      } else if (param_name == name_ + ".cost_scaling_factor_right" &&  // NOLINT
        cost_scaling_factor_right_ != parameter.as_double())
      {
        cost_scaling_factor_right_ = parameter.as_double();
        side_scaling_changed = true;
      } else if (param_name == name_ + ".goal_distance_threshold" &&  // NOLINT
        goal_distance_threshold_ != parameter.as_double())
      {
        goal_distance_threshold_ = parameter.as_double();
        need_reinflation_ = true;
        setCurrent(false);
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + ".enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        need_reinflation_ = true;
        setCurrent(false);
      } else if (param_name == name_ + ".inflate_around_unknown" &&  // NOLINT
        inflate_around_unknown_ != parameter.as_bool())
      {
        inflate_around_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
        setCurrent(false);
      } else if (param_name == name_ + ".inflate_unknown" &&  // NOLINT
        inflate_unknown_ != parameter.as_bool())
      {
        inflate_unknown_ = parameter.as_bool();
        need_reinflation_ = true;
        setCurrent(false);
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + ".num_threads" &&  // NOLINT
        num_threads_ != parameter.as_int())
      {
        int new_value = parameter.as_int();
#ifdef _OPENMP
        int available_cores = omp_get_max_threads();
        if (new_value > available_cores) {
          RCLCPP_WARN(
            logger_,
            "num_threads=%d exceeds available cores (%d). Ignoring.",
            new_value, available_cores);
        } else {
          num_threads_ = new_value;
          RCLCPP_INFO(
            logger_,
            "Updated num_threads to %d %s",
            num_threads_,
            num_threads_ == -1 ? "(auto)" : "");
        }
#else
        RCLCPP_WARN(
          logger_,
          "num_threads parameter ignored - OpenMP support not available. "
          "Inflation layer will use single thread.");
        num_threads_ = new_value;
#endif
      }
    }
  }

  if (side_scaling_changed) {
    cost_scaling_factor_ =
      std::max(cost_scaling_factor_left_, cost_scaling_factor_right_);
    need_reinflation_ = true;
    need_cache_recompute = true;
    setCurrent(false);
  }

  if (need_cache_recompute) {
    matchSize();
  }
}

}  // namespace nav2_costmap_2d
