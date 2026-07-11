// Copyright (c) 2026, Marc Blöchlinger
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
  goal_distance_threshold_(0)
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
    if (inflation_radius_ < 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: inflation_radius must be >= 0");
    }
    if (cost_scaling_factor_left_ < 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: cost_scaling_factor_left must be >= 0");
    }
    if (cost_scaling_factor_right_ < 0.0) {
      throw std::runtime_error(
        "AsymmetricInflationLayer: cost_scaling_factor_right must be >= 0");
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
      nav2::qos::StandardTopicQoS(1));
  }

  setCurrent(true);
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
  if (latest_global_path_ == msg) {return;}

  // Look up the transform from path frame to costmap frame
  std::optional<geometry_msgs::msg::TransformStamped> transform;
  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  const std::string path_frame = msg->header.frame_id;
  if (global_frame != path_frame && !path_frame.empty()) {
    try {
      transform = tf_->lookupTransform(global_frame, path_frame,
          tf2_ros::fromMsg(msg->header.stamp));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
          logger_,
          "AsymmetricInflationLayer: TF lookup failed at path receipt (%s -> %s): %s. "
          "Asymmetric inflation will be disabled for this plan.",
          path_frame.c_str(), global_frame.c_str(), ex.what());
    }
  }

  // Cache the path and transform
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    latest_global_path_ = msg;
    latest_path_transform_ = transform;
  }
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

std::vector<AsymmetricPathSegment>
AsymmetricInflationLayer::extractLocalPathSegments(
  nav2_costmap_2d::Costmap2D & master_grid)
{
  std::vector<AsymmetricPathSegment> local_path_segments;
  nav_msgs::msg::Path current_path;
  std::optional<geometry_msgs::msg::TransformStamped> cached_transform;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!latest_global_path_ || latest_global_path_->poses.size() < 2) {
      return local_path_segments;
    }
    current_path = *latest_global_path_;
    cached_transform = latest_path_transform_;
  }

  // Check if the path is already in costmap frame
  std::string global_frame = layered_costmap_->getGlobalFrameID();
  std::string path_frame = current_path.header.frame_id;
  geometry_msgs::msg::TransformStamped transform;
  bool need_transform = (global_frame != path_frame && !path_frame.empty());

  // Transform the goal pose from the path frame to the costmap frame (e.g., map -> odom)
  geometry_msgs::msg::PoseStamped goal_pose = current_path.poses.back();
  if (need_transform) {
    if (!cached_transform) {
      RCLCPP_WARN(
        logger_,
        "AsymmetricInflationLayer: No cached transform for path frame '%s'. "
        "Falling back to symmetric inflation.",
        path_frame.c_str());
      return local_path_segments;
    }
    tf2::doTransform(goal_pose, goal_pose, *cached_transform);
  }

  // Disable asymmetry near the goal to prevent target oscillations
  double dist_to_goal = std::hypot(
    goal_pose.pose.position.x - current_robot_x_,
    goal_pose.pose.position.y - current_robot_y_);

  if (dist_to_goal <= goal_distance_threshold_) {
    return local_path_segments;
  }

  // Get local window edge coordinates
  const double map_min_x = master_grid.getOriginX();
  const double map_min_y = master_grid.getOriginY();
  const double map_max_x = map_min_x +
    static_cast<double>(master_grid.getSizeInCellsX()) * master_grid.getResolution();
  const double map_max_y = map_min_y +
    static_cast<double>(master_grid.getSizeInCellsY()) * master_grid.getResolution();

  // Extract path segments from local window.
  for (size_t i = 1; i < current_path.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped transformed_start = current_path.poses[i - 1];
    geometry_msgs::msg::PoseStamped transformed_end = current_path.poses[i];
    if (need_transform) {
      tf2::doTransform(current_path.poses[i - 1], transformed_start, *cached_transform);
      tf2::doTransform(current_path.poses[i], transformed_end, *cached_transform);
    }

    const double ax = transformed_start.pose.position.x;
    const double ay = transformed_start.pose.position.y;
    const double bx = transformed_end.pose.position.x;
    const double by = transformed_end.pose.position.y;

    const double min_x = std::min(ax, bx) - inflation_radius_;
    const double max_x = std::max(ax, bx) + inflation_radius_;
    const double min_y = std::min(ay, by) - inflation_radius_;
    const double max_y = std::max(ay, by) + inflation_radius_;

    if (max_x < map_min_x || min_x > map_max_x || max_y < map_min_y || min_y > map_max_y) {
      continue;
    }

    local_path_segments.push_back({{ax, ay}, {bx, by}});
  }
  return local_path_segments;
}

Side
AsymmetricInflationLayer::computeObstacleSide(
  double cx, double cy,
  const std::vector<size_t> & candidates,
  const std::vector<AsymmetricPathSegment> & local_path_segments)
{
  const double inflation_radius_sq = inflation_radius_ * inflation_radius_;

  double min_dist_sq = std::numeric_limits<double>::max();
  double best_cross = 0.0;

  // Evaluate candidate segments provided by the spatial hash.
  for (size_t p : candidates) {
    // Define segment endpoints A (start) and B (end).
    const auto & segment = local_path_segments[p];
    double ax = segment.start.first;
    double ay = segment.start.second;
    double bx = segment.end.first;
    double by = segment.end.second;

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

    // Smallest expected squared segment length
    double min_segment_length_sq = 1e-6;

    // Prevent division by zero for zero-length segments
    if (len_sq < min_segment_length_sq) {
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
    return Side::Neutral;
  }

  // Return the orientation based on the cross product of the closest segment.
  if (best_cross > 0.0) {return Side::Left;}
  if (best_cross < 0.0) {return Side::Right;}

  return Side::Neutral;
}

void
AsymmetricInflationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (!enabled_) {
    return;
  }

  // Pass 1: symmetric baseline via inherited distance-transform inflation
  InflationLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);

  std::vector<AsymmetricPathSegment> local_path_segments = extractLocalPathSegments(master_grid);

  // Abort if we don't have a valid path or if the scaling rates are equal (no asymmetry).
  if (local_path_segments.empty() || cost_scaling_factor_left_ == cost_scaling_factor_right_) {
    setCurrent(true);
    return;
  }

  // Pass 2: disfavored-side asymmetric overlay via distance transform
  unsigned char * master_array = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  // Clamp update window (mirrors InflationLayer::updateCosts)
  const int cmin_i = std::max(0, min_i);
  const int cmin_j = std::max(0, min_j);
  const int cmax_i = std::min(static_cast<int>(size_x), max_i);
  const int cmax_j = std::min(static_cast<int>(size_y), max_j);

  // Padded ROI — same formula as InflationLayer::updateCosts
  const int padding = static_cast<int>(cell_inflation_radius_);
  const int roi_min_i = std::max(0, cmin_i - padding);
  const int roi_min_j = std::max(0, cmin_j - padding);
  const int roi_max_i = std::min(static_cast<int>(size_x), cmax_i + padding);
  const int roi_max_j = std::min(static_cast<int>(size_y), cmax_j + padding);
  const int roi_width = roi_max_i - roi_min_i;
  const int roi_height = roi_max_j - roi_min_j;

  auto spatial_hash = buildPathSpatialHash(local_path_segments);

  MatrixXfRM dist_map = seedDistanceMap(
    master_grid, roi_min_i, roi_min_j, roi_width, roi_height,
    spatial_hash, local_path_segments);

  DistanceTransform::distanceTransform2D(dist_map, roi_height, roi_width);

  applyInflation(
    master_array, dist_map,
    cmin_i, cmin_j, cmax_i, cmax_j,
    roi_min_i, roi_min_j, size_x);

  setCurrent(true);
}

std::unordered_map<uint64_t, std::vector<size_t>>
AsymmetricInflationLayer::buildPathSpatialHash(
  const std::vector<AsymmetricPathSegment> & local_path_segments)
{
  std::unordered_map<uint64_t, std::vector<size_t>> spatial_hash;

  for (size_t p = 0; p < local_path_segments.size(); ++p) {
    // Create segment AB from an original consecutive path pose pair.
    const auto & segment = local_path_segments[p];
    double ax = segment.start.first;
    double ay = segment.start.second;
    double bx = segment.end.first;
    double by = segment.end.second;

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

MatrixXfRM
AsymmetricInflationLayer::seedDistanceMap(
  nav2_costmap_2d::Costmap2D & master_grid,
  int roi_min_i, int roi_min_j, int roi_width, int roi_height,
  const std::unordered_map<uint64_t, std::vector<size_t>> & spatial_hash,
  const std::vector<AsymmetricPathSegment> & local_path_segments)
{
  unsigned char * master_array = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  MatrixXfRM dist_map(roi_height, roi_width);
  dist_map.setConstant(DistanceTransform::DT_INF);

  Side disfavored_side = (cost_scaling_factor_left_ < cost_scaling_factor_right_) ?
    Side::Left : Side::Right;
  const int roi_max_i = roi_min_i + roi_width;
  const int roi_max_j = roi_min_j + roi_height;

  // Helper function to check if a neighbor is "traversable" (i.e., open space)
  auto is_traversable = [&](int nx, int ny) {
      unsigned char c = master_array[master_grid.getIndex(nx, ny)];
      return inflate_around_unknown_ ?
             (c != LETHAL_OBSTACLE && c != NO_INFORMATION) : (c != LETHAL_OBSTACLE);
    };

  // Seed all obstacle boundary cells, that are nearby a path segment
  for (int j = roi_min_j; j < roi_max_j; ++j) {
    for (int i = roi_min_i; i < roi_max_i; ++i) {
      unsigned char cost = master_array[master_grid.getIndex(i, j)];

      // Early exit 1: Skip cells that aren't lethal/unknown obstacles
      if (cost != LETHAL_OBSTACLE && !(inflate_around_unknown_ && cost == NO_INFORMATION)) {
        continue;
      }

      // Check if the cell touches the absolute edges of the costmap
      bool is_on_map_edge = (i == 0 || i == static_cast<int>(size_x) - 1 ||
        j == 0 || j == static_cast<int>(size_y) - 1);

      // An obstacle cell is a boundary if it's on the map edge OR touches free space.
      bool is_boundary = is_on_map_edge ||
        is_traversable(i - 1, j) || is_traversable(i + 1, j) ||
        is_traversable(i, j - 1) || is_traversable(i, j + 1);

      // Early exit 2: Skip interior obstacle cells
      if (!is_boundary) {
        continue;
      }

      // Find segments that are nearby this cell using the spatial hash
      double cx, cy;
      master_grid.mapToWorld(i, j, cx, cy);
      int64_t b_x = static_cast<int64_t>(std::floor(cx / inflation_radius_));
      int64_t b_y = static_cast<int64_t>(std::floor(cy / inflation_radius_));
      uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(b_x)) << 32) |
        static_cast<uint32_t>(b_y);

      // Only enqueue boundary cells on the disfavored side of the path.
      // Cells on favored side already got correctly inflated during the symmetric inflation pass.
      auto it = spatial_hash.find(key);
      if (it == spatial_hash.end()) {
        continue;
      }

      // Determine which side of the path this cell is on
      Side side = computeObstacleSide(cx, cy, it->second, local_path_segments);
      if (side != Side::Neutral && side == disfavored_side) {
        dist_map(j - roi_min_j, i - roi_min_i) = 0.0f;
      }
    }
  }

  return dist_map;
}

void
AsymmetricInflationLayer::applyInflation(
  unsigned char * master_array,
  const MatrixXfRM & distance_map,
  int min_i, int min_j, int max_i, int max_j,
  int roi_min_i, int roi_min_j,
  unsigned int size_x)
{
  if (cost_lut_disfavored_.empty()) {
    return;
  }

  const float cell_inflation_radius_f = static_cast<float>(cell_inflation_radius_);
  const int lut_max = static_cast<int>(cost_lut_disfavored_.size() - 1);
  const unsigned char * lut_data = cost_lut_disfavored_.data();
  const int lut_precision = COST_LUT_PRECISION;

#ifdef _OPENMP
  const int num_threads = getOptimalThreadCount();
  #pragma omp parallel for num_threads(num_threads) schedule(dynamic, 16)
#endif
  for (int j = min_j; j < max_j; ++j) {
    const int row_offset = j * static_cast<int>(size_x);
    const int dist_row = j - roi_min_j;

    for (int i = min_i; i < max_i; ++i) {
      const float distance_cells = distance_map(dist_row, i - roi_min_i);
      if (distance_cells > cell_inflation_radius_f) {
        continue;
      }

      const unsigned int index = row_offset + i;
      const unsigned char old_cost = master_array[index];
      const unsigned int d_scaled = std::min(
        static_cast<unsigned int>(lut_max),
        static_cast<unsigned int>(distance_cells * lut_precision + 0.5f));
      const unsigned char new_cost = lut_data[d_scaled];

      if (new_cost > old_cost) {
        master_array[index] = new_cost;
      }
    }
  }
}

void
AsymmetricInflationLayer::computeAsymmetricCaches()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (cell_inflation_radius_ == 0) {
    return;
  }

  // Build cost LUT for the disfavored side using c_side (the smaller scaling factor).
  // computeCost() always uses cost_scaling_factor_ (c_max), so we inline the formula with c_side.
  const double c_side = std::min(cost_scaling_factor_left_, cost_scaling_factor_right_);
  const unsigned int max_dist_scaled = cell_inflation_radius_ * COST_LUT_PRECISION + 1;

  cost_lut_disfavored_.resize(max_dist_scaled + 1);
  for (unsigned int d_scaled = 0; d_scaled <= max_dist_scaled; ++d_scaled) {
    const double distance = static_cast<double>(d_scaled) / COST_LUT_PRECISION;
    unsigned char cost = 0;
    if (distance == 0.0) {
      cost = LETHAL_OBSTACLE;
    } else if (distance * resolution_ <= inscribed_radius_) {
      cost = INSCRIBED_INFLATED_OBSTACLE;
    } else {
      double factor = exp(-c_side * (distance * resolution_ - inscribed_radius_));
      cost = static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    cost_lut_disfavored_[d_scaled] = cost;
  }
}

rcl_interfaces::msg::SetParametersResult
AsymmetricInflationLayer::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  return InflationLayer::validateParameterUpdatesCallback(parameters);
}

void
AsymmetricInflationLayer::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<rclcpp::Parameter> base_parameters;
  base_parameters.reserve(parameters.size());
  for (const auto & parameter : parameters) {
    if (parameter.get_name() != name_ + ".cost_scaling_factor") {
      base_parameters.push_back(parameter);
    }
  }
  InflationLayer::updateParametersCallback(base_parameters);

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  bool side_scaling_changed = false;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + ".cost_scaling_factor_left" &&  // NOLINT
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
    }
  }

  if (side_scaling_changed) {
    cost_scaling_factor_ =
      std::max(cost_scaling_factor_left_, cost_scaling_factor_right_);
    need_reinflation_ = true;
    setCurrent(false);
    matchSize();
  }
}

}  // namespace nav2_costmap_2d
