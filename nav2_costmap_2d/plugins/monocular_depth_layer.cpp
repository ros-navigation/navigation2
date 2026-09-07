// Copyright (c) 2026 Ocean Code AI Ltd
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

// NOTE: This file was authored with the assistance of an AI system,
// reviewed and validated by the submitter. Disclosed per the Nav2 PR
// template's AI-generated-software policy.

#include "nav2_monocular_depth_layer/monocular_depth_layer.hpp"
#include "nav2_monocular_depth_layer/deprojection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/raytrace_line_2d.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_monocular_depth_layer
{

void MonocularDepthLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"MonocularDepthLayer: failed to lock node"};
  }

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();

  enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);
  depth_topic_ = node->declare_or_get_parameter(
    name_ + "." + "depth_topic", std::string("depth"));
  confidence_topic_ = node->declare_or_get_parameter(
    name_ + "." + "confidence_topic", std::string(""));
  camera_info_topic_ = node->declare_or_get_parameter(
    name_ + "." + "camera_info_topic", std::string("camera_info"));

  min_obstacle_range_ = node->declare_or_get_parameter(
    name_ + "." + "min_obstacle_range", 0.3);
  max_obstacle_range_ = node->declare_or_get_parameter(
    name_ + "." + "max_obstacle_range", 8.0);
  max_clearing_range_ = node->declare_or_get_parameter(
    name_ + "." + "max_clearing_range", 8.0);
  min_obstacle_height_ = node->declare_or_get_parameter(
    name_ + "." + "min_obstacle_height", 0.05);
  max_obstacle_height_ = node->declare_or_get_parameter(
    name_ + "." + "max_obstacle_height", 2.0);
  min_confidence_ = node->declare_or_get_parameter(
    name_ + "." + "min_confidence", 0.5);
  depth_scale_correction_ = node->declare_or_get_parameter(
    name_ + "." + "depth_scale_correction", 1.0);
  subsample_step_ = node->declare_or_get_parameter(
    name_ + "." + "subsample_step", 4);
  min_points_per_cell_ = node->declare_or_get_parameter(
    name_ + "." + "min_points_per_cell", 2);
  observation_persistence_ = node->declare_or_get_parameter(
    name_ + "." + "observation_persistence", 0.0);
  combination_method_ = node->declare_or_get_parameter(
    name_ + "." + "combination_method", 1);
  track_unknown_space_ = node->declare_or_get_parameter(
    name_ + "." + "track_unknown_space", layered_costmap_->isTrackingUnknown());
  clearing_enabled_ = node->declare_or_get_parameter(
    name_ + "." + "clearing_enabled", true);
  transform_tolerance_ = node->declare_or_get_parameter(
    name_ + "." + "transform_tolerance", 0.2);
  transform_tolerance_dur_ = tf2::durationFromSec(transform_tolerance_);

  subsample_step_ = std::max(1, subsample_step_);

  default_value_ = track_unknown_space_ ? NO_INFORMATION : FREE_SPACE;
  matchSize();
  current_ = true;
  was_reset_ = false;

  depth_topic_ = joinWithParentNamespace(depth_topic_);
  camera_info_topic_ = joinWithParentNamespace(camera_info_topic_);

  depth_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_,
    std::bind(&MonocularDepthLayer::depthCallback, this, std::placeholders::_1),
    rclcpp::SensorDataQoS());
  camera_info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_,
    std::bind(&MonocularDepthLayer::cameraInfoCallback, this, std::placeholders::_1),
    rclcpp::SensorDataQoS());

  if (!confidence_topic_.empty()) {
    confidence_topic_ = joinWithParentNamespace(confidence_topic_);
    confidence_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
      confidence_topic_,
      std::bind(&MonocularDepthLayer::confidenceCallback, this, std::placeholders::_1),
      rclcpp::SensorDataQoS());
  }

  RCLCPP_INFO(
    logger_,
    "MonocularDepthLayer '%s' subscribing to depth '%s' and camera_info '%s'",
    name_.c_str(), depth_topic_.c_str(), camera_info_topic_.c_str());
}

void MonocularDepthLayer::depthCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_lock_);
  pending_depth_ = msg;
}

void MonocularDepthLayer::confidenceCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_lock_);
  latest_confidence_ = msg;
}

void MonocularDepthLayer::cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_lock_);
  camera_info_ = msg;
}

double MonocularDepthLayer::depthAt(const sensor_msgs::msg::Image & img, int u, int v) const
{
  const auto row = static_cast<size_t>(v) * img.step;
  if (img.encoding == "32FC1") {
    float d;
    std::memcpy(&d, &img.data[row + static_cast<size_t>(u) * sizeof(float)], sizeof(float));
    if (!std::isfinite(d) || d <= 0.0f) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(d) * depth_scale_correction_;
  } else if (img.encoding == "16UC1") {
    uint16_t d;
    std::memcpy(&d, &img.data[row + static_cast<size_t>(u) * sizeof(uint16_t)], sizeof(uint16_t));
    if (d == 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(d) * 0.001 * depth_scale_correction_;  // mm -> m
  }
  return std::numeric_limits<double>::quiet_NaN();
}

double MonocularDepthLayer::confidenceAt(int u, int v) const
{
  if (!latest_confidence_ || latest_confidence_->encoding != "32FC1") {
    return 1.0;
  }
  const auto & img = *latest_confidence_;
  if (u < 0 || v < 0 ||
    u >= static_cast<int>(img.width) || v >= static_cast<int>(img.height))
  {
    return 1.0;
  }
  const auto idx = static_cast<size_t>(v) * img.step + static_cast<size_t>(u) * sizeof(float);
  float c;
  std::memcpy(&c, &img.data[idx], sizeof(float));
  return std::isfinite(c) ? static_cast<double>(c) : 0.0;
}

bool MonocularDepthLayer::processPendingFrame(
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  sensor_msgs::msg::Image::ConstSharedPtr depth;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  {
    std::lock_guard<std::mutex> lock(data_lock_);
    depth = pending_depth_;
    info = camera_info_;
    pending_depth_.reset();
  }
  if (!depth || !info) {
    return false;
  }
  if (depth->encoding != "32FC1" && depth->encoding != "16UC1") {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "MonocularDepthLayer: unsupported depth encoding '%s' (need 32FC1 or 16UC1)",
      depth->encoding.c_str());
    return false;
  }

  // Look up the camera-optical -> global transform once for the whole frame.
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_->lookupTransform(
      global_frame_, depth->header.frame_id,
      tf2::TimePoint(std::chrono::nanoseconds(rclcpp::Time(depth->header.stamp).nanoseconds())),
      transform_tolerance_dur_);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "MonocularDepthLayer: transform %s -> %s failed: %s",
      depth->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    return false;
  }

  tf2::Transform cam_to_global;
  tf2::fromMsg(tf_msg.transform, cam_to_global);
  const tf2::Vector3 sensor_origin = cam_to_global.getOrigin();

  const double fx = info->k[0], fy = info->k[4];
  const double cx = info->k[2], cy = info->k[5];
  if (fx == 0.0 || fy == 0.0) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "MonocularDepthLayer: CameraInfo has zero focal length");
    return false;
  }

  const int width = static_cast<int>(depth->width);
  const int height = static_cast<int>(depth->height);
  const double min_r2 = min_obstacle_range_ * min_obstacle_range_;
  const double max_r2 = max_obstacle_range_ * max_obstacle_range_;

  // Fresh layer each frame (this layer is intended for a rolling local costmap
  // and does not persist marks across frames unless observation_persistence>0,
  // which is left as documented future work). Reset the accumulator too.
  resetMaps();
  hit_counts_.assign(static_cast<size_t>(size_x_) * size_y_, 0);

  unsigned int sx, sy;
  const bool have_sensor_cell = worldToMap(sensor_origin.x(), sensor_origin.y(), sx, sy);

  // Pass 1: deproject, transform, range/height/confidence filter, and either
  // accumulate a mark hit or ray-clear free space toward the return.
  for (int v = 0; v < height; v += subsample_step_) {
    for (int u = 0; u < width; u += subsample_step_) {
      const double d = depthAt(*depth, u, v);
      if (!std::isfinite(d)) {
        continue;
      }
      const double r2 = d * d;
      if (r2 < min_r2 || r2 > max_r2) {
        continue;
      }
      if (confidenceAt(u, v) < min_confidence_) {
        continue;
      }

      // Optical-frame deprojection (REP 103: x right, y down, z forward).
      const auto pc = deproject(u, v, d, fx, fy, cx, cy);
      const tf2::Vector3 p = cam_to_global * tf2::Vector3(pc[0], pc[1], pc[2]);

      unsigned int mx, my;
      if (!worldToMap(p.x(), p.y(), mx, my)) {
        continue;
      }

      // Clear free space along the ray from the sensor to this return.
      if (clearing_enabled_ && have_sensor_cell && d <= max_clearing_range_) {
        nav2_costmap_2d::Costmap2D::MarkCell clearer(costmap_, FREE_SPACE);
        nav2_util::raytraceLine(clearer, sx, sy, mx, my, size_x_);
      }

      // Only returns inside the height band count as obstacles.
      if (p.z() >= min_obstacle_height_ && p.z() <= max_obstacle_height_) {
        hit_counts_[static_cast<size_t>(my) * size_x_ + mx]++;
      }
    }
  }

  // Pass 2: promote cells with enough spatial agreement to lethal, and grow the
  // update bounds to cover everything we touched.
  bool touched_any = false;
  for (unsigned int j = 0; j < size_y_; ++j) {
    for (unsigned int i = 0; i < size_x_; ++i) {
      const size_t idx = static_cast<size_t>(j) * size_x_ + i;
      double wx, wy;
      if (hit_counts_[idx] >= static_cast<uint16_t>(min_points_per_cell_)) {
        costmap_[idx] = LETHAL_OBSTACLE;
        mapToWorld(i, j, wx, wy);
        touch(wx, wy, min_x, min_y, max_x, max_y);
        touched_any = true;
      } else if (clearing_enabled_ && costmap_[idx] == FREE_SPACE && default_value_ != FREE_SPACE) {
        // A cell we actively cleared; include it in the bounds so it is written.
        mapToWorld(i, j, wx, wy);
        touch(wx, wy, min_x, min_y, max_x, max_y);
        touched_any = true;
      }
    }
  }

  last_frame_stamp_ = clock_->now();
  return touched_any;
}

void MonocularDepthLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  const bool had_new = processPendingFrame(min_x, min_y, max_x, max_y);

  // Staleness check: if no fresh frame within the persistence window, flag the
  // layer as not-current so the planner knows this data may be unsafe.
  const double age = (clock_->now() - last_frame_stamp_).seconds();
  if (had_new) {
    current_ = true;
  } else if (observation_persistence_ > 0.0 && age > observation_persistence_) {
    current_ = false;
  }

  if (was_reset_) {
    was_reset_ = false;
    current_ = false;
  }
}

void MonocularDepthLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  switch (combination_method_) {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 2:  // Max, but do not overwrite known cells with unknown
      updateWithMaxWithoutUnknownOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Max (default)
    default:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
  }
}

void MonocularDepthLayer::reset()
{
  resetMaps();
  {
    std::lock_guard<std::mutex> lock(data_lock_);
    pending_depth_.reset();
  }
  current_ = false;
  was_reset_ = true;
}

}  // namespace nav2_monocular_depth_layer

PLUGINLIB_EXPORT_CLASS(
  nav2_monocular_depth_layer::MonocularDepthLayer, nav2_costmap_2d::Layer)
