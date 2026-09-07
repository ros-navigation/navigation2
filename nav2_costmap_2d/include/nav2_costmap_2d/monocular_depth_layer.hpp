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

#ifndef NAV2_MONOCULAR_DEPTH_LAYER__MONOCULAR_DEPTH_LAYER_HPP_
#define NAV2_MONOCULAR_DEPTH_LAYER__MONOCULAR_DEPTH_LAYER_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/time.hpp"

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_monocular_depth_layer
{

/**
 * @class MonocularDepthLayer
 * @brief A costmap layer that marks and clears cells from a single monocular
 *        depth stream (e.g. a learned DINOv2/DPT depth network) plus its
 *        CameraInfo, with per-pixel confidence gating and spatial agreement
 *        filtering to reject the flicker that dense learned depth produces.
 *
 * This layer intentionally does NOT run any neural network. A separate node is
 * expected to publish metric depth as sensor_msgs/Image (32FC1 in metres, or
 * 16UC1 in millimetres) and, optionally, a matching confidence image (32FC1 in
 * [0, 1]). Keeping inference out of the costmap process preserves Nav2's
 * separation of concerns and lets any monocular estimator drive the layer.
 */
class MonocularDepthLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  MonocularDepthLayer() = default;
  ~MonocularDepthLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void activate() override {}
  void deactivate() override {}
  void reset() override;
  bool isClearable() override {return true;}

private:
  /// @brief Cache the latest depth frame (thread: subscription callback).
  void depthCallback(sensor_msgs::msg::Image::ConstSharedPtr msg);
  /// @brief Cache the latest confidence frame.
  void confidenceCallback(sensor_msgs::msg::Image::ConstSharedPtr msg);
  /// @brief Cache the latest intrinsics.
  void cameraInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  /// @brief Read one pixel of a 32FC1/16UC1 depth image, in metres. Returns
  ///        NaN for invalid/zero returns.
  double depthAt(const sensor_msgs::msg::Image & img, int u, int v) const;
  /// @brief Read one pixel of a 32FC1 confidence image in [0, 1]. Returns 1.0
  ///        if no confidence image is available.
  double confidenceAt(int u, int v) const;

  /// @brief Deproject, transform, filter and write the pending frame into this
  ///        layer's own costmap buffer. Returns the touched world bounds.
  bool processPendingFrame(
    double * min_x, double * min_y, double * max_x, double * max_y);

  // --- Parameters -----------------------------------------------------------
  std::string depth_topic_;
  std::string confidence_topic_;
  std::string camera_info_topic_;
  std::string global_frame_;

  double min_obstacle_range_{0.3};    ///< metres; nearer returns are ignored
  double max_obstacle_range_{8.0};    ///< metres; farther returns are ignored
  double max_clearing_range_{8.0};    ///< metres; ray-clear up to this distance
  double min_obstacle_height_{0.05};  ///< metres in the global frame
  double max_obstacle_height_{2.0};   ///< metres in the global frame
  double min_confidence_{0.5};        ///< [0, 1]; pixels below are dropped
  double depth_scale_correction_{1.0};///< static multiplier for residual scale
  int subsample_step_{4};             ///< process every Nth pixel per axis
  int min_points_per_cell_{2};        ///< spatial-agreement mark threshold
  double observation_persistence_{0.0};  ///< s; 0 keeps only the latest frame
  double transform_tolerance_{0.2};   ///< s
  int combination_method_{1};         ///< 0=overwrite 1=max 2=max-no-unknown
  bool track_unknown_space_{false};
  bool clearing_enabled_{true};

  // --- State ----------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr confidence_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  std::mutex data_lock_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_depth_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_confidence_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Time last_frame_stamp_{0, 0, RCL_ROS_TIME};

  bool rolling_window_{false};
  bool was_reset_{false};
  tf2::Duration transform_tolerance_dur_{tf2::durationFromSec(0.2)};

  // Per-cell hit accumulator, sized to the layer, reused each frame.
  std::vector<uint16_t> hit_counts_;
};

}  // namespace nav2_monocular_depth_layer

#endif  // NAV2_MONOCULAR_DEPTH_LAYER__MONOCULAR_DEPTH_LAYER_HPP_
