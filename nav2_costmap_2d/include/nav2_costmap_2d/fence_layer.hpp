// Copyright (c) 2026, LonelyGuy-SE1
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

#ifndef NAV2_COSTMAP_2D__FENCE_LAYER_HPP_
#define NAV2_COSTMAP_2D__FENCE_LAYER_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

namespace nav2_costmap_2d
{

class FenceLayerTester;

class FenceLayer : public CostmapLayer
{
public:
  FenceLayer();
  virtual ~FenceLayer();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return false;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void matchSize() override;

  friend class FenceLayerTester;

private:
  // Subscription callback (runs on executor thread, outside updateMap lock)
  void polygonCallback(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr & msg);

  // Deferred processing (runs inside updateMap lock via updateBounds)
  void processFence();

  // Pre-rasterize the fence mask into internal costmap
  void rasterizeMask();

  // Point-in-polygon test (ray casting)
  bool isPointInPolygon(double x, double y) const;

  // Dynamic parameter callbacks
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  // Members
  std::string global_frame_;
  nav2::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;

  // Buffered polygon (set in callback, consumed in updateBounds)
  std::mutex polygon_buffer_mutex_;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr polygon_buffer_;
  std::atomic<bool> polygon_updated_{false};

  // Processed polygon state (only accessed under updateMap lock)
  std::vector<double> polygon_x_;
  std::vector<double> polygon_y_;
  bool has_received_polygon_{false};

  // Pre-rasterized mask: true = outside fence (lethal), false = inside (untouched)
  // Stored as flat vector indexed by [j * fence_size_x_ + i]
  std::vector<bool> outside_fence_;
  unsigned int fence_size_x_{0};
  unsigned int fence_size_y_{0};
  double fence_origin_x_{0.0};
  double fence_origin_y_{0.0};
  double fence_resolution_{0.0};

  // Parameter callbacks
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__FENCE_LAYER_HPP_
