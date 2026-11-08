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

#ifndef NAV2_COSTMAP_2D__ROUTE_CORRIDOR_LAYER_HPP_
#define NAV2_COSTMAP_2D__ROUTE_CORRIDOR_LAYER_HPP_

#include <mutex>
#include <vector>
#include "nav2_costmap_2d/layer.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_costmap_2d
{
/** Builds a fixed graph corridor mask and applies pose-dependent outside costs. */
class RouteCorridorLayer : public Layer
{
public:
  void onInitialize() override;
  void reset() override {}  // Keep the startup mask across costmap clears/resets.
  bool isClearable() override {return false;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override;

protected:
  void loadGraph(const std::string & filepath);
  void buildMask(const Costmap2D & master);

  bool insideCorridor(double wx, double wy) const;

  struct Segment
  {
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;
  };
  std::mutex mutex_;
  std::vector<Segment> edges_;
  nav_msgs::msg::OccupancyGrid mask_;
  double max_deviation_{1.0};
  bool built_{false};
  double robot_x_{0.0};
  double robot_y_{0.0};
  bool pose_valid_{false};
  unsigned char recovery_cost_{200};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_pub_;
};
}  // namespace nav2_costmap_2d
#endif  // NAV2_COSTMAP_2D__ROUTE_CORRIDOR_LAYER_HPP_
