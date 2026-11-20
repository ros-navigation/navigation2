/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Ye_X
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
 *   * Neither the name of copyright holder nor the names of its
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
 * Author: Xinyi Ye
 *********************************************************************/

#ifndef NAV2_COSTMAP_2D__ROUTE_CORRIDOR_LAYER_HPP_
#define NAV2_COSTMAP_2D__ROUTE_CORRIDOR_LAYER_HPP_

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_costmap_2d
{
/**
 * @class RouteCorridorLayer
 * @brief Builds a path corridor and applies costs to its three-pixel boundary.
 */
class RouteCorridorLayer : public Layer
{
public:
  /**
   * @brief Load parameters and initialize the path subscription and mask publisher.
   */
  void onInitialize() override;

  /**
   * @brief Keep the latest path across costmap clears and resets.
   */
  void reset() override {}

  /**
   * @brief Report that this layer does not support clearing operations.
   */
  bool isClearable() override {return false;}

  /**
   * @brief Record the robot pose and expand update bounds to the full costmap.
   * @param robot_x Robot X position in the global frame
   * @param robot_y Robot Y position in the global frame
   * @param robot_yaw Robot orientation (unused)
   * @param min_x Minimum world X coordinate of the update window
   * @param min_y Minimum world Y coordinate of the update window
   * @param max_x Maximum world X coordinate of the update window
   * @param max_y Maximum world Y coordinate of the update window
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Apply lethal boundary costs, or recovery costs when the robot is outside the corridor.
   * @param master Master costmap to update
   * @param min_i Minimum cell X index of the update window 
   * @param min_j Minimum cell Y index of the update window 
   * @param max_i Maximum cell X index of the update window 
   * @param max_j Maximum cell Y index of the update window 
   */
  void updateCosts(Costmap2D & master, int min_i, int min_j, int max_i, int max_j) override;

protected:
  /**
   * @brief Convert the new path in to segments (straight lines).
   * @param path Path message in any frame, transformed to the costmap global frame
   */
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

  /** @brief Clear the corridor when the latest navigation goal succeeds or aborts. */
  void goalStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr status);

  /** @brief Reset the master costmap and the other plugins and filters. */
  void clearCostmaps(Costmap2D & master);

  /**
   * @brief Build and publish the masks.
   * @param master Costmap providing the mask dimensions, resolution, and origin
   */
  void buildMask(Costmap2D & master);

  /**
   * @brief Check whether a world position lies inside the corridor mask.
   * @param wx World X coordinate
   * @param wy World Y coordinate
   * @return True if the position is inside a free corridor cell
   */
  bool insideCorridor(double wx, double wy) const;

  struct Segment
  {
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point end;
  };
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> goal_status_subs_;
  std::array<uint8_t, 16> latest_goal_id_{};
  int64_t latest_goal_stamp_{-1};
  bool goal_finished_{false};
  bool goal_clear_pending_{false};
  std::mutex mutex_;
  std::vector<Segment> path_segments_;
  nav_msgs::msg::OccupancyGrid mask_;
  std::vector<bool> corridor_cells_;
  std::vector<size_t> boundary_indices_;
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
