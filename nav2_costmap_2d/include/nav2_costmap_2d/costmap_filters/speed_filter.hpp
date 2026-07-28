/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Samsung Research Russia
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
 *   * Neither the name of the <ORGANIZATION> nor the names of its
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
 * Author: Alexey Merzlyakov
 *********************************************************************/

#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTERS__SPEED_FILTER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTERS__SPEED_FILTER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/path_utils.hpp"

namespace nav2_costmap_2d
{
/**
 * @class SpeedFilter
 * @brief Reads in a speed restriction mask and enables a robot to
 * dynamically adjust speed based on pose in map to slow in dangerous
 * areas. Done via absolute speed setting or percentage of maximum speed
 */
class SpeedFilter : public CostmapFilter
{
public:
  /**
   * @brief A constructor
   */
  SpeedFilter();

  /**
   * @brief Initialize the filter and subscribe to the info topic
   */
  void initializeFilter(
    const std::string & filter_info_topic) override;

  /**
   * @brief Process the keepout layer at the current pose / bounds / grid
   */
  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose & pose) override;

  /**
   * @brief Reset the costmap filter / topic / info
   */
  void resetFilter() override;

  /**
   * @brief If this filter is active
   */
  bool isActive();

protected:
  /**
   * @brief Callback for the filter information
   */
  void filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::ConstSharedPtr & msg);
  /**
   * @brief Callback for the filter mask
   */
  void maskCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  /**
   * @brief Callback for the planned path used by path lookahead
   */
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  /**
   * @brief Look up the speed limit at a single pose in the costmap's global frame
   * @param pose Pose in global_frame_
   * @param speed_limit output: computed speed limit
   * @return true if pose mapped to a valid mask cell, false otherwise
   */
  bool getSpeedLimitAtPose(
    const geometry_msgs::msg::Pose & pose,
    double & speed_limit);
  /**
   * @brief Get the speed limit from the path lookahead
   * @param robot_pose robot pose
   * @param lookahead_dist lookahead distance
   * @param speed_limit output: strictest speed limit found along the lookahead
   * @return true if the lookahead could be evaluated, false otherwise
   */
  bool getSpeedLimitFromLookahead(
    const geometry_msgs::msg::Pose & robot_pose,
    double lookahead_dist,
    double & speed_limit);

  nav2::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  nav2::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  nav2::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  nav2::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_pub_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask_;
  nav_msgs::msg::Path::ConstSharedPtr current_path_;

  // Odometry for variable lookahead distance calculation
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  std::string global_frame_;  // Frame of current layer (master_grid)

  double base_, multiplier_;
  bool percentage_;
  double speed_limit_, speed_limit_prev_;

  // Lookahead distance held when entering a speed zone to avoid oscillations
  double held_lookahead_dist_;
  size_t lookahead_start_idx_;  // Start index for closest pose search, cached for efficiency
  bool enable_path_lookahead_;  // Whether to enable path lookahead
  double max_decel_;       // Deceleration (m/s^2) used to size lookahead
  double min_lookahead_;   // Lower limit on lookahead distance (m)
  double max_lookahead_;   // Upper limit on lookahead distance (m)
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_FILTERS__SPEED_FILTER_HPP_
