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

#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTER_HPP_

#include <string>
#include <mutex>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/layer.hpp"

namespace nav2_costmap_2d
{

/**
 * @brief: CostmapFilter basic class. It is inherited from Layer in order to avoid
 * hidden problems when the shared handling of costmap_ resource (PR #1936)
 */
class CostmapFilter : public Layer
{
public:
  /**
   * @brief A constructor
   */
  CostmapFilter();
  /**
   * @brief A destructor
   */
  ~CostmapFilter();

  /**
   * @brief: Provide a typedef to ease future code maintenance
   */
  typedef std::recursive_mutex mutex_t;
  /**
   * @brief: returns pointer to a mutex
   */
  mutex_t * getMutex()
  {
    return access_;
  }

  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize() final;

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) final;

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) final;

  /**
   * @brief Activate the layer
   */
  virtual void activate() final;
  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate() final;
  /**
   * @brief Reset the layer
   */
  virtual void reset() final;

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() {return false;}

  /** CostmapFilter API **/
  /**
   * @brief: Initializes costmap filter. Creates subscriptions to filter-related topics
   * @param: Name of costmap filter info topic
   */
  virtual void initializeFilter(
    const std::string & filter_info_topic) = 0;

  /**
   * @brief: An algorithm for how to use that map's information. Fills the Costmap2D with
   *         calculated data and makes an action based on processed data
   * @param: Reference to a master costmap2d
   * @param: Low window map boundary OX
   * @param: Low window map boundary OY
   * @param: High window map boundary OX
   * @param: High window map boundary OY
   * @param: Robot 2D-pose
   */
  virtual void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose) = 0;

  /**
   * @brief: Resets costmap filter. Stops all subscriptions
   */
  virtual void resetFilter() = 0;

protected:
  /**
   * @brief: Name of costmap filter info topic
   */
  std::string filter_info_topic_;

  /**
   * @brief: Name of filter mask topic
   */
  std::string mask_topic_;

  /**
   * @brief: mask_frame_->global_frame_ transform tolerance
   */
  tf2::Duration transform_tolerance_;

private:
  /**
   * @brief: Latest robot position
   */
  geometry_msgs::msg::Pose2D latest_pose_;

  /**
   * @brief: Mutex for locking filter's resources
   */
  mutex_t * access_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_FILTER_HPP_
