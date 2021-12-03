// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef NAV2_SMAC_PLANNER__SMOOTHER_HPP_
#define NAV2_SMAC_PLANNER__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_smac_planner/types.hpp"
#include "nav2_smac_planner/constants.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::Smoother
 * @brief A path smoother implementation
 */
class Smoother
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::Smoother
   */
  explicit Smoother(const SmootherParams & params);

  /**
   * @brief A destructor for nav2_smac_planner::Smoother
   */
  ~Smoother() {}

  /**
   * @brief Initialization of the smoother
   * @param min_turning_radius Minimum turning radius (m)
   */
  void initialize(const double & min_turning_radius);

  /**
   * @brief Smoother method
   * @param path Reference to path
   * @param costmap Pointer to minimal costmap
   * @param max_time Maximum time to compute, stop early if over limit
   * @return If smoothing was successful
   */
  bool smooth(
    nav_msgs::msg::Path & path,
    const nav2_costmap_2d::Costmap2D * costmap,
    const double & max_time,
    const bool do_refinement = true);

protected:
  /**
   * @brief Get the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @return dim value
   */
  inline double getFieldByDim(
    const geometry_msgs::msg::PoseStamped & msg,
    const unsigned int & dim);

  /**
   * @brief Set the field value for a given dimension
   * @param msg Current pose to sample
   * @param dim Dimension ID of interest
   * @param value to set the dimention to for the pose
   */
  inline void setFieldByDim(
    geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
    const double & value);

  /**
   * @brief Get the instantaneous curvature valud
   * @param path Path to find curvature in
   * @param i idx in path to find it for
   * @return curvature
   */
  inline double getCurvature(const nav_msgs::msg::Path & path, const unsigned int i);

  /**
   * @brief For a given path, update the path point orientations based on smoothing
   * @param path Path to approximate the path orientation in
   */
  inline void updateApproximatePathOrientations(nav_msgs::msg::Path & path);

  double min_turning_rad_, tolerance_, data_w_, smooth_w_;
  int max_its_;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__SMOOTHER_HPP_
