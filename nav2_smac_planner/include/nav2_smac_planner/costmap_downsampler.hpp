// Copyright (c) 2020, Carlos Luis
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

#ifndef NAV2_SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_
#define NAV2_SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_smac_planner/constants.hpp"

namespace nav2_smac_planner
{

/**
 * @class nav2_smac_planner::CostmapDownsampler
 * @brief A costmap downsampler for more efficient path planning
 */
class CostmapDownsampler
{
public:
  /**
   * @brief A constructor for CostmapDownsampler
   */
  CostmapDownsampler();

  /**
   * @brief A destructor for CostmapDownsampler
   */
  ~CostmapDownsampler();

  /**
   * @brief Configure the downsampled costmap object and the ROS publisher
   * @param node Lifecycle node pointer
   * @param global_frame The ID of the global frame used by the costmap
   * @param topic_name The name of the topic to publish the downsampled costmap
   * @param costmap The costmap we want to downsample
   * @param downsampling_factor Multiplier for the costmap resolution
   */
  void on_configure(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & global_frame,
    const std::string & topic_name,
    nav2_costmap_2d::Costmap2D * const costmap,
    const unsigned int & downsampling_factor);

  /**
   * @brief Activate the publisher of the downsampled costmap
   */
  void on_activate();

  /**
   * @brief Deactivate the publisher of the downsampled costmap
   */
  void on_deactivate();

  /**
   * @brief Cleanup the publisher of the downsampled costmap
   */
  void on_cleanup();

  /**
   * @brief Downsample the given costmap by the downsampling factor, and publish the downsampled costmap
   * @param downsampling_factor Multiplier for the costmap resolution
   * @return A ptr to the downsampled costmap
   */
  nav2_costmap_2d::Costmap2D * downsample(const unsigned int & downsampling_factor);

  /**
   * @brief Resize the downsampled costmap. Used in case the costmap changes and we need to update the downsampled version
   */
  void resizeCostmap();

protected:
  /**
   * @brief Update the sizes X-Y of the costmap and its downsampled version
   */
  void updateCostmapSize();

  /**
   * @brief Explore all subcells of the original costmap and assign the max cost to the new (downsampled) cell
   * @param new_mx The X-coordinate of the cell in the new costmap
   * @param new_my The Y-coordinate of the cell in the new costmap
   */
  void setCostOfCell(
    const unsigned int & new_mx,
    const unsigned int & new_my);

  unsigned int _size_x;
  unsigned int _size_y;
  unsigned int _downsampled_size_x;
  unsigned int _downsampled_size_y;
  unsigned int _downsampling_factor;
  float _downsampled_resolution;
  nav2_costmap_2d::Costmap2D * _costmap;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> _downsampled_costmap;
  std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> _downsampled_costmap_pub;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_
