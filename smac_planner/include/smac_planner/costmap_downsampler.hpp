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

#ifndef SMAC_PLANNER__DOWNSAMPLER_HPP_
#define SMAC_PLANNER__DOWNSAMPLER_HPP_

#include <algorithm>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "smac_planner/constants.hpp"

namespace smac_planner
{

/**
 * @class smac_planner::CostmapDownsampler
 * @brief A costmap downsampler for more efficient path planning
 */
class CostmapDownsampler
{
public:
  /**
   * @brief A constructor for CostmapDownsampler
   * @param node Lifecycle node pointer
   */
  explicit CostmapDownsampler(const nav2_util::LifecycleNode::SharedPtr & node)
  : _costmap(nullptr),
    _downsampled_costmap(nullptr),
    _downsampled_costmap_pub(nullptr),
    _node(node)
  {
  }

  /**
   * @brief A destructor for CostmapDownsampler
   */
  ~CostmapDownsampler()
  {
  }

  /**
   * @brief Initialize the downsampled costmap object and the ROS publisher
   * @param global_frame The ID of the global frame used by the costmap
   * @param topic_name The name of the topic to publish the downsampled costmap
   * @param costmap The costmap we want to downsample
   * @param downsampling_factor Multiplier for the costmap resolution
   */
   void initialize(
    const std::string & global_frame,
    const std::string & topic_name,
    nav2_costmap_2d::Costmap2D * const costmap,
    const unsigned int & downsampling_factor)
  {
    _topic_name = topic_name;
    _costmap = costmap;
    _downsampling_factor = downsampling_factor;
    updateCostmapSize();

    _downsampled_costmap = std::make_unique<nav2_costmap_2d::Costmap2D>
      (_downsampled_size_x, _downsampled_size_y, _downsampled_resolution,
       _costmap->getOriginX(), _costmap->getOriginY(), UNKNOWN);

    _downsampled_costmap_pub = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
      _node, _downsampled_costmap.get(), global_frame, _topic_name, false);
  }

  /**
   * @brief Activate the publisher of the downsampled costmap
   */
  void activatePublisher() {
    _downsampled_costmap_pub->on_activate();
  }

  /**
   * @brief Deactivate the publisher of the downsampled costmap
   */
  void deactivatePublisher() {
    _downsampled_costmap_pub->on_deactivate();
  }

  /**
   * @brief Downsample the given costmap by the downsampling factor, and publish the downsampled costmap
   * @param downsampling_factor Multiplier for the costmap resolution
   * @return A ptr to the downsampled costmap
   */
  nav2_costmap_2d::Costmap2D * downsample(const unsigned int & downsampling_factor)
  {
    _downsampling_factor = downsampling_factor;
    updateCostmapSize();

    // Adjust costmap size if needed
    if (_downsampled_costmap->getSizeInCellsX() != _downsampled_size_x ||
      _downsampled_costmap->getSizeInCellsY() != _downsampled_size_y ||
      _downsampled_costmap->getResolution() != _downsampled_resolution) {
      resizeCostmap();
    }

    // Assign costs
    for (int i = 0; i < _downsampled_size_x; ++i) {
      for (int j = 0; j < _downsampled_size_y; ++j) {
        setCostOfCell(i, j);
      }
    }

    if (_node->count_subscribers(_topic_name) > 0) {
      _downsampled_costmap_pub->publishCostmap();
    }

    return _downsampled_costmap.get();
  }

private:
  /**
   * Update the sizes X-Y of the costmap and its downsampled version
   */
  void updateCostmapSize()
  {
    _size_x = _costmap->getSizeInCellsX();
    _size_y = _costmap->getSizeInCellsY();
    _downsampled_size_x = ceil(static_cast<float>(_size_x) / _downsampling_factor);
    _downsampled_size_y = ceil(static_cast<float>(_size_y) / _downsampling_factor);
    _downsampled_resolution = _downsampling_factor * _costmap->getResolution();
  }

  /**
   * Resize the downsampled costmap. Used in case the costmap changes and we need to update the downsampled version
   */
  void resizeCostmap()
  {
    _downsampled_costmap->resizeMap(
      _downsampled_size_x,
      _downsampled_size_y,
      _downsampled_resolution,
      _costmap->getOriginX(),
      _costmap->getOriginY());
  }

  /**
   * @brief Explore all subcells of the original costmap and assign the max cost to the new (downsampled) cell
   * @param new_mx The X-coordinate of the cell in the new costmap
   * @param new_my The Y-coordinate of the cell in the new costmap
   */
  void setCostOfCell(
    const unsigned int & new_mx,
    const unsigned int & new_my)
  {
    unsigned int mx, my;
    unsigned char cost = 0;
    unsigned int x_offset = new_mx * _downsampling_factor;
    unsigned int y_offset = new_my * _downsampling_factor;

    for (int i = 0; i < _downsampling_factor; ++i) {
      mx = x_offset + i;
      if (mx >= _size_x) {
        continue;
      }
      for (int j = 0; j < _downsampling_factor; ++j) {
        my = y_offset + j;
        if (my >= _size_y) {
          continue;
        }
        cost = std::max(cost, _costmap->getCost(mx, my));
      }
    }

    _downsampled_costmap->setCost(new_mx, new_my, cost);
  }

  unsigned int _size_x;
  unsigned int _size_y;
  unsigned int _downsampled_size_x;
  unsigned int _downsampled_size_y;
  unsigned int _downsampling_factor;
  float _downsampled_resolution;
  std::string _topic_name;
  nav2_util::LifecycleNode::SharedPtr _node;
  nav2_costmap_2d::Costmap2D * _costmap;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> _downsampled_costmap;
  std::unique_ptr<nav2_costmap_2d::Costmap2DPublisher> _downsampled_costmap_pub;
};

}  // namespace smac_planner

#endif // SMAC_PLANNER__DOWNSAMPLER_HPP_
