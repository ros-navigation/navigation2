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

#ifndef SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_
#define SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "smac_planner/constants.hpp"
#include "smac_planner/costmap_downsampler.hpp"

namespace smac_planner
{
/**
 * @class smac_planner::CostmapDownsampler
 * @brief A costmap downsampler for more efficient path planning
 */
template<typename Costmap2DT>
class CostmapDownsampler
{
public:
  /**
   * @brief A constructor for CostmapDownsampler
   */
  CostmapDownsampler()
  : _costmap(nullptr), _downsampled_costmap(nullptr) {}

  /**
   * @brief A destructor for CostmapDownsampler
   */
  ~CostmapDownsampler() {}

  /**
   * @brief Configure the downsampled costmap object and the ROS publisher
   * @param publishCostmap A Functor to publish the costmap
   * @param global_frame The ID of the global frame used by the costmap
   * @param topic_name The name of the topic to publish the downsampled costmap
   * @param costmap The costmap we want to downsample
   * @param downsampling_factor Multiplier for the costmap resolution
   */
  void on_configure(
    std::function<void()> publishCostmap, [[maybe_unused]] const std::string & global_frame,
    [[maybe_unused]] const std::string & topic_name, Costmap2DT * const costmap,
    const unsigned int & downsampling_factor)
  {
    _costmap = costmap;
    _downsampling_factor = downsampling_factor;
    updateCostmapSize();

    _downsampled_costmap = std::make_unique<Costmap2DT>(
      _downsampled_size_x, _downsampled_size_y, _downsampled_resolution, _costmap->getOriginX(),
      _costmap->getOriginY(), UNKNOWN);

    _publish_downsampled_costmap = publishCostmap;
  }

  /**
   * @brief
   *
   */
  void on_cleanup()
  {
    _costmap = nullptr;
    _downsampled_costmap.reset();
  }

  /**
   * @brief Downsample the given costmap by the downsampling factor, and publish the downsampled costmap
   * @param downsampling_factor Multiplier for the costmap resolution
   * @return A ptr to the downsampled costmap
   */
  Costmap2DT * downsample(const unsigned int & downsampling_factor)
  {
    _downsampling_factor = downsampling_factor;
    updateCostmapSize();

    // Adjust costmap size if needed
    if (
      _downsampled_costmap->getSizeInCellsX() != _downsampled_size_x ||
      _downsampled_costmap->getSizeInCellsY() != _downsampled_size_y ||
      _downsampled_costmap->getResolution() != _downsampled_resolution)
    {
      resizeCostmap();
    }

    // Assign costs
    for (uint i = 0; i < _downsampled_size_x; ++i) {
      for (uint j = 0; j < _downsampled_size_y; ++j) {
        setCostOfCell(i, j);
      }
    }

    _publish_downsampled_costmap();
    return _downsampled_costmap.get();
  }

  /**
   * @brief Resize the downsampled costmap. Used in case the costmap changes and we need to update the downsampled version
   */
  void resizeCostmap()
  {
    _downsampled_costmap->resizeMap(
      _downsampled_size_x, _downsampled_size_y, _downsampled_resolution, _costmap->getOriginX(),
      _costmap->getOriginY());
  }

protected:
  /**
   * @brief Update the sizes X-Y of the costmap and its downsampled version
   */
  void updateCostmapSize()
  {
    _size_x = _costmap->getSizeInCellsX();
    _size_y = _costmap->getSizeInCellsY();
    _downsampled_size_x = std::ceil(static_cast<float>(_size_x) / _downsampling_factor);
    _downsampled_size_y = std::ceil(static_cast<float>(_size_y) / _downsampling_factor);
    _downsampled_resolution = _downsampling_factor * _costmap->getResolution();
  }

  /**
   * @brief Explore all subcells of the original costmap and assign the max cost to the new (downsampled) cell
   * @param new_mx The X-coordinate of the cell in the new costmap
   * @param new_my The Y-coordinate of the cell in the new costmap
   */
  void setCostOfCell(const unsigned int & new_mx, const unsigned int & new_my)
  {
    unsigned int mx, my;
    unsigned char cost = 0;
    unsigned int x_offset = new_mx * _downsampling_factor;
    unsigned int y_offset = new_my * _downsampling_factor;

    for (uint i = 0; i < _downsampling_factor; ++i) {
      mx = x_offset + i;
      if (mx >= _size_x) {
        continue;
      }
      for (uint j = 0; j < _downsampling_factor; ++j) {
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
  Costmap2DT * _costmap;
  std::unique_ptr<Costmap2DT> _downsampled_costmap;
  std::function<void()> _publish_downsampled_costmap;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__COSTMAP_DOWNSAMPLER_HPP_
