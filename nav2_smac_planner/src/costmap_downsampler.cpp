// Copyright (c) 2020, Carlos Luis
// Copyright (c) 2020, Samsung Research America
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

#include "nav2_smac_planner/costmap_downsampler.hpp"

#include <string>
#include <memory>
#include <algorithm>

namespace nav2_smac_planner
{

CostmapDownsampler::CostmapDownsampler()
: _costmap(nullptr),
  _downsampled_costmap(nullptr),
  _downsampled_costmap_pub(nullptr)
{
}

CostmapDownsampler::~CostmapDownsampler()
{
}

void CostmapDownsampler::on_configure(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & global_frame,
  const std::string & topic_name,
  nav2_costmap_2d::Costmap2D * const costmap,
  const unsigned int & downsampling_factor,
  const bool & use_min_cost_neighbor)
{
  _costmap = costmap;
  _downsampling_factor = downsampling_factor;
  _use_min_cost_neighbor = use_min_cost_neighbor;
  updateCostmapSize();

  _downsampled_costmap = std::make_unique<nav2_costmap_2d::Costmap2D>(
    _downsampled_size_x, _downsampled_size_y, _downsampled_resolution,
    _costmap->getOriginX(), _costmap->getOriginY(), UNKNOWN);

  if (!node.expired()) {
    _downsampled_costmap_pub = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
      node, _downsampled_costmap.get(), global_frame, topic_name, false);
  }
}

void CostmapDownsampler::on_activate()
{
  if (_downsampled_costmap_pub) {
    _downsampled_costmap_pub->on_activate();
  }
}

void CostmapDownsampler::on_deactivate()
{
  if (_downsampled_costmap_pub) {
    _downsampled_costmap_pub->on_deactivate();
  }
}

void CostmapDownsampler::on_cleanup()
{
  _costmap = nullptr;
  _downsampled_costmap.reset();
  _downsampled_costmap_pub.reset();
}

nav2_costmap_2d::Costmap2D * CostmapDownsampler::downsample(
  const unsigned int downsampling_factor)
{
  _downsampling_factor = downsampling_factor;
  updateCostmapSize();

  // Adjust costmap size if needed
  if (_downsampled_costmap->getSizeInCellsX() != _downsampled_size_x ||
    _downsampled_costmap->getSizeInCellsY() != _downsampled_size_y ||
    _downsampled_costmap->getResolution() != _downsampled_resolution)
  {
    _downsampled_costmap->resizeMap(
      _downsampled_size_x,
      _downsampled_size_y,
      _downsampled_resolution,
      _costmap->getOriginX(),
      _costmap->getOriginY());
  }

  // Assign costs
  for (unsigned int i = 0; i < _downsampled_size_x; ++i) {
    for (unsigned int j = 0; j < _downsampled_size_y; ++j) {
      setCostOfCell(i, j);
    }
  }

  if (_downsampled_costmap_pub) {
    _downsampled_costmap_pub->publishCostmap();
  }
  return _downsampled_costmap.get();
}

}  // namespace nav2_smac_planner
