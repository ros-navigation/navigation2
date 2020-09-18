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

#include "smac_planner/costmap_downsampler.hpp"

namespace smac_planner
{

CostmapDownsampler::CostmapDownsampler(const nav2_util::LifecycleNode::SharedPtr & node)
: _node(node),
  _costmap(nullptr),
  _downsampled_costmap(nullptr),
  _downsampled_costmap_pub(nullptr)
{
}

CostmapDownsampler::~CostmapDownsampler()
{
}

void CostmapDownsampler::initialize(
  const std::string & global_frame,
  const std::string & topic_name,
  nav2_costmap_2d::Costmap2D * const costmap,
  const unsigned int & downsampling_factor)
{
  _topic_name = topic_name;
  _costmap = costmap;
  _downsampling_factor = downsampling_factor;
  updateCostmapSize();

  _downsampled_costmap = std::make_unique<nav2_costmap_2d::Costmap2D>(
    _downsampled_size_x, _downsampled_size_y, _downsampled_resolution,
    _costmap->getOriginX(), _costmap->getOriginY(), UNKNOWN);

  _downsampled_costmap_pub = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
    _node, _downsampled_costmap.get(), global_frame, _topic_name, false);
}

nav2_costmap_2d::Costmap2D * CostmapDownsampler::downsample(
  const unsigned int & downsampling_factor)
{
  _downsampling_factor = downsampling_factor;
  updateCostmapSize();

  // Adjust costmap size if needed
  if (_downsampled_costmap->getSizeInCellsX() != _downsampled_size_x ||
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

  if (_node->count_subscribers(_topic_name) > 0) {
    _downsampled_costmap_pub->publishCostmap();
  }

  return _downsampled_costmap.get();
}

void CostmapDownsampler::updateCostmapSize()
{
  _size_x = _costmap->getSizeInCellsX();
  _size_y = _costmap->getSizeInCellsY();
  _downsampled_size_x = ceil(static_cast<float>(_size_x) / _downsampling_factor);
  _downsampled_size_y = ceil(static_cast<float>(_size_y) / _downsampling_factor);
  _downsampled_resolution = _downsampling_factor * _costmap->getResolution();
}

void CostmapDownsampler::resizeCostmap()
{
  _downsampled_costmap->resizeMap(
    _downsampled_size_x,
    _downsampled_size_y,
    _downsampled_resolution,
    _costmap->getOriginX(),
    _costmap->getOriginY());
}

void CostmapDownsampler::setCostOfCell(
  const unsigned int & new_mx,
  const unsigned int & new_my)
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

}  // namespace smac_planner
