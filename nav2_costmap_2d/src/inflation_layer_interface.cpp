// Copyright (c) 2026, Dexory (Tony Najjar)
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
// limitations under the License.

#include "nav2_costmap_2d/inflation_layer_interface.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_costmap_2d
{

std::shared_ptr<InflationLayerInterface> InflationLayerInterface::getInflationLayer(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
  const std::string layer_name)
{
  const auto layered_costmap = costmap_ros->getLayeredCostmap();
  for (auto layer = layered_costmap->getPlugins()->begin();
    layer != layered_costmap->getPlugins()->end();
    ++layer)
  {
    auto inflation_layer =
      std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayerInterface>(*layer);
    if (inflation_layer) {
      if (layer_name.empty() || inflation_layer->getName() == layer_name) {
        return inflation_layer;
      }
    }
  }
  return nullptr;
}

}  // namespace nav2_costmap_2d
