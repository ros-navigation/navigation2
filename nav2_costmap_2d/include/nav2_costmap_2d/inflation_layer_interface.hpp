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

#ifndef NAV2_COSTMAP_2D__INFLATION_LAYER_INTERFACE_HPP_
#define NAV2_COSTMAP_2D__INFLATION_LAYER_INTERFACE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_costmap_2d
{

// Forward declarations
class Costmap2DROS;
class InflationLayer;
class LegacyInflationLayer;

/**
 * @class InflationLayerInterface
 * @brief Abstract interface for inflation layers, providing common methods
 * needed by navigation components regardless of the inflation implementation.
 */
class InflationLayerInterface : public Layer
{
public:
  typedef std::recursive_mutex mutex_t;

  virtual ~InflationLayerInterface() = default;

  /**
   * @brief Get the cost scaling factor
   * @return The cost scaling factor
   */
  virtual double getCostScalingFactor() = 0;

  /**
   * @brief Get the inflation radius
   * @return The inflation radius in meters
   */
  virtual double getInflationRadius() = 0;

  /**
   * @brief Get the mutex of the inflation information
   * @return Pointer to the mutex
   */
  virtual mutex_t * getMutex() = 0;

  /**
   * @brief Given a distance, compute a cost.
   * @param distance The distance from an obstacle in cells
   * @return A cost value for the distance
   */
  virtual unsigned char computeCost(double distance) const = 0;

  /**
   * @brief Get the inflation layer from a costmap, checking for both
   * InflationLayer and LegacyInflationLayer implementations.
   * @param costmap_ros The costmap ROS wrapper
   * @param layer_name Optional name of the specific layer to find
   * @return Shared pointer to the inflation layer interface, or nullptr if not found
   */
  static std::shared_ptr<InflationLayerInterface> getInflationLayer(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
    const std::string layer_name = "");
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__INFLATION_LAYER_INTERFACE_HPP_
