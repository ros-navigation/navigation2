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


#ifndef NAV2_COSTMAP_2D__INFLATION_LAYER_HPP_
#define NAV2_COSTMAP_2D__INFLATION_LAYER_HPP_

#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/inflation_layer_interface.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/distance_transform.hpp"

namespace nav2_costmap_2d
{

/**
 * @class InflationLayer
 * @brief Layer to convolve costmap by robot's radius or footprint using Eigen-based
 * Felzenszwalb-Huttenlocher O(n) distance transform for improved performance.
 */
class InflationLayer : public InflationLayerInterface
{
public:
  /**
    * @brief A constructor
    */
  InflationLayer();

  /**
    * @brief A destructor
    */
  ~InflationLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  void onInitialize() override;

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
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;
  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Match the size of the master costmap
   */
  void matchSize() override;

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  bool isClearable() override {return false;}

  /**
   * @brief Reset this costmap
   */
  void reset() override
  {
    matchSize();
    current_ = false;
    need_reinflation_ = true;
  }

  /** @brief  Given a distance, compute a cost.
   * @param  distance The distance from an obstacle in cells.
   * @return A cost value for the distance */
  inline unsigned char computeCost(double distance) const override
  {
    unsigned char cost = 0;
    if (distance == 0) {
      cost = LETHAL_OBSTACLE;
    } else if (distance * resolution_ <= inscribed_radius_) {
      cost = INSCRIBED_INFLATED_OBSTACLE;
    } else {
      // make sure cost falls off by Euclidean distance
      double factor =
        exp(-1.0 * cost_scaling_factor_ * (distance * resolution_ - inscribed_radius_));
      cost = static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  static std::shared_ptr<nav2_costmap_2d::InflationLayer> getInflationLayer(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
    const std::string layer_name = "")
  {
    const auto layered_costmap = costmap_ros->getLayeredCostmap();
    for (auto layer = layered_costmap->getPlugins()->begin();
      layer != layered_costmap->getPlugins()->end();
      ++layer)
    {
      auto inflation_layer =
        std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(*layer);
      if (inflation_layer) {
        if (layer_name.empty() || inflation_layer->getName() == layer_name) {
          return inflation_layer;
        }
      }
    }
    return nullptr;
  }

  /**
   * @brief Get the mutex of the inflation information
   */
  mutex_t * getMutex() override
  {
    return access_;
  }

  double getCostScalingFactor() override
  {
    return cost_scaling_factor_;
  }

  double getInflationRadius() override
  {
    return inflation_radius_;
  }

protected:
  /**
   * @brief Apply inflation costs from distance map to costmap
   * @param master_array Pointer to the costmap data
   * @param distance_map Distance transform result
   * @param min_i Minimum x index of update region
   * @param min_j Minimum y index of update region
   * @param max_i Maximum x index of update region
   * @param max_j Maximum y index of update region
   * @param roi_min_i ROI minimum x offset
   * @param roi_min_j ROI minimum y offset
   * @param size_x Width of the costmap
   */
  void applyInflation(
    unsigned char * master_array,
    const MatrixXfRM & distance_map,
    int min_i, int min_j, int max_i, int max_j,
    int roi_min_i, int roi_min_j,
    unsigned int size_x);

  /**
   * @brief Process updates on footprint changes to the inflation layer
   */
  void onFootprintChanged() override;

  /**
   * @brief Convert world distance to cell distance
   */
  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  /**
   * @brief Generate cost lookup table for distance to cost mapping
   */
  void computeCaches();

  /**
   * @brief Determine optimal thread count based on system resources
   * @return Optimal number of OpenMP threads to use
   */
  int getOptimalThreadCount();

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  double inflation_radius_, inscribed_radius_, cost_scaling_factor_;
  bool inflate_unknown_, inflate_around_unknown_;
  unsigned int cell_inflation_radius_;
  int num_threads_;  // Number of OpenMP threads (-1 = auto)
  double resolution_;

  // Cost LUT precision: 100 samples per cell provides smooth gradients
  static constexpr int COST_LUT_PRECISION = 100;
  std::vector<unsigned char> cost_lut_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire costmap should be reinflated next time around.
  bool need_reinflation_;
  mutex_t * access_;
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__INFLATION_LAYER_HPP_
