// Copyright (c) 2023 Andrey Ryzhikov
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

#ifndef NAV2_COSTMAP_2D__DENOISE_LAYER_HPP_
#define NAV2_COSTMAP_2D__DENOISE_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/denoise/image_processing.hpp"

namespace nav2_costmap_2d
{
/**
 * @class DenoiseLayer
 * @brief Layer filters noise-induced standalone obstacles (white costmap pixels) or
 * small obstacles groups
 */
class DenoiseLayer : public Layer
{
  friend class DenoiseLayerTester;  // For test some private methods using gtest

public:
  DenoiseLayer() = default;
  ~DenoiseLayer() = default;

  /**
   * @brief Reset this layer
   */
  void reset() override;

  /**
   * @brief Reports that no clearing operation is required
   */
  bool isClearable() override;

  /**
   * @brief Reports that no expansion is required
   * The method is called to ask the plugin: which area of costmap it needs to update.
   * A layer is essentially a filter, so it never needs to expand bounds.
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  /**
   * @brief Filters noise-induced obstacles in the selected region of the costmap
   * The method is called when costmap recalculation is required.
   * It updates the costmap within its window bounds.
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_x, int min_y, int max_x, int max_y) override;

protected:
  /**
   * @brief Initializes the layer on startup
   * This method is called at the end of plugin initialization.
   * Reads plugin parameters from a config file
   */
  void onInitialize() override;

private:
  /**
     * @brief Removes from the image single obstacles (white pixels) or small obstacles groups
     *
     * Pixels less than 255 will be interpreted as background (free space), 255 - as obstacles.
     * Replaces groups of obstacles smaller than minimal_group_size_ to free space.
     * Groups connectivity type is determined by the connectivity parameter.
     *
     * If minimal_group_size_ is 1 or 0, it does nothing
     * (all standalone obstacles will be preserved, since it satisfies this condition).
     * If minimal_group_size_ equals 2, performs fast filtering based on the dilation operation.
     * Otherwise, it performs a slower segmentation-based operation.
     * @throw std::logic_error in case inner logic errors
   */
  void denoise(Image<uint8_t> & image) const;

  /**
     * @brief Removes from the image groups of white pixels smaller than minimal_group_size_
     * Segments the image into groups of connected pixels
     * Replace pixels in groups whose size smaller than minimal_group_size_ to zero value (background)
     * @throw std::logic_error in case inner logic errors
     * @warning If image.empty() the behavior is undefined
   */
  void removeGroups(Image<uint8_t> & image) const;

  /**
     * @brief Removes from the image freestanding single white pixels
     * Works similarly to removeGroups with minimal_group_size_ = 2, but about 10x faster
     * @throw std::logic_error in case inner logic errors
     * @warning If image.empty() the behavior is undefined
   */
  void removeSinglePixels(Image<uint8_t> & image) const;
  /**
   * @brief Separates image pixels into objects and background
   * @return true if the pixel value is not an obstacle code. False in other case
   */
  bool isBackground(uint8_t pixel) const;

private:
  // The border value of group size. Groups of this and larger size will be kept
  size_t minimal_group_size_{};
  // Pixels connectivity type. Determines how pixels belonging to the same group can be arranged
  ConnectivityType group_connectivity_type_{ConnectivityType::Way8};
  // Memory buffer for temporal image
  mutable MemoryBuffer buffer_;
  // Implementing the removal of grouped noise
  imgproc_impl::GroupsRemover groups_remover_;
  // Interpret NO_INFORMATION code as obstacle
  bool no_information_is_obstacle_{};
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__DENOISE_LAYER_HPP_
