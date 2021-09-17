/*********************************************************************
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 *
 * Author: Andrey Ryzhikov
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__DENOISE_LAYER_HPP_
#define NAV2_COSTMAP_2D__DENOISE_LAYER_HPP_

#include <opencv2/core/core.hpp>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"

class DenoiseLayerTester;  // For test some private methods using gtest

namespace nav2_costmap_2d
{
/**
 * @class DenoiseLayer
 * @brief Layer filters noise-induced standalone obstacles (white costmap pixels) or
 * small obstacles groups
 */
class DenoiseLayer : public Layer
{
  friend class ::DenoiseLayerTester;  // For test some private methods using gtest

private:
  /// Pixels connectivity type (is the way in which pixels in image relate to
  /// their neighbors)
  enum class ConnectivityType: int
  {
    /// neighbors pixels are connected horizontally and vertically
    Way4 = 4,
    /// neighbors pixels are connected horizontally, vertically and diagonally
    Way8 = 8
  };

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
   *
   * The method is called to ask the plugin: which area of costmap it needs to update.
   * A layer is essentially a filter, so it never needs to expand bounds.
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  /**
   * @brief Filters noise-induced obstacles in the selected region of the costmap
   *
   * The method is called when costmap recalculation is required.
   * It updates the costmap within its window bounds.
   * @param master_grid The master costmap grid to update
   * \param min_x X min map coord of the window to update
   * \param min_y Y min map coord of the window to update
   * \param max_x X max map coord of the window to update
   * \param max_y Y max map coord of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_x, int min_y, int max_x, int max_y) override;

protected:
  /**
   * @brief Initializes the layer on startup
   *
   * This method is called at the end of plugin initialization.
   * Reads plugin parameters from a config file
   */
  void onInitialize() override;

private:
  /**
     * @brief Removes from the image single obstacles (white pixels) or small obstacles groups
     *
     * Pixels less than 255 will be interpreted as background (free space), 255 - as obstacles.
     * Replaces groups of obstacles smaller than minimal_group_size to free space.
     * Groups connectivity type is determined by the connectivity parameter.
     *
     * If minimal_group_size is 1 or 0, it does nothing
     * (all standalone obstacles will be preserved, since it satisfies this condition).
     * If minimal_group_size equals 2, performs fast filtering based on the dilation operation.
     * Otherwise, it performs a slower segmentation-based operation.
     *
     * @param image source single channel image with depth CV_8U.
     * @throw std::logic_error in case inner logic errors
   */
  void denoise(cv::Mat & image) const;

  /**
     * @brief Removes from the image groups of white pixels smaller than minimal_group_size
     *
     * Segments the image into groups of connected pixels
     * Replace pixels in groups whose size smaller than minimal_group_size to zero value (background)
     * @param image source single channel binary image with depth CV_8U
     * @throw std::logic_error in case inner logic errors
     * @warning If image.empty() or image.type() != CV_8UC1, the behavior is undefined
   */
  void removeGroups(cv::Mat & image) const;

  /**
     * @brief Removes from the image freestanding single white pixels
     *
     * Works similarly to removeGroups with minimal_group_size = 2, but about 10x faster
     * @param image source single channel binary image with depth CV_8U
     * @throw std::logic_error in case inner logic errors
     * @warning If image.empty() or image.type() != CV_8UC1, the behavior is undefined
   */
  void removeSinglePixels(cv::Mat & image) const;

  /**
     * @brief Calculate truncated histogram of image.
     *
     * Creates a histogram of image_max bins.
     * Bin with index i keep min of (<count of the number of pixels with value i>, bin_max).
     * This truncation avoids overflow and is acceptable in the problem being solved.
     * For example, the image may have 100'000 pixels equal to 0
     * (100'000 > std::numeric_limits<uint16_t>). In this case, an overflow will occur when
     * calculating a traditional histogram with bins of the uint16_t type. But in this function,
     * the bin value will increase to the bin_max value, then stop. Overflow will not happen.
     *
     * Faster (because simpler) than cv::calcHist
     * @param image source single channel image with depth CV_16U
     * @param image_max max image pixel value
     * @param bin_max max histogram bin value
     * @return vector of histogram bins
     * @throw std::logic_error if image.type() != CV_16UC1
     * @warning If source contains a pixel with a value, large then image_max,
     * the behavior is undefined
   */
  std::vector<uint16_t> calculateHistogram(
    const cv::Mat & image, uint16_t image_max, uint16_t bin_max) const;

  /**
     * @brief Convert each pixel of source image to target by lookup table
     *
     * Perform target[i,j] = table[ source[i,j] ]
     * @param source source single channel image with depth CV_16UC1
     * @param target source single channel image with depth CV_8UC1
     * @param table lookup table
     * @throw std::logic_error if the source and target of different sizes
     * @throw std::logic_error if source.type() != CV_16UC1 or target.type() != CV_8UC1
     * @warning If table.size() not more then max pixel value of source image,
     * the behavior is undefined
   */
  void applyLookupTable(
    const cv::Mat & source, cv::Mat & target, const std::vector<uint8_t> & table) const;

  /**
     * @brief Creates a lookup table for binary thresholding
     *
     * The table size is equal to groups_sizes.size()
     * Lookup table[i] = filled_cell_value if groups_sizes[i] >= threshold,
     * empty_cell_value in other case
   */
  std::vector<uint8_t> makeLookupTable(
    const std::vector<uint16_t> & groups_sizes, uint16_t threshold) const;

  /**
     * @brief Convert each pixel of source to corresponding pixel of target using a custom function
     *
     * The source and target must be the same size.
     * For calculation of new target value the operation can use source value and
     * an optionally current target value.
     * This function call operation(source[i, j], target[i, j]) for each pixel
     * where target[i, j] is mutable
     * @tparam SourceElement type of source pixel
     * @tparam TargetElement type of target pixel
     * @tparam Converter function object.
     * Signature should be equivalent to the following:
     * void fn(const SourceElement& src, TargetElement& trg)
     * @param source input image with SourceElement-type pixels
     * @param target output image with TargetElement-type pixels
     * @param operation the binary operation op is applied to pairs of pixels:
     * first (const) from source and second (mutable) from target
     * @throw std::logic_error if the source and target of different sizes
     * @warning If SourceElement/TargetElement type does not match the source/target image type,
     * the behavior is undefined
   */
  template<class SourceElement, class TargetElement, class Converter>
  void convert(const cv::Mat & source, cv::Mat & target, Converter operation) const;

  /**
     * @brief Checks that the image type is as expected
     * @param image the image to be checked
     * @param cv_type expected image type
     * @param error_prefix prefix of the exception text that will be thrown is types are different
     * @throw std::logic_error if the image type and cv_type are different
     * @warning If error_prefix is nullptr, the behavior is undefined
   */
  void checkImageType(const cv::Mat & image, int cv_type, const char * error_prefix) const;

  /**
     * @brief Checks that the a.size() == b.size()
     * @param error_prefix prefix of the exception text that will be thrown is sizes are different
     * @throw std::logic_error if the images sizes are different
     * @warning If error_prefix is nullptr, the behavior is undefined
   */
  void checkImagesSizesEqual(const cv::Mat & a, const cv::Mat & b, const char * error_prefix) const;

private:
  // Pixels connectivity type. Determines how pixels belonging to the same group can be arranged
  size_t minimal_group_size{};
  // The border value of group size. Groups of this and larger size will be kept
  ConnectivityType group_connectivity_type{ConnectivityType::Way8};
  const uint8_t empty_cell_value = 0;
  const uint8_t filled_cell_value = 255;
};

template<class SourceElement, class TargetElement, class Converter>
void DenoiseLayer::convert(const cv::Mat & source, cv::Mat & target, Converter operation) const
{
  checkImagesSizesEqual(source, target, "DenoiseLayer::convert. The source and target");

  for (int row = 0; row < source.rows; ++row) {
    const auto src_begin = source.ptr<const SourceElement>(row);
    const auto src_end = src_begin + source.cols;
    auto trg = target.ptr<TargetElement>(row);

    for (auto src = src_begin; src != src_end; ++src, ++trg) {
      operation(*src, *trg);
    }
  }
}

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__DENOISE_LAYER_HPP_
