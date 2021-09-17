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
#include "nav2_costmap_2d/denoise_layer.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d
{

void
DenoiseLayer::reset()
{
  current_ = false;
}

bool
DenoiseLayer::isClearable()
{
  return false;
}

void
DenoiseLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * /*min_x*/, double * /*min_y*/,
  double * /*max_x*/, double * /*max_y*/) {}

void
DenoiseLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_x, int min_y, int max_x, int max_y)
{
  if (!enabled_) {
    return;
  }
  std::unique_lock<std::recursive_mutex> lock(*master_grid.getMutex());

  unsigned char * master_array = master_grid.getCharMap();
  const int step = static_cast<int>(master_grid.getSizeInCellsX());

  const cv::Size roi_size {max_x - min_x, max_y - min_y};
  cv::Mat roi_image(roi_size, CV_8UC1, static_cast<void *>(master_array + min_x), step);

  try {
    denoise(roi_image);
  } catch (std::exception & ex) {
    RCLCPP_ERROR(logger_, (std::string("Inner error: ") + ex.what()).c_str());
  }

  current_ = true;
}

void
DenoiseLayer::onInitialize()
{
  // Enable/disable plugin
  declareParameter("enabled", rclcpp::ParameterValue(true));
  // Smaller groups should be filtered
  declareParameter("minimal_group_size", rclcpp::ParameterValue(2));
  // Pixels connectivity type
  declareParameter("group_connectivity_type", rclcpp::ParameterValue(8));

  // node != nullptr. If node_ is nullptr, declareParameter (...) will throw
  const auto node = node_.lock();
  node->get_parameter(name_ + "." + "enabled", enabled_);

  auto getInt = [&](const std::string & parameter_name) {
      int param{};
      node->get_parameter(name_ + "." + parameter_name, param);
      return param;
    };

  const int minimal_group_size_param = getInt("minimal_group_size");

  if (minimal_group_size_param <= 1) {
    RCLCPP_WARN(
      logger_,
      "DenoiseLayer::onInitialize(): param minimal_group_size: %i."
      " A value of 1 or less means that all map cells will be left as they are.",
      minimal_group_size_param);
    this->minimal_group_size = 1;
  } else {
    this->minimal_group_size = static_cast<size_t>(minimal_group_size_param);
  }

  const int group_connectivity_type_param = getInt("group_connectivity_type");

  if (group_connectivity_type_param == 4) {
    this->group_connectivity_type = ConnectivityType::Way4;
  } else if (group_connectivity_type_param == 8) {
    this->group_connectivity_type = ConnectivityType::Way8;
  } else {
    RCLCPP_WARN(
      logger_, "DenoiseLayer::onInitialize(): param group_connectivity_type: %i."
      " Possible values are  4 (neighbors pixels are connected horizontally and vertically) "
      "or 8 (neighbors pixels are connected horizontally, vertically and diagonally)."
      "The default value 8 will be used",
      group_connectivity_type_param);
    this->group_connectivity_type = ConnectivityType::Way8;
  }
  current_ = true;
}

void
DenoiseLayer::denoise(cv::Mat & image) const
{
  checkImageType(image, CV_8UC1, "DenoiseLayer::denoise_");

  if (image.empty()) {
    return;
  }

  if (minimal_group_size <= 1) {
    return;  // A smaller group cannot exist. No one pixel will be changed
  }

  if (minimal_group_size == 2) {
    // Performs fast filtration based on erosion function
    removeSinglePixels(image);
  } else {
    // Performs a slower segmentation-based operation
    removeGroups(image);
  }
}

void
DenoiseLayer::removeGroups(cv::Mat & image) const
{
  // Creates image binary (the same type and size as image)
  // binary[i,j] = filled_cell_value if image[i,j] == filled_cell_value,
  // empty_cell_value in other cases
  cv::Mat binary;
  cv::threshold(
    image, binary, filled_cell_value - 1, filled_cell_value, cv::ThresholdTypes::THRESH_BINARY);

  // Creates an image in which each group is labeled with a unique code
  cv::Mat labels;
  // There is an simple alternative: connectedComponentsWithStats.
  // But cv::connectedComponents + calculateHistogram is about 20% faster
  const uint16_t groups_count = static_cast<uint16_t>(
    cv::connectedComponents(binary, labels, static_cast<int>(group_connectivity_type), CV_16U)
  );

  // Calculates the size of each group.
  // Group size is equal to the number of pixels with the same label
  const auto max_label_value = groups_count - 1;  // It's safe. groups_count always non-zero
  std::vector<uint16_t> groups_sizes = calculateHistogram(
    labels, max_label_value, minimal_group_size + 1);
  // The group of pixels labeled 0 corresponds to empty map cells.
  // Zero bin of the histogram is equal to the number of pixels in this group.
  // Because the values of empty map cells should not be changed, we will reset this bin
  groups_sizes.front() = 0;  // don't change image background value

  // Replace the pixel values from the small groups to background code
  const std::vector<uint8_t> lookup_table = makeLookupTable(groups_sizes, minimal_group_size);
  convert<uint16_t, uint8_t>(
    labels, image, [&lookup_table, this](uint16_t src, uint8_t & trg) {
      if (trg == filled_cell_value) {  // This check is required for non-binary input image
        trg = lookup_table[src];
      }
    });
}

void
DenoiseLayer::checkImageType(const cv::Mat & image, int cv_type, const char * error_prefix) const
{
  if (image.type() != cv_type) {
    std::string description = std::string(error_prefix) + " expected image type " +
      std::to_string(cv_type) + " but got " + std::to_string(image.type());
    throw std::logic_error(description);
  }
}

void
DenoiseLayer::checkImagesSizesEqual(
  const cv::Mat & a, const cv::Mat & b, const char * error_prefix) const
{
  if (a.size() != b.size()) {
    std::string description = std::string(error_prefix) + " images sizes are different";
    throw std::logic_error(description);
  }
}

std::vector<uint16_t>
DenoiseLayer::calculateHistogram(const cv::Mat & image, uint16_t image_max, uint16_t bin_max) const
{
  checkImageType(image, CV_16UC1, "DenoiseLayer::calculateHistogram");

  if (image.empty()) {
    return {};
  }
  std::vector<uint16_t> histogram(image_max + 1);

  // Increases the bin value corresponding to the pixel by one
  auto add_pixel_value = [&histogram, bin_max](uint8_t pixel) {
      auto & h = histogram[pixel];
      h = std::min(uint16_t(h + 1), bin_max);
    };

  // Loops through all pixels in the image and updates the histogram at each iteration
  for (int row = 0; row < image.rows; ++row) {
    auto input_line_begin = image.ptr<const uint16_t>(row);
    auto input_line_end = input_line_begin + image.cols;
    std::for_each(input_line_begin, input_line_end, add_pixel_value);
  }
  return histogram;
}

std::vector<uint8_t>
DenoiseLayer::makeLookupTable(const std::vector<uint16_t> & groups_sizes, uint16_t threshold) const
{
  std::vector<uint8_t> lookup_table(groups_sizes.size(), empty_cell_value);

  auto transform_fn = [&threshold, this](uint16_t bin_value) {
      if (bin_value >= threshold) {
        return filled_cell_value;
      }
      return empty_cell_value;
    };
  std::transform(groups_sizes.begin(), groups_sizes.end(), lookup_table.begin(), transform_fn);
  return lookup_table;
}

void
DenoiseLayer::removeSinglePixels(cv::Mat & image) const
{
  const int shape_code = group_connectivity_type == ConnectivityType::Way4 ?
    cv::MorphShapes::MORPH_CROSS : cv::MorphShapes::MORPH_RECT;
  cv::Mat shape = cv::getStructuringElement(shape_code, {3, 3});
  shape.at<uint8_t>(1, 1) = 0;

  cv::Mat max_neighbors_image;
  // Building a map of 4 or 8-connected neighbors.
  // The pixel of the map is 255 if there is an obstacle nearby
  cv::dilate(image, max_neighbors_image, shape);

  convert<uint8_t, uint8_t>(
    max_neighbors_image, image, [this](uint8_t maxNeighbor, uint8_t & img) {
      // img == filled_cell_value is required for non-binary input image
      if (maxNeighbor != filled_cell_value && img == filled_cell_value) {
        img = empty_cell_value;
      }
    });
}

}  // namespace nav2_costmap_2d

// This is the macro allowing a DenoiseLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DenoiseLayer, nav2_costmap_2d::Layer)
