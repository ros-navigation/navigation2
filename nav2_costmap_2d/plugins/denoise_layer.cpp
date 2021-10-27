// Copyright (c) 2021 Andrey Ryzhikov
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

#include "nav2_costmap_2d/denoise_layer.hpp"

#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d
{
static constexpr unsigned char OBSTACLE_CELL = 255;

void
DenoiseLayer::onInitialize()
{
  // Enable/disable plugin
  declareParameter("enabled", rclcpp::ParameterValue(true));
  // Smaller groups should be filtered
  declareParameter("minimal_group_size", rclcpp::ParameterValue(2));
  // Pixels connectivity type
  declareParameter("group_connectivity_type", rclcpp::ParameterValue(8));
  // Max additional memory amount
  declareParameter("memory_usage_mb", rclcpp::ParameterValue(0));

  const auto node = node_.lock();

  if (!node) {
    throw std::runtime_error("DenoiseLayer::onInitialize: Failed to lock node");
  }
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
    this->minimal_group_size_ = 1;
  } else {
    this->minimal_group_size_ = static_cast<size_t>(minimal_group_size_param);
  }

  const int group_connectivity_type_param = getInt("group_connectivity_type");

  if (group_connectivity_type_param == 4) {
    this->group_connectivity_type_ = ConnectivityType::Way4;
  } else if (group_connectivity_type_param == 8) {
    this->group_connectivity_type_ = ConnectivityType::Way8;
  } else {
    RCLCPP_WARN(
      logger_, "DenoiseLayer::onInitialize(): param group_connectivity_type: %i."
      " Possible values are  4 (neighbors pixels are connected horizontally and vertically) "
      "or 8 (neighbors pixels are connected horizontally, vertically and diagonally)."
      "The default value 8 will be used",
      group_connectivity_type_param);
    this->group_connectivity_type_ = ConnectivityType::Way8;
  }

  int memory_usage_mb = getInt("memory_usage_mb");
  size_t memory_usage = std::numeric_limits<size_t>::max();

  if (memory_usage_mb > 0) {
    const size_t byteToMb = 1024 * 1024;
    memory_usage = size_t(memory_usage_mb) * byteToMb;
  }
  buffer = std::make_unique<MemoryBuffer>(memory_usage);
  current_ = true;
}

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

  if (min_x >= max_x || min_y >= max_y) {
    return;
  }

  // wrap roi_image over existing costmap2d buffer
  unsigned char * master_array = master_grid.getCharMap();
  const int step = static_cast<int>(master_grid.getSizeInCellsX());

  const size_t width = max_x - min_x;
  const size_t height = max_y - min_y;
  Image<uint8_t> roi_image(height, width, master_array + min_x, step);

  try {
    denoise(roi_image);
  } catch (std::exception & ex) {
    RCLCPP_ERROR(logger_, (std::string("Inner error: ") + ex.what()).c_str());
  }

  current_ = true;
}

void
DenoiseLayer::denoise(Image<uint8_t> & image) const
{
  if (image.empty()) {
    return;
  }

  if (minimal_group_size_ <= 1) {
    return;  // A smaller group cannot exist. No one pixel will be changed
  }

  if (minimal_group_size_ == 2) {
    // Performs fast filtration based on erosion function
    removeSinglePixels(image);
  } else {
    // Performs a slower segmentation-based operation
    removeGroups(image);
  }
}

template<ConnectivityType connectivity, class Label>
void removeGroupsImpl(Image<uint8_t> & image, MemoryBuffer & buffer, size_t minimal_group_size)
{
  // Creates an image labels in which each obstacles group is labeled with a unique code
  auto components = ConnectedComponents<connectivity, Label>::detect(image, buffer);
  const Label groups_count = components.second;
  const Image<Label> & labels = components.first;

  // Calculates the size of each group.
  // Group size is equal to the number of pixels with the same label
  const Label max_label_value = groups_count - 1;  // It's safe. groups_count always non-zero
  std::vector<size_t> groups_sizes = histogram(
    labels, max_label_value, minimal_group_size + 1);

  // The group of pixels labeled 0 corresponds to empty map cells.
  // Zero bin of the histogram is equal to the number of pixels in this group.
  // Because the values of empty map cells should not be changed, we will reset this bin
  groups_sizes.front() = 0;  // don't change image background value

  // lookup_table[i] = OBSTACLE_CELL if groups_sizes[i] >= minimal_group_size, FREE_SPACE in other case
  std::vector<uint8_t> lookup_table(groups_sizes.size());
  auto transform_fn = [&minimal_group_size](size_t bin_value) {
      return bin_value < minimal_group_size ? FREE_SPACE : OBSTACLE_CELL;
    };
  std::transform(groups_sizes.begin(), groups_sizes.end(), lookup_table.begin(), transform_fn);

  // Replace the pixel values from the small groups to background code
  labels.convert(
    image, [&lookup_table](Label src, uint8_t & trg) {
      if (trg == OBSTACLE_CELL) {  // This check is required for non-binary input image
        trg = lookup_table[src];
      }
    });
}

template<ConnectivityType connectivity>
void removeGroupsPickLabelType(
  Image<uint8_t> & image, MemoryBuffer & buffer,
  size_t minimal_group_size)
{
  if (ConnectedComponents<connectivity, uint32_t>::optimalBufferSize(image) <= buffer.capacity()) {
    removeGroupsImpl<connectivity, uint32_t>(image, buffer, minimal_group_size);
  } else {
    removeGroupsImpl<connectivity, uint16_t>(image, buffer, minimal_group_size);
  }
}

void
DenoiseLayer::removeGroups(Image<uint8_t> & image) const
{
  if (group_connectivity_type_ == ConnectivityType::Way4) {
    removeGroupsPickLabelType<ConnectivityType::Way4>(image, *buffer, minimal_group_size_);
  } else {
    removeGroupsPickLabelType<ConnectivityType::Way8>(image, *buffer, minimal_group_size_);
  }
}

void
DenoiseLayer::removeSinglePixels(Image<uint8_t> & image) const
{
  // Building a map of 4 or 8-connected neighbors.
  // The pixel of the map is 255 if there is an obstacle nearby
  uint8_t * buf = buffer->get<uint8_t>(image.rows() * image.columns());
  Image<uint8_t> max_neighbors_image(image.rows(), image.columns(), buf, image.columns());

  dilate(image, max_neighbors_image, group_connectivity_type_);

  max_neighbors_image.convert(
    image, [this](uint8_t maxNeighbor, uint8_t & img) {
      // img == OBSTACLE_CELL is required for non-binary input image
      if (maxNeighbor != OBSTACLE_CELL && img == OBSTACLE_CELL) {
        img = FREE_SPACE;
      }
    });
}

}  // namespace nav2_costmap_2d

// This is the macro allowing a DenoiseLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DenoiseLayer, nav2_costmap_2d::Layer)
