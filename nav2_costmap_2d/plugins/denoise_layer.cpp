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

#include "nav2_costmap_2d/denoise_layer.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d
{

void
DenoiseLayer::onInitialize()
{
  // Enable/disable plugin
  declareParameter("enabled", rclcpp::ParameterValue(true));
  // Smaller groups should be filtered
  declareParameter("minimal_group_size", rclcpp::ParameterValue(2));
  // Pixels connectivity type
  declareParameter("group_connectivity_type", rclcpp::ParameterValue(8));

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
    minimal_group_size_ = 1;
  } else {
    minimal_group_size_ = static_cast<size_t>(minimal_group_size_param);
  }

  const int group_connectivity_type_param = getInt("group_connectivity_type");

  if (group_connectivity_type_param == 4) {
    group_connectivity_type_ = ConnectivityType::Way4;
  } else {
    group_connectivity_type_ = ConnectivityType::Way8;

    if (group_connectivity_type_param != 8) {
      RCLCPP_WARN(
        logger_, "DenoiseLayer::onInitialize(): param group_connectivity_type: %i."
        " Possible values are  4 (neighbors pixels are connected horizontally and vertically) "
        "or 8 (neighbors pixels are connected horizontally, vertically and diagonally)."
        "The default value 8 will be used",
        group_connectivity_type_param);
    }
  }

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
  no_information_is_obstacle_ = master_grid.getDefaultValue() != NO_INFORMATION;

  // wrap roi_image over existing costmap2d buffer
  unsigned char * master_array = master_grid.getCharMap();
  const int step = static_cast<int>(master_grid.getSizeInCellsX());

  const size_t width = max_x - min_x;
  const size_t height = max_y - min_y;
  Image<uint8_t> roi_image(height, width, master_array + min_y * step + min_x, step);

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

void
DenoiseLayer::removeGroups(Image<uint8_t> & image) const
{
  groups_remover_.removeGroups(
    image, buffer_, group_connectivity_type_, minimal_group_size_,
    [this](uint8_t pixel) {return isBackground(pixel);});
}

void
DenoiseLayer::removeSinglePixels(Image<uint8_t> & image) const
{
  // Building a map of 4 or 8-connected neighbors.
  // The pixel of the map is 255 if there is an obstacle nearby
  uint8_t * buf = buffer_.get<uint8_t>(image.rows() * image.columns());
  Image<uint8_t> max_neighbors_image(image.rows(), image.columns(), buf, image.columns());

  // If NO_INFORMATION (=255) isn't obstacle, we can't use a simple max() to check
  // any obstacle nearby. In this case, we interpret NO_INFORMATION as an empty space.
  if (!no_information_is_obstacle_) {
    auto replace_to_free = [](uint8_t v) {
        return v == NO_INFORMATION ? FREE_SPACE : v;
      };
    auto max = [&](const std::initializer_list<uint8_t> lst) {
        std::array<uint8_t, 3> buf = {
          replace_to_free(*lst.begin()),
          replace_to_free(*(lst.begin() + 1)),
          replace_to_free(*(lst.begin() + 2))
        };
        return *std::max_element(buf.begin(), buf.end());
      };
    dilate(image, max_neighbors_image, group_connectivity_type_, max);
  } else {
    auto max = [](const std::initializer_list<uint8_t> lst) {
        return std::max(lst);
      };
    dilate(image, max_neighbors_image, group_connectivity_type_, max);
  }

  max_neighbors_image.convert(
    image, [this](uint8_t maxNeighbor, uint8_t & img) {
      if (!isBackground(img) && isBackground(maxNeighbor)) {
        img = FREE_SPACE;
      }
    });
}

bool DenoiseLayer::isBackground(uint8_t pixel) const
{
  bool is_obstacle =
    pixel == LETHAL_OBSTACLE ||
    pixel == INSCRIBED_INFLATED_OBSTACLE ||
    (pixel == NO_INFORMATION && no_information_is_obstacle_);
  return !is_obstacle;
}

}  // namespace nav2_costmap_2d

// This is the macro allowing a DenoiseLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DenoiseLayer, nav2_costmap_2d::Layer)
