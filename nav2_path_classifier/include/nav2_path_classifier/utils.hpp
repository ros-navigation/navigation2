// Copyright (c) 2026 Origin
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

#ifndef NAV2_PATH_CLASSIFIER__UTILS_HPP_
#define NAV2_PATH_CLASSIFIER__UTILS_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "nav2_msgs/msg/classified_pose.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_path_classifier::utils
{

/**
 * @brief Convert an HSV color to RGB.
 * @param hue Hue in [0, 1)
 * @param saturation Saturation in [0, 1]
 * @param value Brightness in [0, 1]
 * @return Equivalent RGB color with components in [0, 1]
 */
inline std_msgs::msg::ColorRGBA hsvToRgb(double hue, double saturation, double value)
{
  const double h6 = hue * 6.0;
  const int sector = static_cast<int>(h6);
  const double f = h6 - sector;
  const double p = value * (1.0 - saturation);
  const double q = value * (1.0 - saturation * f);
  const double t = value * (1.0 - saturation * (1.0 - f));

  double r, g, b;
  switch (sector) {
    case 0: r = value; g = t; b = p; break;
    case 1: r = q; g = value; b = p; break;
    case 2: r = p; g = value; b = t; break;
    case 3: r = p; g = q; b = value; break;
    case 4: r = t; g = p; b = value; break;
    default: r = value; g = p; b = q; break;  // sector == 5
  }

  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(r);
  color.g = static_cast<float>(g);
  color.b = static_cast<float>(b);
  color.a = 1.0f;
  return color;
}

/**
 * @brief Get the marker color for a given path class_type
 * @param class_type Class identifier to map to a color
 * @return Color for the class
 */
inline std_msgs::msg::ColorRGBA colorForClass(uint16_t class_type)
{
  constexpr std::array<std::array<float, 3>, 6> kClassPalette = {{
    {{0.2f, 0.4f, 1.0f}},   // blue
    {{0.2f, 0.8f, 0.2f}},   // green
    {{0.9f, 0.1f, 0.1f}},   // red
    {{1.0f, 0.6f, 0.0f}},   // orange
    {{0.8f, 0.2f, 0.8f}},   // magenta
    {{0.2f, 0.8f, 0.8f}}    // cyan
  }};

  if (class_type < kClassPalette.size()) {
    const auto & c = kClassPalette[class_type];
    std_msgs::msg::ColorRGBA color;
    color.r = c[0];
    color.g = c[1];
    color.b = c[2];
    color.a = 1.0f;
    return color;
  }

  // Beyond the curated palette: step the hue by the golden ratio conjugate so
  // colors stay spread across the full hue circle no matter how many classes exist,
  // with no risk of repeating.
  constexpr double kGoldenRatioConjugate = 0.6180339887498949;
  constexpr double kSaturation = 0.85;
  constexpr double kValue = 0.95;
  const double hue = std::fmod(class_type * kGoldenRatioConjugate, 1.0);
  return hsvToRgb(hue, kSaturation, kValue);
}

/**
 * @brief Build per-pose spheres colored by each pose's raw class.
 * @param classified_path Classified poses in path order, before filtering
 * @param header Header for the markers
 * @return Marker array to publish
 */
inline visualization_msgs::msg::MarkerArray createRawPathMarkers(
  const std::vector<nav2_msgs::msg::ClassifiedPose> & classified_path,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray markers;

  // Delete previous markers
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  del.header = header;
  del.ns = "raw_classified_path";
  markers.markers.push_back(del);

  for (size_t i = 0; i < classified_path.size(); ++i) {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = "raw_classified_path";
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.06;
    m.scale.y = 0.06;
    m.scale.z = 0.06;
    m.color = colorForClass(classified_path[i].class_type);
    m.color.a = 0.8;
    m.pose = classified_path[i].pose.pose;
    markers.markers.push_back(m);
  }

  return markers;
}

/**
 * @brief Build a line strip of the path colored by each pose's filtered class.
 * @param classified_path Classified poses in path order, after filtering
 * @param header Header for the markers
 * @return Marker array to publish
 */
inline visualization_msgs::msg::MarkerArray createFilteredPathMarkers(
  const std::vector<nav2_msgs::msg::ClassifiedPose> & classified_path,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray markers;

  // Delete previous markers
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  del.header = header;
  del.ns = "filtered_classified_path";
  markers.markers.push_back(del);

  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns = "filtered_classified_path";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.05;
  m.pose.orientation.w = 1.0;
  for (const auto & classified_pose : classified_path) {
    m.points.push_back(classified_pose.pose.pose.position);
    auto color = colorForClass(classified_pose.class_type);
    color.a = 0.9;
    m.colors.push_back(color);
  }
  markers.markers.push_back(m);

  return markers;
}

}  // namespace nav2_path_classifier::utils

#endif  // NAV2_PATH_CLASSIFIER__UTILS_HPP_
