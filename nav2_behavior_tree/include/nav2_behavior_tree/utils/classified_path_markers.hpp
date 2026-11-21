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

#ifndef NAV2_BEHAVIOR_TREE__UTILS__CLASSIFIED_PATH_MARKERS_HPP_
#define NAV2_BEHAVIOR_TREE__UTILS__CLASSIFIED_PATH_MARKERS_HPP_

#include <array>
#include <cstdint>
#include <random>
#include <vector>

#include "nav2_msgs/msg/classified_pose.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_behavior_tree::utils
{

/**
 * @brief Marker color for a path class, fixed per class across calls and runs
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

  std_msgs::msg::ColorRGBA color;
  color.a = 1.0f;
  if (class_type < kClassPalette.size()) {
    const auto & c = kClassPalette[class_type];
    color.r = c[0];
    color.g = c[1];
    color.b = c[2];
    return color;
  }

  // Raw engine output seeded by the class, as the standard fixes it on every platform
  std::mt19937 gen(class_type);
  const uint32_t bits = gen();
  color.r = static_cast<float>((bits >> 16) & 0xFF) / 255.0f;
  color.g = static_cast<float>((bits >> 8) & 0xFF) / 255.0f;
  color.b = static_cast<float>(bits & 0xFF) / 255.0f;
  return color;
}

/**
 * @brief Per-pose spheres colored by each pose's raw class
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
 * @brief Line strip of the path colored by each pose's class
 * @param classified_path Classified poses in path order, after filtering
 * @param header Header for the markers
 * @return Marker array to publish
 */
inline visualization_msgs::msg::MarkerArray createClassifiedPathMarkers(
  const std::vector<nav2_msgs::msg::ClassifiedPose> & classified_path,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray markers;

  // Delete previous markers
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  del.header = header;
  del.ns = "classified_path";
  markers.markers.push_back(del);

  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns = "classified_path";
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

}  // namespace nav2_behavior_tree::utils

#endif  // NAV2_BEHAVIOR_TREE__UTILS__CLASSIFIED_PATH_MARKERS_HPP_
