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

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "nav2_msgs/msg/classified_pose.hpp"
#include "nav2_behavior_tree/utils/classified_path_markers.hpp"

namespace nav2_behavior_tree::utils
{

using nav2_msgs::msg::ClassifiedPose;
using visualization_msgs::msg::Marker;

// Check that a color has the expected RGB components
static void expectRgb(const std_msgs::msg::ColorRGBA & color, float r, float g, float b)
{
  EXPECT_NEAR(color.r, r, 1e-6);
  EXPECT_NEAR(color.g, g, 1e-6);
  EXPECT_NEAR(color.b, b, 1e-6);
}

// Classified path with one pose per class, where pose i is at x = i
static std::vector<ClassifiedPose> makePath(const std::vector<uint16_t> & classes)
{
  std::vector<ClassifiedPose> path(classes.size());
  for (size_t i = 0; i < classes.size(); ++i) {
    path[i].pose.pose.position.x = static_cast<double>(i);
    path[i].pose.pose.orientation.w = 1.0;
    path[i].class_type = classes[i];
  }
  return path;
}

// Marker header in the map frame
static std_msgs::msg::Header makeHeader()
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  return header;
}

TEST(ClassifiedPathMarkersTest, ColorForClassUsesPaletteForFirstSixClasses)
{
  expectRgb(colorForClass(0), 0.2f, 0.4f, 1.0f);
  expectRgb(colorForClass(1), 0.2f, 0.8f, 0.2f);
  expectRgb(colorForClass(2), 0.9f, 0.1f, 0.1f);
  expectRgb(colorForClass(3), 1.0f, 0.6f, 0.0f);
  expectRgb(colorForClass(4), 0.8f, 0.2f, 0.8f);
  expectRgb(colorForClass(5), 0.2f, 0.8f, 0.8f);
  for (uint16_t c = 0; c < 6; ++c) {
    EXPECT_FLOAT_EQ(colorForClass(c).a, 1.0f);
  }
}

TEST(ClassifiedPathMarkersTest, ColorForClassIsDeterministicAndDistinct)
{
  std::vector<std_msgs::msg::ColorRGBA> colors;
  for (uint16_t c = 0; c < 100; ++c) {
    const auto color = colorForClass(c);
    expectRgb(colorForClass(c), color.r, color.g, color.b);
    colors.push_back(color);
  }
  for (size_t i = 0; i < colors.size(); ++i) {
    for (size_t j = i + 1; j < colors.size(); ++j) {
      const bool same = std::fabs(colors[i].r - colors[j].r) < 1e-6 &&
        std::fabs(colors[i].g - colors[j].g) < 1e-6 &&
        std::fabs(colors[i].b - colors[j].b) < 1e-6;
      EXPECT_FALSE(same) << "classes " << i << " and " << j << " share a color";
    }
  }
}

TEST(ClassifiedPathMarkersTest, RawPathMarkersOneSpherePerPose)
{
  const auto path = makePath({1, 2, 3});
  const auto markers = createRawPathMarkers(path, makeHeader());

  ASSERT_EQ(markers.markers.size(), path.size() + 1);
  EXPECT_EQ(markers.markers[0].action, Marker::DELETEALL);
  for (size_t i = 0; i < path.size(); ++i) {
    const auto & m = markers.markers[i + 1];
    EXPECT_EQ(m.header.frame_id, "map");
    EXPECT_EQ(m.ns, "raw_classified_path");
    EXPECT_EQ(m.id, static_cast<int>(i));
    EXPECT_EQ(m.type, Marker::SPHERE);
    EXPECT_EQ(m.action, Marker::ADD);
    EXPECT_DOUBLE_EQ(m.scale.x, 0.06);
    EXPECT_DOUBLE_EQ(m.scale.y, 0.06);
    EXPECT_DOUBLE_EQ(m.scale.z, 0.06);
    EXPECT_DOUBLE_EQ(m.pose.position.x, path[i].pose.pose.position.x);
    const auto expected = colorForClass(path[i].class_type);
    expectRgb(m.color, expected.r, expected.g, expected.b);
    EXPECT_FLOAT_EQ(m.color.a, 0.8f);
  }
}

TEST(ClassifiedPathMarkersTest, ClassifiedPathMarkersOneColoredStrip)
{
  const auto path = makePath({1, 1, 2, 2, 3});
  const auto markers = createClassifiedPathMarkers(path, makeHeader());

  ASSERT_EQ(markers.markers.size(), 2u);
  EXPECT_EQ(markers.markers[0].action, Marker::DELETEALL);
  EXPECT_EQ(markers.markers[0].ns, "classified_path");

  const auto & strip = markers.markers[1];
  EXPECT_EQ(strip.header.frame_id, "map");
  EXPECT_EQ(strip.ns, "classified_path");
  EXPECT_EQ(strip.id, 0);
  EXPECT_EQ(strip.type, Marker::LINE_STRIP);
  EXPECT_EQ(strip.action, Marker::ADD);
  EXPECT_DOUBLE_EQ(strip.scale.x, 0.05);
  ASSERT_EQ(strip.points.size(), path.size());
  ASSERT_EQ(strip.colors.size(), path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    EXPECT_DOUBLE_EQ(strip.points[i].x, path[i].pose.pose.position.x);
    const auto expected = colorForClass(path[i].class_type);
    expectRgb(strip.colors[i], expected.r, expected.g, expected.b);
    EXPECT_FLOAT_EQ(strip.colors[i].a, 0.9f);
  }
}

}  // namespace nav2_behavior_tree::utils

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
