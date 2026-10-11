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
#include "nav2_path_classifier/utils.hpp"

namespace nav2_path_classifier::utils
{

using nav2_msgs::msg::ClassifiedPose;
using visualization_msgs::msg::Marker;

/**
 * @brief Check that a color has the expected RGB components.
 * @param color Color to check
 * @param r Expected red
 * @param g Expected green
 * @param b Expected blue
 */
static void expectRgb(const std_msgs::msg::ColorRGBA & color, float r, float g, float b)
{
  EXPECT_NEAR(color.r, r, 1e-6);
  EXPECT_NEAR(color.g, g, 1e-6);
  EXPECT_NEAR(color.b, b, 1e-6);
}

/**
 * @brief Build a classified path with one pose per class, where pose i is at x = i.
 * @param classes Class of each pose in order
 * @return Classified path
 */
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

/**
 * @brief Build a marker header in the map frame.
 * @return Header
 */
static std_msgs::msg::Header makeHeader()
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  return header;
}

TEST(UtilsTest, HsvToRgbCoversEverySector)
{
  // Middle of each 60 degree sector at full saturation and brightness
  expectRgb(hsvToRgb(0.5 / 6.0, 1.0, 1.0), 1.0f, 0.5f, 0.0f);
  expectRgb(hsvToRgb(1.5 / 6.0, 1.0, 1.0), 0.5f, 1.0f, 0.0f);
  expectRgb(hsvToRgb(2.5 / 6.0, 1.0, 1.0), 0.0f, 1.0f, 0.5f);
  expectRgb(hsvToRgb(3.5 / 6.0, 1.0, 1.0), 0.0f, 0.5f, 1.0f);
  expectRgb(hsvToRgb(4.5 / 6.0, 1.0, 1.0), 0.5f, 0.0f, 1.0f);
  expectRgb(hsvToRgb(5.5 / 6.0, 1.0, 1.0), 1.0f, 0.0f, 0.5f);
}

TEST(UtilsTest, HsvToRgbZeroSaturationIsGrey)
{
  expectRgb(hsvToRgb(0.3, 0.0, 0.7), 0.7f, 0.7f, 0.7f);
}

TEST(UtilsTest, HsvToRgbAlphaIsOpaque)
{
  EXPECT_FLOAT_EQ(hsvToRgb(0.42, 0.5, 0.5).a, 1.0f);
}

TEST(UtilsTest, ColorForClassUsesPaletteForFirstSixClasses)
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

TEST(UtilsTest, ColorForClassBeyondPaletteUsesGoldenRatioHue)
{
  for (uint16_t c = 6; c < 20; ++c) {
    const auto expected = hsvToRgb(std::fmod(c * 0.6180339887498949, 1.0), 0.85, 0.95);
    const auto color = colorForClass(c);
    expectRgb(color, expected.r, expected.g, expected.b);
    EXPECT_FLOAT_EQ(color.a, 1.0f);
  }
}

TEST(UtilsTest, ColorForClassIsDeterministicAndDistinct)
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

TEST(UtilsTest, RawPathMarkersEmptyPathOnlyDeletes)
{
  const auto markers = createRawPathMarkers({}, makeHeader());
  ASSERT_EQ(markers.markers.size(), 1u);
  EXPECT_EQ(markers.markers[0].action, Marker::DELETEALL);
  EXPECT_EQ(markers.markers[0].ns, "raw_classified_path");
  EXPECT_EQ(markers.markers[0].header.frame_id, "map");
}

TEST(UtilsTest, RawPathMarkersOneSpherePerPose)
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

TEST(UtilsTest, FilteredPathMarkersEmptyPathHasEmptyStrip)
{
  const auto markers = createFilteredPathMarkers({}, makeHeader());
  ASSERT_EQ(markers.markers.size(), 2u);
  EXPECT_EQ(markers.markers[0].action, Marker::DELETEALL);
  EXPECT_TRUE(markers.markers[1].points.empty());
  EXPECT_TRUE(markers.markers[1].colors.empty());
}

TEST(UtilsTest, FilteredPathMarkersOneColoredStrip)
{
  const auto path = makePath({1, 1, 2, 2, 3});
  const auto markers = createFilteredPathMarkers(path, makeHeader());

  ASSERT_EQ(markers.markers.size(), 2u);
  EXPECT_EQ(markers.markers[0].action, Marker::DELETEALL);
  EXPECT_EQ(markers.markers[0].ns, "filtered_classified_path");

  const auto & strip = markers.markers[1];
  EXPECT_EQ(strip.header.frame_id, "map");
  EXPECT_EQ(strip.ns, "filtered_classified_path");
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

}  // namespace nav2_path_classifier::utils

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
