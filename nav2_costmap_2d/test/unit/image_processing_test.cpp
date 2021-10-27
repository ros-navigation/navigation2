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

#include <gtest/gtest.h>
#include <cmath>

#include "nav2_costmap_2d/image_processing.hpp"
#include "image_tests_helper.hpp"

using namespace nav2_costmap_2d;
using namespace imgproc_impl;

TEST(MemoryBuffer, complexTest) {
  MemoryBuffer buf(2);
  ASSERT_EQ(buf.capacity(), 2ul);
  ASSERT_NO_THROW(buf.get<u_int16_t>(1));
  ASSERT_THROW(buf.get<u_int16_t>(2), std::logic_error);
}

TEST(BorderConstant, outOfBoundsAccess) {
  // check access to nullptr row (up)
  {
    BorderConstant<uint8_t> c(nullptr, nullptr, 2);
    uint8_t * any_non_null = reinterpret_cast<uint8_t *>(&c);
    ASSERT_EQ(c.up(any_non_null), uint8_t(0));
  }
  // check out of bounds access
  {
    std::array<uint8_t, 3> data = {1, 2, 3};
    BorderConstant<uint8_t> c(data.data(), data.data(), 2);
    auto leftOutOfBounds = std::prev(data.data());
    auto rightOutOfBounds = data.data() + data.size();

    ASSERT_EQ(c.up(leftOutOfBounds), uint8_t(0));
    ASSERT_EQ(c.up(rightOutOfBounds), uint8_t(0));

    ASSERT_EQ(c.down(leftOutOfBounds), uint8_t(0));
    ASSERT_EQ(c.down(rightOutOfBounds), uint8_t(0));
  }
}

TEST(Histogram, calculateHistogramWithoutTruncation) {
  std::array<uint16_t, 9> data = {0, 2, 1, 0, 3, 4, 1, 2, 0};
  Image<uint16_t> image(3, 3, data.data(), 3);
  const uint16_t max_bin_size = 3;  // three zeros
  const uint16_t max_value = 4;
  const auto hist = histogram(image, max_value, max_bin_size);

  const std::array<uint8_t, 5> expected = {3, 2, 2, 1, 1};
  ASSERT_EQ(hist.size(), expected.size());
  ASSERT_TRUE(std::equal(expected.begin(), expected.end(), hist.begin()));
}

TEST(Histogram, calculateHistogramWithTruncation) {
  std::array<uint16_t, 9> data = {0, 2, 1, 0, 3, 4, 1, 2, 0};
  Image<uint16_t> image(3, 3, data.data(), 3);
  const uint16_t max_bin_size = 2;
  const uint16_t max_value = 4;
  const auto hist = histogram(image, max_value, max_bin_size);

  const std::array<uint8_t, 5> expected = {2, 2, 2, 1, 1};
  ASSERT_EQ(hist.size(), expected.size());
  ASSERT_TRUE(std::equal(expected.begin(), expected.end(), hist.begin()));
}

TEST(Histogram, calculateHistogramOfEmpty) {
  Image<uint16_t> image;
  const uint16_t max_bin_size = 1;
  const uint16_t max_value = 0;

  const auto hist = histogram(image, max_value, max_bin_size);
  ASSERT_TRUE(hist.empty());
}


TEST(EquivalenceLabelTrees, newLabelsTest) {
  std::array<uint8_t, 4> buffer{};
  EquivalenceLabelTrees<uint8_t> eq(buffer.data(), buffer.size());
  ASSERT_EQ(eq.makeLabel(), 1);
  ASSERT_EQ(eq.makeLabel(), 2);
  ASSERT_EQ(eq.makeLabel(), 3);
}

TEST(EquivalenceLabelTrees, unionTest) {
  std::array<uint8_t, 6> buffer{};
  EquivalenceLabelTrees<uint8_t> eq(buffer.data(), buffer.size());
  // create 5 single nodes
  for (size_t i = 1; i < 6; ++i) {
    eq.makeLabel();
  }
  ASSERT_EQ(eq.unionTrees(4, 3), 3);
  ASSERT_EQ(eq.unionTrees(5, 3), 3);
  ASSERT_EQ(eq.unionTrees(4, 1), 1);
  ASSERT_EQ(eq.unionTrees(2, 5), 1);
}

TEST(EquivalenceLabelTrees, emptyBuffer) {
  ASSERT_THROW(EquivalenceLabelTrees<uint8_t>(nullptr, 0), std::logic_error);
}

struct ConnectedComponentsTester : public ::testing::Test
{
  ConnectedComponentsTester()
  : buffer(1024 * 1024) {}

protected:
  MemoryBuffer buffer;
};

TEST_F(ConnectedComponentsTester, way4EmptyTest) {
  Image<uint8_t> input;
  const auto result = ConnectedComponents<ConnectivityType::Way4, uint8_t>::detect(input, buffer);
  ASSERT_EQ(result.second, uint8_t(0));
}

TEST_F(ConnectedComponentsTester, way4SinglePixelTest) {
  {
    Image<uint8_t> input(1, 1);
    input.at(0, 0) = 0;

    const auto result = ConnectedComponents<ConnectivityType::Way4, uint8_t>::detect(input, buffer);

    ASSERT_EQ(result.first.at(0, 0), 0);
    ASSERT_EQ(result.second, 1);
  }
  {
    Image<uint8_t> input(1, 1);
    input.at(0, 0) = 255;

    const auto result = ConnectedComponents<ConnectivityType::Way4, uint8_t>::detect(input, buffer);

    ASSERT_EQ(result.first.at(0, 0), 1);
    ASSERT_EQ(result.second, 2);
  }
}

TEST_F(ConnectedComponentsTester, way4ImageSmallTest) {
  {
    Image<uint8_t> input(1, 2);
    input.at(0, 0) = 0;
    input.at(0, 1) = 255;

    const auto result = ConnectedComponents<ConnectivityType::Way4, uint8_t>::detect(input, buffer);

    ASSERT_EQ(result.second, uint8_t(2));
    ASSERT_EQ(result.first.at(0, 0), 0);
    ASSERT_EQ(result.first.at(0, 1), 1);
  }
  {
    Image<uint8_t> input(2, 1);
    input.at(0, 0) = 0;
    input.at(1, 0) = 255;

    const auto result = ConnectedComponents<ConnectivityType::Way4, uint8_t>::detect(input, buffer);

    ASSERT_EQ(result.second, uint8_t(2));
    ASSERT_EQ(result.first.at(0, 0), 0);
    ASSERT_EQ(result.first.at(1, 0), 1);
  }
}

TEST_F(ConnectedComponentsTester, way4LabelsOverflowTest) {
  // big chessboard image
  Image<uint8_t> input(32, 17);
  uint8_t v = 255;
  input.forEach(
    [&v](uint8_t & pixel) {
      pixel = v;
      if (v == 0) {
        v = 255;
      } else {
        v = 0;
      }
    });

  ASSERT_THROW(
    (ConnectedComponents<ConnectivityType::Way4, uint8_t>::detect(
      input, buffer)),
    std::logic_error);
}

template<class T>
struct UniformLabel
{
  const Image<T> & labels;
  std::map<T, T> labels_map = {{0, 0}};
  size_t next_label = 1;
  T at(size_t i, size_t j)
  {
    const T label = labels.at(i, j);

    if (labels_map.find(label) == labels_map.end()) {
      labels_map[label] = next_label++;
    }
    return labels_map[label];
  }
};

template<class T>
bool isEqualLabels(const Image<T> & lhs, const Image<T> & rhs)
{
  UniformLabel<T> l = {lhs};
  UniformLabel<T> r = {rhs};

  for (size_t i = 0; i < lhs.rows(); ++i) {
    for (size_t j = 0; j < lhs.rows(); ++j) {
      if (l.at(i, j) != r.at(i, j)) {
        return false;
      }
    }
  }
  return true;
}

TEST_F(ConnectedComponentsTester, way4ImageStepsTest) {
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "..xx"
    ".xx."
    "xx.."
    "....");
  const Image<uint8_t> expected_labels = imageFromString<uint8_t>(
    "..xx"
    ".xx."
    "xx.."
    "....");
  auto result = ConnectedComponents<ConnectivityType::Way4, uint8_t>::detect(input, buffer);

  ASSERT_EQ(result.second, uint8_t(2));
  ASSERT_TRUE(isEqualLabels(result.first, expected_labels));
}

/// @brief create mapping '.'->0, 'a'->1, 'b'->2, ... max_symbol->n
std::map<char, uint8_t> makeLabelsMap(char max_symbol)
{
  std::map<char, uint8_t> labels_map = {{'.', 0}};

  for (char s = 'a'; s <= max_symbol; ++s) {
    labels_map.emplace(s, uint8_t(s - 'a' + 1));
  }
  return labels_map;
}

TEST_F(ConnectedComponentsTester, way8ImageStepsTest) {
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "....xx"
    "..xx.."
    "xx...."
    "...xx."
    ".....x"
    "....x.");
  const Image<uint8_t> expected_labels = imageFromString<uint8_t>(
    "....aa"
    "..aa.."
    "aa...."
    "...bb."
    ".....b"
    "....b.", makeLabelsMap('b'));

  auto result = ConnectedComponents<ConnectivityType::Way8, uint8_t>::detect(input, buffer);

  ASSERT_EQ(result.second, uint8_t(3));
  ASSERT_TRUE(isEqualLabels(result.first, expected_labels));
}

TEST_F(ConnectedComponentsTester, way4ImageSieveTest) {
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "x.x.x"
    ".x.x."
    "x.x.x"
    ".x.x."
    "x.x.x");
  const Image<uint8_t> expected_labels = imageFromString<uint8_t>(
    "a.b.c"
    ".d.e."
    "f.g.h"
    ".i.j."
    "k.l.m", makeLabelsMap('m'));

  auto result = ConnectedComponents<ConnectivityType::Way4, uint8_t>::detect(input, buffer);

  ASSERT_EQ(result.second, uint8_t(14));
  ASSERT_TRUE(isEqualLabels(result.first, expected_labels));
}

template<ConnectivityType connectivity>
bool fingerTest(MemoryBuffer & buffer)
{
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "....."
    "....x"
    "..x.x"
    "x.x.x"
    "x.x.x");
  const Image<uint8_t> expected_labels = imageFromString<uint8_t>(
    "....."
    "....c"
    "..b.c"
    "a.b.c"
    "a.b.c", makeLabelsMap('c'));

  auto result = ConnectedComponents<connectivity, uint8_t>::detect(input, buffer);

  return result.second == 4 && isEqualLabels(result.first, expected_labels);
}

TEST_F(ConnectedComponentsTester, way4ImageFingerTest) {
  ASSERT_TRUE(fingerTest<ConnectivityType::Way4>(buffer));
}

TEST_F(ConnectedComponentsTester, way8ImageFingerTest) {
  ASSERT_TRUE(fingerTest<ConnectivityType::Way8>(buffer));
}

template<ConnectivityType connectivity>
bool spiralTest(MemoryBuffer & buffer)
{
  const Image<uint8_t> input = imageFromString<uint8_t>(
    ".xxxxxx"
    "......x"
    ".xxxx.x"
    ".x..x.x"
    ".x.xx.x"
    ".x....x"
    ".xxxxxx");
  const Image<uint8_t> expected_labels = imageFromString<uint8_t>(
    ".xxxxxx"
    "......x"
    ".xxxx.x"
    ".x..x.x"
    ".x.xx.x"
    ".x....x"
    ".xxxxxx");

  auto result = ConnectedComponents<connectivity, uint8_t>::detect(input, buffer);
  return result.second == 2 && isEqualLabels(result.first, expected_labels);
}

TEST_F(ConnectedComponentsTester, way4ImageSpiralTest) {
  ASSERT_TRUE(spiralTest<ConnectivityType::Way4>(buffer));
}

TEST_F(ConnectedComponentsTester, way8ImageSpiralTest) {
  ASSERT_TRUE(spiralTest<ConnectivityType::Way8>(buffer));
}

const Image<uint8_t> crossShape = []() {
    Image<uint8_t> shape(3, 3);
    shape.fill(0);
    shape.at(1, 0) = 255;
    shape.at(1, 2) = 255;
    shape.at(0, 1) = 255;
    shape.at(2, 1) = 255;
    return shape;
  } ();

uint8_t max_list(std::initializer_list<uint8_t> lst)
{
  return std::max(lst);
}

TEST(morphology_operation, emptyImage) {
  Image<uint8_t> input;
  Image<uint8_t> output;

  ASSERT_NO_THROW(morphology_operation(input, output, crossShape, max_list));
}

TEST(morphology_operation, wrongShapeSize) {
  Image<uint8_t> input(1, 1);
  Image<uint8_t> output(1, 1);
  ASSERT_THROW(
    morphology_operation(input, output, Image<uint8_t>(2, 2), max_list),
    std::logic_error);
}

TEST(morphology_operation, wrongSize) {
  Image<uint8_t> input(3, 2);

  {
    Image<uint8_t> output(2, 2);
    ASSERT_THROW(
      morphology_operation(input, output, crossShape, max_list),
      std::logic_error);
  }

  {
    Image<uint8_t> output(3, 3);
    ASSERT_THROW(
      morphology_operation(input, output, crossShape, max_list),
      std::logic_error);
  }
}

TEST(morphology_operation, singlePixelImage) {
  Image<uint8_t> input(1, 1);
  Image<uint8_t> output(1, 1);
  input.fill(255);

  morphology_operation(input, output, crossShape, max_list);

  ASSERT_EQ(output.at(0, 0), 0);
}

TEST(morphology_operation, cornersImage) {
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "x..x"
    "...."
    "...."
    "x..x");
  Image<uint8_t> expected = imageFromString<uint8_t>(
    ".xx."
    "x..x"
    "x..x"
    ".xx.");
  Image<uint8_t> output(input.rows(), input.columns());

  morphology_operation(input, output, crossShape, max_list);

  ASSERT_TRUE(isEqual(output, expected));
}

TEST(morphology_operation, horizontalBordersImage) {
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "x..x"
    "x..x"
    "x..x"
    "x..x");
  Image<uint8_t> expected = imageFromString<uint8_t>(
    "xxxx"
    "xxxx"
    "xxxx"
    "xxxx");
  Image<uint8_t> output(input.rows(), input.columns());

  morphology_operation(input, output, crossShape, max_list);

  ASSERT_TRUE(isEqual(output, expected));
}

TEST(morphology_operation, verticalBordersImage) {
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "xxxx"
    "...."
    "...."
    "xxxx");
  Image<uint8_t> expected = imageFromString<uint8_t>(
    "xxxx"
    "xxxx"
    "xxxx"
    "xxxx");
  Image<uint8_t> output(input.rows(), input.columns());

  morphology_operation(input, output, crossShape, max_list);

  ASSERT_TRUE(isEqual(output, expected));
}
