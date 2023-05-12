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

#include <gtest/gtest.h>
#include <cmath>

#include "nav2_costmap_2d/denoise/image_processing.hpp"
#include "image_tests_helper.hpp"

using namespace nav2_costmap_2d;
using namespace imgproc_impl;

struct ImageProcTester : public ::testing::Test
{
protected:
  std::vector<uint8_t> image_buffer_bytes_;
  std::vector<uint8_t> image_buffer_bytes2_;
  std::vector<uint8_t> image_buffer_bytes3_;
  std::vector<uint16_t> image_buffer_words_;
};

TEST(OutOfBounds, outOfBoundsAccess) {
  // check access to nullptr row (up)
  {
    out_of_bounds_policy::ReplaceToZero<uint8_t> c(nullptr, nullptr, 2);
    uint8_t * any_non_null = reinterpret_cast<uint8_t *>(&c);
    ASSERT_EQ(c.up(any_non_null), uint8_t(0));
  }
  // check out of bounds access
  {
    std::array<uint8_t, 3> data = {1, 2, 3};
    out_of_bounds_policy::ReplaceToZero<uint8_t> c(data.data(), data.data(), 2);
    auto left_out_of_bounds = std::prev(data.data());
    auto right_out_of_bounds = data.data() + data.size();

    ASSERT_EQ(c.up(left_out_of_bounds), uint8_t(0));
    ASSERT_EQ(c.up(right_out_of_bounds), uint8_t(0));

    ASSERT_EQ(c.down(left_out_of_bounds), uint8_t(0));
    ASSERT_EQ(c.down(right_out_of_bounds), uint8_t(0));
  }
}

TEST_F(ImageProcTester, calculateHistogramWithoutTruncation) {
  image_buffer_words_ = {0, 2, 1, 0, 3, 4, 1, 2, 0};
  Image<uint16_t> image = makeImage(3, 3, image_buffer_words_);
  const uint16_t max_bin_size = 3;  // three zeros
  const uint16_t max_value = 4;
  const auto hist = histogram(image, max_value, max_bin_size);

  const std::array<uint8_t, 5> expected = {3, 2, 2, 1, 1};
  ASSERT_EQ(hist.size(), expected.size());
  ASSERT_TRUE(std::equal(expected.begin(), expected.end(), hist.begin()));
}

TEST_F(ImageProcTester, calculateHistogramWithTruncation) {
  image_buffer_words_ = {0, 2, 1, 0, 3, 4, 1, 2, 0};
  Image<uint16_t> image = makeImage(3, 3, image_buffer_words_);
  const uint16_t max_bin_size = 2;
  const uint16_t max_value = 4;
  const auto hist = histogram(image, max_value, max_bin_size);

  const std::array<uint8_t, 5> expected = {2, 2, 2, 1, 1};
  ASSERT_EQ(hist.size(), expected.size());
  ASSERT_TRUE(std::equal(expected.begin(), expected.end(), hist.begin()));
}

TEST_F(ImageProcTester, calculateHistogramOfEmpty) {
  const uint16_t max_bin_size = 1;
  const uint16_t max_value = 0;
  Image<uint16_t> empty;

  const auto hist = histogram(empty, max_value, max_bin_size);
  ASSERT_TRUE(hist.empty());
}


TEST(EquivalenceLabelTrees, newLabelsTest) {
  EquivalenceLabelTrees<uint8_t> eq;
  eq.reset(10, 10, ConnectivityType::Way4);
  ASSERT_EQ(eq.makeLabel(), 1);
  ASSERT_EQ(eq.makeLabel(), 2);
  ASSERT_EQ(eq.makeLabel(), 3);
}

TEST(EquivalenceLabelTrees, unionTest) {
  EquivalenceLabelTrees<uint8_t> eq;
  eq.reset(10, 10, ConnectivityType::Way4);

  // create 5 single nodes
  for (size_t i = 1; i < 6; ++i) {
    eq.makeLabel();
  }
  ASSERT_EQ(eq.unionTrees(4, 3), 3);
  ASSERT_EQ(eq.unionTrees(5, 3), 3);
  ASSERT_EQ(eq.unionTrees(4, 1), 1);
  ASSERT_EQ(eq.unionTrees(2, 5), 1);
}

struct ConnectedComponentsTester : public ImageProcTester
{
protected:
  template<ConnectivityType connectivity>
  bool fingerTest();

  template<ConnectivityType connectivity>
  bool spiralTest();

  inline static bool isBackground(uint8_t pixel)
  {
    return pixel == BACKGROUND_CODE;
  }

  inline Image<uint8_t> makeChessboardLikeImage(
    size_t rows, size_t cols,
    std::vector<uint8_t> & buffer) const;

protected:
  MemoryBuffer buffer_;
  imgproc_impl::EquivalenceLabelTrees<uint8_t> label_trees_;
  static const uint8_t BACKGROUND_CODE = 0;
  static const uint8_t FOREGROUND_CODE = 255;
};

Image<uint8_t> ConnectedComponentsTester::makeChessboardLikeImage(
  size_t rows, size_t cols,
  std::vector<uint8_t> & buffer) const
{
  Image<uint8_t> image = makeImage<uint8_t>(rows, cols, buffer, cols * 3);

  auto inverse = [this](uint8_t v) {
      return (v == BACKGROUND_CODE) ? FOREGROUND_CODE : BACKGROUND_CODE;
    };

  uint8_t current_value = FOREGROUND_CODE;
  for (size_t j = 0; j < cols; ++j) {
    *(image.row(0) + j) = current_value;
    current_value = inverse(current_value);
  }

  for (size_t i = 1; i < rows; ++i) {
    auto up = image.row(i - 1);
    auto current = image.row(i);
    for (size_t j = 0; j < cols; ++j, ++up, ++current) {
      *current = inverse(*up);
    }
  }
  return image;
}

TEST_F(ConnectedComponentsTester, way4EmptyTest) {
  Image<uint8_t> empty;
  const auto result = connectedComponents<ConnectivityType::Way4>(
    empty, buffer_, label_trees_,
    isBackground);
  ASSERT_EQ(result.second, uint8_t(0));
}

TEST_F(ConnectedComponentsTester, way4SinglePixelTest) {
  Image<uint8_t> input = makeImage(1, 1, image_buffer_bytes_);
  {
    input.row(0)[0] = BACKGROUND_CODE;

    const auto result = connectedComponents<ConnectivityType::Way4>(
      input, buffer_, label_trees_,
      isBackground);

    ASSERT_EQ(result.first.row(0)[0], 0);
    ASSERT_EQ(result.second, 1);
  }
  {
    input.row(0)[0] = FOREGROUND_CODE;

    const auto result = connectedComponents<ConnectivityType::Way4>(
      input, buffer_, label_trees_,
      isBackground);

    ASSERT_EQ(result.first.row(0)[0], 1);
    ASSERT_EQ(result.second, 2);
  }
}

TEST_F(ConnectedComponentsTester, way4ImageSmallTest) {
  {
    Image<uint8_t> input = makeImage(1, 2, image_buffer_bytes_);
    input.row(0)[0] = BACKGROUND_CODE;
    input.row(0)[1] = FOREGROUND_CODE;

    const auto result = connectedComponents<ConnectivityType::Way4>(
      input, buffer_, label_trees_,
      isBackground);

    ASSERT_EQ(result.second, uint8_t(2));
    ASSERT_EQ(result.first.row(0)[0], 0);
    ASSERT_EQ(result.first.row(0)[1], 1);
  }
  {
    Image<uint8_t> input = makeImage(2, 1, image_buffer_bytes_);
    input.row(0)[0] = BACKGROUND_CODE;
    input.row(1)[0] = FOREGROUND_CODE;

    const auto result = connectedComponents<ConnectivityType::Way4>(
      input, buffer_, label_trees_,
      isBackground);

    ASSERT_EQ(result.second, uint8_t(2));
    ASSERT_EQ(result.first.row(0)[0], 0);
    ASSERT_EQ(result.first.row(1)[0], 1);
  }
}

TEST_F(ConnectedComponentsTester, way4LabelsOverflowTest) {
  // big chessboard image
  Image<uint8_t> input = makeChessboardLikeImage(32, 17, image_buffer_bytes_);

  ASSERT_THROW(
    (connectedComponents<ConnectivityType::Way4>(input, buffer_, label_trees_, isBackground)),
    LabelOverflow);
}

template<class T>
struct UniformLabel
{
  const Image<T> & labels;
  std::map<T, T> labels_map = {{0, 0}};
  size_t next_label = 1;
  T at(size_t i, size_t j)
  {
    const T label = labels.row(i)[j];

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
    "....", image_buffer_bytes_);
  const Image<uint8_t> expected_labels = imageFromString<uint8_t>(
    "..xx"
    ".xx."
    "xx.."
    "....", image_buffer_bytes2_);
  const auto result = connectedComponents<ConnectivityType::Way4>(
    input, buffer_, label_trees_,
    isBackground);

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
    "....x.", image_buffer_bytes_);
  const Image<uint8_t> expected_labels = imageFromString<uint8_t>(
    "....aa"
    "..aa.."
    "aa...."
    "...bb."
    ".....b"
    "....b.", image_buffer_bytes2_, makeLabelsMap('b'));

  const auto result = connectedComponents<ConnectivityType::Way8>(
    input, buffer_, label_trees_,
    isBackground);

  ASSERT_EQ(result.second, uint8_t(3));
  ASSERT_TRUE(isEqualLabels(result.first, expected_labels));
}

TEST_F(ConnectedComponentsTester, way4ImageSieveTest) {
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "x.x.x"
    ".x.x."
    "x.x.x"
    ".x.x."
    "x.x.x", image_buffer_bytes_);
  const Image<uint8_t> expected_labels = imageFromString<uint8_t>(
    "a.b.c"
    ".d.e."
    "f.g.h"
    ".i.j."
    "k.l.m", image_buffer_bytes2_, makeLabelsMap('m'));

  const auto result = connectedComponents<ConnectivityType::Way4>(
    input, buffer_, label_trees_,
    isBackground);

  ASSERT_EQ(result.second, uint8_t(14));
  ASSERT_TRUE(isEqualLabels(result.first, expected_labels));
}

template<ConnectivityType connectivity>
bool ConnectedComponentsTester::fingerTest()
{
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "....."
    "....x"
    "..x.x"
    "x.x.x"
    "x.x.x", image_buffer_bytes_);
  const Image<uint8_t> expected_labels = imageFromString<uint8_t>(
    "....."
    "....c"
    "..b.c"
    "a.b.c"
    "a.b.c", image_buffer_bytes2_, makeLabelsMap('c'));

  const auto result = connectedComponents<connectivity>(input, buffer_, label_trees_, isBackground);

  return result.second == 4 && isEqualLabels(result.first, expected_labels);
}

TEST_F(ConnectedComponentsTester, way4ImageFingerTest) {
  ASSERT_TRUE(fingerTest<ConnectivityType::Way4>());
}

TEST_F(ConnectedComponentsTester, way8ImageFingerTest) {
  ASSERT_TRUE(fingerTest<ConnectivityType::Way8>());
}

template<ConnectivityType connectivity>
bool ConnectedComponentsTester::spiralTest()
{
  const Image<uint8_t> input = imageFromString<uint8_t>(
    ".xxxxxx"
    "......x"
    ".xxxx.x"
    ".x..x.x"
    ".x.xx.x"
    ".x....x"
    ".xxxxxx", image_buffer_bytes_);
  const Image<uint8_t> expected_labels = imageFromString<uint8_t>(
    ".xxxxxx"
    "......x"
    ".xxxx.x"
    ".x..x.x"
    ".x.xx.x"
    ".x....x"
    ".xxxxxx", image_buffer_bytes2_);

  const auto result = connectedComponents<connectivity>(input, buffer_, label_trees_, isBackground);
  return result.second == 2 && isEqualLabels(result.first, expected_labels);
}

TEST_F(ConnectedComponentsTester, way4ImageSpiralTest) {
  ASSERT_TRUE(spiralTest<ConnectivityType::Way4>());
}

TEST_F(ConnectedComponentsTester, way8ImageSpiralTest) {
  ASSERT_TRUE(spiralTest<ConnectivityType::Way8>());
}

TEST_F(ConnectedComponentsTester, groupsRemoverUint16LabelOverflow) {
  Image<uint8_t> image = makeChessboardLikeImage(512, 512, image_buffer_bytes_);
  GroupsRemover remover;
  MemoryBuffer buffer;
  remover.removeGroups(image, buffer, ConnectivityType::Way4, 2, isBackground);
  const auto bg = BACKGROUND_CODE;
  image.forEach([bg](uint8_t v) {ASSERT_EQ(v, bg);});
}

ShapeBuffer3x3 shape_buffer{};
const Image<uint8_t> cross_shape = createShape(shape_buffer, ConnectivityType::Way4);

uint8_t max_list(std::initializer_list<uint8_t> lst)
{
  return std::max(lst);
}

TEST_F(ImageProcTester, emptyImage) {
  Image<uint8_t> input;
  Image<uint8_t> output;

  ASSERT_NO_THROW(morphologyOperation(input, output, cross_shape, max_list));
}

TEST_F(ImageProcTester, wrongShapeSize) {
  Image<uint8_t> input = makeImage(1, 1, image_buffer_bytes_);
  Image<uint8_t> output = makeImage(1, 1, image_buffer_bytes2_);
  ASSERT_THROW(
    morphologyOperation(input, output, makeImage(2, 2, image_buffer_bytes3_), max_list),
    std::logic_error);
}

TEST_F(ImageProcTester, wrongSize) {
  Image<uint8_t> input = makeImage(3, 2, image_buffer_bytes_);

  {
    Image<uint8_t> output = makeImage(2, 2, image_buffer_bytes2_);
    ASSERT_THROW(
      morphologyOperation(input, output, cross_shape, max_list),
      std::logic_error);
  }

  {
    Image<uint8_t> output = makeImage(3, 3, image_buffer_bytes2_);
    ASSERT_THROW(
      morphologyOperation(input, output, cross_shape, max_list),
      std::logic_error);
  }
}

TEST_F(ImageProcTester, singlePixelImage) {
  image_buffer_bytes_ = {255};
  Image<uint8_t> input = makeImage(1, 1, image_buffer_bytes_);
  Image<uint8_t> output = makeImage(1, 1, image_buffer_bytes2_);

  morphologyOperation(input, output, cross_shape, max_list);

  ASSERT_EQ(output.row(0)[0], 0);
}

TEST_F(ImageProcTester, cornersImage) {
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "x..x"
    "...."
    "...."
    "x..x", image_buffer_bytes_);
  Image<uint8_t> expected = imageFromString<uint8_t>(
    ".xx."
    "x..x"
    "x..x"
    ".xx.", image_buffer_bytes2_);
  Image<uint8_t> output = makeImage(input.rows(), input.columns(), image_buffer_bytes3_);

  morphologyOperation(input, output, cross_shape, max_list);

  ASSERT_TRUE(isEqual(output, expected));
}

TEST_F(ImageProcTester, horizontalBordersImage) {
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "x..x"
    "x..x"
    "x..x"
    "x..x", image_buffer_bytes_);
  Image<uint8_t> expected = imageFromString<uint8_t>(
    "xxxx"
    "xxxx"
    "xxxx"
    "xxxx", image_buffer_bytes2_);
  Image<uint8_t> output = makeImage(input.rows(), input.columns(), image_buffer_bytes3_);

  morphologyOperation(input, output, cross_shape, max_list);

  ASSERT_TRUE(isEqual(output, expected));
}

TEST_F(ImageProcTester, verticalBordersImage) {
  const Image<uint8_t> input = imageFromString<uint8_t>(
    "xxxx"
    "...."
    "...."
    "xxxx", image_buffer_bytes_);
  Image<uint8_t> expected = imageFromString<uint8_t>(
    "xxxx"
    "xxxx"
    "xxxx"
    "xxxx", image_buffer_bytes2_);
  Image<uint8_t> output = makeImage(input.rows(), input.columns(), image_buffer_bytes3_);

  morphologyOperation(input, output, cross_shape, max_list);

  ASSERT_TRUE(isEqual(output, expected));
}
