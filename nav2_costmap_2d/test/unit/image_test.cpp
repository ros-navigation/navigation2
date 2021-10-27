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

#include "nav2_costmap_2d/image.hpp"

using namespace nav2_costmap_2d;

TEST(Image, emptyProps) {
  Image<uint8_t> empty;
  ASSERT_EQ(empty.rows(), 0ul);
  ASSERT_EQ(empty.columns(), 0ul);
  ASSERT_EQ(empty.step(), 0ul);
}

TEST(Image, props) {
  Image<uint8_t> image(3, 2);
  ASSERT_EQ(image.rows(), 3ul);
  ASSERT_EQ(image.columns(), 2ul);
  ASSERT_EQ(image.step(), 2ul);
}

TEST(Image, memoryAccess) {
  std::array<uint8_t, 7> buffer{};
  for (uint8_t i = 0; i < buffer.size(); ++i) {
    buffer[i] = i;
  }
  // buffer[3] is unused
  Image<uint8_t> wrapper(2, 3, buffer.data(), 4);

  ASSERT_EQ(wrapper.row(0), buffer.data());
  ASSERT_EQ(wrapper.row(1), buffer.data() + 4);

  for (uint8_t i = 0; i < 3; ++i) {
    ASSERT_EQ(wrapper.at(0, i), i);
    ASSERT_EQ(wrapper.at(1, i), i + 4);
  }
}

TEST(Image, forEach) {
  Image<uint8_t> image(3, 2);
  const uint8_t non_zero_initial_value = 42;
  uint8_t current(non_zero_initial_value);

  image.forEach(
    [&](uint8_t & pixel) {
      pixel = current++;
    });

  for (size_t i = 0; i < image.rows(); ++i) {
    for (size_t j = 0; j < image.columns(); ++j) {
      ASSERT_EQ(image.at(i, j), non_zero_initial_value + i * image.columns() + j);
    }
  }
}

TEST(Image, convert) {
  std::array<uint16_t, 6> src = {1, 2, 3, 4, 5, 6};
  const Image<uint16_t> source(2, 3, src.data(), 3);
  std::array<uint8_t, 6> trg = {};
  Image<uint8_t> target(2, 3, trg.data(), 3);

  source.convert(
    target, [](uint16_t s, uint8_t & t) {
      t = s * 2;
    });

  const std::array<uint8_t, 6> expected = {2, 4, 6, 8, 10, 12};
  ASSERT_TRUE(std::equal(expected.begin(), expected.end(), trg.begin()));
}

TEST(Image, convertDifferentSizes) {
  const Image<uint16_t> source(2, 3);
  Image<uint8_t> target(3, 2);
  auto do_nothing = [](uint16_t /*src*/, uint8_t & /*trg*/) {};

  // Extra parentheses need to protect commas in template arguments
  ASSERT_THROW((source.convert(target, do_nothing)), std::logic_error);
}

TEST(Image, convertEmptyImages) {
  const Image<uint16_t> source;
  Image<uint8_t> target;
  auto shouldn_t_be_called = [](uint16_t /*src*/, uint8_t & /*trg*/) {
      throw std::logic_error("");
    };

  ASSERT_NO_THROW((source.convert(target, shouldn_t_be_called)));
}

TEST(Image, convertWrongSize) {
  const Image<uint16_t> source(2, 3);

  auto do_nothing = [](uint16_t /*src*/, uint16_t & /*trg*/) {};
  {
    Image<uint16_t> target(3, 3);
    ASSERT_THROW((source.convert(target, do_nothing)), std::logic_error);
  }
  {
    Image<uint16_t> target(2, 2);
    ASSERT_THROW((source.convert(target, do_nothing)), std::logic_error);
  }
}

TEST(Image, part) {
  Image<uint8_t> image(5, 4);
  image.forEach(
    [&](uint8_t & pixel) {
      pixel = 0;
    });

  image.part(1, 1, 3, 2).forEach(
    [](uint8_t & pixel) {
      pixel = 255;
    });

  using BytesVec = std::vector<uint8_t>;
  auto rowAsVec = [&image](size_t row) {
      return BytesVec(image.row(row), image.row(row) + 4);
    };

  ASSERT_EQ(rowAsVec(0), BytesVec({0, 0, 0, 0}));
  ASSERT_EQ(rowAsVec(1), BytesVec({0, 255, 255, 0}));
  ASSERT_EQ(rowAsVec(2), BytesVec({0, 255, 255, 0}));
  ASSERT_EQ(rowAsVec(3), BytesVec({0, 255, 255, 0}));
  ASSERT_EQ(rowAsVec(4), BytesVec({0, 0, 0, 0}));
}

TEST(Image, emptyPart) {
  Image<uint8_t> image(5, 5);
  ASSERT_THROW(image.part(1, 1, 0, 1), std::logic_error);
  ASSERT_THROW(image.part(1, 1, 1, 0), std::logic_error);
}

TEST(Image, outOfBoundsPart) {
  Image<uint8_t> image(5, 5);
  ASSERT_THROW(image.part(3, 3, 2, 4), std::logic_error);
  ASSERT_THROW(image.part(3, 3, 4, 2), std::logic_error);
}

TEST(Image, cloneEmpty) {
  Image<uint16_t> a;
  auto b = a.clone();

  ASSERT_TRUE(b.empty());
}

TEST(Image, clone) {
  std::array<uint16_t, 6> data = {1, 2, 3, 4, 5, 6};
  Image<uint16_t> a(3, 2, data.data(), 2);
  auto b = a.clone();

  auto iter = data.begin();
  b.forEach(
    [&iter](uint16_t v) {
      ASSERT_EQ(v, *iter);
      ++iter;
    });
}
