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

#include "nav2_costmap_2d/denoise/image.hpp"
#include "image_tests_helper.hpp"

using namespace nav2_costmap_2d;

struct ImageTester : public ::testing::Test
{
protected:
  std::vector<uint8_t> image_buffer_bytes;
  std::vector<uint16_t> image_buffer_words;
};

TEST_F(ImageTester, emptyProps) {
  Image<uint8_t> empty;
  ASSERT_EQ(empty.rows(), 0ul);
  ASSERT_EQ(empty.columns(), 0ul);
  ASSERT_EQ(empty.step(), 0ul);
}

TEST_F(ImageTester, memoryAccess) {
  std::array<uint8_t, 7> buffer{};
  for (uint8_t i = 0; i < buffer.size(); ++i) {
    buffer[i] = i;
  }
  // buffer[3] is unused
  Image<uint8_t> wrapper(2, 3, buffer.data(), 4);

  ASSERT_EQ(wrapper.row(0), buffer.data());
  ASSERT_EQ(wrapper.row(1), buffer.data() + 4);
}

TEST_F(ImageTester, forEach) {
  Image<uint8_t> image = makeImage(3, 2, image_buffer_bytes);
  const uint8_t non_zero_initial_value = 42;
  uint8_t current(non_zero_initial_value);

  image.forEach(
    [&](uint8_t & pixel) {
      pixel = current++;
    });

  for (size_t i = 0; i < image.rows(); ++i) {
    for (size_t j = 0; j < image.columns(); ++j) {
      ASSERT_EQ(image.row(i)[j], non_zero_initial_value + i * image.columns() + j);
    }
  }
}

TEST_F(ImageTester, convert) {
  image_buffer_words = {1, 2, 3, 4, 5, 6};
  Image<uint16_t> source = makeImage(2, 3, image_buffer_words);
  Image<uint8_t> target = makeImage(2, 3, image_buffer_bytes);

  source.convert(
    target, [](uint16_t s, uint8_t & t) {
      t = s * 2;
    });

  const std::array<uint8_t, 6> expected = {2, 4, 6, 8, 10, 12};
  ASSERT_TRUE(std::equal(expected.begin(), expected.end(), image_buffer_bytes.begin()));
}

TEST_F(ImageTester, convertDifferentSizes) {
  Image<uint16_t> source = makeImage(2, 3, image_buffer_words);
  Image<uint8_t> target = makeImage(3, 2, image_buffer_bytes);
  auto do_nothing = [](uint16_t /*src*/, uint8_t & /*trg*/) {};

  // Extra parentheses need to protect commas in template arguments
  ASSERT_THROW((source.convert(target, do_nothing)), std::logic_error);
}

TEST_F(ImageTester, convertEmptyImages) {
  const Image<uint16_t> source;
  Image<uint8_t> target;
  auto shouldn_t_be_called = [](uint16_t /*src*/, uint8_t & /*trg*/) {
      throw std::logic_error("");
    };

  ASSERT_NO_THROW((source.convert(target, shouldn_t_be_called)));
}

TEST_F(ImageTester, convertWrongSize) {
  Image<uint16_t> source = makeImage(2, 3, image_buffer_words);

  auto do_nothing = [](uint16_t /*src*/, uint8_t & /*trg*/) {};
  {
    Image<uint8_t> target = makeImage(3, 3, image_buffer_bytes);
    ASSERT_THROW((source.convert(target, do_nothing)), std::logic_error);
  }
  {
    Image<uint8_t> target = makeImage(2, 2, image_buffer_bytes);
    ASSERT_THROW((source.convert(target, do_nothing)), std::logic_error);
  }
}
