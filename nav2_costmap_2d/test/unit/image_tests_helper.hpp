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

#ifndef NAV2_COSTMAP_2D__IMAGE_TESTS_HELPER_HPP_
#define NAV2_COSTMAP_2D__IMAGE_TESTS_HELPER_HPP_

#include "nav2_costmap_2d/denoise/image.hpp"
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <algorithm>

namespace nav2_costmap_2d
{
template<class T>
Image<T> makeImage(size_t rows, size_t columns, std::vector<T> & buffer, size_t step = 0)
{
  step = std::max(step, columns);
  buffer.resize(rows * step);
  return Image<T>(rows, columns, buffer.data(), step);
}

template<class T>
Image<T> clone(const Image<T> & source, std::vector<T> & buffer)
{
  buffer.resize(source.rows() * source.columns());
  Image<T> result(source.rows(), source.columns(), buffer.data(), source.columns());

  for (size_t row = 0; row < source.rows(); ++row) {
    for (size_t column = 0; column < source.columns(); ++column) {
      result.row(row)[column] = source.row(row)[column];
    }
  }
  return result;
}

/**
 * @brief Decodes image from a string
 *
 * Used only for tests.
 * Each character of the string will be replaced with a code from the codes
 * and written to the corresponding pixel of the image
 * The image is always square, i.e. the number of rows is equal to the number of columns
 * For example, string
 * "x.x"
 * ".x."
 * "..."
 * describes a 3x3 image in which a v-shape is drawn with code 255 (with default codes map)
 * @throw std::logic_error if the format of the string is incorrect
 */
template<class T>
Image<T> imageFromString(
  const std::string & s, std::vector<T> & buffer,
  const std::map<char, T> & codes = {{'.', 0}, {'x', 255}})
{
  const size_t side_size = static_cast<size_t>(std::sqrt(s.size()));

  if (size_t(side_size) * side_size != s.size()) {
    throw std::logic_error("Test data error: parseBinaryMatrix: Unexpected input string size");
  }

  const size_t step = static_cast<size_t>(side_size * 3);

  Image<T> image = makeImage(side_size, side_size, buffer, step);
  auto iter = s.begin();
  image.forEach(
    [&](T & pixel) {
      try {
        pixel = codes.at(*iter);
        ++iter;
      } catch (...) {
        throw std::logic_error(
          "Test data error: parseBinaryMatrix: Unexpected symbol: " +
          std::string(1, *iter));
      }
    });
  return image;
}

/**
 * @brief Checks exact match of images
 *
 * @return true if images a and b have the same type, size, and data. Otherwise false
 */
inline bool isEqual(const Image<uint8_t> & a, const Image<uint8_t> & b)
{
  bool equal = a.rows() == b.rows() && a.columns() == b.columns();

  for (size_t row = 0; row < a.rows() && equal; ++row) {
    for (size_t column = 0; column < a.columns() && equal; ++column) {
      equal = a.row(row)[column] == b.row(row)[column];
    }
  }
  return equal;
}

template<class T>
std::ostream & operator<<(std::ostream & out, const Image<T> & image)
{
  for (size_t i = 0; i < image.rows(); ++i) {
    for (size_t j = 0; j < image.columns(); ++j) {
      out << int64_t(image.row(i)[j]) << " ";
    }
    out << std::endl;
  }
  return out;
}

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__IMAGE_TESTS_HELPER_HPP_
