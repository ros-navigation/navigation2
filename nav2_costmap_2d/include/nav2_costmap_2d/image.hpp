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

#ifndef NAV2_COSTMAP_2D__IMAGE_HPP_
#define NAV2_COSTMAP_2D__IMAGE_HPP_

#include <cstddef>
#include <memory>
#include <stdexcept>

namespace nav2_costmap_2d
{

/**
 * @brief Image with pixels of type T
 * Сan own data, be a wrapper over some memory buffer, or refer to a fragment of another image
 * Pixels of one row are stored continuity. But rows continuity is not guaranteed.
 * The distance (number of elements of type T) from row(i) to row(i + 1) is equal to step()
 * @tparam T type of pixel
 */
template<class T>
class Image
{
public:
  /// @brief Create empty image
  Image() = default;

  /**
   * @brief Create image referencing to a third-party buffer
   * @param rows number of image rows
   * @param columns number of image columns
   * @param data existing memory buffer with size at least rows * columns
   * @param step offset from row(i) to row(i + 1) in memory buffer (number of elements of type T).
   * offset = columns if rows are stored continuity
   */
  Image(size_t rows, size_t columns, T * data, size_t step);

  /**
   * @brief Create image
   * @param rows number of image rows
   * @param columns number of image columns
   */
  Image(size_t rows, size_t columns);

  /**
   * @brief Create image referencing to the other
   * Share image data between new and old object.
   * Changing data in a new object will affect the given one and vice versa
   */
  Image(Image & other);

  /**
   * @brief Create image from the other (move constructor)
   */
  Image(Image && other) noexcept;

  /**
   * @brief Create copy of image
   */
  Image clone() const;

  /**
   * @brief Create new image referencing to this image part
   * Share image data between new and old object.
   * Changing data in a new object will affect the given one and vice versa
   * @param top first row of image part
   * @param left first column of image part
   * @param rows number of part rows
   * @param columns number of part columns
   * @throw std::logic_error if part out of image bounds
   */
  Image part(size_t top, size_t left, size_t rows, size_t columns);

  /// @return number of image rows
  size_t rows() const {return rows_;}

  /// @return number of image columns
  size_t columns() const {return columns_;}

  /// @return true if image empty
  bool empty() const {return rows_ == 0 || columns_ == 0;}

  /// @return offset (number of elements of type T) from row(i) to row(i + 1)
  size_t step() const {return step_;}

  /**
   * @return image pixel
   * @warning If row/column exceed the image bounds, the behavior is undefined
   */
  T & at(size_t row, size_t column);

  /// @overload
  const T & at(size_t row, size_t column) const;

  /**
   * @return pointer to first pixel of row
   * @warning If row >= rows(), the behavior is undefined
   */
  T * row(size_t row);

  /// @overload
  const T * row(size_t row) const;

  /// @brief Fill the image with pixels value
  void fill(T value);

  /**
   * @brief Read (and modify, if need) each pixel sequentially
   * @tparam Functor function object.
   * Signature should be equivalent to the following:
   * void fn(T& pixel) or void fn(const T& pixel)
   * @param fn a function that will be applied to each pixel in the image. Can modify image data
   */
  template<class Functor>
  void forEach(Functor && fn);

  /**
   * @brief Read each pixel sequentially
   * @tparam Functor function object.
   * Signature should be equivalent to the following:
   * void fn(const T& pixel)
   * @param fn a function that will be applied to each pixel in the image
   */
  template<class Functor>
  void forEach(Functor && fn) const;
  /**
   * @brief Convert each pixel to corresponding pixel of target using a custom function
   *
   * The source and target must be the same size.
   * For calculation of new target value the operation can use source value and
   * an optionally current target value.
   * This function call operation(this[i, j], target[i, j]) for each pixel
   * where target[i, j] is mutable
   * @tparam TargetElement type of target pixel
   * @tparam Converter function object.
   * Signature should be equivalent to the following:
   * void fn(const T& src, TargetElement& trg)
   * @param target output image with TargetElement-type pixels
   * @param operation the binary operation op is applied to pairs of pixels:
   * first (const) from source and second (mutable) from target
   * @throw std::logic_error if the source and target of different sizes
   */
  template<class TargetElement, class Converter>
  void convert(Image<TargetElement> & target, Converter && converter) const;

private:
  std::shared_ptr<T[]> data_;
  T * data_start_{};
  size_t rows_{};
  size_t columns_{};
  size_t step_{};
};

template<class T>
Image<T>::Image(size_t rows, size_t columns, T * data, size_t step)
: rows_{rows}, columns_{columns}, step_{step}
{
  auto empty_deleter = [](T *) { /* do nothing. This object does not own data_ */};
  data_ = std::shared_ptr<T[]>(data, empty_deleter);
  data_start_ = data;
}

template<class T>
Image<T>::Image(size_t rows, size_t columns)
: rows_{rows}, columns_{columns}, step_{columns}
{
  // Before C++20 we can't call std::make_shared<T[]>...
  data_ = std::make_unique<T[]>(rows * columns); // it's ok if rows * columns == 0
  data_start_ = data_.get();
}

template<class T>
Image<T>::Image(Image & other)
: data_{other.data_}, data_start_{other.data_start_},
  rows_{other.rows_}, columns_{other.columns_}, step_{other.step_} {}

template<class T>
Image<T>::Image(Image && other) noexcept
: data_{std::move(other.data_)}, data_start_{other.data_start_},
  rows_{other.rows_}, columns_{other.columns_}, step_{other.step_} {}

template<class T>
Image<T> Image<T>::clone() const
{
  Image<T> clone(rows(), columns());
  convert(
    clone, [](const T & src, T & trg) {
      trg = src;
    });
  return clone;
}

template<class T>
Image<T> Image<T>::part(size_t top, size_t left, size_t rows, size_t columns)
{
  if (top + rows > this->rows() || left + columns > this->columns()) {
    throw std::logic_error("Image::part. Part is out of image bounds");
  }

  if (rows == 0 || columns == 0) {
    throw std::logic_error("Image::part. Trying to request empty part");
  }
  Image img(rows, columns, &at(top, left), step_);
  img.data_ = this->data_; // overwrite shared pointer to this object member
  return img;
}

template<class T>
T & Image<T>::at(size_t row, size_t column)
{
  return const_cast<T &>( static_cast<const Image<T> &>(*this).at(row, column) );
}

template<class T>
const T & Image<T>::at(size_t row, size_t column) const
{
  return *(this->row(row) + column);
}

template<class T>
T * Image<T>::row(size_t row)
{
  return const_cast<T *>( static_cast<const Image<T> &>(*this).row(row) );
}

template<class T>
const T * Image<T>::row(size_t row) const
{
  return data_start_ + row * step_;
}

template<class T>
void Image<T>::fill(T value)
{
  forEach([&](T & v) {v = value;});
}

template<class T>
template<class Functor>
void Image<T>::forEach(Functor && fn)
{
  static_cast<const Image<T> &>(*this).forEach(
    [&](const T & pixel) {
      fn(const_cast<T &>(pixel));
    });
}

template<class T>
template<class Functor>
void Image<T>::forEach(Functor && fn) const
{
  const T * rowPtr = row(0);

  for (size_t row = 0; row < rows(); ++row) {
    const T * rowEnd = rowPtr + columns();

    for (const T * pixel = rowPtr; pixel != rowEnd; ++pixel) {
      fn(*pixel);
    }
    rowPtr += step();
  }
}

template<class T>
template<class TargetElement, class Converter>
void Image<T>::convert(Image<TargetElement> & target, Converter && converter) const
{
  if (rows() != target.rows() || columns() != target.columns()) {
    throw std::logic_error("Image::convert. The source and target images size are different");
  }
  const T * source_row = row(0);
  TargetElement * target_row = target.row(0);

  for (size_t row = 0; row < rows(); ++row) {
    const T * rowInEnd = source_row + columns();
    const T * src = source_row;
    TargetElement * trg = target_row;

    for (; src != rowInEnd; ++src, ++trg) {
      converter(*src, *trg);
    }
    source_row += step();
    target_row += target.step();
  }
}

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__IMAGE_HPP_
