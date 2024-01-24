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

#ifndef NAV2_COSTMAP_2D__DENOISE__IMAGE_PROCESSING_HPP_
#define NAV2_COSTMAP_2D__DENOISE__IMAGE_PROCESSING_HPP_

#include "image.hpp"
#include <algorithm>
#include <vector>
#include <array>
#include <memory>
#include <limits>

namespace nav2_costmap_2d
{

/**
 * @enum nav2_costmap_2d::ConnectivityType
 * @brief Describes the type of pixel connectivity (is the way in which
 * pixels in image relate to their neighbors)
 */
enum class ConnectivityType : int
{
  /// neighbors pixels are connected horizontally and vertically
  Way4 = 4,
  /// neighbors pixels are connected horizontally, vertically and diagonally
  Way8 = 8
};

/**
 * @brief A memory buffer that can grow to an upper-bounded capacity
 */
class MemoryBuffer
{
public:
  /// @brief Free memory allocated for the buffer
  inline ~MemoryBuffer() {reset();}
  /**
   * @brief Return a pointer to an uninitialized array of count elements
   * Delete the old block of memory and allocates a new one if the size of the old is too small.
   * The returned pointer is valid until the next call to get() or destructor.
   * @tparam T type of element
   * @param count number of elements
   * @throw std::bad_alloc or any other exception thrown by allocator
   */
  template<class T>
  T * get(std::size_t count);

private:
  inline void reset();
  inline void allocate(size_t bytes);

private:
  void * data_{};
  size_t size_{};
};

// forward declarations
namespace imgproc_impl
{
template<class Label>
class EquivalenceLabelTrees;

template<class AggregateFn>
void morphologyOperation(
  const Image<uint8_t> & input, Image<uint8_t> & output,
  const Image<uint8_t> & shape, AggregateFn aggregate);

using ShapeBuffer3x3 = std::array<uint8_t, 9>;
inline Image<uint8_t> createShape(ShapeBuffer3x3 & buffer, ConnectivityType connectivity);
} // namespace imgproc_impl

/**
 * @brief Perform morphological dilation
 * @tparam Max function object
 * @param input input image
 * @param output output image
 * @param connectivity selector for selecting structuring element (Way4-> cross, Way8-> rect)
 * @param max_function takes as input std::initializer_list<uint8_t> with three elements.
 * Returns the greatest value in list
 */
template<class Max>
inline void dilate(
  const Image<uint8_t> & input, Image<uint8_t> & output,
  ConnectivityType connectivity, Max && max_function)
{
  using namespace imgproc_impl;
  ShapeBuffer3x3 shape_buffer;
  Image<uint8_t> shape = createShape(shape_buffer, connectivity);
  morphologyOperation(input, output, shape, max_function);
}

/**
* @brief Compute the connected components labeled image of binary image
* Implements the SAUF algorithm
* (Two Strategies to Speed up Connected Component Labeling Algorithms
* Kesheng Wu, Ekow Otoo, Kenji Suzuki).
* @tparam connectivity pixels connectivity type
* @tparam Label integer type of label
* @tparam IsBg functor with signature bool (uint8_t)
* @param image input image
* @param buffer memory block that will be used to store the result (labeled image)
* and the internal buffer for labels trees
* @param label_trees union-find data structure
* @param is_background returns true if the passed pixel value is background
* @throw LabelOverflow if all possible values of the Label type are used and
* it is impossible to create a new unique
* @return pair(labeled image, total number of labels)
* Labeled image has the same size as image. Label 0 represents the background label,
* labels [1, <return value> - 1] - separate components.
* Total number of labels == 0 for empty image.
* In other cases, label 0 is always counted,
* even if there is no background in the image.
* For example, for an image of one background pixel, the total number of labels == 2.
* Two labels (0, 1) have been counted, although label 0 is not used)
*/
template<ConnectivityType connectivity, class Label, class IsBg>
std::pair<Image<Label>, Label> connectedComponents(
  const Image<uint8_t> & image, MemoryBuffer & buffer,
  imgproc_impl::EquivalenceLabelTrees<Label> & label_trees,
  IsBg && is_background);

// Implementation

template<class T>
T * MemoryBuffer::get(std::size_t count)
{
  // Check the memory allocated by ::operator new can be used to store the type T
  static_assert(
    alignof(std::max_align_t) >= alignof(T),
    "T alignment is more than the fundamental alignment of the platform");

  const size_t required_bytes = sizeof(T) * count;

  if (size_ < required_bytes) {
    allocate(required_bytes);
  }
  return static_cast<T *>(data_);
}

void MemoryBuffer::reset()
{
  ::operator delete(data_);
  size_ = 0;
}

void MemoryBuffer::allocate(size_t bytes)
{
  reset();
  data_ = ::operator new(bytes);
  size_ = bytes;
}

namespace imgproc_impl
{

/**
 * @brief Calculate truncated histogram of image.
 *
 * Creates a histogram of image_max bins.
 * Bin with index i keep min of (<count of the number of pixels with value i>, bin_max).
 * This truncation avoids overflow and is acceptable in the problem being solved.
 * For example, the image with pixel type uint16_t may have 100'000 pixels equal to 0
 * (100'000 > std::numeric_limits<uint16_t>). In this case, an overflow will occur when
 * calculating a traditional histogram with bins of the uint16_t type. But in this function,
 * the bin value will increase to the bin_max value, then stop. Overflow will not happen.
 * @tparam T image pixel type
 * @param image source image
 * @param image_max max image pixel value
 * @param bin_max max histogram bin value
 * @return vector of histogram bins
 * @warning If source contains a pixel with a value, large then image_max,
 * the behavior is undefined
*/
template<class T, class Bin>
std::vector<Bin>
histogram(const Image<T> & image, T image_max, Bin bin_max)
{
  if (image.empty()) {
    return {};
  }
  std::vector<Bin> histogram(size_t(image_max) + 1);

  // Increases the bin value corresponding to the pixel by one
  auto add_pixel_value = [&histogram, bin_max](T pixel) {
      auto & h = histogram[pixel];
      h = std::min(Bin(h + 1), bin_max);
    };

  image.forEach(add_pixel_value);
  return histogram;
}

namespace out_of_bounds_policy
{

/**
 * @brief Boundary case object stub. Used as parameter of class Window.
 * Dereferences a pointer to a pixel without any checks
 * @tparam T image pixel type
 * @sa ReplaceToZero
 */
template<class T>
struct DoNothing
{
  T & up(T * v) const {return *v;}
  T & down(T * v) const {return *v;}
};

/**
 * @brief Boundary case object. Used as parameter of class Window.
 * Dereferences a pointer to a existing pixel. If the pixel is out of bounds, it returns a ref to 0.
 * @tparam T image pixel type
 * @sa DoNothing
 */
template<class T>
class ReplaceToZero
{
public:
  /**
   * @brief Create an object that will replace pointers outside the specified range
   * @param up_row_start pointer to the first pixel of up row. Can be nullptr.
   * @param down_row_start pointer to the first pixel of down row
   * @param columns number of pixels in both rows
   */
  ReplaceToZero(const T * up_row_start, const T * down_row_start, size_t columns)
  : up_row_start_{up_row_start}, up_row_end_{up_row_start + columns},
    down_row_start_{down_row_start}, down_row_end_{down_row_start + columns} {}

  /**
   * @brief Return ref to pixel or to zero value if up_row_start_ is nullptr or the pointer is out of bounds
   * @param v pointer to pixel
   */
  T & up(T * v)
  {
    if (up_row_start_ == nullptr) {
      return zero_;
    }
    return replaceOutOfBounds(v, up_row_start_, up_row_end_);
  }

  /**
   * @brief Return ref to pixel or to zero value if the pointer is out of bounds
   * @param v pointer to pixel
   */
  T & down(T * v)
  {
    return replaceOutOfBounds(v, down_row_start_, down_row_end_);
  }

private:
  /**
   * @brief Replaces an out-of-bounds pointer with a pointer to 0
   * @return a dereferenced pointer or a reference to 0 if the pointer is out of range
   */
  T & replaceOutOfBounds(T * v, const T * begin, const T * end)
  {
    if (v < begin || v >= end) {
      return zero_;
    }
    return *v;
  }

  const T * up_row_start_;
  const T * up_row_end_;
  const T * down_row_start_;
  const T * down_row_end_;
  T zero_{};
};

}  // namespace out_of_bounds_policy

/**
 * @brief Forward scan mask sliding window
 * Provides an interface for access to neighborhood of the current pixel
 * (includes three neighbors of the top row, the pixel to the left and the current one).
 * In the illustration below, the current pixel is e.
 * |a|b|c|
 * |d|e| |
 * | | | |
 * @tparam T image pixel type
 * @tparam Border optional check of access to pixels outside the image boundary (DoNothing or ReplaceToZero)
 */
template<class T, template<class> class Border>
class Window
{
public:
  /**
   * Construct mask window
   * @param up_row pointer to the pixel above the current one (a)
   * @param down_row pointer to the current pixel (e)
   * @param border boundary case object
   */
  inline Window(T * up_row, T * down_row, Border<T> border = {})
  : up_row_{up_row}, down_row_{down_row}, border_{border} {}

  inline T & a() {return border_.up(up_row_ - 1);}
  inline T & b() {return border_.up(up_row_);}
  inline T & c() {return border_.up(up_row_ + 1);}
  inline T & d() {return border_.down(down_row_ - 1);}
  inline T & e() {return *down_row_;}
  inline const T * anchor() const {return down_row_;}

  /// @brief Shifts the window to the right
  inline void next()
  {
    ++up_row_;
    ++down_row_;
  }

private:
  T * up_row_;
  T * down_row_;
  Border<T> border_;
};

/// @brief Discards const
template<class T>
T * dropConst(const T * ptr)
{
  return const_cast<T *>(ptr);
}

/**
 * @brief Create a sliding window with boundary case object
 * @tparam T image pixel type
 * @param up_row pointer to the row above the current one
 * @param down_row pointer to the current row
 * @param columns number of pixels in both rows
 * @param offset offset from rows start to current window pixel
 * @return forward scan mask sliding window
 * @warning Breaks the constant guarantees.
 * Always returns a non-constant window, using which you can potentially change data in up_row, down_row.
 * This could have been avoided by creating a ConstWindow similar to Window.
 * But probably code bloat is the bigger evil
 */
template<class T>
Window<T, out_of_bounds_policy::ReplaceToZero> makeSafeWindow(
  const T * up_row, const T * down_row, size_t columns, size_t offset = 0)
{
  return {
    dropConst(up_row) + offset, dropConst(down_row) + offset,
    out_of_bounds_policy::ReplaceToZero<T>{up_row, down_row, columns}
  };
}

/**
 * @brief Create a sliding window without any out of borders checks
 * @tparam T image pixel type
 * @param up_row pointer to the row above the current one
 * @param down_row pointer to the current row
 * @return forward scan mask sliding window
 * @warning Breaks the constant guarantees. See warning in makeSafeWindow
 */
template<class T>
Window<T, out_of_bounds_policy::DoNothing> makeUnsafeWindow(const T * up_row, const T * down_row)
{
  return {dropConst(up_row), dropConst(down_row)};
}

struct EquivalenceLabelTreesBase
{
  virtual ~EquivalenceLabelTreesBase() = default;
};

struct LabelOverflow : public std::runtime_error
{
  LabelOverflow(const std::string & message)
  : std::runtime_error(message) {}
};

/**
 * @brief Union-find data structure
 * Implementation of union-find data structure, described in reference article.
 * Store rooted trees, where each node of a tree is a provisional label and each edge represents an
 * equivalence between two labels
 * @tparam Label integer type of label
 */
template<class Label>
class EquivalenceLabelTrees : public EquivalenceLabelTreesBase
{
public:
  /**
   * @brief Reset labels tree to initial state
   * @param rows number of image rows
   * @param columns number of image columns
   * @param connectivity pixels connectivity type
   */
  void reset(const size_t rows, const size_t columns, ConnectivityType connectivity)
  {
    // Trying to reserve memory with a margin
    const size_t max_labels_count = maxLabels(rows, columns, connectivity);
    // Number of labels cannot exceed std::numeric_limits<Label>::max()
    labels_size_ = static_cast<Label>(
      std::min(max_labels_count, size_t(std::numeric_limits<Label>::max()))
    );

    try {
      labels_.reserve(labels_size_);
    } catch (...) {
      // ignore any exception
      // perhaps the entire requested amount of memory will not be required
    }

    // Label 0 is reserved for the background pixels, i.e. labels[0] is always 0
    labels_ = {0};
    next_free_ = 1;
  }

  /**
   * @brief Creates new next unused label and returns it back
   * @throw LabelOverflow if all possible labels already used
   * @return label
   */
  Label makeLabel()
  {
    // Check the next_free_ counter does not overflow.
    if (next_free_ == labels_size_) {
      throw LabelOverflow("EquivalenceLabelTrees: Can't create new label");
    }
    labels_.push_back(next_free_);
    return next_free_++;
  }

  /**
   * @brief Unite the two trees containing nodes i and j and return the new root
   * See union function in reference article
   * @param i tree node
   * @param j tree node
   * @return root of joined tree
   */
  Label unionTrees(Label i, Label j)
  {
    Label root = findRoot(i);

    if (i != j) {
      Label root_j = findRoot(j);
      root = std::min(root, root_j);
      setRoot(j, root);
    }
    setRoot(i, root);
    return root;
  }

  /**
   * @brief Convert union-find trees to labels lookup table
   * @return pair(labels lookup table, unique labels count)
   * @warning This method invalidate EquivalenceLabelTrees inner state
   * @warning Returns an internal buffer that will be invalidated
   * on subsequent calls to the methods of EquivalenceLabelTrees
   */
  const std::vector<Label> & getLabels()
  {
    Label k = 1;
    for (Label i = 1; i < next_free_; ++i) {

      if (labels_[i] < i) {
        labels_[i] = labels_[labels_[i]];
      } else {
        labels_[i] = k;
        ++k;
      }
    }
    labels_.resize(k);
    return labels_;
  }

private:
  /**
   * @brief Defines the upper bound for the number of labels
   * @param rows number of image rows
   * @param columns number of image columns
   * @param connectivity pixels connectivity type
   * @return max labels count
   */
  static size_t maxLabels(const size_t rows, const size_t columns, ConnectivityType connectivity)
  {
    size_t max_labels{};

    if (connectivity == ConnectivityType::Way4) {
      /* The maximum of individual components will be reached in the chessboard image,
       * where the white cells correspond to obstacle pixels */
      max_labels = (rows * columns) / 2 + 1;
    } else {
      /* The maximum of individual components will be reached in image like this:
       * x.x.x.x~
       * .......~
       * x.x.x.x~
       * .......~
       * x.x.x.x~
       * ~
       * where 'x' - pixel with obstacle, '.' - background pixel,
       * '~' - row continuation in the same style */
      max_labels = (rows * columns) / 3 + 1;
    }
    ++max_labels; // add zero label
    max_labels = std::min(max_labels, size_t(std::numeric_limits<Label>::max()));
    return max_labels;
  }

  /// @brief Find the root of the tree of node i
  Label findRoot(Label i)
  {
    Label root = i;
    for (; labels_[root] < root; root = labels_[root]) { /*do nothing*/}
    return root;
  }

  /// @brief Set the root of the tree of node i
  void setRoot(Label i, Label root)
  {
    while (labels_[i] < i) {
      auto j = labels_[i];
      labels_[i] = root;
      i = j;
    }
    labels_[i] = root;
  }

private:
  /**
   * Linear trees container. If we have two trees: (2 -> 1) and (4 -> 3), (5 -> 3)
   * and one single node 0, the content of the vector will be:
   * index: 0|1|2|3|4|5
   * value: 0|1|1|3|3|3
   * After unionTrees(1, 3) we have one tree (2 -> 1), (3 -> 1), (4 -> 3), (5 -> 3) and one single node 0:
   * index: 0|1|2|3|4|5
   * value: 0|1|1|1|3|3
   */
  std::vector<Label> labels_;
  Label labels_size_{};
  Label next_free_{};
};

/// @brief The specializations of this class provide the definition of the pixel label
template<ConnectivityType connectivity>
struct ProcessPixel;

/// @brief Define the label of a pixel in an 8-linked image
template<>
struct ProcessPixel<ConnectivityType::Way8>
{
  /**
   * @brief Set the label of the current pixel image.e() based on labels in its neighborhood
   * @tparam ImageWindow Window parameterized by class DoNothing or ReplaceToZero
   * @tparam LabelsWindow Window parameterized by class DoNothing or ReplaceToZero
   * @tparam Label integer type of label
   * @param image input image window. Image data will not be changed. De facto, image is a const ref
   * @param label output label window
   * @param eq_trees union-find structure
   * @throw LabelOverflow if all possible labels already used
   */
  template<class ImageWindow, class LabelsWindow, class Label, class IsBg>
  static void pass(
    ImageWindow & image, LabelsWindow & label, EquivalenceLabelTrees<Label> & eq_trees,
    IsBg && is_bg)
  {
    Label & current = label.e();

    //The decision tree traversal. See reference article for details
    if (!is_bg(image.e())) {
      if (label.b()) {
        current = label.b();
      } else {
        if (!is_bg(image.c())) {
          if (!is_bg(image.a())) {
            current = eq_trees.unionTrees(label.c(), label.a());
          } else {
            if (!is_bg(image.d())) {
              current = eq_trees.unionTrees(label.c(), label.d());
            } else {
              current = label.c();
            }
          }
        } else {
          if (!is_bg(image.a())) {
            current = label.a();
          } else {
            if (!is_bg(image.d())) {
              current = label.d();
            } else {
              current = eq_trees.makeLabel();
            }
          }
        }
      }
    } else {
      current = 0;
    }
  }
};

/// @brief Define the label of a pixel in an 4-linked image
template<>
struct ProcessPixel<ConnectivityType::Way4>
{
  /**
   * @brief Set the label of the current pixel image.e() based on labels in its neighborhood
   * @tparam ImageWindow Window parameterized by class DoNothing or ReplaceToZero
   * @tparam LabelsWindow Window parameterized by class DoNothing or ReplaceToZero
   * @tparam Label integer type of label
   * @param image input image window. Image data will not be changed. De facto, image is a const ref
   * @param label output label window
   * @param eq_trees union-find structure
   * @throw LabelOverflow if all possible labels already used
   */
  template<class ImageWindow, class LabelsWindow, class Label, class IsBg>
  static void pass(
    ImageWindow & image, LabelsWindow & label, EquivalenceLabelTrees<Label> & eq_trees,
    IsBg && is_bg)
  {
    Label & current = label.e();

    // Simplified decision tree traversal. See reference article for details
    if (!is_bg(image.e())) {
      if (!is_bg(image.b())) {
        if (!is_bg(image.d())) {
          current = eq_trees.unionTrees(label.d(), label.b());
        } else {
          current = label.b();
        }
      } else {
        if (!is_bg(image.d())) {
          current = label.d();
        } else {
          current = eq_trees.makeLabel();
        }
      }
    } else {
      current = 0;
    }
  }
};

/**
 * @brief Applies a 1d shape to the neighborhood of each pixel of the input image.
 * Applies a 1d shape (row by row) to the neighborhood of each pixel and passes the result of the overlay to touch_fn.
 * Special case: When processing the first and last pixel of each row, interpreting the missing neighbor as 0.
 * @tparam TouchFn function object.
 * Signature should be equivalent to the following:
 * void fn(uint8_t& out, std::initializer_list<uint8_t> in),
 * where out - pixel of the output image, in - result of overlaying the shape on the neighborhood of source pixel
 * @param input input image
 * @param first_input_row row from which to start processing on the input image
 * @param output output image
 * @param first_output_row row from which to start processing on the output image
 * @param shape structuring element row (size 3, i.e. shape[0], shape[1], shape[2])
 * Should only contain values 0 (ignore neighborhood pixel) or 255 (use pixel).
 * @param touch_fn binary operation that updates a pixel in the output image with an overlay
 */
template<class Apply>
void probeRows(
  const Image<uint8_t> & input, size_t first_input_row,
  Image<uint8_t> & output, size_t first_output_row,
  const uint8_t * shape, Apply touch_fn)
{
  const size_t rows = input.rows() - std::max(first_input_row, first_output_row);
  const size_t columns = input.columns();

  auto apply_shape = [&shape](uint8_t value, uint8_t index) -> uint8_t {
      return value & shape[index];
    };

  auto get_input_row = [&input, first_input_row](size_t row) {
      return input.row(row + first_input_row);
    };
  auto get_output_row = [&output, first_output_row](size_t row) {
      return output.row(row + first_output_row);
    };

  if (columns == 1) {
    for (size_t i = 0; i < rows; ++i) {
      // process single column. Interpret pixel from column -1 and 1 as 0
      auto overlay = {uint8_t(0), apply_shape(*get_input_row(i), 1), uint8_t(0)};
      touch_fn(*get_output_row(i), overlay);
    }
  } else {
    for (size_t i = 0; i < rows; ++i) {
      const uint8_t * in = get_input_row(i);
      const uint8_t * last_column_pixel = in + columns - 1;
      uint8_t * out = get_output_row(i);

      // process first column. Interpret pixel from column -1 as 0
      {
        auto overlay = {uint8_t(0), apply_shape(*in, 1), apply_shape(*(in + 1), 2)};
        touch_fn(*out, overlay);
        ++in;
        ++out;
      }

      // process next columns up to last
      for (; in != last_column_pixel; ++in, ++out) {
        auto overlay = {
          apply_shape(*(in - 1), 0),
          apply_shape(*(in), 1),
          apply_shape(*(in + 1), 2)
        };
        touch_fn(*out, overlay);
      }

      // process last column
      {
        auto overlay = {apply_shape(*(in - 1), 0), apply_shape(*(in), 1), uint8_t(0)};
        touch_fn(*out, overlay);
        ++in;
        ++out;
      }
    }
  }
}

/**
 * @brief Perform morphological operations
 * @tparam AggregateFn function object.
 * Signature should be equivalent to the following:
 * uint8_t fn(std::initializer_list<uint8_t> in),
 * where in - result of overlaying the shape on the neighborhood of source pixel
 * @param input input image
 * @param output output image
 * @param shape structuring element image with size 3x3.
 * Should only contain values 0 (ignore neighborhood pixel) or 255 (use pixel).
 * @param aggregate neighborhood pixels aggregator
 * @throw std::logic_error if the sizes of the input and output images are different or
 * shape size is not equal to 3x3
 */
template<class AggregateFn>
void morphologyOperation(
  const Image<uint8_t> & input, Image<uint8_t> & output,
  const Image<uint8_t> & shape, AggregateFn aggregate)
{
  if (input.rows() != output.rows() || input.columns() != output.columns()) {
    throw std::logic_error(
            "morphologyOperation: the sizes of the input and output images are different");
  }

  if (shape.rows() != 3 || shape.columns() != 3) {
    throw std::logic_error("morphologyOperation: wrong shape size");
  }

  if (input.empty()) {
    return;
  }

  // Simple write the pixel of the output image (first pass only)
  auto set = [&](uint8_t & res, std::initializer_list<uint8_t> lst) {res = aggregate(lst);};
  // Update the pixel of the output image
  auto update = [&](uint8_t & res, std::initializer_list<uint8_t> lst) {
      res = aggregate({res, aggregate(lst), 0});
    };

  // Apply the central shape row.
  // This operation is applicable to all rows of the image, because at any position of the sliding window,
  // its central row is located on the image. So we start from the zero line of input and output
  probeRows(input, 0, output, 0, shape.row(1), set);

  if (input.rows() > 1) {
    // Apply the top shape row.
    // In the uppermost position of the sliding window, its first row is outside the image border.
    // Therefore, we start filling the output image starting from the line 1 and will process input.rows() - 1 lines in total
    probeRows(input, 0, output, 1, shape.row(0), update);
    // Apply the bottom shape row.
    // Similarly, the input image starting from the line 1 and will process input.rows() - 1 lines in total
    probeRows(input, 1, output, 0, shape.row(2), update);
  }
}

/**
 * @brief Return structuring element 3x3 image by predefined figure type
 * @details Used in morphologyOperation
 */
Image<uint8_t> createShape(ShapeBuffer3x3 & buffer, ConnectivityType connectivity)
{
  /**
   * Shape - a binary matrix that is used as a mask. Each element of which is one of two values:
   * code u - the corresponding pixel of the image will be used
   * code i - the corresponding pixel of the image will be ignored
   */
  static constexpr uint8_t u = 255;
  static constexpr uint8_t i = 0;

  if (connectivity == ConnectivityType::Way8) {
    buffer = {
      u, u, u,
      u, i, u,
      u, u, u};
  } else {
    buffer = {
      i, u, i,
      u, i, u,
      i, u, i};
  }
  return Image<uint8_t>(3, 3, buffer.data(), 3);
}

/**
 * @brief Implementation details for connectedComponents
 * @sa connectedComponents
 */
template<ConnectivityType connectivity, class Label, class IsBg>
Label connectedComponentsImpl(
  const Image<uint8_t> & image, Image<Label> & labels,
  imgproc_impl::EquivalenceLabelTrees<Label> & label_trees, const IsBg & is_background)
{
  using namespace imgproc_impl;
  using PixelPass = ProcessPixel<connectivity>;

  // scanning phase
  // scan row 0
  {
    auto img = makeSafeWindow<uint8_t>(nullptr, image.row(0), image.columns());
    auto lbl = makeSafeWindow<Label>(nullptr, labels.row(0), image.columns());

    const uint8_t * first_row_end = image.row(0) + image.columns();

    for (; img.anchor() < first_row_end; img.next(), lbl.next()) {
      PixelPass::pass(img, lbl, label_trees, is_background);
    }
  }

  // scan rows 1, 2, ...
  for (size_t row = 0; row < image.rows() - 1; ++row) {
    // we can safely ignore checks label_mask for first column
    Window<Label, out_of_bounds_policy::DoNothing> label_mask{labels.row(row), labels.row(row + 1)};

    auto up = image.row(row);
    auto current = image.row(row + 1);

    // scan column 0
    {
      auto img = makeSafeWindow(up, current, image.columns());
      PixelPass::pass(img, label_mask, label_trees, is_background);
    }

    // scan columns 1, 2... image.columns() - 2
    label_mask.next();

    auto img = makeUnsafeWindow(std::next(up), std::next(current));
    const uint8_t * current_row_last_element = current + image.columns() - 1;

    for (; img.anchor() < current_row_last_element; img.next(), label_mask.next()) {
      PixelPass::pass(img, label_mask, label_trees, is_background);
    }

    // scan last column
    if (image.columns() > 1) {
      auto last_img = makeSafeWindow(up, current, image.columns(), image.columns() - 1);
      auto last_label = makeSafeWindow(
        labels.row(row), labels.row(row + 1),
        image.columns(), image.columns() - 1);
      PixelPass::pass(last_img, last_label, label_trees, is_background);
    }
  }

  // analysis phase
  const std::vector<Label> & labels_map = label_trees.getLabels();

  // labeling phase
  labels.forEach(
    [&](Label & l) {
      l = labels_map[l];
    });
  return labels_map.size();
}

/**
 * @brief Object to eliminate grouped noise on the image
 * Stores a label tree that is reused
 * @sa connectedComponents
 */
class GroupsRemover
{
public:
  /// @brief Constructs the object and initializes the label tree
  GroupsRemover()
  {
    label_trees_ = std::make_unique<imgproc_impl::EquivalenceLabelTrees<uint16_t>>();
  }

  /**
   * @brief Calls removeGroupsPickLabelType with the Way4/Way8
   * template parameter based on the runtime value of group_connectivity_type
   * @tparam IsBg functor with signature bool (uint8_t)
   * @param[in,out] image image to be denoised
   * @param buffer dynamic memory block that will be used to store the temp labeled image
   * @param group_connectivity_type pixels connectivity type
   * @param minimal_group_size the border value of group size. Groups of this and larger
   * size will be kept
   * @param is_background returns true if the passed pixel value is background
   */
  template<class IsBg>
  void removeGroups(
    Image<uint8_t> & image, MemoryBuffer & buffer,
    ConnectivityType group_connectivity_type, size_t minimal_group_size,
    const IsBg & is_background) const
  {
    if (group_connectivity_type == ConnectivityType::Way4) {
      removeGroupsPickLabelType<ConnectivityType::Way4>(
        image, buffer, minimal_group_size,
        is_background);
    } else {
      removeGroupsPickLabelType<ConnectivityType::Way8>(
        image, buffer, minimal_group_size,
        is_background);
    }
  }

private:
  /**
   * @brief Calls tryRemoveGroupsWithLabelType with the label tree stored in this object.
   * If the stored tree labels are 16 bits and the call fails,
   * change the stored tree type to 32 bit and retry the call.
   * @throw imgproc_impl::LabelOverflow if 32 bit label tree is not enough
   * to complete the operation
   */
  template<ConnectivityType connectivity, class IsBg>
  void removeGroupsPickLabelType(
    Image<uint8_t> & image, MemoryBuffer & buffer,
    size_t minimal_group_size, const IsBg & is_background) const
  {
    bool success{};
    auto label_trees16 =
      dynamic_cast<imgproc_impl::EquivalenceLabelTrees<uint16_t> *>(label_trees_.get());

    if (label_trees16) {
      success = tryRemoveGroupsWithLabelType<connectivity>(
        image, buffer, minimal_group_size,
        *label_trees16, is_background, false);
    }

    if (!success) {
      auto label_trees32 =
        dynamic_cast<imgproc_impl::EquivalenceLabelTrees<uint32_t> *>(label_trees_.get());

      if (!label_trees32) {
        label_trees_ = std::make_unique<imgproc_impl::EquivalenceLabelTrees<uint32_t>>();
        label_trees32 =
          dynamic_cast<imgproc_impl::EquivalenceLabelTrees<uint32_t> *>(label_trees_.get());
      }
      tryRemoveGroupsWithLabelType<connectivity>(
        image, buffer, minimal_group_size, *label_trees32,
        is_background, true);
    }
  }
  /**
   * @brief Calls removeGroupsImpl catching its exceptions if throw_on_label_overflow is true
   * @param throw_on_label_overflow defines the policy for handling exceptions thrown
   * from removeGroupsImpl. If throw_on_label_overflow is true, exceptions are simply
   * rethrown. Otherwise, this function will return false on exception.
   * @return true if removeGroupsImpl throw and throw_on_label_overflow false.
   * False in other case
   */
  template<ConnectivityType connectivity, class Label, class IsBg>
  bool tryRemoveGroupsWithLabelType(
    Image<uint8_t> & image, MemoryBuffer & buffer, size_t minimal_group_size,
    imgproc_impl::EquivalenceLabelTrees<Label> & label_trees,
    const IsBg & is_background,
    bool throw_on_label_overflow) const
  {
    bool success{};
    try {
      removeGroupsImpl<connectivity>(image, buffer, label_trees, minimal_group_size, is_background);
      success = true;
    } catch (imgproc_impl::LabelOverflow &) {
      if (throw_on_label_overflow) {
        throw;
      }
    }
    return success;
  }
  /// @brief Eliminate group noise in the image
  template<ConnectivityType connectivity, class Label, class IsBg>
  void removeGroupsImpl(
    Image<uint8_t> & image, MemoryBuffer & buffer,
    imgproc_impl::EquivalenceLabelTrees<Label> & label_trees, size_t minimal_group_size,
    const IsBg & is_background) const
  {
    // Creates an image labels in which each obstacles group is labeled with a unique code
    Label groups_count;
    auto labels = connectedComponents<connectivity>(image, buffer, label_trees,
        is_background, groups_count);

    // Calculates the size of each group.
    // Group size is equal to the number of pixels with the same label
    const Label max_label_value = groups_count - 1;  // It's safe. groups_count always non-zero
    std::vector<size_t> groups_sizes = histogram(
      labels, max_label_value, size_t(minimal_group_size + 1));

    // The group of pixels labeled 0 corresponds to empty map cells.
    // Zero bin of the histogram is equal to the number of pixels in this group.
    // Because the values of empty map cells should not be changed, we will reset this bin
    groups_sizes.front() = 0;  // don't change image background value

    // noise_labels_table[i] = true if group with label i is noise
    std::vector<bool> noise_labels_table(groups_sizes.size());
    auto transform_fn = [&minimal_group_size](size_t bin_value) {
        return bin_value < minimal_group_size;
      };
    std::transform(
      groups_sizes.begin(), groups_sizes.end(), noise_labels_table.begin(),
      transform_fn);

    // Replace the pixel values from the small groups to background code
    labels.convert(
      image, [&](Label src, uint8_t & trg) {
        if (!is_background(trg) && noise_labels_table[src]) {
          trg = 0;
        }
      });
  }

private:
  mutable std::unique_ptr<imgproc_impl::EquivalenceLabelTreesBase> label_trees_;
};

}  // namespace imgproc_impl

template<ConnectivityType connectivity, class Label, class IsBg>
Image<Label> connectedComponents(
  const Image<uint8_t> & image, MemoryBuffer & buffer,
  imgproc_impl::EquivalenceLabelTrees<Label> & label_trees,
  const IsBg & is_background,
  Label & total_labels)
{
  using namespace imgproc_impl;
  const size_t pixels = image.rows() * image.columns();

  if (pixels == 0) {
    total_labels = 0;
    return Image<Label>{};
  }

  Label * image_buffer = buffer.get<Label>(pixels);
  Image<Label> labels(image.rows(), image.columns(), image_buffer, image.columns());
  label_trees.reset(image.rows(), image.columns(), connectivity);
  total_labels = connectedComponentsImpl<connectivity>(
    image, labels, label_trees,
    is_background);
  return labels;
}

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__DENOISE__IMAGE_PROCESSING_HPP_
