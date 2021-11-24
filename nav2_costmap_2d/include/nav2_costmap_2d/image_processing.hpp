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

#ifndef NAV2_COSTMAP_2D__IMAGE_PROCESSING_HPP_
#define NAV2_COSTMAP_2D__IMAGE_PROCESSING_HPP_

#include "image.hpp"
#include <algorithm>

namespace nav2_costmap_2d
{

/// Pixels connectivity type (is the way in which pixels in image relate to
/// their neighbors)
enum class ConnectivityType: int
{
  /// neighbors pixels are connected horizontally and vertically
  Way4 = 4,
  /// neighbors pixels are connected horizontally, vertically and diagonally
  Way8 = 8
};

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
std::vector<Bin> histogram(const Image<T> & image, T image_max, Bin bin_max);

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

/**
 * @brief A memory buffer that can grow to an upper-bounded capacity
 */
class MemoryBuffer
{
public:
  /**
   * @brief Creates an object with the fixed maximum capacity. Doesn't perform allocations
   * @param capacity buffer capacity in bytes
   */
  inline MemoryBuffer(size_t capacity)
  : capacity_{capacity} {}
  /// @brief Free memory allocated for the buffer
  inline ~MemoryBuffer() {reset();}
  /**
   * @brief Return a pointer to an uninitialized array of count elements
   * Delete the old block of memory and allocates a new one if the size of the old is too small.
   * The returned pointer is valid until the next call to get() or destructor.
   * @tparam T type of element
   * @param count number of elements
   * @throw std::logic_error if buffer size limited and sizeof(T) * count > capacity
   */
  template<class T>
  T * get(std::size_t count);
  /// @brief Return the buffer capacity in bytes
  inline size_t capacity() {return capacity_;}

private:
  inline void reset();
  inline void allocate(size_t bytes);

private:
  void * data{};
  size_t size{};
  size_t capacity_{};
};

// forward declarations
namespace imgproc_impl
{
template<class Label>
class EquivalenceLabelTrees;

template<class AggregateFn>
void morphology_operation(
  const Image<uint8_t> & input, Image<uint8_t> & output,
  const Image<uint8_t> & shape, AggregateFn aggregate);
inline Image<uint8_t> getShape(ConnectivityType connectivity);
} // namespace imgproc_impl

/**
 * @brief Perform morphological dilation
 * @param input input image
 * @param output output image
 * @param connectivity selector for selecting structuring element (Way4-> cross, Way8-> rect)
 */
inline void dilate(
  const Image<uint8_t> & input, Image<uint8_t> & output,
  ConnectivityType connectivity)
{
  using namespace imgproc_impl;
  auto aggregate = [](std::initializer_list<uint8_t> lst) {
      return std::max(lst);
    };
  morphology_operation(input, output, getShape(connectivity), aggregate);
}

template<ConnectivityType connectivity, class Label>
class ConnectedComponents
{
public:
  /**
   * @brief Compute the connected components labeled image of binary image
   * Implements the SAUF algorithm
   * (Two Strategies to Speed up Connected Component Labeling Algorithms
   * Kesheng Wu, Ekow Otoo, Kenji Suzuki).
   * @tparam connectivity pixels connectivity type
   * @tparam Label integer type of label
   * @param image input image. Pixels less than 255 will be interpreted as background
   * @param buffer memory block that will be used to store the result (labeled image)
   * and the internal buffer for labels trees
   * @throw std::logic_error in case of insufficient memory on labels integer overflow
   * @return pair(labeled image, total number of labels)
   * Labeled image has the same size as image. Label 0 represents the background label,
   * labels [1, <return value> - 1] - separate components.
   * Total number of labels == 0 for empty image.
   * In other cases, label 0 is always counted,
   * even if there is no background in the image.
   * For example, for an image of one pixel equal to 255, the total number of labels == 2.
   * Two labels (0, 1) have been counted, although label 0 is not used)
   */
  template<class IsBg>
  static std::pair<Image<Label>, Label> detect(
    const Image<uint8_t> & image, MemoryBuffer & buffer,
    IsBg && is_background);
  /**
   * @brief Return the upper bound of the memory buffer for detecting connected components
   * @param image input image
   */
  static size_t optimalBufferSize(const Image<uint8_t> & image);

private:
  template<class IsBg>
  static Label detectImpl(
    const Image<uint8_t> & image, Image<Label> & labels,
    imgproc_impl::EquivalenceLabelTrees<Label> & label_trees,
    IsBg && is_background);
};

// Implementation

template<class T>
T * MemoryBuffer::get(std::size_t count)
{
  // Check the memory allocated by ::operator new can be used to store the type T
  static_assert(
    alignof(std::max_align_t) >= alignof(T),
    "T alignment is more than the fundamental alignment of the platform");

  const size_t required_bytes = sizeof(T) * count;

  // throw if buffer limited and required_bytes exceeds the limit
  if (required_bytes > capacity_) {
    throw std::logic_error(
            "MemoryBuffer::get: The requested block amount exceeds the maximum allowed");
  }

  if (size < required_bytes) {
    allocate(required_bytes);
  }
  return static_cast<T *>(data);
}

void MemoryBuffer::reset()
{
  ::operator delete(data);
  size = 0;
}

void MemoryBuffer::allocate(size_t bytes)
{
  reset();
  data = ::operator new(bytes);
  this->size = bytes;
}

namespace imgproc_impl
{

/**
   * @brief Boundary case object stub. Used as parameter of class Window.
   * Dereferences a pointer to a pixel without any checks
   * @tparam T image pixel type
   * @sa BorderConstant
   */
template<class T>
struct AsIs
{
  T & up(T * v) const {return *v;}
  T & down(T * v) const {return *v;}
};

/**
 * @brief Boundary case object. Used as parameter of class Window.
 * Dereferences a pointer to a existing pixel. If the pixel is out of bounds, it returns a ref to 0.
 * @tparam T image pixel type
 * @sa AsIs
 */
template<class T>
class BorderConstant
{
public:
  /**
   * @brief Create an object that will replace pointers outside the specified range
   * @param up_row_start pointer to the first pixel of up row. Can be nullptr.
   * @param down_row_start pointer to the first pixel of down row
   * @param columns number of pixels in both rows
   */
  BorderConstant(const T * up_row_start, const T * down_row_start, size_t columns)
  : up_row_start{up_row_start}, up_row_end{up_row_start + columns},
    down_row_start{down_row_start}, down_row_end{down_row_start + columns} {}

  /**
   * @brief Return ref to pixel or to zero value if up_row_start is nullptr or the pointer is out of bounds
   * @param v pointer to pixel
   */
  T & up(T * v)
  {
    if (up_row_start == nullptr) {
      return zero;
    }
    return replace_bounds(v, up_row_start, up_row_end);
  }

  /**
   * @brief Return ref to pixel or to zero value if the pointer is out of bounds
   * @param v pointer to pixel
   */
  T & down(T * v)
  {
    return replace_bounds(v, down_row_start, down_row_end);
  }

private:
  T & replace_bounds(T * v, const T * begin, const T * end)
  {
    if (v < begin || v >= end) {
      return zero;
    }
    return *v;
  }

  const T * up_row_start;
  const T * up_row_end;
  const T * down_row_start;
  const T * down_row_end;
  T zero{};
};

/**
 * @brief Forward scan mask sliding window
 * Provides an interface for access to neighborhood of the current pixel
 * (includes three neighbors of the top row, the pixel to the left and the current one).
 * In the illustration below, the current pixel is e.
 * |a|b|c|
 * |d|e| |
 * | | | |
 * @tparam T image pixel type
 * @tparam Border optional check of access to pixels outside the image boundary (AsIs or BorderConstant)
 */
template<class T, template<class> class Border = AsIs>
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
  : up_row{up_row}, down_row{down_row}, border{border} {}

  inline T & a() {return border.up(up_row - 1);}
  inline T & b() {return border.up(up_row);}
  inline T & c() {return border.up(up_row + 1);}
  inline T & d() {return border.down(down_row - 1);}
  inline T & e() {return border.down(down_row);}

  /// @brief Shifts the window to the right
  inline void next()
  {
    ++up_row;
    ++down_row;
  }

private:
  T * up_row;
  T * down_row;
  Border<T> border;
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
Window<T, BorderConstant> makeSafeWindow(
  const T * up_row, const T * down_row, size_t columns, size_t offset = 0)
{
  return {
    dropConst(up_row) + offset, dropConst(down_row) + offset,
    BorderConstant<T>{up_row, down_row, columns}
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
Window<T> makeUnsafeWindow(const T * up_row, const T * down_row)
{
  return {dropConst(up_row), dropConst(down_row)};
}

/**
 * @brief Union-find data structure
 * Implementation of union-find data structure, described in reference article.
 * Store rooted trees, where each node of a tree is a provisional label and each edge represents an
 * equivalence between two labels
 * @tparam Label integer type of label
 */
template<class Label>
class EquivalenceLabelTrees
{
public:
  /**
   * @brief Construct object. Initialize labels buffer
   * @param labels_buffer pointer to labels buffer
   * @param buffer_size the size of labels buffer
   * @throw std::logic_error if buffer_size == 0
   */
  EquivalenceLabelTrees(Label * labels_buffer, size_t buffer_size)
  {
    if (buffer_size == 0) {
      throw std::logic_error("EquivalenceLabelTrees: empty buffer is not allowed");
    }
    labels = labels_buffer;
    // Label 0 is reserved for the background pixels, i.e. *labels_begin is always 0
    *labels = 0;
    next_free = 1;
    // Number of labels cannot exceed std::numeric_limits<Label>::max()
    labels_size = static_cast<Label>(
      std::min(buffer_size, size_t(std::numeric_limits<Label>::max()))
    );
  }
  /**
   * @brief Defines the upper bound for the number of labels
   * @param rows number of image rows
   * @param columns number of image columns
   * @param connectivity pixels connectivity type
   * @return max labels count
   */
  static size_t max_labels(const size_t rows, const size_t columns, ConnectivityType connectivity)
  {

    size_t max_labels{};

    if (connectivity == ConnectivityType::Way4) {
      /* The maximum of individual components will be reached in the chessboard image,
       * where the white cells correspond to zero pixels */
      max_labels = (rows * columns) / 2 + 1;
    } else {
      /* The maximum of individual components will be reached in image like this:
       * x.x.x.x~
       * .......~
       * x.x.x.x~
       * .......~
       * x.x.x.x~
       * ~
       * where 'x' - pixel with code 255, '.' - pixel with code [0, 254],
       * '~' - row continuation in the same style */
      max_labels = (rows * columns) / 3 + 1;
    }
    ++max_labels; // add zero label
    max_labels = std::min(max_labels, size_t(std::numeric_limits<Label>::max()));
    return max_labels;
  }
  /**
   * @brief Return next unused label
   * @throw std::logic_error if all labels values already
   * @return label
   */
  Label makeLabel()
  {
    // Check the labels array out of bounds.
    // At the same time, this check ensures that the next_free counter does not overflow.
    if (next_free == labels_size) {
      throw std::logic_error("EquivalenceLabelTrees: Can't create new label");
    }
    labels[next_free] = next_free;
    return next_free++;
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
   */
  std::pair<const Label *, Label> getLabels()
  {
    Label k = 1;
    for (Label i = 1; i < next_free; ++i) {

      if (labels[i] < i) {
        labels[i] = labels[labels[i]];
      } else {
        labels[i] = k;
        ++k;
      }
    }
    return std::make_pair(labels, k);
  }

private:
  /// @brief Find the root of the tree of node i
  Label findRoot(Label i)
  {
    Label root = i;
    for (; labels[root] < root; root = labels[root]) { /*do nothing*/}
    return root;
  }

  /// @brief Set the root of the tree of node i
  void setRoot(Label i, Label root)
  {
    while (labels[i] < i) {
      auto j = labels[i];
      labels[i] = root;
      i = j;
    }
    labels[i] = root;
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
  Label * labels;
  Label labels_size;
  Label next_free;
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
   * @tparam ImageWindow Window parameterized by class AsIs or BorderConstant
   * @tparam LabelsWindow Window parameterized by class AsIs or BorderConstant
   * @tparam Label integer type of label
   * @param image input image window. Image data will not be changed. De facto, image is a const ref
   * @param label output label window
   * @param eqTrees union-find structure
   */
  template<class ImageWindow, class LabelsWindow, class Label, class IsBg>
  static void pass(
    ImageWindow & image, LabelsWindow & label, EquivalenceLabelTrees<Label> & eqTrees,
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
            current = eqTrees.unionTrees(label.c(), label.a());
          } else {
            if (!is_bg(image.d())) {
              current = eqTrees.unionTrees(label.c(), label.d());
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
              current = eqTrees.makeLabel();
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
   * @tparam ImageWindow Window parameterized by class AsIs or BorderConstant
   * @tparam LabelsWindow Window parameterized by class AsIs or BorderConstant
   * @tparam Label integer type of label
   * @param image input image window. Image data will not be changed. De facto, image is a const ref
   * @param label output label window
   * @param eqTrees union-find structure
   */
  template<class ImageWindow, class LabelsWindow, class Label, class IsBg>
  static void pass(
    ImageWindow & image, LabelsWindow & label, EquivalenceLabelTrees<Label> & eqTrees,
    IsBg && is_bg)
  {
    Label & current = label.e();

    // Simplified decision tree traversal. See reference article for details
    if (!is_bg(image.e())) {
      if (!is_bg(image.b())) {
        if (!is_bg(image.d())) {
          current = eqTrees.unionTrees(label.d(), label.b());
        } else {
          current = label.b();
        }
      } else {
        if (!is_bg(image.d())) {
          current = label.d();
        } else {
          current = eqTrees.makeLabel();
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
 * @param in pointer to the first row of the input image
 * @param out pointer to the first row of the output image
 * @param rows number of image rows
 * @param columns number of image columns
 * @param shape structuring element row (size 3, i.e. shape[0], shape[1], shape[2])
 * Should only contain values 0 (ignore neighborhood pixel) or 255 (use pixel).
 * @param touch_fn binary operation that updates a pixel in the output image with an overlay
 */
template<class Apply>
void probeRows(
  const uint8_t * in, uint8_t * out,
  size_t rows, size_t columns, const uint8_t * shape, Apply touch_fn)
{
  auto apply_shape = [&shape](uint8_t value, uint8_t index) -> uint8_t {
      return value & shape[index];
    };

  if (columns == 1) {
    for (size_t i = 0; i < rows; ++i) {
      // process single column. Interpret pixel from column -1 and 1 as 0
      auto overlay = {uint8_t(0), apply_shape(*in, 1), uint8_t(0)};
      touch_fn(*out, overlay);
      ++in;
      ++out;
    }
  } else {
    for (size_t i = 0; i < rows; ++i) {
      const uint8_t * last_column_pixel = in + columns - 1;

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
void morphology_operation(
  const Image<uint8_t> & input, Image<uint8_t> & output,
  const Image<uint8_t> & shape, AggregateFn aggregate)
{
  if (input.rows() != output.rows() || input.columns() != output.columns()) {
    throw std::logic_error(
            "morphology_operation: the sizes of the input and output images are different");
  }

  if (shape.rows() != 3 || shape.columns() != 3) {
    throw std::logic_error("morphology_operation: wrong shape size");
  }

  if (input.empty()) {
    return;
  }

  // Simple write the pixel of the output image (first pass only)
  auto set = [&](uint8_t & res, std::initializer_list<uint8_t> lst) {res = aggregate(lst);};
  // Update the pixel of the output image
  auto update = [&](uint8_t & res, std::initializer_list<uint8_t> lst) {
      res = aggregate({res, aggregate(lst)});
    };

  // Apply the second shape row
  probeRows(input.row(0), output.row(0), input.rows(), input.columns(), shape.row(1), set);

  if (input.rows() > 1) {
    // Apply the first (top) shape row
    probeRows(input.row(0), output.row(1), input.rows() - 1, input.columns(), shape.row(0), update);
    // Apply the last shape row
    probeRows(input.row(1), output.row(0), input.rows() - 1, input.columns(), shape.row(2), update);
  }
}

/// @brief Return structuring element 3x3 image by predefined figure type
Image<uint8_t> getShape(ConnectivityType connectivity)
{
  Image<uint8_t> shape(3, 3);

  if (connectivity == ConnectivityType::Way8) {
    shape.fill(255);
    shape.at(1, 1) = 0;
  } else {
    shape.fill(0);
    shape.at(1, 0) = 255;
    shape.at(1, 2) = 255;
    shape.at(0, 1) = 255;
    shape.at(2, 1) = 255;
  }
  return shape;
}

}  // namespace imgproc_impl

template<ConnectivityType connectivity, class Label>
template<class IsBg>
std::pair<Image<Label>, Label> ConnectedComponents<connectivity, Label>::detect(
  const Image<uint8_t> & image, MemoryBuffer & buffer, IsBg && is_background)
{
  using namespace imgproc_impl;
  const size_t pixels = image.rows() * image.columns();

  if (pixels == 0) {
    return {Image<Label>{}, 0};
  }
  const size_t labels_image_bytes = pixels * sizeof(Label);

  if (buffer.capacity() <= labels_image_bytes) {
    throw std::logic_error("connectedComponents(): Not enough memory");
  }

  const size_t rest_buffer_bytes = buffer.capacity() - labels_image_bytes;
  const size_t max_labels = EquivalenceLabelTrees<Label>::max_labels(
    image.rows(), image.columns(), connectivity);

  const size_t labels_buffer_size = std::min(max_labels, rest_buffer_bytes / sizeof(Label));
  Label * buffer_block = buffer.get<Label>(pixels + labels_buffer_size);

  Label * image_buffer = buffer_block;
  Label * labels_block = buffer_block + pixels;
  Image<Label> labels(image.rows(), image.columns(), image_buffer, image.columns());
  EquivalenceLabelTrees<Label> label_trees(labels_block, labels_buffer_size);
  const Label total_labels = detectImpl(image, labels, label_trees, is_background);
  return std::make_pair(labels, total_labels);
}

template<ConnectivityType connectivity, class Label>
template<class IsBg>
Label
ConnectedComponents<connectivity, Label>::detectImpl(
  const Image<uint8_t> & image, Image<Label> & labels,
  imgproc_impl::EquivalenceLabelTrees<Label> & label_trees,
  IsBg && is_background)
{
  using namespace imgproc_impl;
  using PixelPass = ProcessPixel<connectivity>;

  // scanning phase
  // scan row 0
  {
    auto img = makeSafeWindow<uint8_t>(nullptr, image.row(0), image.columns());
    auto lbl = makeSafeWindow<Label>(nullptr, labels.row(0), image.columns());

    for (; &img.e() < image.row(0) + image.columns(); img.next(), lbl.next()) {
      PixelPass::pass(img, lbl, label_trees, is_background);
    }
  }

  // scan rows 1, 2, ...
  for (size_t row = 0; row < image.rows() - 1; ++row) {
    // we can safely ignore checks label_mask for first column
    Window<Label> label_mask{labels.row(row), labels.row(row + 1)};

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
    const uint8_t * current_line_last = current + image.columns() - 1;

    for (; &img.e() < current_line_last; img.next(), label_mask.next()) {
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
  auto labels_and_count = label_trees.getLabels();
  const Label * labels_map = labels_and_count.first;
  const Label labels_count = labels_and_count.second;

  // labeling phase
  labels.forEach(
    [&](Label & l) {
      l = labels_map[l];
    });
  return labels_count;
}

template<ConnectivityType connectivity, class Label>
size_t ConnectedComponents<connectivity, Label>::optimalBufferSize(const Image<uint8_t> & image)
{
  const size_t labels_image_bytes = image.rows() * image.columns() * sizeof(Label);
  const size_t labels_trees_bytes = imgproc_impl::EquivalenceLabelTrees<Label>::max_labels(
    image.rows(), image.columns(), connectivity) * sizeof(Label);
  return labels_image_bytes + labels_trees_bytes;
}

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__IMAGE_PROCESSING_HPP_
