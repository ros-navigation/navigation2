/*********************************************************************
*
* Licensed to the Apache Software Foundation (ASF) under one
* or more contributor license agreements.  See the NOTICE file
* distributed with this work for additional information
* regarding copyright ownership.  The ASF licenses this file
* to you under the Apache License, Version 2.0 (the
* "License"); you may not use this file except in compliance
* with the License.  You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing,
* software distributed under the License is distributed on an
* "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
* KIND, either express or implied.  See the License for the
* specific language governing permissions and limitations
* under the License.
*
* Author: Andrey Ryzhikov
*********************************************************************/

#include <gtest/gtest.h>

#include <string>
#include <vector>
#include <tuple>
#include <stdexcept>
#include <algorithm>

#include "nav2_costmap_2d/denoise_layer.hpp"

/**
 * @brief nav2_costmap_2d::DenoiseLayer class wrapper
 *
 * Provides access to DenoiseLayer private methods for testing them in isolation
 */
class DenoiseLayerTester : public ::testing::Test
{
public:
  using ConnectivityType = nav2_costmap_2d::DenoiseLayer::ConnectivityType;
  template<class SourceElement, class TargetElement, class Converter>
  void convert(const cv::Mat & source, cv::Mat & target, Converter operation) const
  {
    denoise_.convert<SourceElement, TargetElement>(source, target, operation);
  }

  void checkImagesSizesEqual(const cv::Mat & a, const cv::Mat & b, const char * error_prefix) const
  {
    denoise_.checkImagesSizesEqual(a, b, error_prefix);
  }

  void checkImageType(const cv::Mat & image, int cv_type, const char * error_prefix) const
  {
    denoise_.checkImageType(image, cv_type, error_prefix);
  }

  std::vector<uint8_t> makeLookupTable(
    const std::vector<uint16_t> & groups_sizes, uint16_t threshold) const
  {
    return denoise_.makeLookupTable(groups_sizes, threshold);
  }

  std::vector<uint16_t> calculateHistogram(
    const cv::Mat & image, uint16_t image_max, uint16_t bin_max) const
  {
    return denoise_.calculateHistogram(image, image_max, bin_max);
  }

  void removeSinglePixels(cv::Mat & image, ConnectivityType connectivity)
  {
    denoise_.group_connectivity_type = connectivity;
    denoise_.removeSinglePixels(image);
  }

  void removeGroups(cv::Mat & image, ConnectivityType connectivity, size_t minimal_group_size)
  {
    denoise_.group_connectivity_type = connectivity;
    denoise_.minimal_group_size = minimal_group_size;
    denoise_.removeGroups(image);
  }

  void denoise(cv::Mat & image, ConnectivityType connectivity, size_t minimal_group_size)
  {
    denoise_.group_connectivity_type = connectivity;
    denoise_.minimal_group_size = minimal_group_size;
    denoise_.denoise(image);
  }

  bool reset()
  {
    denoise_.current_ = true;
    denoise_.reset();
    return denoise_.current_;
  }

  static void initialize(nav2_costmap_2d::DenoiseLayer & d)
  {
    d.onInitialize();
  }

  static bool & touchCurrent(nav2_costmap_2d::DenoiseLayer & d)
  {
    return d.current_;
  }

  static void configure(
    nav2_costmap_2d::DenoiseLayer & d, ConnectivityType connectivity, size_t minimal_group_size)
  {
    d.enabled_ = true;
    d.group_connectivity_type = connectivity;
    d.minimal_group_size = minimal_group_size;
  }

  static std::tuple<bool, DenoiseLayerTester::ConnectivityType, size_t> getParameters(
    const nav2_costmap_2d::DenoiseLayer & d)
  {
    return std::make_tuple(d.enabled_, d.group_connectivity_type, d.minimal_group_size);
  }

private:
  nav2_costmap_2d::DenoiseLayer denoise_;
};

TEST_F(DenoiseLayerTester, checkEqualImagesSize) {
  const cv::Mat a(3, 2, CV_8UC1);
  const cv::Mat b(3, 2, CV_16UC1);

  ASSERT_NO_THROW(checkImagesSizesEqual(a, b, ""));
}

TEST_F(DenoiseLayerTester, checkEqualImagesSizeForDifferentImages) {
  const cv::Mat a(3, 2, CV_8UC1);
  const cv::Mat b(2, 3, CV_16UC1);

  ASSERT_THROW(checkImagesSizesEqual(a, b, ""), std::logic_error);
}

TEST_F(DenoiseLayerTester, checkImageType) {
  const cv::Mat a(1, 1, CV_16UC1);

  ASSERT_NO_THROW(checkImageType(a, CV_16UC1, ""));
}

TEST_F(DenoiseLayerTester, checkImageTypeForDifferentTypes) {
  const cv::Mat a(1, 1, CV_16UC1);

  ASSERT_THROW(checkImageType(a, CV_32FC1, ""), std::logic_error);
}

TEST_F(DenoiseLayerTester, convert) {
  const cv::Mat source = (cv::Mat_<uint16_t>(2, 3) << 1, 2, 3, 4, 5, 6);
  cv::Mat target(2, 3, CV_8UC1);

  convert<uint16_t, uint8_t>(
    source, target, [](uint16_t src, uint8_t & trg) {
      trg = src * 2;
    });

  const std::array<uint8_t, 6> expected = {2, 4, 6, 8, 10, 12};
  ASSERT_TRUE(std::equal(expected.begin(), expected.end(), target.ptr()));
}

TEST_F(DenoiseLayerTester, convertDifferentSizes) {
  const cv::Mat source(2, 3, CV_8UC1);
  cv::Mat target(3, 2, CV_8UC1);
  auto do_nothing = [](uint16_t /*src*/, uint8_t & /*trg*/) {};

  // Extra parentheses need to protect commas in template arguments
  ASSERT_THROW((convert<uint8_t, uint8_t>(source, target, do_nothing)), std::logic_error);
}

TEST_F(DenoiseLayerTester, convertEmptyImages) {
  const cv::Mat source;
  cv::Mat target;
  auto shouldn_t_be_called = [](uint16_t /*src*/, uint8_t & /*trg*/) {
      throw std::logic_error("");
    };

  // Extra parentheses need to protect commas in template arguments
  ASSERT_NO_THROW((convert<uint16_t, uint8_t>(source, target, shouldn_t_be_called)));
}

TEST_F(DenoiseLayerTester, makeLookupTable) {
  const auto result = makeLookupTable({1, 0, 2, 3, 4}, 2);

  ASSERT_EQ(result, std::vector<uint8_t>({0, 0, 255, 255, 255}));
}

TEST_F(DenoiseLayerTester, makeLookupTableFromEmpty) {
  const auto result = makeLookupTable({}, 2);

  ASSERT_TRUE(result.empty());
}

TEST_F(DenoiseLayerTester, calculateHistogramWithoutTruncation) {
  const cv::Mat source = (cv::Mat_<uint16_t>(3, 3) << 0, 2, 1, 0, 3, 4, 1, 2, 0);
  const size_t max_bin_size = 3;  // three zeros
  const size_t max_value = 4;

  const auto histogram = calculateHistogram(source, max_value, max_bin_size);

  const std::array<uint8_t, 5> expected = {3, 2, 2, 1, 1};
  ASSERT_EQ(histogram.size(), expected.size());
  ASSERT_TRUE(std::equal(expected.begin(), expected.end(), histogram.begin()));
}

TEST_F(DenoiseLayerTester, calculateHistogramWithTruncation) {
  const cv::Mat source = (cv::Mat_<uint16_t>(3, 3) << 0, 2, 1, 0, 3, 4, 1, 2, 0);
  const size_t max_bin_size = 2;  // truncate zero bin
  const size_t max_value = 4;

  const auto histogram = calculateHistogram(source, max_value, max_bin_size);

  const std::array<uint8_t, 5> expected = {2, 2, 2, 1, 1};
  ASSERT_EQ(histogram.size(), expected.size());
  ASSERT_TRUE(std::equal(expected.begin(), expected.end(), histogram.begin()));
}

TEST_F(DenoiseLayerTester, calculateHistogramOfEmpty) {
  const cv::Mat source(0, 0, CV_16UC1);
  const size_t max_bin_size = 1;
  const size_t max_value = 0;

  const auto histogram = calculateHistogram(source, max_value, max_bin_size);
  ASSERT_TRUE(histogram.empty());
}

TEST_F(DenoiseLayerTester, calculateHistogramWrongType) {
  const cv::Mat source(1, 1, CV_8UC1, cv::Scalar(0));
  const size_t max_bin_size = 1;
  const size_t max_value = 0;

  ASSERT_THROW(calculateHistogram(source, max_value, max_bin_size), std::logic_error);
}

/**
 * @brief Decodes a single channel 8-bit image with values 0 and 255 from a string
 *
 * Used only for tests.
 * String format: '.' - pixel with code 0, 'x' - pixel with code 255.
 * The image is always square, i.e. the number of rows is equal to the number of columns
 * For example, string
 * "x.x"
 * ".x."
 * "..."
 * describes a 3x3 image in which a v-shape is drawn with code 255
 * @throw std::logic_error if the format of the string is incorrect
 */
cv::Mat parseBinaryMatrix(const std::string & s)
{
  const int side_size = static_cast<int>(std::sqrt(s.size()));

  if (size_t(side_size) * side_size != s.size()) {
    throw std::logic_error("Test data error: parseBinaryMatrix: Unexpected input string size");
  }
  cv::Mat mat(side_size, side_size, CV_8UC1);

  std::transform(
    s.begin(), s.end(), mat.ptr(), [](char symbol) -> uint8_t {
      if (symbol == '.') {
        return 0;
      } else if (symbol == 'x') {
        return 255;
      } else {
        throw std::logic_error(
          "Test data error: parseBinaryMatrix: Unexpected symbol: " + std::string(1, symbol));
      }
    });
  return mat;
}

/**
 * @brief Checks exact match of images
 *
 * @return true if images a and b have the same type, size, and data. Otherwise false
 */
bool isEqual(const cv::Mat & a, const cv::Mat & b)
{
  return a.size() == b.size() && a.type() == b.type() && cv::countNonZero(a != b) == 0;
}

TEST_F(DenoiseLayerTester, removeSinglePixels4way) {
  const cv::Mat in = parseBinaryMatrix(
    "x.x."
    "x..x"
    ".x.."
    "xx.x");
  const cv::Mat exp = parseBinaryMatrix(
    "x..."
    "x..."
    ".x.."
    "xx..");

  cv::Mat out = in.clone();
  removeSinglePixels(out, ConnectivityType::Way4);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removeSinglePixels8way) {
  const cv::Mat in = parseBinaryMatrix(
    "x.x."
    "x..x"
    ".x.."
    "xx.x");
  const cv::Mat exp = parseBinaryMatrix(
    "x.x."
    "x..x"
    ".x.."
    "xx..");

  cv::Mat out = in.clone();
  removeSinglePixels(out, ConnectivityType::Way8);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removeSinglePixelsFromExtremelySmallImage) {
  {
    const cv::Mat in = parseBinaryMatrix(
      "x");
    const cv::Mat exp = parseBinaryMatrix(
      ".");

    cv::Mat out = in.clone();
    removeSinglePixels(out, ConnectivityType::Way8);

    ASSERT_TRUE(isEqual(out, exp));
  }

  {
    const cv::Mat in = parseBinaryMatrix(
      "x."
      ".x");
    const cv::Mat exp = parseBinaryMatrix(
      "x."
      ".x");

    cv::Mat out = in.clone();
    removeSinglePixels(out, ConnectivityType::Way8);

    ASSERT_TRUE(isEqual(out, exp));
  }

  {
    const cv::Mat in = parseBinaryMatrix(
      "x."
      ".x");
    const cv::Mat exp = parseBinaryMatrix(
      ".."
      "..");

    cv::Mat out = in.clone();
    removeSinglePixels(out, ConnectivityType::Way4);

    ASSERT_TRUE(isEqual(out, exp));
  }
}

TEST_F(DenoiseLayerTester, removeSinglePixelsFromNonBinary) {
  cv::Mat in = cv::Mat(3, 3, CV_8UC1, cv::Scalar(254));
  in.at<uint8_t>(1, 1) = 255;

  cv::Mat exp = cv::Mat(3, 3, CV_8UC1, cv::Scalar(254));
  exp.at<uint8_t>(1, 1) = 0;

  cv::Mat out = in.clone();
  removeSinglePixels(out, ConnectivityType::Way4);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removePixelsGroup4way) {
  const cv::Mat in = parseBinaryMatrix(
    ".xx..xx"
    "..x.x.."
    "x..x..x"
    "x......"
    "...x.xx"
    "xxx..xx"
    "....xx.");
  const cv::Mat exp = parseBinaryMatrix(
    ".xx...."
    "..x...."
    "......."
    "......."
    ".....xx"
    "xxx..xx"
    "....xx.");

  cv::Mat out = in.clone();
  removeGroups(out, ConnectivityType::Way4, 3);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removePixelsGroup8way) {
  const cv::Mat in = parseBinaryMatrix(
    ".xx..xx"
    "..x.x.."
    "x..x..x"
    "x......"
    "...x.xx"
    "xxx..xx"
    "....xx.");
  const cv::Mat exp = parseBinaryMatrix(
    ".xx..xx"
    "..x.x.."
    "...x..."
    "......."
    "...x.xx"
    "xxx..xx"
    "....xx.");

  cv::Mat out = in.clone();
  removeGroups(out, ConnectivityType::Way8, 3);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removePixelsGroupFromExtremelySmallImage) {
  {
    const cv::Mat in = parseBinaryMatrix(
      "x");
    const cv::Mat exp = parseBinaryMatrix(
      ".");

    cv::Mat out = in.clone();
    removeGroups(out, ConnectivityType::Way8, 3);

    ASSERT_TRUE(isEqual(out, exp));
  }

  {
    const cv::Mat in = parseBinaryMatrix(
      "x."
      ".x");
    const cv::Mat exp = parseBinaryMatrix(
      ".."
      "..");

    cv::Mat out = in.clone();
    removeGroups(out, ConnectivityType::Way8, 3);

    ASSERT_TRUE(isEqual(out, exp));
  }
}

TEST_F(DenoiseLayerTester, removePixelsGroupFromNonBinary) {
  cv::Mat in = cv::Mat(3, 3, CV_8UC1, cv::Scalar(254));
  in.at<uint8_t>(1, 1) = 255;

  cv::Mat exp = cv::Mat(3, 3, CV_8UC1, cv::Scalar(254));
  exp.at<uint8_t>(1, 1) = 0;

  cv::Mat out = in.clone();
  removeGroups(out, ConnectivityType::Way4, 2);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, denoiseSingles) {
  const cv::Mat in = parseBinaryMatrix(
    "xx."
    "..."
    "..x");
  const cv::Mat exp = parseBinaryMatrix(
    "xx."
    "..."
    "...");

  cv::Mat out = in.clone();
  denoise(out, ConnectivityType::Way4, 2);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, denoiseGroups) {
  const cv::Mat in = parseBinaryMatrix(
    "xx."
    "x.x"
    "..x");
  const cv::Mat exp = parseBinaryMatrix(
    "xx."
    "x.."
    "...");

  cv::Mat out = in.clone();
  denoise(out, ConnectivityType::Way4, 3);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, denoiseWrongImageType) {
  cv::Mat in(1, 1, CV_16UC1);

  ASSERT_THROW(denoise(in, ConnectivityType::Way4, 2), std::logic_error);
}

TEST_F(DenoiseLayerTester, denoiseEmpty) {
  cv::Mat in(0, 0, CV_8UC1);

  ASSERT_NO_THROW(denoise(in, ConnectivityType::Way4, 2));
}

TEST_F(DenoiseLayerTester, denoiseNothing) {
  cv::Mat in(1, 1, CV_8UC1);

  ASSERT_NO_THROW(denoise(in, ConnectivityType::Way4, 1));
}

TEST_F(DenoiseLayerTester, constructorAndDestructor) {
  ASSERT_NO_THROW(
    []() {
      nav2_costmap_2d::DenoiseLayer layer;
    });
}

TEST_F(DenoiseLayerTester, reset) {
  ASSERT_FALSE(reset());
}

TEST_F(DenoiseLayerTester, isClearable) {
  nav2_costmap_2d::DenoiseLayer layer;

  ASSERT_FALSE(layer.isClearable());
}

TEST_F(DenoiseLayerTester, updateBounds) {
  nav2_costmap_2d::DenoiseLayer layer;

  const std::array<double, 4> region = {1., 2., 3., 4.};
  auto r = region;

  ASSERT_NO_THROW(layer.updateBounds(0., 0., 0., &r[0], &r[1], &r[2], &r[3]));
  ASSERT_EQ(r, region);
}

TEST_F(DenoiseLayerTester, updateCostsIfDisabled) {
  nav2_costmap_2d::DenoiseLayer layer;
  nav2_costmap_2d::Costmap2D costmap(1, 1, 1., 0., 0., 255);

  layer.updateCosts(costmap, 0, 0, 1, 1);

  ASSERT_EQ(costmap.getCost(0), 255);
}

TEST_F(DenoiseLayerTester, updateCosts) {
  nav2_costmap_2d::DenoiseLayer layer;
  nav2_costmap_2d::Costmap2D costmap(1, 1, 1., 0., 0., 255);
  DenoiseLayerTester::configure(layer, DenoiseLayerTester::ConnectivityType::Way4, 2);

  layer.updateCosts(costmap, 0, 0, 1, 1);

  ASSERT_EQ(costmap.getCost(0), 0);
}

// Copy paste from declare_parameter_test.cpp
class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture rcl_cpp_fixture;

std::shared_ptr<nav2_costmap_2d::DenoiseLayer> constructLayer(
  std::shared_ptr<nav2_util::LifecycleNode> node =
  std::make_shared<nav2_util::LifecycleNode>("test_node"))
{
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto layers = std::make_shared<nav2_costmap_2d::LayeredCostmap>("frame", false, false);

  auto deleter = [node, tf, layers](nav2_costmap_2d::DenoiseLayer * ptr)
    {
      delete ptr;
    };
  auto layer = std::shared_ptr<nav2_costmap_2d::DenoiseLayer>(
    new nav2_costmap_2d::DenoiseLayer, deleter);
  layer->initialize(layers.get(), "test_layer", tf.get(), node, nullptr, nullptr);
  return layer;
}

TEST_F(DenoiseLayerTester, initializeDefault) {
  auto layer = constructLayer();

  DenoiseLayerTester::initialize(*layer);

  ASSERT_EQ(
    DenoiseLayerTester::getParameters(*layer),
    std::make_tuple(true, DenoiseLayerTester::ConnectivityType::Way8, 2));
}

TEST_F(DenoiseLayerTester, initializeCustom) {
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_node");
  auto layer = constructLayer(node);
  node->set_parameter(
    rclcpp::Parameter(layer->getFullName("minimal_group_size"), rclcpp::ParameterValue(5)));
  node->set_parameter(
    rclcpp::Parameter(layer->getFullName("group_connectivity_type"), rclcpp::ParameterValue(4)));

  DenoiseLayerTester::initialize(*layer);

  ASSERT_EQ(
    DenoiseLayerTester::getParameters(*layer),
    std::make_tuple(true, DenoiseLayerTester::ConnectivityType::Way4, 5));
}

TEST_F(DenoiseLayerTester, initializeInvalid) {
  auto node = std::make_shared<nav2_util::LifecycleNode>("test_node");
  auto layer = constructLayer(node);
  node->set_parameter(
    rclcpp::Parameter(layer->getFullName("minimal_group_size"), rclcpp::ParameterValue(-1)));
  node->set_parameter(
    rclcpp::Parameter(layer->getFullName("group_connectivity_type"), rclcpp::ParameterValue(3)));

  DenoiseLayerTester::initialize(*layer);

  ASSERT_EQ(
    DenoiseLayerTester::getParameters(*layer),
    std::make_tuple(true, DenoiseLayerTester::ConnectivityType::Way8, 1));
}
