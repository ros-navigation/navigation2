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

#include <string>
#include <vector>
#include <tuple>
#include <stdexcept>
#include <algorithm>

#include "nav2_costmap_2d/denoise_layer.hpp"
#include "image_tests_helper.hpp"

namespace nav2_costmap_2d
{
/**
 * @brief nav2_costmap_2d::DenoiseLayer class wrapper
 *
 * Provides access to DenoiseLayer private methods for testing them in isolation
 */
class DenoiseLayerTester : public ::testing::Test
{
public:
  void removeSinglePixels(
    Image<uint8_t> & image, ConnectivityType connectivity,
    bool no_information_is_obstacle = true)
  {
    denoise_.group_connectivity_type_ = connectivity;
    denoise_.no_information_is_obstacle_ = no_information_is_obstacle;
    denoise_.removeSinglePixels(image);
  }

  void removeGroups(
    Image<uint8_t> & image, ConnectivityType connectivity,
    size_t minimal_group_size, bool no_information_is_obstacle = true)
  {
    denoise_.group_connectivity_type_ = connectivity;
    denoise_.minimal_group_size_ = minimal_group_size;
    denoise_.no_information_is_obstacle_ = no_information_is_obstacle;
    denoise_.removeGroups(image);
  }

  void denoise(
    Image<uint8_t> & image, ConnectivityType connectivity,
    size_t minimal_group_size, bool no_information_is_obstacle = true)
  {
    denoise_.group_connectivity_type_ = connectivity;
    denoise_.minimal_group_size_ = minimal_group_size;
    denoise_.no_information_is_obstacle_ = no_information_is_obstacle;
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
    d.group_connectivity_type_ = connectivity;
    d.minimal_group_size_ = minimal_group_size;
  }

  static std::tuple<bool, ConnectivityType, size_t> getParameters(
    const nav2_costmap_2d::DenoiseLayer & d)
  {
    return std::make_tuple(d.enabled_, d.group_connectivity_type_, d.minimal_group_size_);
  }

protected:
  std::vector<uint8_t> image_buffer_bytes;
  std::vector<uint8_t> image_buffer_bytes2;
  std::vector<uint8_t> image_buffer_bytes3;

private:
  nav2_costmap_2d::DenoiseLayer denoise_;
};

}

using namespace nav2_costmap_2d;

TEST_F(DenoiseLayerTester, removeSinglePixels4way) {
  const auto in = imageFromString<uint8_t>(
    "x.x."
    "x..x"
    ".x.."
    "xx.x", image_buffer_bytes);
  const auto exp = imageFromString<uint8_t>(
    "x..."
    "x..."
    ".x.."
    "xx..", image_buffer_bytes2);
  auto out = clone(in, image_buffer_bytes3);
  removeSinglePixels(out, ConnectivityType::Way4);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removeSinglePixels4wayNoInformationIsEmpty) {
  const std::map<char, uint8_t> legend = {{'.', 0}, {'n', NO_INFORMATION}, {'x', LETHAL_OBSTACLE}};
  const auto in = imageFromString<uint8_t>(
    "x.x."
    "xnnx"
    "nxnn"
    "xx.x", image_buffer_bytes, legend);
  const auto exp = imageFromString<uint8_t>(
    "x..."
    "xnn."
    "nxnn"
    "xx..", image_buffer_bytes2, legend);
  auto out = clone(in, image_buffer_bytes3);
  removeSinglePixels(out, ConnectivityType::Way4, false);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removeSinglePixels8way) {
  const auto in = imageFromString<uint8_t>(
    "x.x."
    "x..x"
    ".x.."
    "xx.x", image_buffer_bytes);
  const auto exp = imageFromString<uint8_t>(
    "x.x."
    "x..x"
    ".x.."
    "xx..", image_buffer_bytes2);

  auto out = clone(in, image_buffer_bytes3);
  removeSinglePixels(out, ConnectivityType::Way8);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removeSinglePixelsFromExtremelySmallImage) {
  {
    const auto in = imageFromString<uint8_t>(
      "x", image_buffer_bytes);
    const auto exp = imageFromString<uint8_t>(
      ".", image_buffer_bytes2);

    auto out = clone(in, image_buffer_bytes3);
    removeSinglePixels(out, ConnectivityType::Way8);

    ASSERT_TRUE(isEqual(out, exp));
  }

  {
    const auto in = imageFromString<uint8_t>(
      "x."
      ".x", image_buffer_bytes);
    const auto exp = imageFromString<uint8_t>(
      "x."
      ".x", image_buffer_bytes2);

    auto out = clone(in, image_buffer_bytes3);
    removeSinglePixels(out, ConnectivityType::Way8);

    ASSERT_TRUE(isEqual(out, exp));
  }

  {
    const auto in = imageFromString<uint8_t>(
      "x."
      ".x", image_buffer_bytes);
    const auto exp = imageFromString<uint8_t>(
      ".."
      "..", image_buffer_bytes2);

    auto out = clone(in, image_buffer_bytes3);
    removeSinglePixels(out, ConnectivityType::Way4);

    ASSERT_TRUE(isEqual(out, exp));
  }
}

TEST_F(DenoiseLayerTester, removeSinglePixelsFromNonBinary) {
  // buffer for 9 pixels with neutral (between FREE_SPACE and INSCRIBED_INFLATED_OBSTACLE) value
  image_buffer_bytes.assign(9, 250);
  Image<uint8_t> in = makeImage<uint8_t>(3, 3, image_buffer_bytes);
  in.row(1)[1] = NO_INFORMATION;
  Image<uint8_t> exp = clone(in, image_buffer_bytes2);
  exp.row(1)[1] = FREE_SPACE;

  auto out = clone(in, image_buffer_bytes3);
  removeSinglePixels(out, ConnectivityType::Way4);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removePixelsGroup4way) {
  const auto in = imageFromString<uint8_t>(
    ".xx..xx"
    "..x.x.."
    "x..x..x"
    "x......"
    "...x.xx"
    "xxx..xx"
    "....xx.", image_buffer_bytes);
  const auto exp = imageFromString<uint8_t>(
    ".xx...."
    "..x...."
    "......."
    "......."
    ".....xx"
    "xxx..xx"
    "....xx.", image_buffer_bytes2);

  auto out = clone(in, image_buffer_bytes3);
  removeGroups(out, ConnectivityType::Way4, 3);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removePixelsGroup4wayNoInformationIsEmpty) {
  const std::map<char, uint8_t> legend = {{'.', 0}, {'n', NO_INFORMATION}, {'x', LETHAL_OBSTACLE}};
  const auto in = imageFromString<uint8_t>(
    ".xxnnxx"
    "..xnx.."
    "x..x..x"
    "x......"
    "nnnxnxx"
    "xxx..xx"
    "....xx.", image_buffer_bytes, legend);
  const auto exp = imageFromString<uint8_t>(
    ".xxnn.."
    "..xn..."
    "......."
    "......."
    "nnn.nxx"
    "xxx..xx"
    "....xx.", image_buffer_bytes2, legend);

  auto out = clone(in, image_buffer_bytes3);
  removeGroups(out, ConnectivityType::Way4, 3, false);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removePixelsGroup8way) {
  const auto in = imageFromString<uint8_t>(
    ".xx..xx"
    "..x.x.."
    "x..x..x"
    "x......"
    "...x.xx"
    "xxx..xx"
    "....xx.", image_buffer_bytes);
  const auto exp = imageFromString<uint8_t>(
    ".xx..xx"
    "..x.x.."
    "...x..."
    "......."
    "...x.xx"
    "xxx..xx"
    "....xx.", image_buffer_bytes2);

  auto out = clone(in, image_buffer_bytes3);
  removeGroups(out, ConnectivityType::Way8, 3);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, removePixelsGroupFromExtremelySmallImage) {
  {
    const auto in = imageFromString<uint8_t>(
      "x", image_buffer_bytes);
    const auto exp = imageFromString<uint8_t>(
      ".", image_buffer_bytes2);

    auto out = clone(in, image_buffer_bytes3);
    removeGroups(out, ConnectivityType::Way8, 3);

    ASSERT_TRUE(isEqual(out, exp));
  }

  {
    const auto in = imageFromString<uint8_t>(
      "x."
      ".x", image_buffer_bytes);
    const auto exp = imageFromString<uint8_t>(
      ".."
      "..", image_buffer_bytes2);

    auto out = clone(in, image_buffer_bytes3);
    removeGroups(out, ConnectivityType::Way8, 3);

    ASSERT_TRUE(isEqual(out, exp));
  }
}

TEST_F(DenoiseLayerTester, removePixelsGroupFromNonBinary) {
  // buffer for 9 pixels with neutral (between FREE_SPACE and INSCRIBED_INFLATED_OBSTACLE) value
  image_buffer_bytes.assign(9, 250);
  Image<uint8_t> in = makeImage<uint8_t>(3, 3, image_buffer_bytes);
  in.row(1)[1] = 255;

  Image<uint8_t> exp = clone(in, image_buffer_bytes2);
  exp.row(1)[1] = 0;

  auto out = clone(in, image_buffer_bytes3);
  removeGroups(out, ConnectivityType::Way4, 2);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, denoiseSingles) {
  const auto in = imageFromString<uint8_t>(
    "xx."
    "..."
    "..x", image_buffer_bytes);
  const auto exp = imageFromString<uint8_t>(
    "xx."
    "..."
    "...", image_buffer_bytes2);

  auto out = clone(in, image_buffer_bytes3);
  denoise(out, ConnectivityType::Way4, 2);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, denoiseGroups) {
  const auto in = imageFromString<uint8_t>(
    "xx."
    "x.x"
    "..x", image_buffer_bytes);
  const auto exp = imageFromString<uint8_t>(
    "xx."
    "x.."
    "...", image_buffer_bytes2);

  auto out = clone(in, image_buffer_bytes3);
  denoise(out, ConnectivityType::Way4, 3);

  ASSERT_TRUE(isEqual(out, exp)) <<
    "input:" << std::endl << in << std::endl <<
    "output:" << std::endl << out << std::endl <<
    "expected:" << std::endl << exp;
}

TEST_F(DenoiseLayerTester, denoiseEmpty) {
  Image<uint8_t> in;

  ASSERT_NO_THROW(denoise(in, ConnectivityType::Way4, 2));
}

TEST_F(DenoiseLayerTester, denoiseNothing) {
  Image<uint8_t> in = makeImage<uint8_t>(1, 1, image_buffer_bytes);

  ASSERT_NO_THROW(denoise(in, ConnectivityType::Way4, 1));
}

TEST_F(DenoiseLayerTester, constructorAndDestructor) {
  ASSERT_NO_THROW(
    // []()
    {
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
  nav2_costmap_2d::Costmap2D costmap(1, 1, 1., 0., 0.);
  costmap.setCost(0, 0, NO_INFORMATION);
  DenoiseLayerTester::configure(layer, ConnectivityType::Way4, 2);

  layer.updateCosts(costmap, 0, 0, 1, 1);

  ASSERT_EQ(costmap.getCost(0), FREE_SPACE);
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
  layer->initialize(layers.get(), "test_layer", tf.get(), node, nullptr);
  return layer;
}

TEST_F(DenoiseLayerTester, initializeDefault) {
  auto layer = constructLayer();

  DenoiseLayerTester::initialize(*layer);

  ASSERT_EQ(
    DenoiseLayerTester::getParameters(*layer),
    std::make_tuple(true, ConnectivityType::Way8, 2));
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
    std::make_tuple(true, ConnectivityType::Way4, 5));
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
    std::make_tuple(true, ConnectivityType::Way8, 1));
}
