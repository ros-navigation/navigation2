// Copyright (c) 2022 Samsung R&D Institute Russia
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
// limitations under the License. Reserved.

#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <tuple>
#include <functional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav2_util/occ_grid_values.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/costmap_filters/binary_filter.hpp"

using namespace std::chrono_literals;

static const char FILTER_NAME[]{"binary_filter"};
static const char INFO_TOPIC[]{"costmap_filter_info"};
static const char MASK_TOPIC[]{"mask"};
static const char BINARY_STATE_TOPIC[]{"binary_state"};

static const double NO_TRANSLATION = 0.0;
static const double TRANSLATION_X = 1.0;
static const double TRANSLATION_Y = 1.0;

static const uint8_t INCORRECT_TYPE = 200;

class InfoPublisher : public rclcpp::Node
{
public:
  InfoPublisher(uint8_t type, const char * mask_topic, double base, double multiplier)
  : Node("costmap_filter_info_pub")
  {
    publisher_ = this->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      INFO_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<nav2_msgs::msg::CostmapFilterInfo> msg =
      std::make_unique<nav2_msgs::msg::CostmapFilterInfo>();
    msg->type = type;
    msg->filter_mask_topic = mask_topic;
    msg->base = static_cast<float>(base);
    msg->multiplier = static_cast<float>(multiplier);

    publisher_->publish(std::move(msg));
  }

  ~InfoPublisher()
  {
    publisher_.reset();
  }

private:
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;
};  // InfoPublisher

class MaskPublisher : public rclcpp::Node
{
public:
  explicit MaskPublisher(const nav_msgs::msg::OccupancyGrid & mask)
  : Node("mask_pub")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      MASK_TOPIC,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    publisher_->publish(mask);
  }

  ~MaskPublisher()
  {
    publisher_.reset();
  }

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};  // MaskPublisher

class BinaryStateSubscriber : public rclcpp::Node
{
public:
  explicit BinaryStateSubscriber(const std::string & binary_state_topic, bool default_state)
  : Node("binary_state_sub"), binary_state_updated_(false)
  {
    subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      binary_state_topic, rclcpp::QoS(10),
      std::bind(&BinaryStateSubscriber::binaryStateCallback, this, std::placeholders::_1));

    // Initialize with default state
    msg_ = std::make_shared<std_msgs::msg::Bool>();
    msg_->data = default_state;
  }

  void binaryStateCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
  {
    msg_ = msg;
    binary_state_updated_ = true;
  }

  std_msgs::msg::Bool::SharedPtr getBinaryState()
  {
    return msg_;
  }

  inline bool binaryStateUpdated()
  {
    return binary_state_updated_;
  }

  inline void resetBinaryStateIndicator()
  {
    binary_state_updated_ = false;
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
  std_msgs::msg::Bool::SharedPtr msg_;
  bool binary_state_updated_;
};  // BinaryStateSubscriber

class TestMask : public nav_msgs::msg::OccupancyGrid
{
public:
  TestMask(
    unsigned int width, unsigned int height, double resolution,
    const std::string & mask_frame)
  : width_(width), height_(height)
  {
    // Fill filter mask info
    header.frame_id = mask_frame;
    info.resolution = resolution;
    info.width = width_;
    info.height = height_;
    info.origin.position.x = 0.0;
    info.origin.position.y = 0.0;
    info.origin.position.z = 0.0;
    info.origin.orientation.x = 0.0;
    info.origin.orientation.y = 0.0;
    info.origin.orientation.z = 0.0;
    info.origin.orientation.w = 1.0;

    // Fill test mask as follows:
    //
    //  mask           (10,11)
    //   *----------------*
    //   |91|92|...|99|100|
    //   |...             |
    //   |...             |
    //   |11|12|13|...| 20|
    //   | 1| 2| 3|...| 10|
    //   |-1| 0| 0|...|  0|
    //   *----------------*
    // (0,0)
    data.resize(width_ * height_, nav2_util::OCC_GRID_UNKNOWN);

    unsigned int mx, my;
    data[0] = nav2_util::OCC_GRID_UNKNOWN;
    for (mx = 1; mx < width_; mx++) {
      data[mx] = nav2_util::OCC_GRID_FREE;
    }
    unsigned int it;
    for (my = 1; my < height_; my++) {
      for (mx = 0; mx < width_; mx++) {
        it = mx + my * width_;
        data[it] = makeData(mx, my);
      }
    }
  }

  inline int8_t makeData(unsigned int mx, unsigned int my)
  {
    return mx + (my - 1) * width_ + 1;
  }

private:
  const unsigned int width_;
  const unsigned int height_;
};  // TestMask

class TestNode : public ::testing::Test
{
public:
  TestNode()
  : default_state_(false) {}

  ~TestNode() {}

protected:
  void createMaps(const std::string & mask_frame);
  void publishMaps(uint8_t type, const char * mask_topic, double base, double multiplier);
  void rePublishInfo(uint8_t type, const char * mask_topic, double base, double multiplier);
  void rePublishMask();
  void setDefaultState(bool default_state);  // NOTE: must be called before createBinaryFilter()
  bool createBinaryFilter(const std::string & global_frame, double flip_threshold);
  void createTFBroadcaster(const std::string & mask_frame, const std::string & global_frame);
  void publishTransform();

  // Test methods
  void testFullMask(
    double base, double multiplier, double flip_threshold, double tr_x, double tr_y);
  void testSimpleMask(
    double base, double multiplier, double flip_threshold, double tr_x, double tr_y);
  void testOutOfMask();
  void testIncorrectTF();
  void testResetFilter();

  void resetMaps();
  void reset();

  std::shared_ptr<nav2_costmap_2d::BinaryFilter> binary_filter_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> master_grid_;

  bool default_state_;

private:
  void waitSome(const std::chrono::nanoseconds & duration);
  std_msgs::msg::Bool::SharedPtr getBinaryState();
  std_msgs::msg::Bool::SharedPtr waitBinaryState();
  bool getSign(
    unsigned int x, unsigned int y, double base, double multiplier, double flip_threshold);
  void verifyBinaryState(bool sign, std_msgs::msg::Bool::SharedPtr state);

  const unsigned int width_ = 10;
  const unsigned int height_ = 11;
  const double resolution_ = 1.0;

  nav2_util::LifecycleNode::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<geometry_msgs::msg::TransformStamped> transform_;

  std::shared_ptr<TestMask> mask_;

  std::shared_ptr<InfoPublisher> info_publisher_;
  std::shared_ptr<MaskPublisher> mask_publisher_;
  std::shared_ptr<BinaryStateSubscriber> binary_state_subscriber_;
};

void TestNode::createMaps(const std::string & mask_frame)
{
  // Make map and mask put as follows:
  //  master_grid     (12,13)
  //    *----------------*
  //    |                |
  //    |  mask  (10,11) |
  //    |   *-------*    |
  //    |   |///////|    |
  //    |   |///////|    |
  //    |   |///////|    |
  //    |   *-------*    |
  //    | (0,0)          |
  //    |                |
  //    *----------------*
  // (-2,-2)

  // Create master_grid_
  master_grid_ = std::make_shared<nav2_costmap_2d::Costmap2D>(
    width_ + 4, height_ + 4, resolution_, -2.0, -2.0, nav2_costmap_2d::FREE_SPACE);

  // Create mask_
  mask_ = std::make_shared<TestMask>(width_, height_, resolution_, mask_frame);
}

void TestNode::publishMaps(
  uint8_t type, const char * mask_topic, double base, double multiplier)
{
  info_publisher_ = std::make_shared<InfoPublisher>(type, mask_topic, base, multiplier);
  mask_publisher_ = std::make_shared<MaskPublisher>(*mask_);
}

void TestNode::rePublishInfo(
  uint8_t type, const char * mask_topic, double base, double multiplier)
{
  info_publisher_.reset();
  info_publisher_ = std::make_shared<InfoPublisher>(type, mask_topic, base, multiplier);
  // Allow both CostmapFilterInfo and filter mask subscribers
  // to receive a new message
  waitSome(100ms);
}

void TestNode::rePublishMask()
{
  mask_publisher_.reset();
  mask_publisher_ = std::make_shared<MaskPublisher>(*mask_);
  // Allow filter mask subscriber to receive a new message
  waitSome(100ms);
}

void TestNode::waitSome(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= rclcpp::Duration(duration)) {
    rclcpp::spin_some(node_->get_node_base_interface());
    rclcpp::spin_some(binary_state_subscriber_);
    std::this_thread::sleep_for(10ms);
  }
}

std_msgs::msg::Bool::SharedPtr TestNode::getBinaryState()
{
  std::this_thread::sleep_for(100ms);
  rclcpp::spin_some(binary_state_subscriber_);
  return binary_state_subscriber_->getBinaryState();
}

std_msgs::msg::Bool::SharedPtr TestNode::waitBinaryState()
{
  const std::chrono::nanoseconds timeout = 500ms;

  rclcpp::Time start_time = node_->now();
  binary_state_subscriber_->resetBinaryStateIndicator();
  while (rclcpp::ok() && node_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (binary_state_subscriber_->binaryStateUpdated()) {
      binary_state_subscriber_->resetBinaryStateIndicator();
      return binary_state_subscriber_->getBinaryState();
    }
    rclcpp::spin_some(binary_state_subscriber_);
    std::this_thread::sleep_for(10ms);
  }
  return nullptr;
}

void TestNode::setDefaultState(bool default_state)
{
  default_state_ = default_state;
}

bool TestNode::createBinaryFilter(const std::string & global_frame, double flip_threshold)
{
  node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  nav2_costmap_2d::LayeredCostmap layers(global_frame, false, false);

  node_->declare_parameter(
    std::string(FILTER_NAME) + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  node_->set_parameter(
    rclcpp::Parameter(std::string(FILTER_NAME) + ".transform_tolerance", 0.5));
  node_->declare_parameter(
    std::string(FILTER_NAME) + ".filter_info_topic", rclcpp::ParameterValue(INFO_TOPIC));
  node_->set_parameter(
    rclcpp::Parameter(std::string(FILTER_NAME) + ".filter_info_topic", INFO_TOPIC));
  node_->declare_parameter(
    std::string(FILTER_NAME) + ".default_state", rclcpp::ParameterValue(default_state_));
  node_->set_parameter(
    rclcpp::Parameter(std::string(FILTER_NAME) + ".default_state", default_state_));
  node_->declare_parameter(
    std::string(FILTER_NAME) + ".binary_state_topic", rclcpp::ParameterValue(BINARY_STATE_TOPIC));
  node_->set_parameter(
    rclcpp::Parameter(std::string(FILTER_NAME) + ".binary_state_topic", BINARY_STATE_TOPIC));
  node_->declare_parameter(
    std::string(FILTER_NAME) + ".flip_threshold", rclcpp::ParameterValue(flip_threshold));
  node_->set_parameter(
    rclcpp::Parameter(std::string(FILTER_NAME) + ".flip_threshold", flip_threshold));

  binary_filter_ = std::make_shared<nav2_costmap_2d::BinaryFilter>();
  binary_filter_->initialize(&layers, FILTER_NAME, tf_buffer_.get(), node_, nullptr);
  binary_filter_->initializeFilter(INFO_TOPIC);

  binary_state_subscriber_ =
    std::make_shared<BinaryStateSubscriber>(BINARY_STATE_TOPIC, default_state_);

  // Wait until mask will be received by BinaryFilter
  const std::chrono::nanoseconds timeout = 500ms;
  rclcpp::Time start_time = node_->now();
  while (!binary_filter_->isActive()) {
    if (node_->now() - start_time > rclcpp::Duration(timeout)) {
      return false;
    }
    rclcpp::spin_some(node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

void TestNode::createTFBroadcaster(const std::string & mask_frame, const std::string & global_frame)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  transform_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
  transform_->header.frame_id = mask_frame;
  transform_->child_frame_id = global_frame;

  transform_->header.stamp = node_->now() + rclcpp::Duration(100ms);
  transform_->transform.translation.x = TRANSLATION_X;
  transform_->transform.translation.y = TRANSLATION_Y;
  transform_->transform.translation.z = 0.0;
  transform_->transform.rotation.x = 0.0;
  transform_->transform.rotation.y = 0.0;
  transform_->transform.rotation.z = 0.0;
  transform_->transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(*transform_);

  // Allow tf_buffer_ to be filled by listener
  waitSome(100ms);
}

void TestNode::publishTransform()
{
  if (tf_broadcaster_) {
    transform_->header.stamp = node_->now() + rclcpp::Duration(100ms);
    tf_broadcaster_->sendTransform(*transform_);
  }
}

bool TestNode::getSign(
  unsigned int x, unsigned int y, double base, double multiplier, double flip_threshold)
{
  const int8_t cost = mask_->makeData(x, y);
  return base + cost * multiplier > flip_threshold;
}

void TestNode::verifyBinaryState(bool sign, std_msgs::msg::Bool::SharedPtr state)
{
  ASSERT_TRUE(state != nullptr);
  if (sign) {
    EXPECT_FALSE(state->data == default_state_);
  } else {
    EXPECT_TRUE(state->data == default_state_);
  }
}

void TestNode::testFullMask(
  double base, double multiplier, double flip_threshold, double tr_x, double tr_y)
{
  const int min_i = 0;
  const int min_j = 0;
  const int max_i = width_ + 4;
  const int max_j = height_ + 4;

  geometry_msgs::msg::Pose2D pose;
  std_msgs::msg::Bool::SharedPtr binary_state;

  unsigned int x, y;
  bool prev_sign = false;
  bool sign;

  // data = 0
  x = 1;
  y = 0;
  pose.x = x - tr_x;
  pose.y = y - tr_y;
  publishTransform();
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  sign = getSign(x, y, base, multiplier, flip_threshold);
  if (sign != prev_sign) {
    // Binary filter just flipped
    binary_state = waitBinaryState();
    prev_sign = sign;
  } else {
    // Binary filter state should not be changed
    binary_state = getBinaryState();
  }
  verifyBinaryState(sign, binary_state);

  // data in range [1..100] (sparsed for testing speed)
  for (y = 1; y < height_; y += 2) {
    for (x = 0; x < width_; x += 2) {
      pose.x = x - tr_x;
      pose.y = y - tr_y;
      publishTransform();
      binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);

      sign = getSign(x, y, base, multiplier, flip_threshold);
      if (prev_sign != sign) {
        // Binary filter just flipped
        binary_state = waitBinaryState();
        prev_sign = sign;
      } else {
        // Binary filter state should not be changed
        binary_state = getBinaryState();
      }
      verifyBinaryState(sign, binary_state);
    }
  }

  // data = -1 (unknown)
  bool prev_state = binary_state->data;
  pose.x = -tr_x;
  pose.y = -tr_y;
  publishTransform();
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  binary_state = getBinaryState();
  ASSERT_TRUE(binary_state != nullptr);
  ASSERT_EQ(binary_state->data, prev_state);  // Binary state won't be updated
}

void TestNode::testSimpleMask(
  double base, double multiplier, double flip_threshold, double tr_x, double tr_y)
{
  const int min_i = 0;
  const int min_j = 0;
  const int max_i = width_ + 4;
  const int max_j = height_ + 4;

  geometry_msgs::msg::Pose2D pose;
  std_msgs::msg::Bool::SharedPtr binary_state;

  unsigned int x, y;
  bool prev_sign = false;
  bool sign;

  // data = 0
  x = 1;
  y = 0;
  pose.x = x - tr_x;
  pose.y = y - tr_y;
  publishTransform();
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  sign = getSign(x, y, base, multiplier, flip_threshold);
  if (sign != prev_sign) {
    // Binary filter just flipped
    binary_state = waitBinaryState();
    prev_sign = sign;
  } else {
    // Binary filter state should not be changed
    binary_state = getBinaryState();
  }
  verifyBinaryState(sign, binary_state);

  // data = <some_middle_value>
  x = width_ / 2 - 1;
  y = height_ / 2 - 1;
  pose.x = x - tr_x;
  pose.y = y - tr_y;
  publishTransform();
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);

  sign = getSign(x, y, base, multiplier, flip_threshold);
  if (prev_sign != sign) {
    // Binary filter just flipped
    binary_state = waitBinaryState();
    prev_sign = sign;
  } else {
    // Binary filter state should not be changed
    binary_state = getBinaryState();
  }
  verifyBinaryState(sign, binary_state);

  // data = 100
  x = width_ - 1;
  y = height_ - 1;
  pose.x = x - tr_x;
  pose.y = y - tr_y;
  publishTransform();
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);

  sign = getSign(x, y, base, multiplier, flip_threshold);
  if (prev_sign != sign) {
    // Binary filter just flipped
    binary_state = waitBinaryState();
    prev_sign = sign;
  } else {
    // Binary filter state should not be changed
    binary_state = getBinaryState();
  }
  verifyBinaryState(sign, binary_state);

  // data = -1 (unknown)
  bool prev_state = binary_state->data;
  pose.x = -tr_x;
  pose.y = -tr_y;
  publishTransform();
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  binary_state = getBinaryState();
  ASSERT_TRUE(binary_state != nullptr);
  ASSERT_EQ(binary_state->data, prev_state);  // Binary state won't be updated
}

void TestNode::testOutOfMask()
{
  // base, multiplier and flip_threshold should have values as below for this test
  const double base = 0.0;
  const double multiplier = 1.0;
  const double flip_threshold = 10.0;

  const int min_i = 0;
  const int min_j = 0;
  const int max_i = width_ + 4;
  const int max_j = height_ + 4;

  geometry_msgs::msg::Pose2D pose;
  std_msgs::msg::Bool::SharedPtr binary_state;

  // data = <some_middle_value>
  pose.x = width_ / 2 - 1;
  pose.y = height_ / 2 - 1;
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  binary_state = waitBinaryState();
  verifyBinaryState(getSign(pose.x, pose.y, base, multiplier, flip_threshold), binary_state);

  // Then go to out of mask bounds and ensure that binary state is set back to default
  pose.x = -2.0;
  pose.y = -2.0;
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  binary_state = getBinaryState();
  ASSERT_TRUE(binary_state != nullptr);
  ASSERT_EQ(binary_state->data, default_state_);

  pose.x = width_ + 1.0;
  pose.y = height_ + 1.0;
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  binary_state = getBinaryState();
  ASSERT_TRUE(binary_state != nullptr);
  ASSERT_EQ(binary_state->data, default_state_);
}

void TestNode::testIncorrectTF()
{
  const int min_i = 0;
  const int min_j = 0;
  const int max_i = width_ + 4;
  const int max_j = height_ + 4;

  geometry_msgs::msg::Pose2D pose;
  std_msgs::msg::Bool::SharedPtr binary_state;

  // data = <some_middle_value>
  pose.x = width_ / 2 - 1;
  pose.y = height_ / 2 - 1;
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  binary_state = waitBinaryState();
  ASSERT_TRUE(binary_state == nullptr);
}

void TestNode::testResetFilter()
{
  // base, multiplier and flip_threshold should have values as below for this test
  const double base = 0.0;
  const double multiplier = 1.0;
  const double flip_threshold = 10.0;

  const int min_i = 0;
  const int min_j = 0;
  const int max_i = width_ + 4;
  const int max_j = height_ + 4;

  geometry_msgs::msg::Pose2D pose;
  std_msgs::msg::Bool::SharedPtr binary_state;

  // Switch-on binary filter
  pose.x = width_ / 2 - 1;
  pose.y = height_ / 2 - 1;
  publishTransform();
  binary_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  binary_state = waitBinaryState();
  verifyBinaryState(getSign(pose.x, pose.y, base, multiplier, flip_threshold), binary_state);

  // Reset binary filter and check its state was resetted to default
  binary_filter_->resetFilter();
  binary_state = waitBinaryState();
  ASSERT_TRUE(binary_state != nullptr);
  ASSERT_EQ(binary_state->data, default_state_);
}

void TestNode::resetMaps()
{
  mask_.reset();
  master_grid_.reset();
}

void TestNode::reset()
{
  resetMaps();
  info_publisher_.reset();
  mask_publisher_.reset();
  binary_state_subscriber_.reset();
  binary_filter_.reset();
  node_.reset();
  tf_listener_.reset();
  tf_broadcaster_.reset();
  tf_buffer_.reset();
}

TEST_F(TestNode, testBinaryState)
{
  // Initialize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::BINARY_FILTER, MASK_TOPIC, 0.0, 1.0);
  ASSERT_TRUE(createBinaryFilter("map", 10.0));

  // Test BinaryFilter
  testSimpleMask(0.0, 1.0, 10.0, NO_TRANSLATION, NO_TRANSLATION);

  // Clean-up
  binary_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testBinaryStateScaled)
{
  // Initialize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::BINARY_FILTER, MASK_TOPIC, 100.0, -1.0);
  ASSERT_TRUE(createBinaryFilter("map", 35.0));

  // Test BinaryFilter
  testFullMask(100.0, -1.0, 35.0, NO_TRANSLATION, NO_TRANSLATION);

  // Clean-up
  binary_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testInvertedBinaryState)
{
  // Initialize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::BINARY_FILTER, MASK_TOPIC, 0.0, 1.0);
  setDefaultState(true);
  ASSERT_TRUE(createBinaryFilter("map", 10.0));

  // Test BinaryFilter
  testSimpleMask(0.0, 1.0, 10.0, NO_TRANSLATION, NO_TRANSLATION);

  // Clean-up
  binary_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testOutOfBounds)
{
  // Initialize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::BINARY_FILTER, MASK_TOPIC, 0.0, 1.0);
  ASSERT_TRUE(createBinaryFilter("map", 10.0));

  // Test BinaryFilter
  testOutOfMask();

  // Clean-up
  binary_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testInfoRePublish)
{
  // Initialize test system
  createMaps("map");
  // Publish Info with incorrect dummy mask topic
  publishMaps(nav2_costmap_2d::BINARY_FILTER, "dummy_topic", 0.0, 1.0);
  ASSERT_FALSE(createBinaryFilter("map", 10.0));

  // Re-publish filter info with correct mask topic
  // and ensure that everything works fine
  rePublishInfo(nav2_costmap_2d::BINARY_FILTER, MASK_TOPIC, 0.0, 1.0);

  // Test BinaryFilter
  testSimpleMask(0.0, 1.0, 10.0, NO_TRANSLATION, NO_TRANSLATION);

  // Clean-up
  binary_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testMaskRePublish)
{
  // Create mask in incorrect frame
  createMaps("dummy");
  publishMaps(nav2_costmap_2d::BINARY_FILTER, MASK_TOPIC, 0.0, 1.0);
  EXPECT_TRUE(createBinaryFilter("map", 10.0));

  // Create mask in correct frame
  resetMaps();
  createMaps("map");
  // Re-publish correct filter mask and ensure that everything works fine
  rePublishMask();

  // Test BinaryFilter
  testSimpleMask(0.0, 1.0, 10.0, NO_TRANSLATION, NO_TRANSLATION);

  // Clean-up
  binary_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testIncorrectFilterType)
{
  // Initialize test system
  createMaps("map");
  publishMaps(INCORRECT_TYPE, MASK_TOPIC, 0.0, 1.0);
  ASSERT_FALSE(createBinaryFilter("map", 10.0));

  // Clean-up
  binary_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testDifferentFrame)
{
  // Initialize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::BINARY_FILTER, MASK_TOPIC, 0.0, 1.0);
  ASSERT_TRUE(createBinaryFilter("odom", 10.0));
  createTFBroadcaster("map", "odom");

  // Test BinaryFilter
  testSimpleMask(0.0, 1.0, 10.0, TRANSLATION_X, TRANSLATION_Y);

  // Clean-up
  binary_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testIncorrectFrame)
{
  // Initialize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::BINARY_FILTER, MASK_TOPIC, 0.0, 1.0);
  ASSERT_TRUE(createBinaryFilter("odom", 10.0));
  // map->odom TF does not exit

  // Test BinaryFilter with incorrect TF chain
  testIncorrectTF();

  // Clean-up
  binary_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testResetState)
{
  // Initialize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::BINARY_FILTER, MASK_TOPIC, 0.0, 1.0);
  ASSERT_TRUE(createBinaryFilter("map", 10.0));

  testResetFilter();

  // Clean-up
  // do not need to resetFilter(): this was already done in testResetFilter()
  reset();
}

int main(int argc, char ** argv)
{
  // Initialize the system
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  // Actual testing
  bool test_result = RUN_ALL_TESTS();

  // Shutdown
  rclcpp::shutdown();

  return test_result;
}
