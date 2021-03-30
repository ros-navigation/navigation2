// Copyright (c) 2020 Samsung Research Russia
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
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/costmap_filters/speed_filter.hpp"

using namespace std::chrono_literals;

static const std::string FILTER_NAME = "speed_filter";
static const std::string INFO_TOPIC = "costmap_filter_info";
static const std::string MASK_TOPIC = "mask";
static const std::string SPEED_LIMIT_TOPIC = "speed_limit";

static const double NO_TRANSLATION = 0.0;
static const double TRANSLATION_X = 1.0;
static const double TRANSLATION_Y = 1.0;

static const uint8_t INCORRECT_TYPE = 200;

static constexpr double EPSILON = 1e-5;

class InfoPublisher : public rclcpp::Node
{
public:
  InfoPublisher(uint8_t type, double base, double multiplier)
  : Node("costmap_filter_info_pub")
  {
    publisher_ = this->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      INFO_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<nav2_msgs::msg::CostmapFilterInfo> msg =
      std::make_unique<nav2_msgs::msg::CostmapFilterInfo>();
    msg->type = type;
    msg->filter_mask_topic = MASK_TOPIC;
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
  MaskPublisher(const nav_msgs::msg::OccupancyGrid & mask)
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

class SpeedLimitSubscriber : public rclcpp::Node
{
public:
  SpeedLimitSubscriber(const std::string & speed_limit_topic)
  : Node("speed_limit_sub"), speed_limit_updated_(false)
  {
    subscriber_ = this->create_subscription<nav2_msgs::msg::SpeedLimit>(
      speed_limit_topic, rclcpp::QoS(10),
      std::bind(&SpeedLimitSubscriber::speedLimitCallback, this, std::placeholders::_1));
  }

  void speedLimitCallback(
    const nav2_msgs::msg::SpeedLimit::SharedPtr msg)
  {
    msg_ = msg;
    speed_limit_updated_ = true;
  }

  nav2_msgs::msg::SpeedLimit::SharedPtr getSpeedLimit()
  {
    return msg_;
  }

  inline bool speedLimitUpdated()
  {
    return speed_limit_updated_;
  }

  inline void resetSpeedLimitIndicator()
  {
    speed_limit_updated_ = false;
  }

private:
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr subscriber_;
  nav2_msgs::msg::SpeedLimit::SharedPtr msg_;
  bool speed_limit_updated_;
};  // SpeedLimitSubscriber

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
    data[0] = -1;
    for (mx = 1; mx < width_; mx++) {
      data[mx] = 0;
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
  TestNode() {}

  ~TestNode() {}

protected:
  void createMaps(const std::string & mask_frame);
  void publishMaps(uint8_t type, double base, double multiplier);
  void rePublishInfo(uint8_t type, double base, double multiplier);
  void rePublishMask();
  bool createSpeedFilter(const std::string & global_frame);
  void createTFBroadcaster(const std::string & mask_frame, const std::string & global_frame);
  void publishTransform();

  // Test methods
  void testFullMask(
    uint8_t type, double base, double multiplier,
    double tr_x, double tr_y);
  void testSimpleMask(
    uint8_t type, double base, double multiplier,
    double tr_x, double tr_y);
  void testOutOfMask(uint8_t type, double base, double multiplier);
  void testIncorrectLimits(uint8_t type, double base, double multiplier);

  void reset();

  std::shared_ptr<nav2_costmap_2d::SpeedFilter> speed_filter_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> master_grid_;

private:
  void waitSome(const std::chrono::nanoseconds & duration);
  void verifySpeedLimit(
    uint8_t type, double base, double multiplier,
    unsigned int x, unsigned int y,
    nav2_msgs::msg::SpeedLimit::SharedPtr speed_limit);
  nav2_msgs::msg::SpeedLimit::SharedPtr getSpeedLimit();
  nav2_msgs::msg::SpeedLimit::SharedPtr waitSpeedLimit();

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
  std::shared_ptr<SpeedLimitSubscriber> speed_limit_subscriber_;
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

void TestNode::publishMaps(uint8_t type, double base, double multiplier)
{
  info_publisher_ = std::make_shared<InfoPublisher>(type, base, multiplier);
  mask_publisher_ = std::make_shared<MaskPublisher>(*mask_);
}

void TestNode::rePublishInfo(uint8_t type, double base, double multiplier)
{
  info_publisher_.reset();
  info_publisher_ = std::make_shared<InfoPublisher>(type, base, multiplier);
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

nav2_msgs::msg::SpeedLimit::SharedPtr TestNode::getSpeedLimit()
{
  std::this_thread::sleep_for(100ms);
  rclcpp::spin_some(speed_limit_subscriber_);
  return speed_limit_subscriber_->getSpeedLimit();
}

nav2_msgs::msg::SpeedLimit::SharedPtr TestNode::waitSpeedLimit()
{
  const std::chrono::nanoseconds timeout = 500ms;

  rclcpp::Time start_time = node_->now();
  speed_limit_subscriber_->resetSpeedLimitIndicator();
  while (rclcpp::ok() && node_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (speed_limit_subscriber_->speedLimitUpdated()) {
      speed_limit_subscriber_->resetSpeedLimitIndicator();
      return speed_limit_subscriber_->getSpeedLimit();
    }
    rclcpp::spin_some(speed_limit_subscriber_);
    std::this_thread::sleep_for(10ms);
  }
  return nullptr;
}

void TestNode::waitSome(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  while (rclcpp::ok() && node_->now() - start_time <= rclcpp::Duration(duration)) {
    rclcpp::spin_some(node_->get_node_base_interface());
    rclcpp::spin_some(speed_limit_subscriber_);
    std::this_thread::sleep_for(10ms);
  }
}

bool TestNode::createSpeedFilter(const std::string & global_frame)
{
  node_ = std::make_shared<nav2_util::LifecycleNode>("test_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  nav2_costmap_2d::LayeredCostmap layers(global_frame, false, false);

  node_->declare_parameter(
    FILTER_NAME + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  node_->set_parameter(
    rclcpp::Parameter(FILTER_NAME + ".transform_tolerance", 0.5));
  node_->declare_parameter(
    FILTER_NAME + ".filter_info_topic", rclcpp::ParameterValue(INFO_TOPIC));
  node_->set_parameter(
    rclcpp::Parameter(FILTER_NAME + ".filter_info_topic", INFO_TOPIC));
  node_->declare_parameter(
    FILTER_NAME + ".speed_limit_topic", rclcpp::ParameterValue(SPEED_LIMIT_TOPIC));
  node_->set_parameter(
    rclcpp::Parameter(FILTER_NAME + ".speed_limit_topic", SPEED_LIMIT_TOPIC));

  speed_filter_ = std::make_shared<nav2_costmap_2d::SpeedFilter>();
  speed_filter_->initialize(&layers, FILTER_NAME, tf_buffer_.get(), node_, nullptr, nullptr);
  speed_filter_->initializeFilter(INFO_TOPIC);

  speed_limit_subscriber_ = std::make_shared<SpeedLimitSubscriber>(SPEED_LIMIT_TOPIC);

  // Wait until mask will be received by SpeedFilter
  const std::chrono::nanoseconds timeout = 500ms;
  rclcpp::Time start_time = node_->now();
  while (!speed_filter_->isActive()) {
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

void TestNode::verifySpeedLimit(
  uint8_t type, double base, double multiplier,
  unsigned int x, unsigned int y,
  nav2_msgs::msg::SpeedLimit::SharedPtr speed_limit)
{
  int8_t cost = mask_->makeData(x, y);
  // expected_limit is being calculated by using float32 base and multiplier
  double expected_limit = cost * multiplier + base;
  if (type == nav2_costmap_2d::SPEED_FILTER_PERCENT) {
    if (expected_limit < 0.0 || expected_limit > 100.0) {
      expected_limit = nav2_costmap_2d::NO_SPEED_LIMIT;
    }
    EXPECT_TRUE(speed_limit->percentage);
    EXPECT_TRUE(speed_limit->speed_limit >= 0.0);
    EXPECT_TRUE(speed_limit->speed_limit <= 100.0);
    EXPECT_NEAR(speed_limit->speed_limit, expected_limit, EPSILON);
  } else if (type == nav2_costmap_2d::SPEED_FILTER_ABSOLUTE) {
    if (expected_limit < 0.0) {
      expected_limit = nav2_costmap_2d::NO_SPEED_LIMIT;
    }
    EXPECT_FALSE(speed_limit->percentage);
    EXPECT_TRUE(speed_limit->speed_limit >= 0.0);
    EXPECT_NEAR(speed_limit->speed_limit, expected_limit, EPSILON);
  } else {
    FAIL() << "The type of costmap filter is unknown";
  }
}

void TestNode::testFullMask(
  uint8_t type, double base, double multiplier,
  double tr_x, double tr_y)
{
  const int min_i = 0;
  const int min_j = 0;
  const int max_i = width_ + 4;
  const int max_j = height_ + 4;

  geometry_msgs::msg::Pose2D pose;
  nav2_msgs::msg::SpeedLimit::SharedPtr speed_limit;

  // data = 0
  pose.x = 1 - tr_x;
  pose.y = -tr_y;
  publishTransform();
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  speed_limit = getSpeedLimit();
  ASSERT_TRUE(speed_limit == nullptr);

  // data in range [1..100]
  unsigned int x, y;
  for (y = 1; y < height_; y++) {
    for (x = 0; x < width_; x++) {
      pose.x = x - tr_x;
      pose.y = y - tr_y;
      publishTransform();
      speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
      speed_limit = waitSpeedLimit();
      ASSERT_TRUE(speed_limit != nullptr);
      verifySpeedLimit(type, base, multiplier, x, y, speed_limit);
    }
  }

  // data = 0
  pose.x = 1 - tr_x;
  pose.y = -tr_y;
  publishTransform();
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  speed_limit = waitSpeedLimit();
  ASSERT_TRUE(speed_limit != nullptr);
  EXPECT_EQ(speed_limit->speed_limit, nav2_costmap_2d::NO_SPEED_LIMIT);

  // data = -1
  pose.x = -tr_x;
  pose.y = -tr_y;
  publishTransform();
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  speed_limit = getSpeedLimit();
  ASSERT_TRUE(speed_limit != nullptr);
  EXPECT_EQ(speed_limit->speed_limit, nav2_costmap_2d::NO_SPEED_LIMIT);
}

void TestNode::testSimpleMask(
  uint8_t type, double base, double multiplier,
  double tr_x, double tr_y)
{
  const int min_i = 0;
  const int min_j = 0;
  const int max_i = width_ + 4;
  const int max_j = height_ + 4;

  geometry_msgs::msg::Pose2D pose;
  nav2_msgs::msg::SpeedLimit::SharedPtr speed_limit;

  // data = 0
  pose.x = 1 - tr_x;
  pose.y = -tr_y;
  publishTransform();
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  speed_limit = getSpeedLimit();
  ASSERT_TRUE(speed_limit == nullptr);

  // data = <some_middle_value>
  unsigned int x = width_ / 2 - 1;
  unsigned int y = height_ / 2 - 1;
  pose.x = x - tr_x;
  pose.y = y - tr_y;
  publishTransform();
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  speed_limit = waitSpeedLimit();
  ASSERT_TRUE(speed_limit != nullptr);
  verifySpeedLimit(type, base, multiplier, x, y, speed_limit);

  // data = 100
  x = width_ - 1;
  y = height_ - 1;
  pose.x = x - tr_x;
  pose.y = y - tr_y;
  publishTransform();
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  speed_limit = waitSpeedLimit();
  ASSERT_TRUE(speed_limit != nullptr);
  verifySpeedLimit(type, base, multiplier, x, y, speed_limit);

  // data = 0
  pose.x = 1 - tr_x;
  pose.y = -tr_y;
  publishTransform();
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  speed_limit = waitSpeedLimit();
  ASSERT_TRUE(speed_limit != nullptr);
  EXPECT_EQ(speed_limit->speed_limit, nav2_costmap_2d::NO_SPEED_LIMIT);

  // data = -1
  pose.x = -tr_x;
  pose.y = -tr_y;
  publishTransform();
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  speed_limit = getSpeedLimit();
  ASSERT_TRUE(speed_limit != nullptr);
  EXPECT_EQ(speed_limit->speed_limit, nav2_costmap_2d::NO_SPEED_LIMIT);
}

void TestNode::testOutOfMask(uint8_t type, double base, double multiplier)
{
  const int min_i = 0;
  const int min_j = 0;
  const int max_i = width_ + 4;
  const int max_j = height_ + 4;

  geometry_msgs::msg::Pose2D pose;
  nav2_msgs::msg::SpeedLimit::SharedPtr old_speed_limit, speed_limit;

  // data = <some_middle_value>
  pose.x = width_ / 2 - 1;
  pose.y = height_ / 2 - 1;
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  old_speed_limit = waitSpeedLimit();
  ASSERT_TRUE(old_speed_limit != nullptr);
  verifySpeedLimit(type, base, multiplier, pose.x, pose.y, old_speed_limit);

  // Then go to out of mask bounds and ensure that speed limit was not updated
  pose.x = -2.0;
  pose.y = -2.0;
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  speed_limit = getSpeedLimit();
  ASSERT_TRUE(speed_limit == old_speed_limit);

  pose.x = width_ + 1.0;
  pose.y = height_ + 1.0;
  speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
  speed_limit = getSpeedLimit();
  ASSERT_TRUE(speed_limit == old_speed_limit);
}

void TestNode::testIncorrectLimits(uint8_t type, double base, double multiplier)
{
  const int min_i = 0;
  const int min_j = 0;
  const int max_i = width_ + 4;
  const int max_j = height_ + 4;

  geometry_msgs::msg::Pose2D pose;
  nav2_msgs::msg::SpeedLimit::SharedPtr speed_limit;

  std::vector<std::tuple<unsigned int, unsigned int>> points;

  // Some middle point corresponding to correct speed limit value
  points.push_back(std::make_tuple(width_ / 2 - 1, height_ / 2 - 1));
  // (0, 1) point corresponding to incorrect limit value: data = 1, value < 0
  points.push_back(std::make_tuple(0, 1));
  // Some middle point corresponding to correct speed limit value
  points.push_back(std::make_tuple(width_ / 2 - 1, height_ / 2 - 1));
  // (width_ - 1, height_ - 1) point corresponding to incorrect limit value:
  // data = 100, value > 100
  points.push_back(std::make_tuple(width_ - 1, height_ - 1));

  for (auto it = points.begin(); it != points.end(); ++it) {
    pose.x = static_cast<double>(std::get<0>(*it));
    pose.y = static_cast<double>(std::get<1>(*it));
    speed_filter_->process(*master_grid_, min_i, min_j, max_i, max_j, pose);
    speed_limit = waitSpeedLimit();
    ASSERT_TRUE(speed_limit != nullptr);
    verifySpeedLimit(type, base, multiplier, pose.x, pose.y, speed_limit);
  }
}

void TestNode::reset()
{
  mask_.reset();
  master_grid_.reset();
  info_publisher_.reset();
  mask_publisher_.reset();
  speed_limit_subscriber_.reset();
  speed_filter_.reset();
  node_.reset();
  tf_listener_.reset();
  tf_broadcaster_.reset();
  tf_buffer_.reset();
}

TEST_F(TestNode, testPercentSpeedLimit)
{
  // Initilize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::SPEED_FILTER_PERCENT, 0.0, 1.0);
  EXPECT_TRUE(createSpeedFilter("map"));

  // Test SpeedFilter
  testFullMask(nav2_costmap_2d::SPEED_FILTER_PERCENT, 0.0, 1.0, NO_TRANSLATION, NO_TRANSLATION);

  // Clean-up
  speed_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testIncorrectPercentSpeedLimit)
{
  // Initilize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::SPEED_FILTER_PERCENT, -50.0, 2.0);
  EXPECT_TRUE(createSpeedFilter("map"));

  // Test SpeedFilter
  testIncorrectLimits(nav2_costmap_2d::SPEED_FILTER_PERCENT, -50.0, 2.0);

  // Clean-up
  speed_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testAbsoluteSpeedLimit)
{
  // Initilize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::SPEED_FILTER_ABSOLUTE, 1.23, 4.5);
  EXPECT_TRUE(createSpeedFilter("map"));

  // Test SpeedFilter
  testFullMask(nav2_costmap_2d::SPEED_FILTER_ABSOLUTE, 1.23, 4.5, NO_TRANSLATION, NO_TRANSLATION);

  // Clean-up
  speed_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testIncorrectAbsoluteSpeedLimit)
{
  // Initilize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::SPEED_FILTER_ABSOLUTE, -50.0, 2.0);
  EXPECT_TRUE(createSpeedFilter("map"));

  // Test SpeedFilter
  testIncorrectLimits(nav2_costmap_2d::SPEED_FILTER_ABSOLUTE, -50.0, 2.0);

  // Clean-up
  speed_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testOutOfBounds)
{
  // Initilize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::SPEED_FILTER_PERCENT, 0.0, 1.0);
  EXPECT_TRUE(createSpeedFilter("map"));

  // Test SpeedFilter
  testOutOfMask(nav2_costmap_2d::SPEED_FILTER_PERCENT, 0.0, 1.0);

  // Clean-up
  speed_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testInfoRePublish)
{
  // Initilize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::SPEED_FILTER_ABSOLUTE, 1.23, 4.5);
  EXPECT_TRUE(createSpeedFilter("map"));

  // Re-publish filter info (with incorrect base and multiplier)
  // and test that everything is working after
  rePublishInfo(nav2_costmap_2d::SPEED_FILTER_PERCENT, 0.1, 0.2);

  // Test SpeedFilter
  testSimpleMask(
    nav2_costmap_2d::SPEED_FILTER_PERCENT, 0.1, 0.2, NO_TRANSLATION, NO_TRANSLATION);

  // Clean-up
  speed_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testMaskRePublish)
{
  // Initilize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::SPEED_FILTER_ABSOLUTE, 1.23, 4.5);
  EXPECT_TRUE(createSpeedFilter("map"));

  // Re-publish filter mask and test that everything is working after
  rePublishMask();

  // Test SpeedFilter
  testSimpleMask(
    nav2_costmap_2d::SPEED_FILTER_ABSOLUTE, 1.23, 4.5, NO_TRANSLATION, NO_TRANSLATION);

  // Clean-up
  speed_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testIncorrectFilterType)
{
  // Initilize test system
  createMaps("map");
  publishMaps(INCORRECT_TYPE, 1.23, 4.5);
  EXPECT_FALSE(createSpeedFilter("map"));

  // Clean-up
  speed_filter_->resetFilter();
  reset();
}

TEST_F(TestNode, testDifferentFrame)
{
  // Initilize test system
  createMaps("map");
  publishMaps(nav2_costmap_2d::SPEED_FILTER_PERCENT, 0.0, 1.0);
  EXPECT_TRUE(createSpeedFilter("odom"));
  createTFBroadcaster("map", "odom");

  // Test SpeedFilter
  testFullMask(nav2_costmap_2d::SPEED_FILTER_PERCENT, 0.0, 1.0, TRANSLATION_X, TRANSLATION_Y);

  // Clean-up
  speed_filter_->resetFilter();
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
