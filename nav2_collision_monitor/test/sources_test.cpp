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
// limitations under the License.

#include <gtest/gtest.h>

#include <math.h>
#include <cmath>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/scan.hpp"
#include "nav2_collision_monitor/pointcloud.hpp"
#include "nav2_collision_monitor/range.hpp"
#include "nav2_collision_monitor/polygon_source.hpp"

using namespace std::chrono_literals;

static constexpr double EPSILON = std::numeric_limits<float>::epsilon();

static const char BASE_FRAME_ID[]{"base_link"};
static const char SOURCE_FRAME_ID[]{"base_source"};
static const char GLOBAL_FRAME_ID[]{"odom"};
static const char SCAN_NAME[]{"LaserScan"};
static const char SCAN_TOPIC[]{"scan"};
static const char POINTCLOUD_NAME[]{"PointCloud"};
static const char POINTCLOUD_TOPIC[]{"pointcloud"};
static const char RANGE_NAME[]{"Range"};
static const char RANGE_TOPIC[]{"range"};
static const char POLYGON_NAME[]{"Polygon"};
static const char POLYGON_TOPIC[]{"polygon"};
static const tf2::Duration TRANSFORM_TOLERANCE{tf2::durationFromSec(0.1)};
static const rclcpp::Duration DATA_TIMEOUT{rclcpp::Duration::from_seconds(5.0)};

class TestNode : public nav2_util::LifecycleNode
{
public:
  TestNode()
  : nav2_util::LifecycleNode("test_node")
  {
  }

  ~TestNode()
  {
    scan_pub_.reset();
    pointcloud_pub_.reset();
    range_pub_.reset();
    polygon_pub_.reset();
  }

  void publishScan(const rclcpp::Time & stamp, const double range)
  {
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      SCAN_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<sensor_msgs::msg::LaserScan> msg =
      std::make_unique<sensor_msgs::msg::LaserScan>();

    msg->header.frame_id = SOURCE_FRAME_ID;
    msg->header.stamp = stamp;

    msg->angle_min = 0.0;
    msg->angle_max = 2 * M_PI;
    msg->angle_increment = M_PI / 2;
    msg->time_increment = 0.0;
    msg->scan_time = 0.0;
    msg->range_min = 0.1;
    msg->range_max = 1.1;
    std::vector<float> ranges(4, range);
    msg->ranges = ranges;

    scan_pub_->publish(std::move(msg));
  }

  void publishPointCloud(const rclcpp::Time & stamp)
  {
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      POINTCLOUD_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<sensor_msgs::msg::PointCloud2> msg =
      std::make_unique<sensor_msgs::msg::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*msg);

    msg->header.frame_id = SOURCE_FRAME_ID;
    msg->header.stamp = stamp;

    modifier.setPointCloud2Fields(
      3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(3);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    // Point 0: (0.5, 0.5, 0.2)
    *iter_x = 0.5;
    *iter_y = 0.5;
    *iter_z = 0.2;
    ++iter_x; ++iter_y; ++iter_z;

    // Point 1: (-0.5, -0.5, 0.3)
    *iter_x = -0.5;
    *iter_y = -0.5;
    *iter_z = 0.3;
    ++iter_x; ++iter_y; ++iter_z;

    // Point 2: (1.0, 1.0, 10.0)
    *iter_x = 1.0;
    *iter_y = 1.0;
    *iter_z = 10.0;

    pointcloud_pub_->publish(std::move(msg));
  }

  void publishPointCloudForMinRange(const rclcpp::Time & stamp)
  {
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      POINTCLOUD_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<sensor_msgs::msg::PointCloud2> msg =
      std::make_unique<sensor_msgs::msg::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*msg);

    msg->header.frame_id = SOURCE_FRAME_ID;
    msg->header.stamp = stamp;

    modifier.setPointCloud2Fields(
      3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(4);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    // Point 0: Very close to origin (0.05, 0.0, 0.15) - range ≈ 0.158,
    // should be filtered
    *iter_x = 0.05;
    *iter_y = 0.0;
    *iter_z = 0.15;
    ++iter_x; ++iter_y; ++iter_z;

    // Point 1: Close to origin (0.15, 0.0, 0.15) - range ≈ 0.212,
    // should pass with min_range=0.16
    *iter_x = 0.15;
    *iter_y = 0.0;
    *iter_z = 0.15;
    ++iter_x; ++iter_y; ++iter_z;

    // Point 2: At min_range threshold (0.2, 0.0, 0.15) - range ≈ 0.25,
    // should pass with min_range=0.16
    *iter_x = 0.2;
    *iter_y = 0.0;
    *iter_z = 0.15;
    ++iter_x; ++iter_y; ++iter_z;

    // Point 3: Beyond min_range (0.5, 0.0, 0.15) - range ≈ 0.522,
    // should pass
    *iter_x = 0.5;
    *iter_y = 0.0;
    *iter_z = 0.15;

    pointcloud_pub_->publish(std::move(msg));
  }

  void publishRange(const rclcpp::Time & stamp, const double range)
  {
    range_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
      RANGE_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<sensor_msgs::msg::Range> msg =
      std::make_unique<sensor_msgs::msg::Range>();

    msg->header.frame_id = SOURCE_FRAME_ID;
    msg->header.stamp = stamp;

    msg->radiation_type = 0;
    msg->field_of_view = M_PI / 10;
    msg->min_range = 0.1;
    msg->max_range = 1.1;
    msg->range = range;

    range_pub_->publish(std::move(msg));
  }

  void publishPolygon(const rclcpp::Time & stamp)
  {
    polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonInstanceStamped>(
      POLYGON_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<geometry_msgs::msg::PolygonInstanceStamped> msg =
      std::make_unique<geometry_msgs::msg::PolygonInstanceStamped>();

    msg->header.frame_id = SOURCE_FRAME_ID;
    msg->header.stamp = stamp;

    geometry_msgs::msg::Point32 point;
    point.x = 1.0;
    point.y = -1.0;
    point.z = 0.0;
    msg->polygon.polygon.points.push_back(point);
    point.x = 1.0;
    point.y = 1.0;
    point.z = 0.0;
    msg->polygon.polygon.points.push_back(point);
    point.x = -1.0;
    point.y = 1.0;
    point.z = 0.0;
    msg->polygon.polygon.points.push_back(point);
    point.x = -1.0;
    point.y = -1.0;
    point.z = 0.0;
    msg->polygon.polygon.points.push_back(point);
    msg->polygon.id = 1u;
    polygon_pub_->publish(std::move(msg));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonInstanceStamped>::SharedPtr polygon_pub_;
};  // TestNode

class ScanWrapper : public nav2_collision_monitor::Scan
{
public:
  ScanWrapper(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & data_timeout,
    const bool base_shift_correction)
  : nav2_collision_monitor::Scan(
      node, source_name, tf_buffer, base_frame_id, global_frame_id,
      transform_tolerance, data_timeout, base_shift_correction)
  {}

  bool dataReceived() const
  {
    return data_ != nullptr;
  }
};  // ScanWrapper

class PointCloudWrapper : public nav2_collision_monitor::PointCloud
{
public:
  PointCloudWrapper(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & data_timeout,
    const bool base_shift_correction)
  : nav2_collision_monitor::PointCloud(
      node, source_name, tf_buffer, base_frame_id, global_frame_id,
      transform_tolerance, data_timeout, base_shift_correction)
  {}

  bool dataReceived() const
  {
    return data_ != nullptr;
  }
};  // PointCloudWrapper

class RangeWrapper : public nav2_collision_monitor::Range
{
public:
  RangeWrapper(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & data_timeout,
    const bool base_shift_correction)
  : nav2_collision_monitor::Range(
      node, source_name, tf_buffer, base_frame_id, global_frame_id,
      transform_tolerance, data_timeout, base_shift_correction)
  {}

  bool dataReceived() const
  {
    return data_ != nullptr;
  }
};  // RangeWrapper

class PolygonWrapper : public nav2_collision_monitor::PolygonSource
{
public:
  PolygonWrapper(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & data_timeout,
    const bool base_shift_correction)
  : nav2_collision_monitor::PolygonSource(
      node, source_name, tf_buffer, base_frame_id, global_frame_id,
      transform_tolerance, data_timeout, base_shift_correction)
  {}

  bool dataReceived() const
  {
    return data_.size() > 0;
  }
};  // PolygonWrapper

class Tester : public ::testing::Test
{
public:
  Tester();
  ~Tester();
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

protected:
  // Data sources creation routine
  void createSources(const bool base_shift_correction = true);

  // Setting TF chains
  void sendTransforms(const rclcpp::Time & stamp);

  // Data sources working routines
  bool waitScan(const std::chrono::nanoseconds & timeout);
  bool waitPointCloud(const std::chrono::nanoseconds & timeout);
  bool waitRange(const std::chrono::nanoseconds & timeout);
  bool waitPolygon(const std::chrono::nanoseconds & timeout);
  void checkScan(const std::vector<nav2_collision_monitor::Point> & data);
  void checkPointCloud(const std::vector<nav2_collision_monitor::Point> & data);
  void checkRange(const std::vector<nav2_collision_monitor::Point> & data);
  void checkPolygon(const std::vector<nav2_collision_monitor::Point> & data);

  std::shared_ptr<TestNode> test_node_;
  std::shared_ptr<ScanWrapper> scan_;
  std::shared_ptr<PointCloudWrapper> pointcloud_;
  std::shared_ptr<RangeWrapper> range_;
  std::shared_ptr<PolygonWrapper> polygon_;
};  // Tester

Tester::Tester()
{
  test_node_ = std::make_shared<TestNode>();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(test_node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Tester::~Tester()
{
  scan_.reset();
  pointcloud_.reset();
  range_.reset();

  test_node_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();
}

void Tester::createSources(const bool base_shift_correction)
{
  // Create Scan object
  test_node_->declare_parameter(
    std::string(SCAN_NAME) + ".topic", rclcpp::ParameterValue(SCAN_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(SCAN_NAME) + ".topic", SCAN_TOPIC));

  scan_ = std::make_shared<ScanWrapper>(
    test_node_, SCAN_NAME, tf_buffer_,
    BASE_FRAME_ID, GLOBAL_FRAME_ID,
    TRANSFORM_TOLERANCE, DATA_TIMEOUT, base_shift_correction);
  scan_->configure();

  // Create PointCloud object
  test_node_->declare_parameter(
    std::string(POINTCLOUD_NAME) + ".topic", rclcpp::ParameterValue(POINTCLOUD_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POINTCLOUD_NAME) + ".topic", POINTCLOUD_TOPIC));
  test_node_->declare_parameter(
    std::string(POINTCLOUD_NAME) + ".min_height", rclcpp::ParameterValue(0.1));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POINTCLOUD_NAME) + ".min_height", 0.1));
  test_node_->declare_parameter(
    std::string(POINTCLOUD_NAME) + ".max_height", rclcpp::ParameterValue(1.0));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POINTCLOUD_NAME) + ".max_height", 1.0));

  pointcloud_ = std::make_shared<PointCloudWrapper>(
    test_node_, POINTCLOUD_NAME, tf_buffer_,
    BASE_FRAME_ID, GLOBAL_FRAME_ID,
    TRANSFORM_TOLERANCE, DATA_TIMEOUT, base_shift_correction);
  pointcloud_->configure();

  // Create Range object
  test_node_->declare_parameter(
    std::string(RANGE_NAME) + ".topic", rclcpp::ParameterValue(RANGE_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(RANGE_NAME) + ".topic", RANGE_TOPIC));

  test_node_->declare_parameter(
    std::string(RANGE_NAME) + ".obstacles_angle", rclcpp::ParameterValue(M_PI / 199));

  range_ = std::make_shared<RangeWrapper>(
    test_node_, RANGE_NAME, tf_buffer_,
    BASE_FRAME_ID, GLOBAL_FRAME_ID,
    TRANSFORM_TOLERANCE, DATA_TIMEOUT, base_shift_correction);
  range_->configure();

  // Create Polygon object
  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".topic", rclcpp::ParameterValue(POLYGON_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POLYGON_NAME) + ".topic", POLYGON_TOPIC));

  test_node_->declare_parameter(
    std::string(POLYGON_NAME) + ".sampling_distance", rclcpp::ParameterValue(0.1));

  polygon_ = std::make_shared<PolygonWrapper>(
    test_node_, POLYGON_NAME, tf_buffer_,
    BASE_FRAME_ID, GLOBAL_FRAME_ID,
    TRANSFORM_TOLERANCE, DATA_TIMEOUT, base_shift_correction);
  polygon_->configure();
}

void Tester::sendTransforms(const rclcpp::Time & stamp)
{
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(test_node_);

  geometry_msgs::msg::TransformStamped transform;

  // base_frame -> source_frame transform
  transform.header.frame_id = BASE_FRAME_ID;
  transform.child_frame_id = SOURCE_FRAME_ID;

  transform.header.stamp = stamp;
  transform.transform.translation.x = 0.1;
  transform.transform.translation.y = 0.1;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf_broadcaster->sendTransform(transform);

  // global_frame -> base_frame transform
  transform.header.frame_id = GLOBAL_FRAME_ID;
  transform.child_frame_id = BASE_FRAME_ID;

  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;

  tf_broadcaster->sendTransform(transform);
}

bool Tester::waitScan(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = test_node_->now();
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (scan_->dataReceived()) {
      return true;
    }
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitPointCloud(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = test_node_->now();
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (pointcloud_->dataReceived()) {
      return true;
    }
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitRange(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = test_node_->now();
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (range_->dataReceived()) {
      return true;
    }
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

bool Tester::waitPolygon(const std::chrono::nanoseconds & timeout)
{
  rclcpp::Time start_time = test_node_->now();
  while (rclcpp::ok() && test_node_->now() - start_time <= rclcpp::Duration(timeout)) {
    if (polygon_->dataReceived()) {
      return true;
    }
    rclcpp::spin_some(test_node_->get_node_base_interface());
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

void Tester::checkScan(const std::vector<nav2_collision_monitor::Point> & data)
{
  ASSERT_EQ(data.size(), 4u);

  // Point 0: (1.0 + 0.1, 0.0 + 0.1)
  EXPECT_NEAR(data[0].x, 1.1, EPSILON);
  EXPECT_NEAR(data[0].y, 0.1, EPSILON);

  // Point 1: (0.0 + 0.1, 1.0 + 0.1)
  EXPECT_NEAR(data[1].x, 0.1, EPSILON);
  EXPECT_NEAR(data[1].y, 1.1, EPSILON);

  // Point 2: (-1.0 + 0.1, 0.0 + 0.1)
  EXPECT_NEAR(data[2].x, -0.9, EPSILON);
  EXPECT_NEAR(data[2].y, 0.1, EPSILON);

  // Point 3: (0.0 + 0.1, -1.0 + 0.1)
  EXPECT_NEAR(data[3].x, 0.1, EPSILON);
  EXPECT_NEAR(data[3].y, -0.9, EPSILON);
}

void Tester::checkPointCloud(const std::vector<nav2_collision_monitor::Point> & data)
{
  ASSERT_EQ(data.size(), 2u);

  // Point 0: (0.5 + 0.1, 0.5 + 0.1)
  EXPECT_NEAR(data[0].x, 0.6, EPSILON);
  EXPECT_NEAR(data[0].y, 0.6, EPSILON);

  // Point 1: (-0.5 + 0.1, -0.5 + 0.1)
  EXPECT_NEAR(data[1].x, -0.4, EPSILON);
  EXPECT_NEAR(data[1].y, -0.4, EPSILON);

  // Point 2 should be out of scope by height
}

void Tester::checkRange(const std::vector<nav2_collision_monitor::Point> & data)
{
  ASSERT_EQ(data.size(), 21u);

  const double angle_increment = M_PI / 199;
  double angle = -M_PI / (10 * 2);
  int i;
  for (i = 0; i < 199 / 10 + 1; i++) {
    ASSERT_NEAR(data[i].x, 1.0 * std::cos(angle) + 0.1, EPSILON);
    ASSERT_NEAR(data[i].y, 1.0 * std::sin(angle) + 0.1, EPSILON);
    angle += angle_increment;
  }
  // Check for the latest FoW/2 point
  angle = M_PI / (10 * 2);
  ASSERT_NEAR(data[i].x, 1.0 * std::cos(angle) + 0.1, EPSILON);
  ASSERT_NEAR(data[i].y, 1.0 * std::sin(angle) + 0.1, EPSILON);
}

void Tester::checkPolygon(const std::vector<nav2_collision_monitor::Point> & data)
{
  ASSERT_EQ(data.size(), 84u);
}

TEST_F(Tester, testGetData)
{
  rclcpp::Time curr_time = test_node_->now();

  createSources();

  sendTransforms(curr_time);

  // Publish data for sources
  test_node_->publishScan(curr_time, 1.0);
  test_node_->publishPointCloud(curr_time);
  test_node_->publishRange(curr_time, 1.0);
  test_node_->publishPolygon(curr_time);

  // Wait until all sources will receive the data
  ASSERT_TRUE(waitScan(500ms));
  ASSERT_TRUE(waitPointCloud(500ms));
  ASSERT_TRUE(waitRange(500ms));
  ASSERT_TRUE(waitPolygon(500ms));

  // Check Scan data
  std::vector<nav2_collision_monitor::Point> data;
  scan_->getData(curr_time, data);
  checkScan(data);

  // Check Pointcloud data
  data.clear();
  pointcloud_->getData(curr_time, data);
  checkPointCloud(data);

  // Check Range data
  data.clear();
  range_->getData(curr_time, data);
  checkRange(data);

  // Check Polygon data
  data.clear();
  polygon_->getData(curr_time, data);
  checkPolygon(data);
}

TEST_F(Tester, testGetOutdatedData)
{
  rclcpp::Time curr_time = test_node_->now();

  createSources();

  sendTransforms(curr_time);

  // Publish outdated data for sources
  test_node_->publishScan(curr_time - DATA_TIMEOUT - 1s, 1.0);
  test_node_->publishPointCloud(curr_time - DATA_TIMEOUT - 1s);
  test_node_->publishRange(curr_time - DATA_TIMEOUT - 1s, 1.0);
  test_node_->publishPolygon(curr_time - DATA_TIMEOUT - 1s);

  // Wait until all sources will receive the data
  ASSERT_TRUE(waitScan(500ms));
  ASSERT_TRUE(waitPointCloud(500ms));
  ASSERT_TRUE(waitRange(500ms));
  ASSERT_TRUE(waitPolygon(500ms));

  // Scan data should be empty
  std::vector<nav2_collision_monitor::Point> data;
  scan_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Pointcloud data should be empty
  pointcloud_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Range data should be empty
  range_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Polygon data should be empty
  polygon_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);
}

TEST_F(Tester, testIncorrectFrameData)
{
  rclcpp::Time curr_time = test_node_->now();

  createSources();

  // Send incorrect transform
  sendTransforms(curr_time - 1s);

  // Publish data for sources
  test_node_->publishScan(curr_time, 1.0);
  test_node_->publishPointCloud(curr_time);
  test_node_->publishRange(curr_time, 1.0);
  test_node_->publishPolygon(curr_time);

  // Wait until all sources will receive the data
  ASSERT_TRUE(waitScan(500ms));
  ASSERT_TRUE(waitPointCloud(500ms));
  ASSERT_TRUE(waitRange(500ms));
  ASSERT_TRUE(waitPolygon(500ms));

  // Scan data should be empty
  std::vector<nav2_collision_monitor::Point> data;
  scan_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Pointcloud data should be empty
  pointcloud_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Range data should be empty
  range_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Polygon data should be empty
  polygon_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);
}

TEST_F(Tester, testIncorrectData)
{
  rclcpp::Time curr_time = test_node_->now();

  createSources();

  sendTransforms(curr_time);

  // Publish data for sources
  test_node_->publishScan(curr_time, 2.0);
  test_node_->publishPointCloud(curr_time);
  test_node_->publishRange(curr_time, 2.0);

  // Wait until all sources will receive the data
  ASSERT_TRUE(waitScan(500ms));
  ASSERT_TRUE(waitRange(500ms));

  // Scan data should be empty
  std::vector<nav2_collision_monitor::Point> data;
  scan_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);

  // Range data should be empty
  range_->getData(curr_time, data);
  ASSERT_EQ(data.size(), 0u);
}

TEST_F(Tester, testIgnoreTimeShift)
{
  rclcpp::Time curr_time = test_node_->now();

  createSources(false);

  // Send incorrect transform
  sendTransforms(curr_time - 1s);

  // Publish data for sources
  test_node_->publishScan(curr_time, 1.0);
  test_node_->publishPointCloud(curr_time);
  test_node_->publishRange(curr_time, 1.0);
  test_node_->publishPolygon(curr_time);

  // Wait until all sources will receive the data
  ASSERT_TRUE(waitScan(500ms));
  ASSERT_TRUE(waitPointCloud(500ms));
  ASSERT_TRUE(waitRange(500ms));
  ASSERT_TRUE(waitPolygon(500ms));

  // Scan data should be consistent
  std::vector<nav2_collision_monitor::Point> data;
  scan_->getData(curr_time, data);
  checkScan(data);

  // Pointcloud data should be consistent
  data.clear();
  pointcloud_->getData(curr_time, data);
  checkPointCloud(data);

  // Range data should be consistent
  data.clear();
  range_->getData(curr_time, data);
  checkRange(data);

  // Polygon data should be consistent
  data.clear();
  polygon_->getData(curr_time, data);
  checkPolygon(data);
}

TEST_F(Tester, testPointCloudMinRange)
{
  rclcpp::Time curr_time = test_node_->now();

  // Create PointCloud object with min_range = 0.2
  test_node_->declare_parameter(
    std::string(POINTCLOUD_NAME) + ".topic", rclcpp::ParameterValue(POINTCLOUD_TOPIC));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POINTCLOUD_NAME) + ".topic", POINTCLOUD_TOPIC));
  test_node_->declare_parameter(
    std::string(POINTCLOUD_NAME) + ".min_height", rclcpp::ParameterValue(0.1));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POINTCLOUD_NAME) + ".min_height", 0.1));
  test_node_->declare_parameter(
    std::string(POINTCLOUD_NAME) + ".max_height", rclcpp::ParameterValue(1.0));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POINTCLOUD_NAME) + ".max_height", 1.0));
  test_node_->declare_parameter(
    std::string(POINTCLOUD_NAME) + ".min_range", rclcpp::ParameterValue(0.16));
  test_node_->set_parameter(
    rclcpp::Parameter(std::string(POINTCLOUD_NAME) + ".min_range", 0.16));

  pointcloud_ = std::make_shared<PointCloudWrapper>(
    test_node_, POINTCLOUD_NAME, tf_buffer_,
    BASE_FRAME_ID, GLOBAL_FRAME_ID,
    TRANSFORM_TOLERANCE, DATA_TIMEOUT, true);
  pointcloud_->configure();

  sendTransforms(curr_time);

  // Publish pointcloud data with points at various ranges
  test_node_->publishPointCloudForMinRange(curr_time);

  // Wait until pointcloud receives the data
  ASSERT_TRUE(waitPointCloud(500ms));

  // Check PointCloud data - should only contain points with range >= min_range (0.16)
  std::vector<nav2_collision_monitor::Point> data;
  pointcloud_->getData(curr_time, data);

  // Should have 3 points: first point (range ≈ 0.158) filtered out, remaining 3 pass
  ASSERT_EQ(data.size(), 3u);

  // Point 1: (0.15 + 0.1, 0.0 + 0.1) = (0.25, 0.1) - range ≈ 0.212, passes
  EXPECT_NEAR(data[0].x, 0.25, EPSILON);
  EXPECT_NEAR(data[0].y, 0.1, EPSILON);

  // Point 2: (0.2 + 0.1, 0.0 + 0.1) = (0.3, 0.1) - range ≈ 0.25, passes
  EXPECT_NEAR(data[1].x, 0.3, EPSILON);
  EXPECT_NEAR(data[1].y, 0.1, EPSILON);

  // Point 3: (0.5 + 0.1, 0.0 + 0.1) = (0.6, 0.1) - range ≈ 0.522, passes
  EXPECT_NEAR(data[2].x, 0.6, EPSILON);
  EXPECT_NEAR(data[2].y, 0.1, EPSILON);
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
