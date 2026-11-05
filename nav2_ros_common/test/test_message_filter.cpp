// Copyright (c) 2026 Open Navigation LLC
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

#include <memory>

#include "gtest/gtest.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/message_filter.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

TEST(MessageFilterTest, PendingScanCanBeDestroyedAfterManager)
{
  auto node = std::make_shared<nav2::LifecycleNode>("message_filter_cleanup_test");
  auto manager = nav2::create_transform_manager(node);
  std::weak_ptr<nav2::TransformManager> weak_manager = manager;
  std::weak_ptr<tf2_ros::Buffer> weak_buffer = manager->getRawBufferSharedPtr();
  message_filters::SimpleFilter<sensor_msgs::msg::LaserScan> input;
  auto filter = nav2::create_message_filter<sensor_msgs::msg::LaserScan>(
    input, *manager, "odom", 10, node, 30.0);
  std::weak_ptr<nav2::MessageFilter<sensor_msgs::msg::LaserScan>> weak_filter = filter;

  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.frame_id = "missing_sensor";
  scan->header.stamp = node->now();
  std::weak_ptr<sensor_msgs::msg::LaserScan> weak_scan = scan;
  filter->add(scan);
  scan.reset();
  ASSERT_FALSE(weak_scan.expired());

  manager.reset();
  EXPECT_TRUE(weak_manager.expired());
  ASSERT_FALSE(weak_buffer.expired());
  EXPECT_FALSE(weak_buffer.lock()->isUsingDedicatedThread());
  EXPECT_FALSE(weak_scan.expired());

  // The base destructor must still be able to cancel the pending TF request.
  filter.reset();
  EXPECT_TRUE(weak_filter.expired());
  EXPECT_TRUE(weak_scan.expired());
  EXPECT_TRUE(weak_buffer.expired());
}

TEST(MessageFilterTest, BufferSurvivesUntilLastFilterIsDestroyed)
{
  auto node = std::make_shared<nav2::LifecycleNode>("shared_message_filter_cleanup_test");
  auto manager = nav2::create_transform_manager(node);
  std::weak_ptr<tf2_ros::Buffer> weak_buffer = manager->getRawBufferSharedPtr();
  message_filters::SimpleFilter<sensor_msgs::msg::LaserScan> scans;
  message_filters::SimpleFilter<sensor_msgs::msg::PointCloud2> clouds;
  auto scan_filter = nav2::create_message_filter<sensor_msgs::msg::LaserScan>(
    scans, *manager, "odom", 10, node, 30.0);
  auto cloud_filter = nav2::create_message_filter<sensor_msgs::msg::PointCloud2>(
    clouds, *manager, "odom", 10, node, 30.0);

  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id = "missing_sensor";
  cloud->header.stamp = node->now();
  cloud_filter->add(cloud);

  manager.reset();
  scan_filter.reset();
  EXPECT_FALSE(weak_buffer.expired());
  cloud_filter.reset();
  EXPECT_TRUE(weak_buffer.expired());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
