// Copyright (c) 2025 Fabian Koenig
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

#include <random>
#include "gtest/gtest.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ObservationBufferFixture : public ::testing::Test
{
protected:
  ObservationBufferFixture()
  {
    auto clock_handle = node_->get_clock()->get_clock_handle();
    auto ret = rcl_enable_ros_time_override(clock_handle);
    ret = rcl_set_ros_time_override(clock_handle, 0);
    (void) ret;
    geometry_msgs::msg::TransformStamped tf_msg{};
    tf_msg.child_frame_id = sensor_frame_;
    tf_msg.header.frame_id = global_frame_;
    tf_msg.transform = tf2::toMsg(tf2::Transform::getIdentity());
    tf_buffer_.setTransform(tf_msg, "", true);
  }

  ~ObservationBufferFixture()
  {
    node_->shutdown();
  }

  void setClock(const rclcpp::Time & time)
  {
    auto clock_handle = node_->get_clock()->get_clock_handle();
    auto ret = rcl_set_ros_time_override(clock_handle, time.nanoseconds());
    (void) ret;
  }

  static sensor_msgs::msg::PointCloud2 generatePointCloud(
    double x, double y, double z,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    *iter_x = x;
    *iter_y = y;
    *iter_z = z;

    cloud.header.frame_id = sensor_frame_;
    cloud.header.stamp = stamp;

    return cloud;
  }

  static sensor_msgs::msg::PointCloud2 generateRandomPointCloud(
    size_t num_points,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    std::random_device                  rand_dev{};
    std::mt19937                        generator{rand_dev()};
    std::uniform_int_distribution<int> distr{0, 10000};
    for (auto i = 0u; i < num_points; ++i) {
      *iter_x = distr(generator) / 100.0;
      *iter_y = distr(generator) / 100.0;
      *iter_z = distr(generator) / 100.0;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud.header.frame_id = sensor_frame_;
    cloud.header.stamp = stamp;

    return cloud;
  }

  static constexpr const char * global_frame_{"map"};
  static constexpr const char * sensor_frame_{"sensor"};

  nav2::LifecycleNode::SharedPtr node_{std::make_shared<nav2::LifecycleNode>(
      "observation_buffer_test_node")};
  tf2_ros::Buffer tf_buffer_{node_->get_clock()};
  nav2_costmap_2d::ObservationBuffer buffer_{node_, "/observations", 10.0, 0.1, 0.0,
    std::numeric_limits<double>::max(), 10.0, 1.0, 5.0, 2.0, tf_buffer_, global_frame_,
    sensor_frame_, tf2::Duration{0}};
};

TEST_F(ObservationBufferFixture, isCurrent)
{
   setClock(rclcpp::Time{0, 100000});
   buffer_.bufferCloud(generatePointCloud(0.5, 0.5, 0.5, rclcpp::Time{0, 0}));
   // Expect time between clouds is 0.1s, time of the cloud is 0.0s and current time is 0.0001s
   // so the buffer is up to date
   EXPECT_TRUE(buffer_.isCurrent());
   setClock(rclcpp::Time{1, 0});
   // Now the current time is 1.0s and the buffer is outdated
   EXPECT_FALSE(buffer_.isCurrent());
}

TEST_F(ObservationBufferFixture, purgeStaleObservations)
{
   setClock(rclcpp::Time{1, 0});
   buffer_.bufferCloud(generatePointCloud(0.5, 0.5, 0.5, rclcpp::Time{1, 0}));
   setClock(rclcpp::Time{5, 0});
   buffer_.bufferCloud(generatePointCloud(0.5, 0.5, 0.5, rclcpp::Time{5, 0}));
   setClock(rclcpp::Time{10, 0});
   buffer_.bufferCloud(generatePointCloud(0.5, 0.5, 0.5, rclcpp::Time{10, 0}));
   std::vector<nav2_costmap_2d::Observation::ConstSharedPtr> observations{};
   // The observation_keep_time is 10s and the age of the oldest cloud is 9s
   // so all 3 added observations should be returned
   buffer_.getObservations(observations);
   EXPECT_EQ(observations.size(), 3u);
   setClock(rclcpp::Time{12, 0});
   observations.clear();
   // Now the oldest cloud is 11s old and only 2 observations should be returned
   buffer_.getObservations(observations);
   EXPECT_EQ(observations.size(), 2u);
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
