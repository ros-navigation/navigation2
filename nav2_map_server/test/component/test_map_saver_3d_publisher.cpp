// Copyright (c) 2020 Shivam Pandey
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

#include <experimental/filesystem>
#include <string>
#include <memory>

#include "test_constants/test_constants.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "nav2_map_server/map_3d/map_io_3d.hpp"

#define TEST_DIR TEST_DIRECTORY

using namespace nav2_map_server;  // NOLINT
using std::experimental::filesystem::path;

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("map_publisher")
  {
    std::string pub_map_pcd_file = path(TEST_DIR) / path(g_valid_pcd_yaml_file);
    sensor_msgs::msg::PointCloud2 pcd_msg_;
    geometry_msgs::msg::Pose origin_msg_;
    map_3d::LOAD_MAP_STATUS status =
      map_3d::loadMapFromYaml(pub_map_pcd_file, pcd_msg_, origin_msg_);
    if (status != map_3d::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Can not load %s map file", pub_map_pcd_file.c_str());
      return;
    }

    // Creating publisher of message type sensor_msgs::msg::PointCloud2
    pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // Creating publisher of message type geometry_msgs::msg::Pose
    origin_pub_ = create_publisher<geometry_msgs::msg::Pose>(
      "map_origin",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pcd_pub_->publish(pcd_msg_);
    origin_pub_->publish(origin_msg_);
  }

protected:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr origin_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto pub_node = std::make_shared<TestPublisher>();
  rclcpp::spin(pub_node);
  rclcpp::shutdown();
  return 0;
}
