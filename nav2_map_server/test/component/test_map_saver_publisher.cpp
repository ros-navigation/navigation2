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
// limitations under the License.

#include <filesystem>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/map_io.hpp"
#include "test_constants/test_constants.h"

#define TEST_DIR TEST_DIRECTORY

using namespace nav2_map_server;  // NOLINT
using std::filesystem::path;

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("map_publisher")
  {
    std::string pub_map_file = path(TEST_DIR) / path(g_valid_yaml_file);
    nav_msgs::msg::OccupancyGrid msg;
    LOAD_MAP_STATUS status = loadMapFromYaml(pub_map_file, msg);
    if (status != LOAD_MAP_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Can not load %s map file", pub_map_file.c_str());
      return;
    }

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    map_pub_->publish(msg);
  }

protected:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto pub_node = std::make_shared<TestPublisher>();
  rclcpp::spin(pub_node);
  rclcpp::shutdown();
  return 0;
}
