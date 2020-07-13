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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap_filter_semantic_info.hpp"

using namespace std::chrono_literals;

class DummySemanticMapServer : public rclcpp::Node
{
public:
  DummySemanticMapServer()
  : Node("semantic_map_server")
  {
    publisher_ = this->create_publisher<nav2_msgs::msg::CostmapFilterSemanticInfo>(
        "/semantic_info", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    std::unique_ptr<nav2_msgs::msg::CostmapFilterSemanticInfo> msg =
      std::make_unique<nav2_msgs::msg::CostmapFilterSemanticInfo>();
    msg->type = 0;
    msg->map_filter_topic = "/map_filter";
    msg->base = 0.0;
    msg->multiplier = 1.0;

    publisher_->publish(std::move(msg));
  }

private:
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterSemanticInfo>::SharedPtr publisher_;
};  // DummySemanticMapServer

int main(int argc, char * argv[])
{
  printf("This is dummy semantic map server\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummySemanticMapServer>());
  rclcpp::shutdown();

  return 0;
}
