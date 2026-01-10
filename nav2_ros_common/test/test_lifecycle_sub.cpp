// Copyright (c) 2025 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "nav2_ros_common/lifecycle_node.hpp"

class TestNode : public nav2:: LifecycleNode
{
public: 
  TestNode()
  :nav2::LifecycleNode("test_lifecycle_sub")
  {
    RCLCPP_INFO(get_logger(), "Test node created");
  }

  void initialize()
  {
    RCLCPP_INFO(get_logger(), "Creating lifecycle subscription");

    sub_ = create_subscription<std_msgs::msg::String>(
      "test_topic",
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Received: %s", msg->data.c_str());
      });

    RCLCPP_INFO(get_logger(), "Subscription created");
  }

protected:
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configuring");
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Activating - subscription will now process messages");
    return nav2::CallbackReturn::SUCCESS;
  }

  nav2:: CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating - subscription will ignore messages");
    return nav2::CallbackReturn::SUCCESS;
  }

private:
  nav2::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create node using shared_ptr
  auto node = std::make_shared<TestNode>();

  // Initialize when shared_ptr is created
  node->initialize();

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}