// Copyright (c) 2020 Samsung Research
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

#include <memory>

#include "gtest/gtest.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"


TEST(LifecylceTest, CheckInitialTfTimeout) {
  rclcpp::init(0, nullptr);

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(rclcpp::NodeOptions());
  costmap->set_parameter({"initial_transform_timeout", 0.0});

  std::thread spin_thread{[costmap]() {rclcpp::spin(costmap->get_node_base_interface());}};

  {
    const auto state_after_configure = costmap->configure();
    ASSERT_EQ(state_after_configure.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    // Without providing the transform from global to robot base the activation should fail
    // and the costmap should transition into the inactive state.
    const auto state_after_activate = costmap->activate();
    ASSERT_EQ(state_after_activate.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }

  // Set a dummy transform from global to robot base
  geometry_msgs::msg::TransformStamped transform_global_to_robot{};
  transform_global_to_robot.header.frame_id = costmap->getGlobalFrameID();
  transform_global_to_robot.child_frame_id = costmap->getBaseFrameID();
  costmap->getTfBuffer()->setTransform(transform_global_to_robot, "test", true);
  // Now the costmap should successful transition into the active state
  {
    const auto state_after_activate = costmap->activate();
    ASSERT_EQ(state_after_activate.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
}
