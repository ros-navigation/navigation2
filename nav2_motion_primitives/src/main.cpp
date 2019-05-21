// Copyright (c) 2018 Intel Corporation
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

#include "rclcpp/rclcpp.hpp"
#include "nav2_motion_primitives/back_up.hpp"
#include "nav2_motion_primitives/spin.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto motion_primitives_node = rclcpp::Node::make_shared("motion_primitives");

  auto spin = std::make_shared<nav2_motion_primitives::Spin>(
    motion_primitives_node);

  auto back_up = std::make_shared<nav2_motion_primitives::BackUp>(
    motion_primitives_node);

  rclcpp::spin(motion_primitives_node);
  rclcpp::shutdown();

  return 0;
}
