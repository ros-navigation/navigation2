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
// limitations under the License.

#include <memory>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "nav2_simple_navigator/simple_navigator.hpp"

int main(int argc, char ** argv)
{
  try {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nav2_simple_navigator::SimpleNavigator>());
    rclcpp::shutdown();

    return 0;
  } catch (std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("simple_navigator"), e.what());
    return 0;
  }
}
