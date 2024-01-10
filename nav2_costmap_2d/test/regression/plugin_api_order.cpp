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
// limitations under the License. Reserved.

#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

TEST(CostmapPluginsTester, checkPluginAPIOrder)
{
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap_ros");

  // Workaround to avoid setting base_link->map transform
  costmap_ros->set_parameter(rclcpp::Parameter("robot_base_frame", "map"));
  // Specifying order verification plugin in the parameters
  std::vector<std::string> plugins_str;
  plugins_str.push_back("order_layer");
  costmap_ros->set_parameter(rclcpp::Parameter("plugins", plugins_str));
  costmap_ros->declare_parameter(
    "order_layer.plugin",
    rclcpp::ParameterValue(std::string("nav2_costmap_2d::OrderLayer")));

  // Do actual test: ensure that plugin->updateBounds()/updateCosts()
  // will be called after plugin->activate()
  costmap_ros->on_configure(costmap_ros->get_current_state());
  costmap_ros->on_activate(costmap_ros->get_current_state());

  // Do cleanup
  costmap_ros->on_deactivate(costmap_ros->get_current_state());
  costmap_ros->on_cleanup(costmap_ros->get_current_state());
  costmap_ros->on_shutdown(costmap_ros->get_current_state());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
