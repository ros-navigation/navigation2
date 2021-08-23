// Copyright (c) 2020, Samsung Research America
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
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/map_server.hpp"
#include "nav2_amcl/amcl_node.hpp"
#include "nav2_controller/nav2_controller.hpp"
#include "nav2_planner/planner_server.hpp"
#include "nav2_recoveries/recovery_server.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_bt_navigator/bt_navigator.hpp"
#include "nav2_waypoint_follower/waypoint_follower.hpp"

template<typename NodeT>
std::shared_ptr<std::thread> create_spin_thread(NodeT & node)
{
  return std::make_shared<std::thread>(
    [node]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node->get_node_base_interface());
      executor.spin();
      executor.remove_node(node->get_node_base_interface());
    });
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // localiztion nodes
  std::vector<std::string> localiztion_node_names;
  auto map_node = std::make_shared<nav2_map_server::MapServer>();
  localiztion_node_names.push_back(map_node->get_name());
  auto amcl_node = std::make_shared<nav2_amcl::AmclNode>();
  localiztion_node_names.push_back(amcl_node->get_name());
  // lifecycle manager of localiztion
  auto loc_manager_options = rclcpp::NodeOptions();
  loc_manager_options.arguments(
    {"--ros-args", "-r", std::string("__node:=") + "lifecycle_manager_localization", "--"});
  loc_manager_options.append_parameter_override("node_names", localiztion_node_names);
  auto lifecycle_manager_localization_node =
    std::make_shared<nav2_lifecycle_manager::LifecycleManager>(loc_manager_options);

  // navigation nodes
  std::vector<std::string> navigation_node_names;
  auto controller_node = std::make_shared<nav2_controller::ControllerServer>();
  navigation_node_names.push_back(controller_node->get_name());
  auto planner_node = std::make_shared<nav2_planner::PlannerServer>();
  navigation_node_names.push_back(planner_node->get_name());
  auto recoveries_node = std::make_shared<recovery_server::RecoveryServer>();
  navigation_node_names.push_back(recoveries_node->get_name());
  auto bt_navigator_node = std::make_shared<nav2_bt_navigator::BtNavigator>();
  navigation_node_names.push_back(bt_navigator_node->get_name());
  auto waypoint_follower_node = std::make_shared<nav2_waypoint_follower::WaypointFollower>();
  navigation_node_names.push_back(waypoint_follower_node->get_name());
  // lifecycle manager of navigation
  auto nav_manager_options = rclcpp::NodeOptions();
  nav_manager_options.arguments(
    {"--ros-args", "-r", std::string("__node:=") + "lifecycle_manager_navigation", "--"});
  nav_manager_options.append_parameter_override("node_names", navigation_node_names);
  auto lifecycle_manager_navigation_node =
    std::make_shared<nav2_lifecycle_manager::LifecycleManager>(nav_manager_options);

  // spin threads
  std::vector<std::shared_ptr<std::thread>> threads;
  threads.push_back(create_spin_thread(map_node));
  threads.push_back(create_spin_thread(amcl_node));
  threads.push_back(create_spin_thread(lifecycle_manager_localization_node));
  threads.push_back(create_spin_thread(controller_node));
  threads.push_back(create_spin_thread(planner_node));
  threads.push_back(create_spin_thread(recoveries_node));
  threads.push_back(create_spin_thread(bt_navigator_node));
  threads.push_back(create_spin_thread(waypoint_follower_node));
  threads.push_back(create_spin_thread(lifecycle_manager_navigation_node));
  for (auto t : threads) {
    t->join();
  }

  rclcpp::shutdown();
  return 0;
}
