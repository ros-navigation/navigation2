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

#include <random>
#include <string>
#include <vector>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("test_updown"), "Initializing test");
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_service_client");
  nav2_lifecycle_manager::LifecycleManagerClient client_nav("lifecycle_manager_navigation", node);
  nav2_lifecycle_manager::LifecycleManagerClient client_loc("lifecycle_manager_localization", node);
  bool test_passed = true;

  // Wait for a few seconds to let all of the nodes come up
  std::this_thread::sleep_for(5s);

  // Start the nav2 system, bringing it to the ACTIVE state
  client_nav.startup();
  client_loc.startup();

  // Wait for a couple secs to make sure the nodes have processed all discovery
  // info before starting
  RCLCPP_INFO(rclcpp::get_logger("test_updown"), "Waiting for nodes to be active");
  std::this_thread::sleep_for(2s);

  // The system should now be active
  int retries = 0;
  while ((client_nav.is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) &&
    (client_loc.is_active() != nav2_lifecycle_manager::SystemStatus::ACTIVE) &&
    (retries < 10))
  {
    std::this_thread::sleep_for(2s);
    retries++;
  }
  if (retries == 10) {
    // the system isn't active
    RCLCPP_ERROR(rclcpp::get_logger("test_updown"), "System startup failed");
    test_passed = false;
  }

  // Shut down the nav2 system, bringing it to the FINALIZED state
  client_nav.shutdown();
  client_loc.shutdown();

  if (test_passed) {
    RCLCPP_INFO(
      rclcpp::get_logger("test_updown"),
      "****************************************************  TEST PASSED!");
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("test_updown"),
      "****************************************************  TEST FAILED!");
  }
  rclcpp::shutdown();
  return 0;
}
