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
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Logger logger = rclcpp::get_logger("test_updown");
  RCLCPP_INFO(logger, "Initializing test");
  auto node = std::make_shared<rclcpp::Node>("lifecycle_manager_service_client");
  nav2_lifecycle_manager::LifecycleManagerClient client_nav2("lifecycle_manager_nav2", node);

  // Wait for a few seconds to let all of the nodes come up
  std::this_thread::sleep_for(5s);

  // Start the nav2 system, bringing it to the ACTIVE state
  client_nav2.startup();

  // Wait for a couple secs to make sure the nodes have processed all discovery
  // info before starting
  RCLCPP_INFO(logger, "Waiting for nodes to be active");
  std::this_thread::sleep_for(2s);

  // The system should now be active
  int retries_number = 10;
  bool test_passed = false;
  while (retries_number) {
    if (client_nav2.is_active() == nav2_lifecycle_manager::SystemStatus::ACTIVE) {
      test_passed = true;
      break;
    }

    RCLCPP_WARN(logger, "Not all nodes are active. Repeat status request.");
    std::this_thread::sleep_for(2s);
    retries_number--;
  }

  if (!test_passed) {
    // the system isn't active
    RCLCPP_ERROR(logger, "System startup failed");
  }

  // Shut down the nav2 system, bringing it to the FINALIZED state
  client_nav2.shutdown();

  if (test_passed) {
    RCLCPP_INFO(
      logger,
      "****************************************************  TEST PASSED!");
  } else {
    RCLCPP_INFO(
      logger,
      "****************************************************  TEST FAILED!");
  }
  rclcpp::shutdown();
  return 0;
}
