// Copyright (c) 2026 Oke Iyanuoluwa Enoch
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

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class LifecycleManagerWrapper : public nav2_lifecycle_manager::LifecycleManager
{
public:
  using nav2_lifecycle_manager::LifecycleManager::LifecycleManager;

  bool hasServiceClients() const
  {
    return !node_map_.empty();
  }
};

TEST(LifecycleManagerPreshutdown, shutdown_while_waiting_for_managed_node)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter(
      "node_names", std::vector<std::string>{"missing_node_a", "missing_node_b"}),
    rclcpp::Parameter("autostart", false),
  });
  auto manager = std::make_shared<LifecycleManagerWrapper>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(manager);
  std::thread spin_thread([&executor]() {executor.spin();});

  // Initialization is now blocked waiting on a service which is never available
  std::this_thread::sleep_for(500ms);
  rclcpp::shutdown();
  spin_thread.join();

  EXPECT_FALSE(manager->hasServiceClients());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
