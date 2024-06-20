// Copyright (c) 2024 Open Navigation LLC
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
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "opennav_docking/dock_database.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// These sets of tests are admittedly incomplete without a dummy docking plugin.
// Integration tests handle coverage more fully than the database lookups and population.
// However, Utils unit tests already validate the key database population functions.

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace std::chrono_literals;

namespace opennav_docking
{

class DbShim : public opennav_docking::DockDatabase
{
public:
  DbShim()
  : DockDatabase()
  {}

  void populateOne()
  {
    Dock dock;
    dock.type = "first_dock_t";
    dock_plugins_.insert({"first_dock_t", nullptr});
    dock_instances_.insert({"first_dock", dock});
  }

  void populateTwo()
  {
    Dock dock;
    dock.type = "second_dock_t";
    dock_plugins_.insert({"second_dock_t", nullptr});
    dock_instances_.insert({"second_dock", dock});
  }
};

TEST(DatabaseTests, ObjectLifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  opennav_docking::DockDatabase db;
  db.initialize(node, nullptr);
  db.activate();
  db.deactivate();

  EXPECT_EQ(db.plugin_size(), 0u);
  EXPECT_EQ(db.instance_size(), 0u);
}

TEST(DatabaseTests, initializeBogusPlugins)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  std::vector<std::string> plugins{"dockv1", "dockv2"};
  node->declare_parameter("dock_plugins", rclcpp::ParameterValue(plugins));
  opennav_docking::DockDatabase db;
  db.initialize(node, nullptr);

  plugins.clear();
  node->set_parameter(rclcpp::Parameter("dock_plugins", rclcpp::ParameterValue(plugins)));
  db.initialize(node, nullptr);
}

TEST(DatabaseTests, findTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  DbShim db;
  db.populateOne();

  db.findDockPlugin("");
  db.findDockPlugin("first_dock_t");
  EXPECT_THROW(db.findDock("first_dock"), opennav_docking_core::DockNotValid);
  EXPECT_THROW(db.findDock("bogus_dock"), opennav_docking_core::DockNotInDB);

  db.populateTwo();
  db.findDockPlugin("");
}

TEST(DatabaseTests, reloadDbService)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  std::vector<std::string> plugins{"dockv1"};
  node->declare_parameter("dock_plugins", rclcpp::ParameterValue(plugins));
  node->declare_parameter(
    "dockv1.plugin",
    rclcpp::ParameterValue("opennav_docking::SimpleChargingDock"));
  opennav_docking::DockDatabase db;
  db.initialize(node, nullptr);

  // Call service with a filepath
  auto client =
    node->create_client<nav2_msgs::srv::ReloadDockDatabase>("test/reload_database");

  auto request = std::make_shared<nav2_msgs::srv::ReloadDockDatabase::Request>();
  request->filepath = ament_index_cpp::get_package_share_directory("opennav_docking") +
    "/test_dock_file.yaml";
  EXPECT_TRUE(client->wait_for_service(1s));
  auto result = client->async_send_request(request);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node, result, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(result.get()->success);

  // Try again with a bogus file
  auto request2 = std::make_shared<nav2_msgs::srv::ReloadDockDatabase::Request>();
  request2->filepath = ament_index_cpp::get_package_share_directory("opennav_docking") +
    "/file_does_not_exist.yaml";
  EXPECT_TRUE(client->wait_for_service(1s));
  auto result2 = client->async_send_request(request2);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node, result2, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(result2.get()->success);
}

}  // namespace opennav_docking
