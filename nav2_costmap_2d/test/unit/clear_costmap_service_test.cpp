// Copyright (c) 2026 Sushant Vijay Chavan
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

#include <gtest/gtest.h>

#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/clear_costmap_around_pose.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

using namespace std::chrono_literals;

static constexpr char clear_entirely_srv_name[]{"/costmap/clear_entirely_costmap"};
static constexpr char clear_around_pose_srv_name[]{"/costmap/clear_around_pose_costmap"};
static constexpr char clear_around_robot_srv_name[]{"/costmap/clear_around_costmap"};
static constexpr char clear_except_region_srv_name[]{"/costmap/clear_except_costmap"};

using ClearEntireCostMapSrv = nav2_msgs::srv::ClearEntireCostmap;
using ClearCostmapAroundPoseSrv = nav2_msgs::srv::ClearCostmapAroundPose;
using ClearCostmapAroundRobotSrv = nav2_msgs::srv::ClearCostmapAroundRobot;
using ClearCostmapExceptRegionSrv = nav2_msgs::srv::ClearCostmapExceptRegion;

template<typename ServiceT>
class ClearCostmapServiceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap");

    // Send identity transform to ensure costmap can transform robot poses during service calls
    auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(costmap_);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.header.stamp = costmap_->now();
    transform.transform.rotation.w = 1.0;
    tf_broadcaster->sendTransform(transform);

    clear_costmap_client_ = costmap_->create_client<ServiceT>(service_name_);
    costmap_->on_configure(rclcpp_lifecycle::State());
    ASSERT_TRUE(clear_costmap_client_->wait_for_service(10s));

    waitForTransform();
  }

  void testPluginsInitialization()
  {
    const auto & plugins = *(costmap_->getLayeredCostmap()->getPlugins());

    ASSERT_EQ(plugins.size(),
      3u) << "Expecting three plugin to be loaded by default for service: " << service_name_;

    const std::vector<std::string> expected_plugin_names = {"static_layer", "obstacle_layer",
      "inflation_layer"};

    const std::vector<bool> expected_clearable = {false, true, false};

    for (size_t i = 0; i < plugins.size(); ++i) {
      EXPECT_EQ(plugins[i]->getName(),
        expected_plugin_names[i]) << "Plugin name mismatch at index " << i << " for service: " <<
        service_name_;
      EXPECT_EQ(plugins[i]->isClearable(),
        expected_clearable[i]) << "Plugin clearable status mismatch at index " << i <<
        " for service: " << service_name_;
    }
  }

  void testClearingWithEmptyPluginList()
  {
    // Expect service call to succeed by default when no plugins are specified
    testClearingWithPluginList({}, "'Clearing with empty plugin list' test", true);
  }

  void testClearingWithClearablePlugin()
  {
    // Expect service call to succeed since obstacle_layer is clearable
    testClearingWithPluginList({"obstacle_layer"}, "'Clearing with clearable plugin' test", true);
  }

  void testClearingWithNonClearablePlugin()
  {
    // Expect service call to fail since inflation_layer is not clearable
    testClearingWithPluginList({"inflation_layer"}, "'Clearing with non-clearable plugin' test",
      false);
  }

  void testClearingWithClearableAndNonClearablePluginList()
  {
    // Expect service call to fail since inflation_layer is not clearable
    testClearingWithPluginList({"obstacle_layer", "inflation_layer"},
      "'Clearing with clearable and non-clearable plugin list' test", false);
  }

  void testClearingWithUnknownPlugin()
  {
    // Expect service call to fail since voxel_layer is unknown
    testClearingWithPluginList({"voxel_layer"}, "'Clearing with unknown plugin' test", false);
  }

  void testClearingWithKnownAndUnknownPluginList()
  {
    // Expect service call to fail since voxel_layer is unknown
    testClearingWithPluginList({"obstacle_layer", "voxel_layer"},
      "'Clearing with known and unknown plugin' test", false);
  }

private:
  void waitForTransform()
  {
    auto transformAvailable = [this]()-> bool {
        return costmap_->getTfBuffer()->canTransform("map", "base_link", rclcpp::Time(0), 1s);
      };

    rclcpp::Time start_time = costmap_->now();
    const auto max_wait_time = 5s;
    while (rclcpp::ok() && (costmap_->now() - start_time) < max_wait_time &&
      !transformAvailable())
    {
      RCLCPP_WARN_THROTTLE(costmap_->get_logger(), *costmap_->get_clock(), 1000,
        "Waiting for transform between 'map' and 'base_link' to become available...");
      std::this_thread::sleep_for(10ms);
    }

    ASSERT_TRUE(transformAvailable()) <<
      "Transform between 'map' and 'base_link' not available after waiting for " <<
      max_wait_time.count() << " seconds";
  }

  ServiceT::Request::SharedPtr createDefaultSrvRequest();

  void testClearingWithPluginList(
    const std::vector<std::string> & plugin_names,
    const std::string & test_operation, bool expected_success)
  {
    auto request = createDefaultSrvRequest();
    request->plugins = plugin_names;
    validateServiceCall(request, test_operation, expected_success);
  }

  void validateServiceCall(
    ServiceT::Request::SharedPtr request, const std::string & test_operation,
    bool expected_success)
  {
    auto result_future = clear_costmap_client_->async_call(request);
    if (rclcpp::spin_until_future_complete(costmap_,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result_future.get();
      EXPECT_EQ(response->success,
        expected_success) << test_operation << " failed for service: " << service_name_;
    } else {
      FAIL() << "Service call failed when performing " << test_operation << " for service: " <<
        service_name_;
    }
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  nav2::ServiceClient<ServiceT>::SharedPtr clear_costmap_client_;
  static std::string service_name_;
};

// Specialized class for ClearEntireCostmap service tests
using ClearEntireCostmapTest = ClearCostmapServiceTest<ClearEntireCostMapSrv>;
template<> std::string ClearEntireCostmapTest::service_name_ = clear_entirely_srv_name;
template<>
ClearEntireCostMapSrv::Request::SharedPtr ClearEntireCostmapTest::createDefaultSrvRequest()
{
  return std::make_shared<ClearEntireCostMapSrv::Request>();
}

// Specialized class for ClearCostmapAroundPose service tests
using ClearCostmapAroundPoseTest = ClearCostmapServiceTest<ClearCostmapAroundPoseSrv>;
template<> std::string ClearCostmapAroundPoseTest::service_name_ = clear_around_pose_srv_name;
template<>
ClearCostmapAroundPoseSrv::Request::SharedPtr ClearCostmapAroundPoseTest::createDefaultSrvRequest()
{
  auto request = std::make_shared<ClearCostmapAroundPoseSrv::Request>();
  request->pose.header.frame_id = costmap_->getGlobalFrameID();
  request->pose.pose.position.x = 0.0;
  request->pose.pose.position.y = 0.0;
  request->pose.pose.orientation.w = 1.0;  // Identity orientation
  request->reset_distance = 1.0;
  return request;
}

// Specialized class for ClearCostmapAroundRobot service tests
using ClearCostmapAroundRobotTest =
  ClearCostmapServiceTest<ClearCostmapAroundRobotSrv>;
template<> std::string ClearCostmapAroundRobotTest::service_name_ = clear_around_robot_srv_name;
template<>
ClearCostmapAroundRobotSrv::Request::SharedPtr
ClearCostmapAroundRobotTest::createDefaultSrvRequest()
{
  auto request = std::make_shared<ClearCostmapAroundRobotSrv::Request>();
  request->reset_distance = 1.0;
  return request;
}

// Specialized class for ClearCostmapExceptRegion service tests
using ClearCostmapExceptRegionTest = ClearCostmapServiceTest<ClearCostmapExceptRegionSrv>;
template<> std::string ClearCostmapExceptRegionTest::service_name_ = clear_except_region_srv_name;
template<>
ClearCostmapExceptRegionSrv::Request::SharedPtr ClearCostmapExceptRegionTest::
createDefaultSrvRequest()
{
  auto request = std::make_shared<ClearCostmapExceptRegionSrv::Request>();
  request->reset_distance = 1.0;
  return request;
}

// Tests for the ClearEntireCostmap service
TEST_F(ClearEntireCostmapTest, TestPluginsInitialization)
{
  testPluginsInitialization();
}

TEST_F(ClearEntireCostmapTest, TestClearingWithEmptyPluginList)
{
  testClearingWithEmptyPluginList();
}

TEST_F(ClearEntireCostmapTest, TestClearingWithClearablePlugin)
{
  testClearingWithClearablePlugin();
}

TEST_F(ClearEntireCostmapTest, TestClearingWithNonClearablePlugin)
{
  testClearingWithNonClearablePlugin();
}

TEST_F(ClearEntireCostmapTest, TestClearingWithClearableAndNonClearablePluginList)
{
  testClearingWithClearableAndNonClearablePluginList();
}

TEST_F(ClearEntireCostmapTest, TestClearingWithUnknownPlugin)
{
  testClearingWithUnknownPlugin();
}

TEST_F(ClearEntireCostmapTest, TestClearingWithKnownAndUnknownPluginList)
{
  testClearingWithKnownAndUnknownPluginList();
}

// Tests for the ClearCostmapAroundPose service
TEST_F(ClearCostmapAroundPoseTest, TestPluginsInitialization)
{
  testPluginsInitialization();
}

TEST_F(ClearCostmapAroundPoseTest, TestClearingWithEmptyPluginList)
{
  testClearingWithEmptyPluginList();
}

TEST_F(ClearCostmapAroundPoseTest, TestClearingWithClearablePlugin)
{
  testClearingWithClearablePlugin();
}

TEST_F(ClearCostmapAroundPoseTest, TestClearingWithNonClearablePlugin)
{
  testClearingWithNonClearablePlugin();
}

TEST_F(ClearCostmapAroundPoseTest, TestClearingWithClearableAndNonClearablePluginList)
{
  testClearingWithClearableAndNonClearablePluginList();
}

TEST_F(ClearCostmapAroundPoseTest, TestClearingWithUnknownPlugin)
{
  testClearingWithUnknownPlugin();
}

TEST_F(ClearCostmapAroundPoseTest, TestClearingWithKnownAndUnknownPluginList)
{
  testClearingWithKnownAndUnknownPluginList();
}

// Tests for the ClearCostmapAroundRobot service
TEST_F(ClearCostmapAroundRobotTest, TestPluginsInitialization)
{
  testPluginsInitialization();
}

TEST_F(ClearCostmapAroundRobotTest, TestClearingWithEmptyPluginList)
{
  testClearingWithEmptyPluginList();
}

TEST_F(ClearCostmapAroundRobotTest, TestClearingWithClearablePlugin)
{
  testClearingWithClearablePlugin();
}

TEST_F(ClearCostmapAroundRobotTest, TestClearingWithNonClearablePlugin)
{
  testClearingWithNonClearablePlugin();
}

TEST_F(ClearCostmapAroundRobotTest, TestClearingWithClearableAndNonClearablePluginList)
{
  testClearingWithClearableAndNonClearablePluginList();
}

TEST_F(ClearCostmapAroundRobotTest, TestClearingWithUnknownPlugin)
{
  testClearingWithUnknownPlugin();
}

TEST_F(ClearCostmapAroundRobotTest, TestClearingWithKnownAndUnknownPluginList)
{
  testClearingWithKnownAndUnknownPluginList();
}

// Tests for the ClearCostmapExceptRegion service
TEST_F(ClearCostmapExceptRegionTest, TestPluginsInitialization)
{
  testPluginsInitialization();
}

TEST_F(ClearCostmapExceptRegionTest, TestClearingWithEmptyPluginList)
{
  testClearingWithEmptyPluginList();
}

TEST_F(ClearCostmapExceptRegionTest, TestClearingWithClearablePlugin)
{
  testClearingWithClearablePlugin();
}

TEST_F(ClearCostmapExceptRegionTest, TestClearingWithNonClearablePlugin)
{
  testClearingWithNonClearablePlugin();
}

TEST_F(ClearCostmapExceptRegionTest, TestClearingWithClearableAndNonClearablePluginList)
{
  testClearingWithClearableAndNonClearablePluginList();
}

TEST_F(ClearCostmapExceptRegionTest, TestClearingWithUnknownPlugin)
{
  testClearingWithUnknownPlugin();
}

TEST_F(ClearCostmapExceptRegionTest, TestClearingWithKnownAndUnknownPluginList)
{
  testClearingWithKnownAndUnknownPluginList();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
