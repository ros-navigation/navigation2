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

#include <gtest/gtest.h>
#include <experimental/filesystem>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>

#include "test_constants/test_constants.h"
#include "nav2_util/lifecycle_service_client.hpp"

#include "nav2_msgs/srv/get_map3_d.hpp"
#include "nav2_msgs/srv/load_map3_d.hpp"
#include "nav2_msgs/msg/pcd2.hpp"

#define TEST_DIR TEST_DIRECTORY

using std::experimental::filesystem::path;

using lifecycle_msgs::msg::Transition;

class RclCppFixture
{
 public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

class MapServer3DTestFixture : public ::testing::Test
{
 public:
  static void SetUpTestCase()
  {
    node_ = rclcpp::Node::make_shared("map_client_test");
    lifecycle_client_ =
      std::make_shared<nav2_util::LifecycleServiceClient>("map_server", node_);
    RCLCPP_INFO(node_->get_logger(), "Creating Test Node");

    std::this_thread::sleep_for(std::chrono::seconds(5));  // allow node to start up
    const std::chrono::seconds timeout(5);
    lifecycle_client_->change_state(Transition::TRANSITION_CONFIGURE, timeout);
    lifecycle_client_->change_state(Transition::TRANSITION_ACTIVATE, timeout);
  }

  static void TearDownTestCase()
  {
    lifecycle_client_->change_state(Transition::TRANSITION_DEACTIVATE);
    lifecycle_client_->change_state(Transition::TRANSITION_CLEANUP);
  }

  template<class T>
  typename T::Response::SharedPtr send_request(
      rclcpp::Node::SharedPtr node,
      typename rclcpp::Client<T>::SharedPtr client,
      typename T::Request::SharedPtr request)
  {
    auto result = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      return result.get();
    } else {
      return nullptr;
    }
  }

 protected:
  // Check that map_msg corresponds to reference pattern
  // Input: map_msg
  static void verifyMapMsg(const nav2_msgs::msg::PCD2 & map_msg)
  {
    std::vector<float> origin;
    origin.push_back(map_msg.origin.x);
    origin.push_back(map_msg.origin.y);
    origin.push_back(map_msg.origin.z);

    std::vector<float> orientation;
    orientation.push_back(map_msg.orientation.w);
    orientation.push_back(map_msg.orientation.x);
    orientation.push_back(map_msg.orientation.y);
    orientation.push_back(map_msg.orientation.z);
    ASSERT_EQ(origin, g_valid_origin_pcd);
    ASSERT_EQ(orientation, g_valid_orientation_pcd);

    ASSERT_EQ(map_msg.map.width, g_valid_pcd_width);
    ASSERT_EQ(map_msg.map.data.size(), g_valid_pcd_data_size);
  }

  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<nav2_util::LifecycleServiceClient> lifecycle_client_;
};


rclcpp::Node::SharedPtr MapServer3DTestFixture::node_ = nullptr;
std::shared_ptr<nav2_util::LifecycleServiceClient> MapServer3DTestFixture::lifecycle_client_ =
    nullptr;


// Send map getting service request and verify obtained PointCloud
TEST_F(MapServer3DTestFixture, GetMap3D)
{
  RCLCPP_INFO(node_->get_logger(), "Testing GetMap service");
  auto req = std::make_shared<nav2_msgs::srv::GetMap3D::Request>();
  auto client = node_->create_client<nav2_msgs::srv::GetMap3D>(
      "/map_server/map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for map service");
  ASSERT_TRUE(client->wait_for_service());

  auto resp = send_request<nav2_msgs::srv::GetMap3D>(node_, client, req);

  nav2_msgs::msg::PCD2 map_msg;
  map_msg.map = resp->map;
  map_msg.origin = resp->origin;
  map_msg.orientation = resp->orientation;
  verifyMapMsg(map_msg);
}

// Send map loading service request and verify obtained PointCloud
TEST_F(MapServer3DTestFixture, LoadMap3D)
{
  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap3D::Request>();
  auto client = node_->create_client<nav2_msgs::srv::LoadMap3D>(
      "/map_server/load_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
  ASSERT_TRUE(client->wait_for_service());

  req->map_url = path(TEST_DIR) / path(g_valid_pcd_yaml_file);
  auto resp = send_request<nav2_msgs::srv::LoadMap3D>(node_, client, req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap3D::Response::RESULT_SUCCESS);

  nav2_msgs::msg::PCD2 map_msg;
  map_msg.map = resp->map;
  map_msg.origin = resp->origin;
  map_msg.orientation = resp->orientation;
  verifyMapMsg(map_msg);
}

// Send map loading service request without specifying which map to load
TEST_F(MapServer3DTestFixture, LoadMapNull3D)
{
  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap3D::Request>();
  auto client = node_->create_client<nav2_msgs::srv::LoadMap3D>(
      "/map_server/load_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
  ASSERT_TRUE(client->wait_for_service());

  req->map_url = "";
  RCLCPP_INFO(node_->get_logger(), "Sending load_map request with null file name");
  auto resp = send_request<nav2_msgs::srv::LoadMap3D>(node_, client, req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap3D::Response::RESULT_MAP_DOES_NOT_EXIST);
}