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

#include <string>
#include <memory>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include "test_constants/test_constants.h"
#include "nav2_map_server/map_server.hpp"
#include "nav2_util/lifecycle_service_client.hpp"
#include "nav2_msgs/srv/load_map.hpp"
using namespace std::chrono_literals;
using namespace rclcpp;  // NOLINT

#define TEST_DIR TEST_DIRECTORY

using std::filesystem::path;

using lifecycle_msgs::msg::Transition;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

class MapServerTestFixture : public ::testing::Test
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
    lifecycle_client_->change_state(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
    lifecycle_client_.reset();
    node_.reset();
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
  void verifyMapMsg(const nav_msgs::msg::OccupancyGrid & map_msg)
  {
    ASSERT_FLOAT_EQ(map_msg.info.resolution, g_valid_image_res);
    ASSERT_EQ(map_msg.info.width, g_valid_image_width);
    ASSERT_EQ(map_msg.info.height, g_valid_image_height);
    for (unsigned int i = 0; i < map_msg.info.width * map_msg.info.height; i++) {
      ASSERT_EQ(g_valid_image_content[i], map_msg.data[i]);
    }
  }

  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<nav2_util::LifecycleServiceClient> lifecycle_client_;
};


rclcpp::Node::SharedPtr MapServerTestFixture::node_ = nullptr;
std::shared_ptr<nav2_util::LifecycleServiceClient> MapServerTestFixture::lifecycle_client_ =
  nullptr;


// Send map getting service request and verify obtained OccupancyGrid
TEST_F(MapServerTestFixture, GetMap)
{
  RCLCPP_INFO(node_->get_logger(), "Testing GetMap service");
  auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto client = node_->create_client<nav_msgs::srv::GetMap>(
    "/map_server/map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for map service");
  ASSERT_TRUE(client->wait_for_service());

  auto resp = send_request<nav_msgs::srv::GetMap>(node_, client, req);

  verifyMapMsg(resp->map);
}

// Send map loading service request and verify obtained OccupancyGrid
TEST_F(MapServerTestFixture, LoadMap)
{
  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
    "/map_server/load_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
  ASSERT_TRUE(client->wait_for_service());

  req->map_url = path(TEST_DIR) / path(g_valid_yaml_file);
  auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS);
  verifyMapMsg(resp->map);
}

// Send map loading service request without specifying which map to load
TEST_F(MapServerTestFixture, LoadMapNull)
{
  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
    "/map_server/load_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
  ASSERT_TRUE(client->wait_for_service());

  req->map_url = "";
  RCLCPP_INFO(node_->get_logger(), "Sending load_map request with null file name");
  auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST);
}

// Send map loading service request with non-existing yaml file
TEST_F(MapServerTestFixture, LoadMapInvalidYaml)
{
  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
    "/map_server/load_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
  ASSERT_TRUE(client->wait_for_service());

  req->map_url = "invalid_file.yaml";
  RCLCPP_INFO(node_->get_logger(), "Sending load_map request with invalid yaml file name");
  auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA);
}

// Send map loading service request with yaml file containing non-existing map
TEST_F(MapServerTestFixture, LoadMapInvalidImage)
{
  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
    "/map_server/load_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
  ASSERT_TRUE(client->wait_for_service());

  req->map_url = path(TEST_DIR) / "invalid_image.yaml";
  RCLCPP_INFO(node_->get_logger(), "Sending load_map request with invalid image file name");
  auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA);
}

/**
 * Test behaviour of server if yaml_filename is set to an empty string.
 */
TEST_F(MapServerTestFixture, NoInitialMap)
{
  // turn off node into unconfigured state
  lifecycle_client_->change_state(Transition::TRANSITION_DEACTIVATE);
  lifecycle_client_->change_state(Transition::TRANSITION_CLEANUP);

  auto client = node_->create_client<nav_msgs::srv::GetMap>("/map_server/map");
  auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_, "map_server");
  ASSERT_TRUE(parameters_client->wait_for_service(3s));

  // set yaml_filename-parameter to empty string (essentially restart the node)
  RCLCPP_INFO(node_->get_logger(), "Removing yaml_filename-parameter before restarting");
  parameters_client->set_parameters({Parameter("yaml_filename", ParameterValue(""))});

  // only configure node, to test behaviour of service while node is not active
  lifecycle_client_->change_state(Transition::TRANSITION_CONFIGURE, 3s);

  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service while not being active");
  auto load_map_req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  auto load_map_cl = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

  ASSERT_TRUE(load_map_cl->wait_for_service(3s));
  auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, load_map_cl, load_map_req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_UNDEFINED_FAILURE);

  // activate server and load map:
  lifecycle_client_->change_state(Transition::TRANSITION_ACTIVATE, 3s);
  RCLCPP_INFO(node_->get_logger(), "active again");

  load_map_req->map_url = path(TEST_DIR) / path(g_valid_yaml_file);
  auto load_res = send_request<nav2_msgs::srv::LoadMap>(node_, load_map_cl, load_map_req);

  ASSERT_EQ(load_res->result, nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS);
  verifyMapMsg(load_res->map);
}
