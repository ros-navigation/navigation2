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
#include "nav2_map_server/map_server.hpp"
#include "nav2_util/lifecycle_service_client.hpp"
#include  "nav2_msgs/srv/load_map.hpp"

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

class TestNode : public ::testing::Test
{
public:
  TestNode()
  {
    node_ = rclcpp::Node::make_shared("map_client_test");
    map_server_node_name_ = "map_server";
    lifecycle_client_ =
      std::make_shared<nav2_util::LifecycleServiceClient>(map_server_node_name_, node_);
    RCLCPP_INFO(node_->get_logger(), "Creating Test Node");


    std::this_thread::sleep_for(std::chrono::seconds(1));  // allow node to start up
    const std::chrono::seconds timeout(5);
    lifecycle_client_->change_state(Transition::TRANSITION_CONFIGURE, timeout);
    lifecycle_client_->change_state(Transition::TRANSITION_ACTIVATE, timeout);
  }

  ~TestNode()
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
    if (
      rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      return result.get();
    } else {
      return nullptr;
    }
  }

protected:
  std::string map_server_node_name_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<nav2_util::LifecycleServiceClient> lifecycle_client_;
};

TEST_F(TestNode, GetMap)
{
  RCLCPP_INFO(node_->get_logger(), "Testing GetMap service");
  auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto client = node_->create_client<nav_msgs::srv::GetMap>(
    "/" + map_server_node_name_ + "/map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for map service");
  ASSERT_TRUE(client->wait_for_service());

  auto resp = send_request<nav_msgs::srv::GetMap>(node_, client, req);

  ASSERT_FLOAT_EQ(resp->map.info.resolution, g_valid_image_res);
  ASSERT_EQ(resp->map.info.width, g_valid_image_width);
  ASSERT_EQ(resp->map.info.height, g_valid_image_height);

  for (unsigned int i = 0; i < resp->map.info.width * resp->map.info.height; i++) {
    ASSERT_EQ(g_valid_image_content[i], resp->map.data[i]);
  }
}

TEST_F(TestNode, LoadMap)
{
  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
    "/" + map_server_node_name_ + "/load_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
  ASSERT_TRUE(client->wait_for_service());

  req->map_url = path(TEST_DIR) / path(g_valid_yaml_file);
  auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS);
  ASSERT_FLOAT_EQ(resp->map.info.resolution, g_valid_image_res);
  ASSERT_EQ(resp->map.info.height, g_valid_image_height);

  for (unsigned int i = 0; i < resp->map.info.width * resp->map.info.height; i++) {
    ASSERT_EQ(g_valid_image_content[i], resp->map.data[i]);
  }
}

TEST_F(TestNode, LoadMapNull)
{
  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
    "/" + map_server_node_name_ + "/load_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
  ASSERT_TRUE(client->wait_for_service());

  req->map_url = "";
  RCLCPP_INFO(node_->get_logger(), "Sending load_map request with null file name");
  auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST);
}

TEST_F(TestNode, LoadMapInvalidYaml)
{
  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
    "/" + map_server_node_name_ + "/load_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
  ASSERT_TRUE(client->wait_for_service());

  req->map_url = "invalid_file.yaml";
  RCLCPP_INFO(node_->get_logger(), "Sending load_map request with invalid yaml file name");
  auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA);
}

TEST_F(TestNode, LoadMapInvalidImage)
{
  RCLCPP_INFO(node_->get_logger(), "Testing LoadMap service");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::LoadMap>(
    "/" + map_server_node_name_ + "/load_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for load_map service");
  ASSERT_TRUE(client->wait_for_service());

  req->map_url = path(TEST_DIR) / "invalid_image.yaml";
  RCLCPP_INFO(node_->get_logger(), "Sending load_map request with invalid image file name");
  auto resp = send_request<nav2_msgs::srv::LoadMap>(node_, client, req);

  ASSERT_EQ(resp->result, nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA);
}
