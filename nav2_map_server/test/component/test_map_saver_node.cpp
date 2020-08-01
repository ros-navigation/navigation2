// Copyright (c) 2020 Samsung Research Russia
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
#include "nav2_map_server/map_saver.hpp"
#include "nav2_util/lifecycle_service_client.hpp"

#include "nav2_msgs/srv/save_map.hpp"
#include "nav2_msgs/srv/save_map3_d.hpp"
#include "nav2_msgs/msg/pcd2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#define TEST_DIR TEST_DIRECTORY

using std::experimental::filesystem::path;
using lifecycle_msgs::msg::Transition;
using namespace nav2_map_server;  // NOLINT

class RclCppFixture {
 public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};

RclCppFixture g_rclcppfixture;

class MapSaverTestFixture : public ::testing::Test
{
 public:
  static void SetUpTestCase()
  {
    node_ = rclcpp::Node::make_shared("map_client_test");
    lifecycle_client_ =
        std::make_shared<nav2_util::LifecycleServiceClient>("map_saver", node_);
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
  static void verifyMapMsg(const nav_msgs::msg::OccupancyGrid &map_msg)
  {
    ASSERT_FLOAT_EQ(map_msg.info.resolution, g_valid_image_res);
    ASSERT_EQ(map_msg.info.width, g_valid_image_width);
    ASSERT_EQ(map_msg.info.height, g_valid_image_height);
    for (unsigned int i = 0; i < map_msg.info.width * map_msg.info.height; i++) {
      ASSERT_EQ(g_valid_image_content[i], map_msg.data[i]);
    }
  }

  static void verifyMapMsg(const nav2_msgs::msg::PCD2 &map_msg)
  {
    std::vector<float> vp;
    vp.push_back(map_msg.view_point.translation.x);
    vp.push_back(map_msg.view_point.translation.y);
    vp.push_back(map_msg.view_point.translation.z);

    vp.push_back(map_msg.view_point.rotation.w);
    vp.push_back(map_msg.view_point.rotation.x);
    vp.push_back(map_msg.view_point.rotation.y);
    vp.push_back(map_msg.view_point.rotation.z);
    ASSERT_EQ(vp, g_valid_view_point);

//    TODO: add additional test
  }

  static rclcpp::Node::SharedPtr node_;
  static std::shared_ptr<nav2_util::LifecycleServiceClient> lifecycle_client_;
};

rclcpp::Node::SharedPtr MapSaverTestFixture::node_ = nullptr;
std::shared_ptr<nav2_util::LifecycleServiceClient> MapSaverTestFixture::lifecycle_client_ =
    nullptr;

// Send map saving service request.
// Load saved map and verify obtained OccupancyGrid.
TEST_F(MapSaverTestFixture, SaveMap)
{
  RCLCPP_INFO(node_->get_logger(), "Testing SaveMap service");
  auto req = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::SaveMap>(
      "/map_saver/save_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for save_map service");
  ASSERT_TRUE(client->wait_for_service());

  // 1. Send valid save_map service request
  req->map_topic = "map";
  req->map_url = path(g_tmp_dir) / path(g_valid_map_name);
  req->image_format = "png";
  req->map_mode = "trinary";
  req->free_thresh = g_default_free_thresh;
  req->occupied_thresh = g_default_occupied_thresh;
  auto resp = send_request<nav2_msgs::srv::SaveMap>(node_, client, req);
  ASSERT_EQ(resp->result, true);

  // 2. Load saved map and verify it
  nav_msgs::msg::OccupancyGrid map_msg;
  LOAD_MAP_STATUS status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);
  verifyMapMsg(map_msg);
}

// Send map(pcd) saving service request.
// Load saved map and verify obtained OccupancyGrid.
TEST_F(MapSaverTestFixture, SaveMap3D)
{
  RCLCPP_INFO(node_->get_logger(), "Testing SaveMap service");
  auto req = std::make_shared<nav2_msgs::srv::SaveMap3D::Request>();
  auto client = node_->create_client<nav2_msgs::srv::SaveMap3D>(
    "/map_saver/save_map3D");

  RCLCPP_INFO(node_->get_logger(), "Waiting for save_map service");
  ASSERT_TRUE(client->wait_for_service());

  // 1. Send valid save_map service request
  req->map_topic = "map3D";
  req->map_url = path(g_tmp_dir) / path(g_valid_pcd_map_name);

  // set view point translation
  req->view_point.translation.x = g_valid_view_point[0];
  req->view_point.translation.y = g_valid_view_point[1];
  req->view_point.translation.z = g_valid_view_point[2];

  // set view point rotation
  req->view_point.rotation.w = g_valid_view_point[3];
  req->view_point.rotation.x = g_valid_view_point[4];
  req->view_point.rotation.y = g_valid_view_point[5];
  req->view_point.rotation.z = g_valid_view_point[6];

  req->file_format = "pcd";
  req->as_binary = false;

  auto resp = send_request<nav2_msgs::srv::SaveMap3D>(node_, client, req);
  ASSERT_EQ(resp->result, true);

  // 2. Load saved map and verify it
  nav2_msgs::msg::PCD2 map_msg;
  nav2_map_server_3D::LOAD_MAP_STATUS status =
      nav2_map_server_3D::loadMapFromYaml(
          path(g_tmp_dir) / path(g_valid_yaml_file),
          map_msg.map,
          map_msg.view_point);

  ASSERT_EQ(status, nav2_map_server_3D::LOAD_MAP_SUCCESS);
  verifyMapMsg(map_msg);
}

// Send map saving service request with default parameters.
// Load saved map and verify obtained OccupancyGrid.
TEST_F(MapSaverTestFixture, SaveMapDefaultParameters)
{
  RCLCPP_INFO(node_->get_logger(), "Testing SaveMap service");
  auto req = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::SaveMap>(
    "/map_saver/save_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for save_map service");
  ASSERT_TRUE(client->wait_for_service());

  // 1. Send save_map serivce request with default parameters
  req->map_topic = "";
  req->map_url = path(g_tmp_dir) / path(g_valid_map_name);
  req->image_format = "";
  req->map_mode = "";
  req->free_thresh = 0.0;
  req->occupied_thresh = 0.0;
  auto resp = send_request<nav2_msgs::srv::SaveMap>(node_, client, req);
  ASSERT_EQ(resp->result, true);

  // 2. Load saved map and verify it
  nav_msgs::msg::OccupancyGrid map_msg;
  LOAD_MAP_STATUS status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);
  verifyMapMsg(map_msg);
}

// Send map(pcd) saving service request with default parameters.
// Load saved map(pcd) and verify obtained PointCloud.
TEST_F(MapSaverTestFixture, SaveMapDefaultParameters3D)
{
  RCLCPP_INFO(node_->get_logger(), "Testing SaveMap service");
  auto req = std::make_shared<nav2_msgs::srv::SaveMap3D::Request>();
  auto client = node_->create_client<nav2_msgs::srv::SaveMap3D>(
    "/map_saver/save_map3D");

  RCLCPP_INFO(node_->get_logger(), "Waiting for save_map service");
  ASSERT_TRUE(client->wait_for_service());

  // 1. Send save_map service request with default parameters
  req->map_topic = "";
  req->map_url = path(g_tmp_dir) / path(g_valid_pcd_map_name);

  // set view point translation
  req->view_point.translation.x = g_valid_view_point[0];
  req->view_point.translation.y = g_valid_view_point[1];
  req->view_point.translation.z = g_valid_view_point[2];

  // set view point rotation
  req->view_point.rotation.w = g_valid_view_point[3];
  req->view_point.rotation.x = g_valid_view_point[4];
  req->view_point.rotation.y = g_valid_view_point[5];
  req->view_point.rotation.z = g_valid_view_point[6];

  req->file_format = "";
  req->as_binary = false;

  auto resp = send_request<nav2_msgs::srv::SaveMap3D>(node_, client, req);
  ASSERT_EQ(resp->result, true);

  // 2. Load saved map and verify it
  nav2_msgs::msg::PCD2 map_msg;
  nav2_map_server_3D::LOAD_MAP_STATUS status =
      nav2_map_server_3D::loadMapFromYaml(
        path(g_tmp_dir) / path(g_valid_yaml_file),
        map_msg.map,
        map_msg.view_point);

  ASSERT_EQ(status, nav2_map_server_3D::LOAD_MAP_SUCCESS);
  verifyMapMsg(map_msg);
}

// Send map saving service requests with different sets of parameters.
// In case of map is expected to be saved correctly, load map from a saved
// file and verify obtained OccupancyGrid.
TEST_F(MapSaverTestFixture, SaveMapInvalidParameters)
{
  RCLCPP_INFO(node_->get_logger(), "Testing SaveMap service");
  auto req = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  auto client = node_->create_client<nav2_msgs::srv::SaveMap>(
    "/map_saver/save_map");

  RCLCPP_INFO(node_->get_logger(), "Waiting for save_map service");
  ASSERT_TRUE(client->wait_for_service());

  // 1. Trying to send save_map serivce request with different sets of parameters
  // In case of map is expected to be saved correctly, verify it
  req->map_topic = "invalid_map";
  req->map_url = path(g_tmp_dir) / path(g_valid_map_name);
  req->image_format = "png";
  req->map_mode = "trinary";
  req->free_thresh = g_default_free_thresh;
  req->occupied_thresh = g_default_occupied_thresh;
  auto resp = send_request<nav2_msgs::srv::SaveMap>(node_, client, req);
  ASSERT_EQ(resp->result, false);

  req->map_topic = "map";
  req->image_format = "invalid_format";
  resp = send_request<nav2_msgs::srv::SaveMap>(node_, client, req);
  ASSERT_EQ(resp->result, true);
  nav_msgs::msg::OccupancyGrid map_msg;
  LOAD_MAP_STATUS status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);
  verifyMapMsg(map_msg);

  req->image_format = "png";
  req->map_mode = "invalid_mode";
  resp = send_request<nav2_msgs::srv::SaveMap>(node_, client, req);
  ASSERT_EQ(resp->result, true);
  status = loadMapFromYaml(path(g_tmp_dir) / path(g_valid_yaml_file), map_msg);
  ASSERT_EQ(status, LOAD_MAP_SUCCESS);
  verifyMapMsg(map_msg);

  req->map_mode = "trinary";
  req->free_thresh = 2.0;
  req->occupied_thresh = 2.0;
  resp = send_request<nav2_msgs::srv::SaveMap>(node_, client, req);
  ASSERT_EQ(resp->result, false);

  req->free_thresh = -2.0;
  req->occupied_thresh = -2.0;
  resp = send_request<nav2_msgs::srv::SaveMap>(node_, client, req);
  ASSERT_EQ(resp->result, false);

  req->free_thresh = 0.7;
  req->occupied_thresh = 0.2;
  resp = send_request<nav2_msgs::srv::SaveMap>(node_, client, req);
  ASSERT_EQ(resp->result, false);
}

// TODO: Add more tests
