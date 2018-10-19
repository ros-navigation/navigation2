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
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "nav2_map_server/map_representations/occ_grid_server.hpp"
#include "test_constants/test_constants.h"

// TODO(bpwilcox): test publisher

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
  }

  template<class T>
  typename T::Response::SharedPtr send_request(

    rclcpp::Node::SharedPtr node,
    typename rclcpp::Client<T>::SharedPtr client,
    typename T::Request::SharedPtr request)
  {
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      return result.get();
    } else {
      return NULL;
    }
  }

protected:
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TestNode, ResultReturned)
{
  auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto client = node_->create_client<nav_msgs::srv::GetMap>("occ_grid");
  ASSERT_TRUE(client->wait_for_service());
  auto resp = send_request<nav_msgs::srv::GetMap>(node_, client, req);
  ASSERT_FLOAT_EQ(resp->map.info.resolution, g_valid_image_res);
  ASSERT_EQ(resp->map.info.width, g_valid_image_width);
  ASSERT_EQ(resp->map.info.height, g_valid_image_height);
  for (unsigned int i = 0; i < resp->map.info.width * resp->map.info.height; i++) {
    ASSERT_EQ(g_valid_image_content[i], resp->map.data[i]);
  }
}
