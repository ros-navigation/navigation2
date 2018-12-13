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

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_dynamic_params/dynamic_params_client.hpp"
#include "rclcpp/rclcpp.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

class ClientTest : public ::testing::Test
{
public:
  ClientTest()
  {
    node_ = rclcpp::Node::make_shared("dynamic_param_client_test");
    dynamic_params_client_ = std::make_unique<nav2_dynamic_params::DynamicParamsClient>(node_);
  }

protected:
  std::unique_ptr<nav2_dynamic_params::DynamicParamsClient> dynamic_params_client_;
  rclcpp::Node::SharedPtr node_;
  bool callback_result_ = false;
};

TEST_F(ClientTest, testAddParameters)
{
  node_->set_parameters({rclcpp::Parameter("baz", 1)});

  dynamic_params_client_->add_parameters();
  dynamic_params_client_->add_parameters({"foobar"});
  
  auto dynamic_param_map = dynamic_params_client_->get_param_map();
  EXPECT_EQ(1, dynamic_param_map.count("/dynamic_param_client_test/baz"));
  EXPECT_EQ(1, dynamic_param_map.count("/dynamic_param_client_test/foobar"));
}

 
TEST_F(ClientTest, testAddParamsOtherNodes)
{
  dynamic_params_client_->add_parameters_on_node("test_namespace", "test_node");
  dynamic_params_client_->add_parameters("test_namespace", "test_node", {"foobar"});
  dynamic_params_client_->add_parameters("test_node", {"foo"});

  auto dynamic_param_map = dynamic_params_client_->get_param_map();
  EXPECT_EQ(1, dynamic_param_map.count("/test_node/foo"));
  EXPECT_EQ(1, dynamic_param_map.count("/test_namespace/test_node/bar"));
  EXPECT_EQ(1, dynamic_param_map.count("/test_namespace/test_node/foobar"));  
}


TEST_F(ClientTest, testGetParams)
{
  node_->set_parameters({rclcpp::Parameter("baz", 5)});
  node_->set_parameters({rclcpp::Parameter("foobaz", 5.0)});

  dynamic_params_client_->add_parameters_on_node("test_namespace", "test_node");
  dynamic_params_client_->add_parameters("test_namespace", "test_node", {"foobar"});
  dynamic_params_client_->add_parameters("test_node", {"foo"});
  dynamic_params_client_->add_parameters();
  dynamic_params_client_->add_parameters({"foobaz"});
  dynamic_params_client_->add_parameters("some_node", {"barbaz"});

  double foo, foobaz;
  int bar, baz, barbaz;
  std::string foobar;

  dynamic_params_client_->get_event_param("baz", baz);
  dynamic_params_client_->get_event_param("test_namespace", "test_node", "bar", bar);
  dynamic_params_client_->get_event_param("test_node", "foo", foo);

  EXPECT_EQ(5, baz);
  EXPECT_EQ(1.0, foo);
  EXPECT_EQ(1, bar);

  auto result  = dynamic_params_client_->get_event_param_or<std::string>("test_namespace/test_node", "foobar", foobar, "test");
  EXPECT_EQ(false, result);
  result = dynamic_params_client_->get_event_param_or("foobaz", foobaz, 7.0);
  EXPECT_EQ(true, result);
  result = dynamic_params_client_->get_event_param("some_node", "barbaz", barbaz);
  EXPECT_EQ(false, result);
  EXPECT_EQ("test", foobar);
  EXPECT_EQ(5.0, foobaz);
  
}

TEST_F(ClientTest, testEventCallbacks)
{
  dynamic_params_client_->add_parameters({"baz"});
  dynamic_params_client_->add_parameters("test_node", {"foo"});

  std::function<void()> callback = [this]()-> void
    {
      callback_result_ = true;
    };

  dynamic_params_client_->set_callback(callback, false);
  
  node_->set_parameters({rclcpp::Parameter("baz", 2)});

  while(!callback_result_)
  {
    rclcpp::spin_some(node_);
  }

  EXPECT_EQ(true, callback_result_);

  auto param_client_A = std::make_shared<rclcpp::SyncParametersClient>(node_, "/test_node");
  auto param_client_B = std::make_shared<rclcpp::SyncParametersClient>(node_, "/test_namespace/test_node");

  callback_result_ = false;
  param_client_A->set_parameters({rclcpp::Parameter("foo", 3.0)});
/*   while(!callback_result_)
  {
    rclcpp::spin_some(node_);
  } */
  rclcpp::Rate r(10);
  rclcpp::executors::SingleThreadedExecutor exec;
  while (!callback_result_ && rclcpp::ok()) {
    exec.spin_node_once(node_->get_node_base_interface(), std::chrono::milliseconds(100));
    r.sleep();
  }

  EXPECT_EQ(true, callback_result_);

}
