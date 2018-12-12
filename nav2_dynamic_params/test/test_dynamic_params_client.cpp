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
    params_client_ = std::make_unique<nav2_dynamic_params::DynamicParamsClient>(node_);
  }

protected:
  std::unique_ptr<nav2_dynamic_params::DynamicParamsClient> params_client_;
  rclcpp::Node::SharedPtr node_;
};

TEST_F(ClientTest, testAddParameters)
{
  //auto node_A = rclcpp::Node::make_shared("node_A");
  //auto node_B = rclcpp::Node::make_shared("node_B", "namespace");
  //std::map<std::string, rclcpp::Parameter> params;

  //params["/node_A/foo"] = rclcpp::Parameter("foo", 1.0);
  //params["/namespace/node_B/bar"] = rclcpp::Parameter("bar", 1);
  //params["/namespace/node_B/foobar"] = rclcpp::Parameter("foobar", 1);
  //node_A->set_parameters({params["/node_A/foo"]});
  //node_B->set_parameters({params["/namespace/node_B/bar"]});

  //node_B->set_parameters({rclcpp::Parameter("foobar", 1)});
  
/*   rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_A);
  exec.add_node(node_B);
  exec.spin(); */
  //node_B->set_parameters({rclcpp::Parameter("foobar", 1)});
  node_->set_parameters({rclcpp::Parameter("baz", 1)});

  params_client_->add_parameters({"baz"});
  //params_client_->add_parameters("node_A", {"foo"});
  //params_client_->add_parameters("namespace", "node_B", {"bar"});
  //params_client_->add_parameters_on_node("namespace", "node_B");

  auto dynamic_param_map = params_client_->get_param_map();
  EXPECT_EQ(1, dynamic_param_map.count("/baz"));
}

/* 
TEST_F(ClientTest, testGetParameters)
{

}

TEST_F(ClientTest, testDifferentNamespaces)
{

}

TEST_F(ClientTest, testDuplicateParams)
{

} */

// Define a user event callback
/* void event_callback()
{
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "\nEvent Callback!");

  if (dynamic_params_client->is_in_event("foo")) {
    RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "'foo' is in this event!");
  }

  double foo;
  dynamic_params_client->get_event_param("foo", foo);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "foo: %f", foo);

  int bar_B;
  dynamic_params_client->get_event_param_or("bar", bar_B, 2);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "bar_B: %d", bar_B);

  int bar_C;
  dynamic_params_client->get_event_param_or("some_namespace", "bar", bar_C, 3);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "bar_C: %d", bar_C);

  std::string baz;
  dynamic_params_client->get_event_param_or("some_namespace/baz", baz, std::string("default"));
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "baz: %s", baz.c_str());

  // Parameter not set on server
  double foobar;
  dynamic_params_client->get_event_param_or("foobar", foobar, 5.5);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "foobar: %f", foobar);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("example_dynamic_params_client", "some_other_namespace");

  // Add Dynamic Reconfigure Client
  dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node);
  // Add parameters by node. Note that there are different ways to add parameters
  // The namespace must be provided, if applicable
  dynamic_params_client->add_parameters("example_node_A", {"foo"});
  // If node is not available for service, then none of its parameters will be registered
  dynamic_params_client->add_parameters_on_node("example_node_B");
  dynamic_params_client->add_parameters("some_namespace", "example_node_C", {"baz", "bar"});
  // without node path, adding only parameters will grab parameters from member node
  dynamic_params_client->add_parameters({"foobaz"});

  dynamic_params_client->set_callback(std::bind(event_callback));

  // Check list of parameters
  auto list = dynamic_params_client->get_param_names();
  std::stringstream ss;
  for (auto & param_name : list) {
    ss << "\n" << param_name;
  }

  RCLCPP_INFO(node->get_logger(), ss.str().c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();

  delete dynamic_params_client;
  return 0;
}
 */