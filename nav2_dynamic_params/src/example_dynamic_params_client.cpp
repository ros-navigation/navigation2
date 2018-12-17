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
#include "nav2_dynamic_params/dynamic_params_client.hpp"
#include "rclcpp/rclcpp.hpp"

nav2_dynamic_params::DynamicParamsClient * dynamic_params_client;

// Define a user event callback
void event_callback()
{
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "\nEvent Callback!");

  if (dynamic_params_client->is_in_event("foo")) {
    RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "'foo' is in this event!");
  }

  if (dynamic_params_client->is_in_event("example_node_B", "bar")) {
    RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"),
      "'example_node_B/bar' is in this event!");
  }

  double foo;
  dynamic_params_client->get_event_param("example_node_A", "foo", foo);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "foo: %f", foo);

  int bar_B;
  dynamic_params_client->get_event_param_or("example_node_B", "bar", bar_B, 2);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "bar_B: %d", bar_B);

  int bar_C;
  dynamic_params_client->get_event_param_or("some_namespace/example_node_C", "bar", bar_C, 3);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "bar_C: %d", bar_C);

  std::string baz;
  dynamic_params_client->get_event_param_or("some_namespace", "example_node_C",
    "baz", baz, std::string("default"));
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "baz: %s", baz.c_str());

  // Parameter not set on node
  double foobar;
  dynamic_params_client->get_event_param_or("foobar", foobar, 5.5);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "foobar: %f", foobar);

  int foobaz;
  dynamic_params_client->get_event_param_or("foobaz", foobaz, 25);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "foobaz: %d", foobaz);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("example_dynamic_params_client", "some_other_namespace");
  node->set_parameter_if_not_set("foobaz", 50);

  // Add Dynamic Reconfigure Client
  dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node);
  // Add parameters by node. Note that there are different ways to add parameters
  // The namespace must be provided, if applicable
  dynamic_params_client->add_parameters("example_node_A", {"foo"});
  // If node is not available for service, then none of its parameters will be registered
  dynamic_params_client->add_parameters_on_node("example_node_B");
  dynamic_params_client->add_parameters("some_namespace", "example_node_C", {"baz", "bar"});
  // without node path, adding only parameters will grab parameters from member node
  dynamic_params_client->add_parameters({"foobar", "foobaz"});

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
