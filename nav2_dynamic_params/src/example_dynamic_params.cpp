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
#include "nav2_dynamic_params/dynamic_params_client.hpp"
#include "nav2_dynamic_params/dynamic_params_validator.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

nav2_dynamic_params::DynamicParamsClient * dynamic_params_client;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("example_dynamic_params_client");

  // Create Parameter Client to Node
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);

  // Add Parameters to Server
  node->set_parameters({rclcpp::Parameter("foo", 1.0), rclcpp::Parameter("bar", 2)});

  // Create DynamicParamsClient
  dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(parameters_client);
  dynamic_params_client->addParametersFromServer({"foo", "bar"});

  // Create DynamicParamsValidator
  auto param_validator_ = new nav2_dynamic_params::DynamicParamsValidator(node);
  param_validator_->add_param("foo", rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator_->add_param("bar", rclcpp::ParameterType::PARAMETER_INTEGER, {0, 10});

  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto sub = parameters_client->on_parameter_event(
    [node](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params_client"), "Event Callback!");

      double foo;
      dynamic_params_client->get_event_param(event, "foo", foo);
      RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params_client"), "foo: %f", foo);

      int bar;
      dynamic_params_client->get_event_param(event, "bar", bar, 4);
      RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params_client"), "bar: %d", bar);

      // Parameter not set on server
      double foobar;
      dynamic_params_client->get_event_param(event, "foobar", foobar, 5.5);
      RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params_client"), "foobar: %f", foobar);
    });

  // Change Parameters
  RCLCPP_INFO(node->get_logger(), "First Service Request:");
  node->set_parameters({rclcpp::Parameter("foo", 2.0), rclcpp::Parameter("bar", 3)});

  // Try to set illegal values
  RCLCPP_INFO(node->get_logger(), "Second Service Request:");
  node->set_parameters({rclcpp::Parameter("foo", 1), rclcpp::Parameter("bar", 11)});

  // Add New Parameter
  RCLCPP_INFO(node->get_logger(), "Third Service Request:");
  node->set_parameters({rclcpp::Parameter("foobar", 28.0)});

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
