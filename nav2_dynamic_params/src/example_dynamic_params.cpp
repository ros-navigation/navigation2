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

  // Add Parameters to Server
  node->set_parameters({rclcpp::Parameter("foo", 1.0), rclcpp::Parameter("bar", 2)});

  // Add Dynamic Reconfigure Client
  dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node);

  std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> callback = [node](
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params_client"), "Event Callback!");

      if (dynamic_params_client->is_in_event(event, "foo")) {
        RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params_client"),
          "'foo' is in this event!");
      }

      double foo;
      dynamic_params_client->get_event_param(event, "foo", foo);
      RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params_client"), "foo: %f", foo);

      int bar;
      dynamic_params_client->get_event_param_or(event, "bar", bar, 4);
      RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params_client"), "bar: %d", bar);

      // Parameter not set on server
      double foobar;
      dynamic_params_client->get_event_param_or(event, "foobar", foobar, 5.5);
      RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params_client"), "foobar: %f", foobar);
    };

  dynamic_params_client->set_callback(callback);

  // Create DynamicParamsValidator
  auto param_validator = new nav2_dynamic_params::DynamicParamsValidator(node);
  param_validator->add_param("foo", rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator->add_param("bar", rclcpp::ParameterType::PARAMETER_INTEGER, {0, 10});

  // Change Parameters
  RCLCPP_INFO(node->get_logger(), "First Service Request:");
  node->set_parameters({rclcpp::Parameter("foo", 2.0), rclcpp::Parameter("bar", 3)});

  // Try to set illegal values
  RCLCPP_INFO(node->get_logger(), "Second Service Request:");
  node->set_parameters({rclcpp::Parameter("foo", 1), rclcpp::Parameter("bar", 11)});

  // Add New Parameter
  RCLCPP_INFO(node->get_logger(), "Third Service Request:");
  node->set_parameters({rclcpp::Parameter("foobar", 28.0)});
  param_validator->add_static_params({"foobar"});

  RCLCPP_INFO(node->get_logger(), "Fourth Service Request:");
  node->set_parameters({rclcpp::Parameter("foobar", 5.0)});

  rclcpp::spin(node);
  rclcpp::shutdown();

  delete dynamic_params_client;
  delete param_validator;

  return 0;
}
