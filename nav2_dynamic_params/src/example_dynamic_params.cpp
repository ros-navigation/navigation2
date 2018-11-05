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
#include <vector>
#include "nav2_dynamic_params/dynamic_params_client.hpp"
#include "nav2_dynamic_params/dynamic_params_validator.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sstream>

using namespace std::chrono_literals;
using rcl_interfaces::msg::SetParametersResult;

nav2_dynamic_params::DynamicParamsClient * dynamic_params_client;

// Define a user event callback
void event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "Event Callback!");

  if (dynamic_params_client->is_in_event(event, "foo")) {
    RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "'foo' is in this event!");
  }

  double foo;
  dynamic_params_client->get_event_param("foo", foo);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "foo: %f", foo);

  int bar;
  dynamic_params_client->get_event_param_or("bar", bar, 4);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "bar: %d", bar);

  // Parameter not set on server
  double foobar;
  dynamic_params_client->get_event_param_or("foobar", foobar, 5.5);
  RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"), "foobar: %f", foobar);
}

// Define a custom validation callback
SetParametersResult custom_validation_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = SetParametersResult();
  result.successful = true;

  for (const auto & parameter : parameters) {
    // Filter for parameter "foo"
    if (parameter.get_name() == "foo") {
      auto value = parameter.get_value<double>();
      // Reject any set requests between 10 & 20
      if (value > 10.0 && value < 20.0) {
        RCLCPP_INFO(rclcpp::get_logger("example_dynamic_params"),
          "Parameter Change Denied::Failing Custom Validation: %s",
          parameter.get_name().c_str());
        result.successful = false;
        return result;
      }
    }
  }
  return result;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("example_dynamic_params");

  // Set parameters on the node
  node->set_parameters({rclcpp::Parameter("foo", 1.0), rclcpp::Parameter("bar", 2)});

  // Add Dynamic Reconfigure Client
  dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node);
  //dynamic_params_client->add_parameters({"foo", "bar"});
  //dynamic_params_client->add_parameters("foobar");
  dynamic_params_client->add_parameters("", {"foo", "bar", "foobar"});

  dynamic_params_client->set_callback(std::bind(event_callback, std::placeholders::_1));


  auto list = dynamic_params_client->get_param_names();

  std::stringstream ss;
  for (auto & param_name : list)
  {
    ss << "\n" << param_name;
  }
  // Create DynamicParamsValidator
  auto param_validator = new nav2_dynamic_params::DynamicParamsValidator(node);
  param_validator->add_param("foo", rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator->add_param("bar", rclcpp::ParameterType::PARAMETER_INTEGER, {0, 10});
  param_validator->set_validation_callback(
    std::bind(custom_validation_callback, std::placeholders::_1));

  // Change Parameters
  RCLCPP_INFO(node->get_logger(), "1st Service Request:");
  node->set_parameters({rclcpp::Parameter("foo", 2.0), rclcpp::Parameter("bar", 3)});

  // Try to set illegal values
  RCLCPP_INFO(node->get_logger(), "2nd Service Request:");
  node->set_parameters({rclcpp::Parameter("foo", 1), rclcpp::Parameter("bar", 11)});

  // Add New Parameter to node
  RCLCPP_INFO(node->get_logger(), "3rd Service Request:");
  node->set_parameters({rclcpp::Parameter("foobar", 28.0)});

  // Make "foobar" a static parameter & attempt to set again
  param_validator->add_static_params({"foobar"});
  RCLCPP_INFO(node->get_logger(), "4th Service Request:");
  node->set_parameters({rclcpp::Parameter("foobar", 5.0)});

  // Try to set "foo" to illegal value from custom validation callback
  RCLCPP_INFO(node->get_logger(), "5th Service Request:");
  node->set_parameters({rclcpp::Parameter("foo", 15.0)});

  rclcpp::spin(node);
  rclcpp::shutdown();

  delete dynamic_params_client;
  delete param_validator;

  return 0;
}
