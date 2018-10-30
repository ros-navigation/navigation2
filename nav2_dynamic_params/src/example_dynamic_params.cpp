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

using namespace std::chrono_literals;

nav2_dynamic_params::DynamicParamsClient * dynamic_params_client;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("example_dynamic_params_client");

  // Set parameters on the node
  node->set_parameters({rclcpp::Parameter("foo", 1.0), rclcpp::Parameter("bar", 2)});

  // Add Dynamic Reconfigure Client
  dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node);
  dynamic_params_client->add_parameters({"foo", "bar", "foobar"});

  std::function<void(const rcl_interfaces::msg::ParameterEvent::SharedPtr)> callback = [node](
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      RCLCPP_INFO(node->get_logger(), "Event Callback!");

      if (dynamic_params_client->is_in_event(event, "foo")) {
        RCLCPP_INFO(node->get_logger(), "'foo' is in this event!");
      }

      double foo;
      dynamic_params_client->get_event_param(event, "foo", foo);
      RCLCPP_INFO(node->get_logger(), "foo: %f", foo);

      int bar;
      dynamic_params_client->get_event_param_or(event, "bar", bar, 4);
      RCLCPP_INFO(node->get_logger(), "bar: %d", bar);

      // Parameter not set on server
      double foobar;
      dynamic_params_client->get_event_param_or(event, "foobar", foobar, 5.5);
      RCLCPP_INFO(node->get_logger(), "foobar: %f", foobar);
    };

  dynamic_params_client->set_callback(callback);

  // Create DynamicParamsValidator
  auto param_validator = new nav2_dynamic_params::DynamicParamsValidator(node);
  param_validator->add_param("foo", rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator->add_param("bar", rclcpp::ParameterType::PARAMETER_INTEGER, {0, 10});

  // Create a custom validation callback
  std::function<rcl_interfaces::msg::SetParametersResult(
      const std::vector<rclcpp::Parameter> &)> custom_validation_callback = [node](
    const std::vector<rclcpp::Parameter> & parameters)
    -> rcl_interfaces::msg::SetParametersResult
    {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;

      for (const auto & parameter : parameters) {
        // Filter for parameter "foo"
        if (parameter.get_name() == "foo") {
          auto value = parameter.get_value<double>();
          // Reject any set requests between 10 & 20
          if (value > 10.0 && value < 20.0) {
            RCLCPP_INFO(node->get_logger(),
              "Parameter Change Denied::Failing Custom Validation: %s",
              parameter.get_name().c_str());
            result.successful = false;
            return result;
          }
        }
      }
      return result;
    };

  param_validator->set_validation_callback(custom_validation_callback);

  // Change Parameters
  RCLCPP_INFO(node->get_logger(), "1st Service Request:");
  node->set_parameters({rclcpp::Parameter("foo", 2.0), rclcpp::Parameter("bar", 3)});

  // Try to set illegal values
  RCLCPP_INFO(node->get_logger(), "2nd Service Request:");
  node->set_parameters({rclcpp::Parameter("foo", 1), rclcpp::Parameter("bar", 11)});

  // Add New Parameter
  RCLCPP_INFO(node->get_logger(), "3rd Service Request:");
  node->set_parameters({rclcpp::Parameter("foobar", 28.0)});
  param_validator->add_static_params({"foobar"});

  RCLCPP_INFO(node->get_logger(), "4th Service Request:");
  node->set_parameters({rclcpp::Parameter("foobar", 5.0)});

  RCLCPP_INFO(node->get_logger(), "5th Service Request:");
  node->set_parameters({rclcpp::Parameter("foo", 15.0)});

  rclcpp::spin(node);
  rclcpp::shutdown();

  delete dynamic_params_client;
  delete param_validator;

  return 0;
}
