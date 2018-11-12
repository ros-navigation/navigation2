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
#include <vector>
#include "nav2_dynamic_params/dynamic_params_validator.hpp"
#include "rclcpp/rclcpp.hpp"

using rcl_interfaces::msg::SetParametersResult;
using namespace std::chrono_literals;

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

  auto node_A = rclcpp::Node::make_shared("example_node_A");
  auto node_B = rclcpp::Node::make_shared("example_node_B");
  auto node_C = rclcpp::Node::make_shared("example_node_C", "some_namespace");

  // Create DynamicParamsValidator
  auto param_validator_A = new nav2_dynamic_params::DynamicParamsValidator(node_A, true);
  param_validator_A->add_param("foo", rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator_A->set_validation_callback(
    std::bind(custom_validation_callback, std::placeholders::_1));

  auto param_validator_B = new nav2_dynamic_params::DynamicParamsValidator(node_B);
  param_validator_B->add_param("bar", rclcpp::ParameterType::PARAMETER_INTEGER, {0, 10});

  auto param_validator_C = new nav2_dynamic_params::DynamicParamsValidator(node_C, true);
  param_validator_C->add_param("bar", rclcpp::ParameterType::PARAMETER_INTEGER, {-25, 25});
  param_validator_C->add_param("baz", rclcpp::ParameterType::PARAMETER_STRING);

  // Set parameters on the node
  node_A->set_parameter_if_not_set("foo", 1.0);
  node_B->set_parameter_if_not_set("bar", 1);
  node_C->set_parameter_if_not_set("baz", "my_string");
  node_C->set_parameter_if_not_set("bar", -1);
  rclcpp::sleep_for(100ms);
  // Change Parameters
  RCLCPP_INFO(node_A->get_logger(), "1st Service Request:");
  node_A->set_parameters({rclcpp::Parameter("foo", 2.0)});
  node_B->set_parameters({rclcpp::Parameter("bar", 3)});
  node_C->set_parameters_atomically({
    rclcpp::Parameter("bar", 5),
    rclcpp::Parameter("baz", "my_new_string")
  });
  rclcpp::sleep_for(100ms);
  // Try to set illegal values
  RCLCPP_INFO(node_A->get_logger(), "2nd Service Request:");
  node_A->set_parameters({rclcpp::Parameter("foo", 1)});
  node_A->set_parameters({rclcpp::Parameter("foobar", 28.0)});
  node_B->set_parameters({rclcpp::Parameter("bar", 11)});
  rclcpp::sleep_for(100ms);
  // Set new parameter on B
  RCLCPP_INFO(node_A->get_logger(), "3rd Service Request:");
  node_B->set_parameters({rclcpp::Parameter("foobar", 28.0)});
  rclcpp::sleep_for(100ms);
  // Make "foobar" a static parameter & attempt to set again
  param_validator_B->add_static_params({"foobar"});
  RCLCPP_INFO(node_A->get_logger(), "4th Service Request:");
  node_B->set_parameters({rclcpp::Parameter("foobar", 5.0)});
  rclcpp::sleep_for(100ms);
  // Try to set "foo" to illegal value from custom validation callback
  RCLCPP_INFO(node_A->get_logger(), "5th Service Request:");
  node_A->set_parameters({rclcpp::Parameter("foo", 15.0)});
  rclcpp::sleep_for(100ms);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node_A);
  exec.add_node(node_B);
  exec.add_node(node_C);
  exec.spin();
  rclcpp::shutdown();

  delete param_validator_A;
  delete param_validator_B;
  delete param_validator_C;

  return 0;
}
