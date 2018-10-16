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

#include "nav2_dynamic_params/dynamic_params_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sstream>

using namespace std::chrono_literals;

nav2_dynamic_params::DynamicParamsClient * dynamic_params_client;

void on_parameter_event(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event, rclcpp::Logger logger)
{
  // TODO(wjwwood): The message should have an operator<<, which would replace all of this.
  std::stringstream ss;
  ss << "\nParameter event:\n new parameters:";
  for (auto & new_parameter : event->new_parameters) {
    ss << "\n  " << new_parameter.name;
  }
  ss << "\n changed parameters:";
  for (auto & changed_parameter : event->changed_parameters) {
    ss << "\n  " << changed_parameter.name;
  }
  ss << "\n deleted parameters:";
  for (auto & deleted_parameter : event->deleted_parameters) {
    ss << "\n  " << deleted_parameter.name;
  }
  ss << "\n";
  RCLCPP_INFO(logger, ss.str().c_str());
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_dynamic_params_client");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  dynamic_params_client = new nav2_dynamic_params::DynamicParamsClient(node->get_name(), node);



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
    double foo;
    dynamic_params_client->get_event_param(event, "foo", foo);
    RCLCPP_INFO(rclcpp::get_logger("NUT"), "foo: %f", foo);
    
    double bar; // Default Value
    dynamic_params_client->get_event_param(event, "bar", bar, 5.0);
    RCLCPP_INFO(rclcpp::get_logger("NUT"), "bar: %f", bar);

    on_parameter_event(event, node->get_logger());
  }); 

  // Add new Parameter
  node->set_parameters({rclcpp::Parameter("foo", 1.0)});
  
  // Add another new parameter
  //node->set_parameters({rclcpp::Parameter("bar", 2.0)});

  // Change Parameter
  node->set_parameters({rclcpp::Parameter("foo", 3.0)});


  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}