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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node_A = rclcpp::Node::make_shared("test_node");
  auto node_B = rclcpp::Node::make_shared("test_node", "test_namespace");

  node_A->set_parameters({rclcpp::Parameter("foo", 1.0)});
  node_B->set_parameters({rclcpp::Parameter("bar", 1)});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_A);
  exec.add_node(node_B);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
