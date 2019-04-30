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
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node_A = rclcpp_lifecycle::LifecycleNode::make_shared(
    "test_node", nav2_util::get_node_options_default());
  auto node_B = rclcpp_lifecycle::LifecycleNode::make_shared(
    "test_node", "test_namespace", nav2_util::get_node_options_default());

  node_A->set_parameters({rclcpp::Parameter("foo", 1.0)});
  node_B->set_parameters({rclcpp::Parameter("bar", 1)});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_A->get_node_base_interface());
  exec.add_node(node_B->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
