// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include <math.h>

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_planner/planner_server.hpp"
#include "rclcpp/rclcpp.hpp"

class PlannerShim : public nav2_planner::PlannerServer
{
public:
  PlannerShim()
  : nav2_planner::PlannerServer(rclcpp::NodeOptions())
  {
  }

  // Since we cannot call configure/activate due to costmaps
  // requiring TF
  void setDynamicCallback()
  {
    auto node = shared_from_this();
    // Add callback for dynamic parameters
    dyn_params_handler_ = node->add_on_set_parameters_callback(
      std::bind(&PlannerShim::dynamicParamsShim, this, std::placeholders::_1));
  }

  rcl_interfaces::msg::SetParametersResult
  dynamicParamsShim(std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    dynamicParametersCallback(parameters);
    return result;
  }
};

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(WPTest, test_dynamic_parameters)
{
  auto planner = std::make_shared<PlannerShim>();
  planner->setDynamicCallback();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    planner->get_node_base_interface(), planner->get_node_topics_interface(),
    planner->get_node_graph_interface(),
    planner->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("expected_planner_frequency", 100.0)});

  rclcpp::spin_until_future_complete(
    planner->get_node_base_interface(),
    results);

  EXPECT_EQ(planner->get_parameter("expected_planner_frequency").as_double(), 100.0);

  // test edge case for = 0
  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("expected_planner_frequency", -1.0)});

  rclcpp::spin_until_future_complete(
    planner->get_node_base_interface(),
    results);

  EXPECT_EQ(planner->get_parameter("expected_planner_frequency").as_double(), -1.0);
}
