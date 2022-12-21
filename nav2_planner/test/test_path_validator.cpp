// Copyright (c) 2022 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.


#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_planner/path_validator.hpp"

using PathValidator = nav2_planner::PathValidator;
using Costs = nav2_planner::Costs;

class RclCppFixture
{
 public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

TEST(PathValidatorTest, is_cost_change_signifant)
{
  PathValidator path_validator;

  // free space test
  Costs original_costs(30);
  std::fill(original_costs.begin(), original_costs.end(), 0);

  Costs current_costs(30);
  std::fill( current_costs.begin(), current_costs.end(), 0);
  float z_score = 2.55;

  bool is_cost_change_significant =
      path_validator.isCostChangeSignifant(original_costs, current_costs, z_score);

  EXPECT_TRUE(is_cost_change_significant);
}
