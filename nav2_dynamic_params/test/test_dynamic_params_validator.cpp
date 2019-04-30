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
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_dynamic_params/dynamic_params_validator.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"

using rcl_interfaces::msg::SetParametersResult;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};

RclCppFixture g_rclcppfixture;

// Define a custom validation callback
SetParametersResult custom_validation_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = SetParametersResult();
  result.successful = true;

  for (const auto & parameter : parameters) {
    // Filter for parameter "foo"
    if (parameter.get_name() == "custom_param") {
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

class ValidatorTest : public ::testing::Test
{
public:
  ValidatorTest()
  {
    node_ = rclcpp::Node::make_shared(
      "dynamic_param_validator_test", nav2_util::get_node_options_default());
    param_validator_ = std::make_unique<nav2_dynamic_params::DynamicParamsValidator>(node_);
  }

protected:
  std::unique_ptr<nav2_dynamic_params::DynamicParamsValidator> param_validator_;
  rclcpp::Node::SharedPtr node_;
};


TEST_F(ValidatorTest, testValidType)
{
  param_validator_->add_param("double", rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator_->add_param("int", rclcpp::ParameterType::PARAMETER_INTEGER);
  param_validator_->add_param("string", rclcpp::ParameterType::PARAMETER_STRING);
  param_validator_->add_param("bool", rclcpp::ParameterType::PARAMETER_BOOL);
  param_validator_->add_param("double_array", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  param_validator_->add_param("int_array", rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
  param_validator_->add_param("string_array", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  param_validator_->add_param("bool_array", rclcpp::ParameterType::PARAMETER_BOOL_ARRAY);

  // Set valid type values
  auto valid_results = node_->set_parameters({
    rclcpp::Parameter("double", 1.0),
    rclcpp::Parameter("int", 1),
    rclcpp::Parameter("string", "test"),
    rclcpp::Parameter("bool", true),
    rclcpp::Parameter("double_array", std::vector<double>({1.0, 2.0})),
    rclcpp::Parameter("int_array", std::vector<int>({1, 2})),
    rclcpp::Parameter("string_array", std::vector<std::string>({"test_1", "test_2"})),
    rclcpp::Parameter("bool_array", std::vector<bool>({true, false}))
  });

  // Set invalid type values
  auto invalid_results = node_->set_parameters({
    rclcpp::Parameter("double", 1),
    rclcpp::Parameter("int", 1.0),
    rclcpp::Parameter("string", 1),
    rclcpp::Parameter("bool", 1),
    rclcpp::Parameter("double_array", 1),
    rclcpp::Parameter("int_array", 1),
    rclcpp::Parameter("string_array", 1),
    rclcpp::Parameter("bool_array", 1),
  });

  for (auto result : valid_results) {
    EXPECT_EQ(true, result.successful);
  }
  for (auto result : invalid_results) {
    EXPECT_EQ(false, result.successful);
  }
}

TEST_F(ValidatorTest, testValidRange)
{
  param_validator_->add_param(
    "double_bound", rclcpp::ParameterType::PARAMETER_DOUBLE, {0.0, 10.0});
  param_validator_->add_param(
    "double_low_bound", rclcpp::ParameterType::PARAMETER_DOUBLE, {0.0, 10.0}, 0);
  param_validator_->add_param(
    "double_high_bound", rclcpp::ParameterType::PARAMETER_DOUBLE, {0.0, 10.0}, 1);
  param_validator_->add_param(
    "int_bound", rclcpp::ParameterType::PARAMETER_INTEGER, {0, 10});
  param_validator_->add_param(
    "int_low_bound", rclcpp::ParameterType::PARAMETER_INTEGER, {0, 10}, 0);
  param_validator_->add_param(
    "int_high_bound", rclcpp::ParameterType::PARAMETER_INTEGER, {0, 10}, 1);
  param_validator_->add_param(
    "string_invalid_bound", rclcpp::ParameterType::PARAMETER_STRING, {0, 10});

  auto valid_results = node_->set_parameters({
    rclcpp::Parameter("double_bound", 5.0),
    rclcpp::Parameter("double_low_bound", -100.0),
    rclcpp::Parameter("double_high_bound", 100.0),
    rclcpp::Parameter("int_bound", 5),
    rclcpp::Parameter("int_low_bound", -100),
    rclcpp::Parameter("int_high_bound", 100),
  });

  // Set invalid type values
  auto invalid_results = node_->set_parameters({
    rclcpp::Parameter("double_bound", 11.0),
    rclcpp::Parameter("double_low_bound", 11.0),
    rclcpp::Parameter("double_high_bound", -1.0),
    rclcpp::Parameter("int_bound", 11),
    rclcpp::Parameter("int_low_bound", 11),
    rclcpp::Parameter("int_high_bound", -1),
    rclcpp::Parameter("string_invalid_bound", "test"),
  });

  for (auto result : valid_results) {
    EXPECT_EQ(true, result.successful);
  }
  for (auto result : invalid_results) {
    EXPECT_EQ(false, result.successful);
  }
}

TEST_F(ValidatorTest, testStaticParams)
{
  param_validator_->add_param(
    "static_param", rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator_->add_param(
    "non_static_param", rclcpp::ParameterType::PARAMETER_DOUBLE);

  param_validator_->add_static_params({"static_param"});

  auto invalid_result = node_->set_parameters_atomically(
    {rclcpp::Parameter("static_param", 1.0)});
  auto valid_result = node_->set_parameters_atomically(
    {rclcpp::Parameter("non_static_param", 1.0)});

  EXPECT_EQ(false, invalid_result.successful);
  EXPECT_EQ(true, valid_result.successful);
}

TEST_F(ValidatorTest, testCustomValidation)
{
  param_validator_->add_param("param", rclcpp::ParameterType::PARAMETER_DOUBLE);
  param_validator_->add_param("custom_param", rclcpp::ParameterType::PARAMETER_DOUBLE);

  param_validator_->set_validation_callback(
    std::bind(custom_validation_callback, std::placeholders::_1));

  auto valid_result = node_->set_parameters_atomically(
    {rclcpp::Parameter("param", 15.0)});
  auto valid_result_custom = node_->set_parameters_atomically(
    {rclcpp::Parameter("custom_param", 10.0)});
  auto invalid_result = node_->set_parameters_atomically(
    {rclcpp::Parameter("custom_param", 15.0)});

  EXPECT_EQ(false, invalid_result.successful);
  EXPECT_EQ(true, valid_result_custom.successful);
  EXPECT_EQ(true, valid_result.successful);
}
