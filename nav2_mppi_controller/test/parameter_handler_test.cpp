// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"

// Tests parameter handler object

using namespace mppi;  // NOLINT

class ParametersHandlerWrapper : public ParametersHandler
{
public:
  ParametersHandlerWrapper() = default;

  explicit ParametersHandlerWrapper(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
  : ParametersHandler(parent, name) {}

  template<typename T>
  auto asWrapped(rclcpp::Parameter parameter)
  {
    return ParametersHandler::as<T>(parameter);
  }
};

using namespace mppi;  // NOLINT

TEST(ParameterHandlerTest, asTypeConversionTest)
{
  ParametersHandlerWrapper a;
  rclcpp::Parameter int_p("test.int_parameter", rclcpp::ParameterValue(1));
  rclcpp::Parameter double_p("test.double_parameter", rclcpp::ParameterValue(10.0));
  rclcpp::Parameter bool_p("test.bool_parameter", rclcpp::ParameterValue(false));
  rclcpp::Parameter string_p("test.string_parameter", rclcpp::ParameterValue(std::string("hello")));

  rclcpp::Parameter intv_p("test.intv_parameter", rclcpp::ParameterValue(std::vector<int>{1}));
  rclcpp::Parameter doublev_p(
    "test.doublev_parameter", rclcpp::ParameterValue(std::vector<double>{10.0}));
  rclcpp::Parameter boolv_p("test.boolv_parameter",
    rclcpp::ParameterValue(std::vector<bool>{false}));
  rclcpp::Parameter stringv_p(
    "test.stringv_parameter",
    rclcpp::ParameterValue(std::vector<std::string>{std::string("hello")}));

  EXPECT_EQ(a.asWrapped<int>(int_p), 1);
  EXPECT_EQ(a.asWrapped<double>(double_p), 10.0);
  EXPECT_EQ(a.asWrapped<bool>(bool_p), false);
  EXPECT_EQ(a.asWrapped<std::string>(string_p), std::string("hello"));

  EXPECT_EQ(a.asWrapped<std::vector<int64_t>>(intv_p)[0], 1);
  EXPECT_EQ(a.asWrapped<std::vector<double>>(doublev_p)[0], 10.0);
  EXPECT_EQ(a.asWrapped<std::vector<bool>>(boolv_p)[0], false);
  EXPECT_EQ(a.asWrapped<std::vector<std::string>>(stringv_p)[0], std::string("hello"));
}

TEST(ParameterHandlerTest, PrePostDynamicCallbackTest)
{
  bool pre_triggered = false, post_triggered = false, dynamic_triggered = false;
  auto preCb = [&]() {
      if (post_triggered) {
        throw std::runtime_error("Post-callback triggered before pre-callback!");
      }
      pre_triggered = true;
    };

  auto postCb = [&]() {
      if (!pre_triggered) {
        throw std::runtime_error("Pre-callback was not triggered before post-callback!");
      }
      post_triggered = true;
    };

  auto dynamicCb = [&](const rclcpp::Parameter & /*param*/,
      rcl_interfaces::msg::SetParametersResult & /*result*/) {
      dynamic_triggered = true;
    };

  rclcpp::Parameter random_param(".blah_blah", rclcpp::ParameterValue(true));
  rclcpp::Parameter random_param2(".use_sim_time", rclcpp::ParameterValue(true));
  bool val = false;

  ParametersHandlerWrapper a;
  a.addPreCallback(preCb);
  a.addPostCallback(postCb);
  a.addParamCallback(".use_sim_time", dynamicCb);
  a.setParamCallback(val, ".blah_blah");

  // Dynamic callback should not trigger, wrong parameter, but val should be updated
  a.dynamicParamsCallback(std::vector<rclcpp::Parameter>{random_param});
  EXPECT_FALSE(dynamic_triggered);
  EXPECT_TRUE(pre_triggered);
  EXPECT_TRUE(post_triggered);
  EXPECT_TRUE(val);

  // Now dynamic parameter bool should be updated, right param called!
  pre_triggered = false, post_triggered = false;
  a.dynamicParamsCallback(std::vector<rclcpp::Parameter>{random_param2});
  EXPECT_TRUE(dynamic_triggered);
  EXPECT_TRUE(pre_triggered);
  EXPECT_TRUE(post_triggered);
}

TEST(ParameterHandlerTest, GetSystemParamsTest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  std::string name = "test";
  node->declare_parameter("param1", rclcpp::ParameterValue(true));
  node->declare_parameter(name + ".param2", rclcpp::ParameterValue(7));

  // Get parameters in global namespace and in subnamespaces
  ParametersHandler handler(node, name);
  auto getParameter = handler.getParamGetter("");
  bool p1 = false;
  int p2 = 0;
  getParameter(p1, "param1", false);
  getParameter(p2, name + ".param2", 0);
  EXPECT_EQ(p1, true);
  EXPECT_EQ(p2, 7);

  // Get parameters in subnamespaces using name semantics of getter
  auto getParameter2 = handler.getParamGetter(name);
  p2 = 0;
  getParameter2(p2, "param2", 0);
  EXPECT_EQ(p2, 7);
}

TEST(ParameterHandlerTest, DynamicAndStaticParametersTest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");

  node->declare_parameter("test.dynamic_int", rclcpp::ParameterValue(7));
  node->declare_parameter("test.static_int", rclcpp::ParameterValue(7));
  std::string name = "test";
  ParametersHandlerWrapper handler(node, name);
  handler.start();

  // Get parameters and check they have initial values
  auto getParameter = handler.getParamGetter("");
  int p1 = 0, p2 = 0;
  getParameter(p1, "test.dynamic_int", 0, ParameterType::Dynamic);
  getParameter(p2, "test.static_int", 0, ParameterType::Static);
  EXPECT_EQ(p1, 7);
  EXPECT_EQ(p2, 7);

  // Now change them both via dynamic parameters
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  std::shared_future<rcl_interfaces::msg::SetParametersResult> result_future =
    rec_param->set_parameters_atomically(
  {
    rclcpp::Parameter("my_node.verbose", true),
    rclcpp::Parameter("test.dynamic_int", 10),
    rclcpp::Parameter("test.static_int", 10)
  });

  auto rc = rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    result_future);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, rc);

  auto result = result_future.get();
  EXPECT_EQ(result.successful, false);
  EXPECT_FALSE(result.reason.empty());
  EXPECT_EQ(
    result.reason, std::string("Rejected change to static parameter: ") +
    "{\"name\": \"test.static_int\", \"type\": \"integer\", \"value\": \"10\"}");

  // Now, only param1 should change, param 2 should be the same
  EXPECT_EQ(p1, 10);
  EXPECT_EQ(p2, 7);
}

TEST(ParameterHandlerTest, DynamicAndStaticParametersNotVerboseTest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  node->declare_parameter("test.dynamic_int", rclcpp::ParameterValue(7));
  node->declare_parameter("test.static_int", rclcpp::ParameterValue(7));
  std::string name = "test";
  ParametersHandlerWrapper handler(node, name);
  handler.start();

  // Get parameters and check they have initial values
  auto getParameter = handler.getParamGetter("");
  int p1 = 0, p2 = 0;
  getParameter(p1, "test.dynamic_int", 0, ParameterType::Dynamic);
  getParameter(p2, "test.static_int", 0, ParameterType::Static);
  EXPECT_EQ(p1, 7);
  EXPECT_EQ(p2, 7);

  // Now change them both via dynamic parameters
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  std::shared_future<rcl_interfaces::msg::SetParametersResult> result_future =
    rec_param->set_parameters_atomically(
  {
    // Don't set default param rclcpp::Parameter("my_node.verbose", false),
    rclcpp::Parameter("test.dynamic_int", 10),
    rclcpp::Parameter("test.static_int", 10)
  });

  auto rc = rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    result_future);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, rc);

  auto result = result_future.get();
  EXPECT_EQ(result.successful, false);
  EXPECT_FALSE(result.reason.empty());
  EXPECT_EQ(
    result.reason, std::string("Rejected change to static parameter: ") +
    "{\"name\": \"test.static_int\", \"type\": \"integer\", \"value\": \"10\"}");

  // Now, only param1 should change, param 2 should be the same
  EXPECT_EQ(p1, 10);
  EXPECT_EQ(p2, 7);
}

TEST(ParameterHandlerTest, DynamicAndStaticParametersNotDeclaredTest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");

  node->declare_parameter("test.dynamic_int", rclcpp::ParameterValue(7));
  node->declare_parameter("test.static_int", rclcpp::ParameterValue(7));
  std::string name = "test";
  ParametersHandlerWrapper handler(node, name);
  handler.start();

  // Set verbose true to get more information about bad parameter usage
  auto getParameter = handler.getParamGetter("");
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  std::shared_future<rcl_interfaces::msg::SetParametersResult>
  result_future = rec_param->set_parameters_atomically(
  {
    rclcpp::Parameter("my_node.verbose", true),
  });

  auto rc = rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    result_future);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, rc);

  auto result = result_future.get();
  EXPECT_EQ(result.successful, true);
  EXPECT_TRUE(result.reason.empty());

  // Try to set some parameters that have not been declared via the service client
  result_future = rec_param->set_parameters_atomically(
  {
    rclcpp::Parameter("test.static_int", 10),
    rclcpp::Parameter("test.not_declared", true),
    rclcpp::Parameter("test.not_declared2", true),
  });

  rc = rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    result_future);
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, rc);

  result = result_future.get();
  EXPECT_EQ(result.successful, false);
  EXPECT_FALSE(result.reason.empty());
  // The ParameterNotDeclaredException handler in rclcpp/parameter_service.cpp
  // overrides any other reasons and does not provide details to the service client.
  EXPECT_EQ(result.reason, std::string("One or more parameters were not declared before setting"));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
