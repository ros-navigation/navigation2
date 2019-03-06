// Copyright (c) 2019 Intel Corporation
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

#include <string>
#include "nav2_util/service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "gtest/gtest.h"

using nav2_util::ServiceClient;
using std::string;

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

class TestServiceClient : public ServiceClient<std_srvs::srv::Empty>
{
public:
  TestServiceClient(
    const std::string & name,
    const rclcpp::Node::SharedPtr & provided_node = rclcpp::Node::SharedPtr())
  : ServiceClient(name, provided_node) {}

  string name() {return node_->get_name();}
  const rclcpp::Node::SharedPtr & getNode() {return node_;}
};

TEST(ServiceClient, are_non_alphanumerics_removed)
{
  TestServiceClient t("/foo/bar");
  string adjustedPrefix = "_foo_bar_Node_";
  ASSERT_EQ(t.name().length(), adjustedPrefix.length() + 8);
  ASSERT_EQ(0, t.name().compare(0, adjustedPrefix.length(), adjustedPrefix));
}

TEST(ServiceClient, can_ServiceClient_use_passed_in_node)
{
  auto node = rclcpp::Node::make_shared("test_node");
  TestServiceClient t("bar", node);
  ASSERT_EQ(t.getNode(), node);
  ASSERT_EQ(t.name(), "test_node");
}
