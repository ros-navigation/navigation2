// Copyright 2023 Open Navigation LLC
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

#include <gtest/gtest.h>
#include "nav2_util/copy_all_parameter_values.hpp"
#include "rclcpp/rclcpp.hpp"

class TestNode : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestNode, TestParamCopying)
{
  auto node1 = std::make_shared<rclcpp::Node>("test_node1");
  auto node2 = std::make_shared<rclcpp::Node>("test_node2");

  // Tests for (1) multiple types, (2) recursion, (3) overriding values
  node1->declare_parameter("Foo1", rclcpp::ParameterValue(std::string(("bar1"))));
  node1->declare_parameter("Foo2", rclcpp::ParameterValue(0.123));
  node1->declare_parameter("Foo", rclcpp::ParameterValue(std::string(("bar"))));
  node1->declare_parameter("Foo.bar", rclcpp::ParameterValue(std::string(("steve"))));
  node2->declare_parameter("Foo", rclcpp::ParameterValue(std::string(("barz2"))));

  // Show Node2 is empty of Node1's parameters, but contains its own
  EXPECT_FALSE(node2->has_parameter("Foo1"));
  EXPECT_FALSE(node2->has_parameter("Foo2"));
  EXPECT_FALSE(node2->has_parameter("Foo.bar"));
  EXPECT_TRUE(node2->has_parameter("Foo"));
  EXPECT_EQ(node2->get_parameter("Foo").as_string(), std::string("barz2"));

  bool override = false;
  nav2_util::copy_all_parameter_values(node1, node2, override);

  // Test new parameters exist, of expected value, and original param is not overridden
  EXPECT_TRUE(node2->has_parameter("Foo1"));
  EXPECT_EQ(node2->get_parameter("Foo1").as_string(), std::string("bar1"));
  EXPECT_TRUE(node2->has_parameter("Foo2"));
  EXPECT_EQ(node2->get_parameter("Foo2").as_double(), 0.123);
  EXPECT_TRUE(node2->has_parameter("Foo.bar"));
  EXPECT_EQ(node2->get_parameter("Foo.bar").as_string(), std::string("steve"));
  EXPECT_TRUE(node2->has_parameter("Foo"));
  EXPECT_EQ(node2->get_parameter("Foo").as_string(), std::string("barz2"));

  // Test if parameter overrides are permissible that Node2's value is overridden
  override = true;
  nav2_util::copy_all_parameter_values(node1, node2, override);
  EXPECT_EQ(node2->get_parameter("Foo").as_string(), std::string("bar"));
}

TEST_F(TestNode, TestParamCopyingExceptions)
{
  auto node1 = std::make_shared<rclcpp::Node>("test_node1");
  auto node2 = std::make_shared<rclcpp::Node>("test_node2");

  // Tests for Parameter value conflicts handled
  node1->declare_parameter("Foo", rclcpp::ParameterValue(std::string(("bar"))));
  node2->declare_parameter("Foo", rclcpp::ParameterValue(0.123));

  bool override = true;
  EXPECT_NO_THROW(
    nav2_util::copy_all_parameter_values(node1, node2, override));

  // Tests for Parameter read-only handled
  node1->declare_parameter("Foo1", rclcpp::ParameterValue(std::string(("bar"))));
  node2->declare_parameter("Foo1", rclcpp::ParameterValue(0.123));
  EXPECT_NO_THROW(nav2_util::copy_all_parameter_values(node1, node2, override));
}
