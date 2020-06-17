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

#include "nav2_util/node_utils.hpp"
#include "gtest/gtest.h"

using nav2_util::sanitize_node_name;
using nav2_util::generate_internal_node_name;
using nav2_util::generate_internal_node;
using nav2_util::add_namespaces;
using nav2_util::time_to_string;

TEST(SanitizeNodeName, SanitizeNodeName)
{
  ASSERT_EQ(sanitize_node_name("bar"), "bar");
  ASSERT_EQ(sanitize_node_name("/foo/bar"), "_foo_bar");
}

TEST(TimeToString, IsLengthCorrect)
{
  ASSERT_EQ(time_to_string(0).length(), 0u);
  ASSERT_EQ(time_to_string(1).length(), 1u);
  ASSERT_EQ(time_to_string(10).length(), 10u);
  ASSERT_EQ(time_to_string(20)[0], '0');
}

TEST(TimeToString, TimeToStringDifferent)
{
  auto time1 = time_to_string(8);
  auto time2 = time_to_string(8);
  ASSERT_NE(time1, time2);
}

TEST(GenerateInternalNodeName, GenerateNodeName)
{
  auto defaultName = generate_internal_node_name();
  ASSERT_EQ(defaultName[0], '_');
  ASSERT_EQ(defaultName.length(), 9u);
}

TEST(AddNamespaces, AddNamespaceSlash)
{
  ASSERT_EQ(add_namespaces("hi", "bye"), "hi/bye");
  ASSERT_EQ(add_namespaces("hi/", "bye"), "/hi/bye");
}
