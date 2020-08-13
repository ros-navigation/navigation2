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

#include "nav2_util/string_utils.hpp"
#include "gtest/gtest.h"

using nav2_util::split;
using nav2_util::Tokens;

TEST(Split, SplitFunction)
{
  ASSERT_EQ(split("", ':'), Tokens({""}));
  ASSERT_EQ(split("foo", ':'), Tokens{"foo"});
  ASSERT_EQ(split("foo:bar", ':'), Tokens({"foo", "bar"}));
  ASSERT_EQ(split("foo:bar:", ':'), Tokens({"foo", "bar", ""}));
  ASSERT_EQ(split(":", ':'), Tokens({"", ""}));
  ASSERT_EQ(split("foo::bar", ':'), Tokens({"foo", "", "bar"}));
  ASSERT_TRUE(nav2_util::strip_leading_slash(std::string("/hi")) == std::string("hi"));
}
