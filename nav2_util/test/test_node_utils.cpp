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

using nav2_util::sanitizeNodeName;
using nav2_util::generateInternalNodeName;
using nav2_util::generateInternalNode;
using nav2_util::timeToString;

TEST(SanitizeNodeName, SanitizeNodeName)
{
  ASSERT_EQ(sanitizeNodeName("bar"), "bar");
  ASSERT_EQ(sanitizeNodeName("/foo/bar"), "_foo_bar");
}

TEST(TimeToString, IsLengthCorrect)
{
  ASSERT_EQ(timeToString(0).length(), 0u);
  ASSERT_EQ(timeToString(1).length(), 1u);
  ASSERT_EQ(timeToString(10).length(), 10u);
  ASSERT_EQ(timeToString(20)[0], '0');
}

TEST(TimeToString, TimeToStringDifferent)
{
  auto time1 = timeToString(8);
  auto time2 = timeToString(8);
  ASSERT_NE(time1, time2);
}

TEST(GenerateInternalNodeName, GenerateNodeName)
{
  auto defaultName = generateInternalNodeName();
  ASSERT_EQ(defaultName[0], '_');
  ASSERT_EQ(defaultName.length(), 9u);
}
