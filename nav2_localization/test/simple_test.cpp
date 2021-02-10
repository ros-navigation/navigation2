// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA
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

#include <gtest/gtest.h>
#include <climits>

// bad function:
// for example: how to deal with overflow?
int add(int a, int b)
{
  return a + b;
}

TEST(NumberCmpTest, ShouldPass) {
  ASSERT_EQ(3, add(1, 2));
}

TEST(NumberCmpTest, ShouldFail) {
  ASSERT_EQ(INT_MAX, add(INT_MAX, 1));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
