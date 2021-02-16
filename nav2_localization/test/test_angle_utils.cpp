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

#include <cmath>
#include "gtest/gtest.h"
#include "nav2_localization/angle_utils.hpp"

using nav2_localization::AngleUtils;

const double EPSILON = 1e-3;

TEST(AngleUtilsTest, DiffEqZero)
{
  double a = M_PI / 2;
  double b = M_PI / 2;

  double expected = 0.0;
  double actual = AngleUtils::angleDiff(a, b);
  EXPECT_NEAR(expected, actual, EPSILON);
}

TEST(AngleUtilsTest, DiffWithinRange)
{
  // the difference is within [-pi, pi)
  double a = M_PI / 2;
  double b = M_PI / 4;

  double expected = M_PI / 4;
  double actual = AngleUtils::angleDiff(a, b);
  EXPECT_NEAR(expected, actual, EPSILON);

  a = -M_PI / 2;
  b = M_PI / 4;

  expected = -3 * M_PI / 4;
  actual = AngleUtils::angleDiff(a, b);
  EXPECT_NEAR(expected, actual, EPSILON);
}

TEST(AngleUtilsTest, DiffAboveRange)
{
  // the difference is > pi
  double a = 2 * M_PI;
  double b = M_PI / 4;

  double expected = -M_PI / 4;
  double actual = AngleUtils::angleDiff(a, b);
  EXPECT_NEAR(expected, actual, EPSILON);
}

TEST(AngleUtilsTest, DiffBelowRange)
{
  // the difference is < -pi
  double a = M_PI / 4;
  double b = 2 * M_PI;

  double expected = M_PI / 4;
  double actual = AngleUtils::angleDiff(a, b);
  EXPECT_NEAR(expected, actual, EPSILON);
}

TEST(AngleUtilsTest, DiffOnTheEdgeOfRange)
{
  // the difference is == +/-pi
  double a = 2 * M_PI;
  double b = M_PI;

  double expected = M_PI;
  double actual = AngleUtils::angleDiff(a, b);
  EXPECT_NEAR(expected, actual, EPSILON);

  expected = -M_PI;
  actual = AngleUtils::angleDiff(b, a);
  EXPECT_NEAR(expected, actual, EPSILON);
}

TEST(AngleUtilsTest, AngleEqZero)
{
  double a = 0.0;
  double b = M_PI / 2;

  double expected = -M_PI / 2;
  double actual = AngleUtils::angleDiff(a, b);
  EXPECT_NEAR(expected, actual, EPSILON);

  expected = M_PI / 2;
  actual = AngleUtils::angleDiff(b, a);
  EXPECT_NEAR(expected, actual, EPSILON);

  a = 0.0;
  b = 0.0;
  expected = 0.0;
  actual = AngleUtils::angleDiff(a, b);
  EXPECT_NEAR(expected, actual, EPSILON);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
