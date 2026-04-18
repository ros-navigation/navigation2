// Copyright (c) 2025 Nav2 Contributors
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
#include <cmath>

#include "clothoid_math.hpp"

namespace nav2_geometric_planners
{

// ---------------------------------------------------------------------------
// ClothoidSegment::buildG1 — trivial case (zero-length)
// ---------------------------------------------------------------------------

TEST(ClothoidMathTest, ZeroLengthSegment)
{
  ClothoidSegment seg;
  // Start == end => zero-length segment, solver should return gracefully.
  EXPECT_TRUE(seg.buildG1(1.0, 2.0, 0.5, 1.0, 2.0, 0.5));
  EXPECT_NEAR(seg.length_, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// ClothoidSegment::eval — endpoint should match goal after buildG1
// ---------------------------------------------------------------------------

TEST(ClothoidMathTest, EndpointMatchesGoal)
{
  const double x0 = 0.0, y0 = 0.0, th0 = 0.0;
  const double x1 = 2.0, y1 = 1.0, th1 = M_PI / 4.0;

  ClothoidSegment seg;
  ASSERT_TRUE(seg.buildG1(x0, y0, th0, x1, y1, th1));
  ASSERT_GT(seg.length_, 0.0);

  double xe{}, ye{};
  seg.eval(seg.length_, xe, ye);

  EXPECT_NEAR(xe, x1, 0.05);
  EXPECT_NEAR(ye, y1, 0.05);
  EXPECT_NEAR(seg.theta(seg.length_), th1, 0.05);
}

// ---------------------------------------------------------------------------
// ClothoidSegment::kappa — curvature at s=0 equals kappa0_
// ---------------------------------------------------------------------------

TEST(ClothoidMathTest, CurvatureAtStart)
{
  ClothoidSegment seg;
  seg.kappa0_ = 0.3;
  seg.dkappa_ = 0.1;

  EXPECT_NEAR(seg.kappa(0.0), 0.3, 1e-12);
  EXPECT_NEAR(seg.kappa(1.0), 0.4, 1e-12);
}

// ---------------------------------------------------------------------------
// ClothoidSegment::theta — heading increases linearly for zero dkappa
// ---------------------------------------------------------------------------

TEST(ClothoidMathTest, HeadingLinearForConstantCurvature)
{
  ClothoidSegment seg;
  seg.theta0_ = 0.0;
  seg.kappa0_ = 0.5;
  seg.dkappa_ = 0.0;
  seg.length_ = 2.0;

  EXPECT_NEAR(seg.theta(1.0), 0.5, 1e-12);
  EXPECT_NEAR(seg.theta(2.0), 1.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Straight-line clothoid (same heading at start and end)
// ---------------------------------------------------------------------------

TEST(ClothoidMathTest, StraightLineSegment)
{
  ClothoidSegment seg;
  ASSERT_TRUE(seg.buildG1(0.0, 0.0, 0.0, 3.0, 0.0, 0.0));

  double xe{}, ye{};
  seg.eval(seg.length_, xe, ye);

  EXPECT_NEAR(xe, 3.0, 0.05);
  EXPECT_NEAR(ye, 0.0, 0.05);
  EXPECT_NEAR(seg.theta(seg.length_), 0.0, 0.05);
}

}  // namespace nav2_geometric_planners

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
