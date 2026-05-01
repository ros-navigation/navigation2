// Copyright (c) 2026 Sanchit Badamikar
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
#include <vector>

#include "bezier_math.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace nav2_geometric_planners
{

// Helper to build a Point.
static geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = 0.0;
  return p;
}

// ---------------------------------------------------------------------------
// evaluateBezier — at t=0 returns first control point
// ---------------------------------------------------------------------------

TEST(BezierMathTest, EvalAtZeroReturnsStart)
{
  const std::vector<geometry_msgs::msg::Point> pts = {
    makePoint(1.0, 2.0), makePoint(3.0, 4.0), makePoint(5.0, 0.0)
  };

  const auto result = evaluateBezier(pts, 0.0);
  EXPECT_NEAR(result.x, 1.0, 1e-9);
  EXPECT_NEAR(result.y, 2.0, 1e-9);
}

// ---------------------------------------------------------------------------
// evaluateBezier — at t=1 returns last control point
// ---------------------------------------------------------------------------

TEST(BezierMathTest, EvalAtOneReturnsEnd)
{
  const std::vector<geometry_msgs::msg::Point> pts = {
    makePoint(1.0, 2.0), makePoint(3.0, 4.0), makePoint(5.0, 0.0)
  };

  const auto result = evaluateBezier(pts, 1.0);
  EXPECT_NEAR(result.x, 5.0, 1e-9);
  EXPECT_NEAR(result.y, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// evaluateBezier — linear Bezier (2 points) gives midpoint at t=0.5
// ---------------------------------------------------------------------------

TEST(BezierMathTest, LinearBezierMidpoint)
{
  const std::vector<geometry_msgs::msg::Point> pts = {
    makePoint(0.0, 0.0), makePoint(4.0, 2.0)
  };

  const auto mid = evaluateBezier(pts, 0.5);
  EXPECT_NEAR(mid.x, 2.0, 1e-9);
  EXPECT_NEAR(mid.y, 1.0, 1e-9);
}

// ---------------------------------------------------------------------------
// evaluateBezier — quadratic Bezier midpoint formula: B(0.5) = P0/4 + P1/2 + P2/4
// ---------------------------------------------------------------------------

TEST(BezierMathTest, QuadraticBezierMidpoint)
{
  // P0=(0,0), P1=(1,2), P2=(2,0)
  // B(0.5) = 0.25*(0,0) + 0.5*(1,2) + 0.25*(2,0) = (1.0, 1.0)
  const std::vector<geometry_msgs::msg::Point> pts = {
    makePoint(0.0, 0.0), makePoint(1.0, 2.0), makePoint(2.0, 0.0)
  };

  const auto mid = evaluateBezier(pts, 0.5);
  EXPECT_NEAR(mid.x, 1.0, 1e-9);
  EXPECT_NEAR(mid.y, 1.0, 1e-9);
}

// ---------------------------------------------------------------------------
// evaluateBezier — cubic Bezier endpoint constraint
// ---------------------------------------------------------------------------

TEST(BezierMathTest, CubicBezierEndpoint)
{
  const std::vector<geometry_msgs::msg::Point> pts = {
    makePoint(0.0, 0.0), makePoint(1.0, 3.0),
    makePoint(2.0, -1.0), makePoint(3.0, 0.0)
  };

  const auto start = evaluateBezier(pts, 0.0);
  const auto end   = evaluateBezier(pts, 1.0);

  EXPECT_NEAR(start.x, 0.0, 1e-9);
  EXPECT_NEAR(start.y, 0.0, 1e-9);
  EXPECT_NEAR(end.x, 3.0, 1e-9);
  EXPECT_NEAR(end.y, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// evaluateBezier — curve lies on the convex hull (convex-hull property)
// The midpoint must lie between the extreme x coordinates.
// ---------------------------------------------------------------------------

TEST(BezierMathTest, ConvexHullProperty)
{
  const std::vector<geometry_msgs::msg::Point> pts = {
    makePoint(0.0, 0.0), makePoint(1.0, 4.0), makePoint(2.0, 0.0)
  };

  for (int i = 0; i <= 10; ++i) {
    const double t = static_cast<double>(i) / 10.0;
    const auto p = evaluateBezier(pts, t);
    EXPECT_GE(p.x, 0.0);
    EXPECT_LE(p.x, 2.0);
    EXPECT_GE(p.y, 0.0);
    EXPECT_LE(p.y, 4.0);
  }
}

}  // namespace nav2_geometric_planners

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
