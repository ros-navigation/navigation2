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
#include <vector>

#include "cubic_spline_math.hpp"
#include "nav2_core/planner_exceptions.hpp"

namespace nav2_geometric_planners
{

// ---------------------------------------------------------------------------
// stampToSec — basic conversion
// ---------------------------------------------------------------------------

TEST(CubicSplineMathTest, StampToSec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 5;
  stamp.nanosec = 500000000u;  // 0.5 s

  EXPECT_NEAR(stampToSec(stamp), 5.5, 1e-9);
}

// ---------------------------------------------------------------------------
// isValidStamp — zero vs non-zero
// ---------------------------------------------------------------------------

TEST(CubicSplineMathTest, IsValidStamp)
{
  builtin_interfaces::msg::Time zero_stamp;
  zero_stamp.sec = 0;
  zero_stamp.nanosec = 0;
  EXPECT_FALSE(isValidStamp(zero_stamp));

  builtin_interfaces::msg::Time valid_stamp;
  valid_stamp.sec = 1;
  valid_stamp.nanosec = 0;
  EXPECT_TRUE(isValidStamp(valid_stamp));
}

// ---------------------------------------------------------------------------
// CubicSpline1D — interpolates exactly at knots
// ---------------------------------------------------------------------------

TEST(CubicSplineMathTest, InterpolatesAtKnots)
{
  const std::vector<double> t = {0.0, 1.0, 2.0, 3.0};
  const std::vector<double> y = {0.0, 1.0, 0.0, 1.0};

  CubicSpline1D spline;
  spline.build(t, y);

  for (std::size_t i = 0; i < t.size(); ++i) {
    EXPECT_NEAR(spline.evaluate(t[i]), y[i], 1e-9)
      << "Failed at knot index " << i;
  }
}

// ---------------------------------------------------------------------------
// CubicSpline1D — clamping at boundaries
// ---------------------------------------------------------------------------

TEST(CubicSplineMathTest, ClampsBeyondRange)
{
  const std::vector<double> t = {0.0, 1.0, 2.0};
  const std::vector<double> y = {0.0, 1.0, 4.0};

  CubicSpline1D spline;
  spline.build(t, y);

  // Value below range should equal spline at t_min.
  EXPECT_NEAR(spline.evaluate(-1.0), spline.evaluate(0.0), 1e-9);
  // Value above range should equal spline at t_max.
  EXPECT_NEAR(spline.evaluate(5.0), spline.evaluate(2.0), 1e-9);
}

// ---------------------------------------------------------------------------
// CubicSpline1D — straight line gives exact linear result
// ---------------------------------------------------------------------------

TEST(CubicSplineMathTest, LinearDataGivesLinearSpline)
{
  const std::vector<double> t = {0.0, 1.0, 2.0, 3.0};
  const std::vector<double> y = {0.0, 1.0, 2.0, 3.0};  // y = t

  CubicSpline1D spline;
  spline.build(t, y);

  EXPECT_NEAR(spline.evaluate(0.5), 0.5, 1e-9);
  EXPECT_NEAR(spline.evaluate(1.5), 1.5, 1e-9);
  EXPECT_NEAR(spline.evaluate(2.5), 2.5, 1e-9);
}

// ---------------------------------------------------------------------------
// CubicSpline1D — requires at least 2 points
// ---------------------------------------------------------------------------

TEST(CubicSplineMathTest, ThrowsWithTooFewPoints)
{
  CubicSpline1D spline;
  EXPECT_THROW(spline.build({1.0}, {1.0}), std::invalid_argument);
}

}  // namespace nav2_geometric_planners

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
