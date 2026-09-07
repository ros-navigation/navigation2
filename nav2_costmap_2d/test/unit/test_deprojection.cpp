// Copyright (c) 2026 Ocean Code AI Ltd
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

// NOTE: This file was authored with the assistance of an AI system,
// reviewed and validated by the submitter. Disclosed per the Nav2 PR
// template's AI-generated-software policy.

#include <gtest/gtest.h>
#include "nav2_monocular_depth_layer/deprojection.hpp"

using nav2_monocular_depth_layer::deproject;

// A pixel at the principal point projects straight down the optical axis.
TEST(Deprojection, PrincipalPointIsOnAxis)
{
  const double fx = 500.0, fy = 500.0, cx = 320.0, cy = 240.0;
  const auto p = deproject(cx, cy, 2.0, fx, fy, cx, cy);
  EXPECT_NEAR(p[0], 0.0, 1e-9);
  EXPECT_NEAR(p[1], 0.0, 1e-9);
  EXPECT_NEAR(p[2], 2.0, 1e-9);
}

// A pixel one focal length to the right of centre sits at 45 degrees: x == z.
TEST(Deprojection, FortyFiveDegreesRight)
{
  const double fx = 500.0, fy = 500.0, cx = 320.0, cy = 240.0;
  const auto p = deproject(cx + fx, cy, 3.0, fx, fy, cx, cy);
  EXPECT_NEAR(p[0], 3.0, 1e-9);   // +x is image-right in the optical frame
  EXPECT_NEAR(p[1], 0.0, 1e-9);
  EXPECT_NEAR(p[2], 3.0, 1e-9);
}

// +y is image-down in the optical frame (REP 103).
TEST(Deprojection, PositiveYIsDown)
{
  const double fx = 500.0, fy = 400.0, cx = 320.0, cy = 240.0;
  const auto p = deproject(cx, cy + fy, 5.0, fx, fy, cx, cy);
  EXPECT_NEAR(p[1], 5.0, 1e-9);
}

// Depth scales the deprojected point linearly.
TEST(Deprojection, LinearInDepth)
{
  const double fx = 600.0, fy = 600.0, cx = 320.0, cy = 240.0;
  const auto near = deproject(100.0, 50.0, 1.0, fx, fy, cx, cy);
  const auto far = deproject(100.0, 50.0, 4.0, fx, fy, cx, cy);
  EXPECT_NEAR(far[0], 4.0 * near[0], 1e-9);
  EXPECT_NEAR(far[1], 4.0 * near[1], 1e-9);
  EXPECT_NEAR(far[2], 4.0 * near[2], 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
