// Copyright (c) 2026 Nisarg Panchal
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

#include "gtest/gtest.h"
#include "nav2_core/planner_exceptions.hpp"

using nav2_core::ComputePathThroughPosesResult;

TEST(PlannerExceptionsTest, DefaultCodes)
{
  EXPECT_EQ(nav2_core::PlannerException("").errorCode(), ComputePathThroughPosesResult::UNKNOWN);
  EXPECT_EQ(
    nav2_core::InvalidPlanner("").errorCode(), ComputePathThroughPosesResult::INVALID_PLANNER);
  EXPECT_EQ(
    nav2_core::StartOccupied("").errorCode(), ComputePathThroughPosesResult::START_OCCUPIED);
  EXPECT_EQ(
    nav2_core::GoalOccupied("").errorCode(), ComputePathThroughPosesResult::GOAL_OCCUPIED);
  EXPECT_EQ(
    nav2_core::StartOutsideMapBounds("").errorCode(),
    ComputePathThroughPosesResult::START_OUTSIDE_MAP);
  EXPECT_EQ(
    nav2_core::GoalOutsideMapBounds("").errorCode(),
    ComputePathThroughPosesResult::GOAL_OUTSIDE_MAP);
  EXPECT_EQ(
    nav2_core::NoValidPathCouldBeFound("").errorCode(),
    ComputePathThroughPosesResult::NO_VALID_PATH);
  EXPECT_EQ(nav2_core::PlannerTimedOut("").errorCode(), ComputePathThroughPosesResult::TIMEOUT);
  EXPECT_EQ(nav2_core::PlannerTFError("").errorCode(), ComputePathThroughPosesResult::TF_ERROR);
  EXPECT_EQ(
    nav2_core::NoViapointsGiven("").errorCode(),
    ComputePathThroughPosesResult::NO_VIAPOINTS_GIVEN);
}
