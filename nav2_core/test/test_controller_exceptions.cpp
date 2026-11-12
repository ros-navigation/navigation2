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

#include <cstdint>

#include "gtest/gtest.h"
#include "nav2_core/controller_exceptions.hpp"

using nav2_core::FollowPathResult;

namespace
{

uint16_t codeOf(const nav2_core::ControllerException & e)
{
  return e.errorCode();
}

}  // namespace

TEST(ControllerExceptionsTest, DefaultCodes)
{
  EXPECT_EQ(codeOf(nav2_core::ControllerException("")), FollowPathResult::UNKNOWN);
  EXPECT_EQ(
    codeOf(nav2_core::InvalidController("")), FollowPathResult::INVALID_CONTROLLER);
  EXPECT_EQ(codeOf(nav2_core::ControllerTFError("")), FollowPathResult::TF_ERROR);
  EXPECT_EQ(
    codeOf(nav2_core::FailedToMakeProgress("")), FollowPathResult::FAILED_TO_MAKE_PROGRESS);
  EXPECT_EQ(codeOf(nav2_core::PatienceExceeded("")), FollowPathResult::PATIENCE_EXCEEDED);
  EXPECT_EQ(codeOf(nav2_core::InvalidPath("")), FollowPathResult::INVALID_PATH);
  EXPECT_EQ(codeOf(nav2_core::NoValidControl("")), FollowPathResult::NO_VALID_CONTROL);
  EXPECT_EQ(
    codeOf(nav2_core::ControllerTimedOut("")), FollowPathResult::CONTROLLER_TIMED_OUT);
}

TEST(ControllerExceptionsTest, CustomCode)
{
  EXPECT_EQ(
    codeOf(nav2_core::ControllerException("", FollowPathResult::TIMEOUT)),
    FollowPathResult::TIMEOUT);
  EXPECT_EQ(
    codeOf(nav2_core::NoValidControl("", FollowPathResult::FAILED_TO_MAKE_PROGRESS)),
    FollowPathResult::FAILED_TO_MAKE_PROGRESS);
}

TEST(ControllerExceptionsTest, RethrowKeepsCode)
{
  try {
    try {
      throw nav2_core::NoValidControl("msg", FollowPathResult::FAILED_TO_MAKE_PROGRESS);
    } catch (nav2_core::NoValidControl &) {
      throw;
    }
  } catch (nav2_core::ControllerException & e) {
    EXPECT_EQ(e.errorCode(), FollowPathResult::FAILED_TO_MAKE_PROGRESS);
    EXPECT_STREQ(e.what(), "msg");
  }
}
