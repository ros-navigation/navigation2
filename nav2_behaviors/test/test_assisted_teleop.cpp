// Copyright (c) 2026 Clutterbot
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

#include <cmath>

#include "gtest/gtest.h"
#include "nav2_behaviors/plugins/assisted_teleop.hpp"
#include "nav2_util/geometry_utils.hpp"

class AssistedTeleopShim : public nav2_behaviors::AssistedTeleop
{
public:
  using nav2_behaviors::AssistedTeleop::projectPose;
};

TEST(AssistedTeleopTest, projectPoseDiffDrive)
{
  AssistedTeleopShim behavior;

  geometry_msgs::msg::Pose pose;
  pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI_4);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;

  auto projected = behavior.projectPose(pose, twist, 1.0);

  EXPECT_NEAR(projected.position.x, M_SQRT1_2, 1e-9);
  EXPECT_NEAR(projected.position.y, M_SQRT1_2, 1e-9);
}

TEST(AssistedTeleopTest, projectPoseHolonomic)
{
  AssistedTeleopShim behavior;

  geometry_msgs::msg::Pose pose;
  pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI_4);

  // Distinct magnitudes, so a formula that swaps the two axes cannot pass.
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.linear.y = 2.0;

  auto projected = behavior.projectPose(pose, twist, 1.0);

  EXPECT_NEAR(projected.position.x, -M_SQRT1_2, 1e-9);
  EXPECT_NEAR(projected.position.y, 3.0 * M_SQRT1_2, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
