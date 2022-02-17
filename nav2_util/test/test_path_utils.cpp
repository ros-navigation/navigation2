// Copyright (c) 2022 Adam Aposhian
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/path_utils.hpp"
#include "gtest/gtest.h"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

using namespace nav2_util;

TEST(PathUtils, test_generate_straight)
{
  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = "test_frame";

  auto path = generate_path(
    start, 1.0, {
    std::make_unique<Straight>(1.0)
  });
  EXPECT_EQ(path.poses.size(), 2u);
  for (const auto & pose : path.poses) {
    EXPECT_EQ(pose.header.frame_id, start.header.frame_id);
  }
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.z, 0.0);

  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.z, 0.0);
}
