// Copyright (c) 2026
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

#include "gtest/gtest.h"
#include "nav2_amcl/amcl_node.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(AmclNodeRandomSeed, configure_uses_parameter_override)
{
  rclcpp::NodeOptions options;
  // Ensure we cover the deterministic-seed branch in initParticleFilter().
  options.parameter_overrides({{"random_seed", 42}});

  auto amcl = std::make_shared<nav2_amcl::AmclNode>(options);
  amcl->configure();

  EXPECT_EQ(amcl->get_parameter("random_seed").as_int(), 42);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
