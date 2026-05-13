// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <Eigen/Dense>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/noise_low_pass_filter.hpp"

using namespace mppi;  // NOLINT

TEST(NoiseLowPassFilterTest, NoopsWhenUnconfigured)
{
  NoiseLowPassFilter filter;
  Eigen::ArrayXXf signal(2, 4);
  signal << 0.1f, -0.2f, 0.3f, -0.4f,
    1.0f, 0.5f, -0.5f, -1.0f;
  const Eigen::ArrayXXf original = signal;

  filter.filter(signal);

  EXPECT_TRUE(signal.isApprox(original));
}

TEST(NoiseLowPassFilterTest, KeepsRequestedCutoffAfterClamp)
{
  NoiseLowPassFilter filter;
  const double requested_cutoff = 1000.0;

  filter.configure(true, 0.1f, requested_cutoff, 2, rclcpp::get_logger("test"));
  const double slow_effective_cutoff = filter.getEffectiveCutoffFrequency();
  EXPECT_TRUE(filter.isActive());
  EXPECT_DOUBLE_EQ(filter.getRequestedCutoffFrequency(), requested_cutoff);
  EXPECT_EQ(filter.getOrder(), 2);
  EXPECT_LT(slow_effective_cutoff, 5.0);

  const float faster_model_dt = 0.01f;
  const double faster_nyquist = 1.0 / (2.0 * static_cast<double>(faster_model_dt));
  filter.resetDt(faster_model_dt, rclcpp::get_logger("test"));
  EXPECT_TRUE(filter.isActive());
  EXPECT_DOUBLE_EQ(filter.getRequestedCutoffFrequency(), requested_cutoff);
  EXPECT_LT(filter.getEffectiveCutoffFrequency(), faster_nyquist);
  EXPECT_GT(filter.getEffectiveCutoffFrequency(), slow_effective_cutoff);
}

TEST(NoiseLowPassFilterTest, ResetDtCanEnableFilterAfterInvalidModelDt)
{
  NoiseLowPassFilter filter;

  filter.configure(true, 0.0f, 1.0, 2, rclcpp::get_logger("test"));
  EXPECT_FALSE(filter.isActive());
  EXPECT_DOUBLE_EQ(filter.getRequestedCutoffFrequency(), 1.0);
  EXPECT_EQ(filter.getOrder(), 2);

  filter.resetDt(0.1f, rclcpp::get_logger("test"));
  EXPECT_TRUE(filter.isActive());
  EXPECT_DOUBLE_EQ(filter.getRequestedCutoffFrequency(), 1.0);
  EXPECT_EQ(filter.getOrder(), 2);
}

TEST(NoiseLowPassFilterTest, RejectsInvalidOrderInput)
{
  NoiseLowPassFilter filter;
  filter.configure(true, 0.1f, 1.0, 0, rclcpp::get_logger("test"));
  EXPECT_FALSE(filter.isActive());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
