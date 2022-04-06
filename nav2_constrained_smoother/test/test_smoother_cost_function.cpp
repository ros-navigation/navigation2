// Copyright (c) 2021 RoboTech Vision
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <future>
#include <thread>
#include <algorithm>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_constrained_smoother/smoother_cost_function.hpp"

class TestableSmootherCostFunction : nav2_constrained_smoother::SmootherCostFunction
{
public:
  TestableSmootherCostFunction(
    const Eigen::Vector2d & original_pos,
    double next_to_last_length_ratio,
    bool reversing,
    const nav2_costmap_2d::Costmap2D * costmap,
    const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> & costmap_interpolator,
    const nav2_constrained_smoother::SmootherParams & params,
    double costmap_weight)
  : SmootherCostFunction(
      original_pos, next_to_last_length_ratio, reversing,
      costmap, costmap_interpolator,
      params, costmap_weight)
  {
  }

  inline double getCurvatureResidual(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_next,
    const Eigen::Vector2d & pt_prev) const
  {
    double r = 0.0;
    addCurvatureResidual<double>(weight, pt, pt_next, pt_prev, r);
    return r;
  }
};

class Test : public ::testing::Test
{
protected:
  void SetUp()
  {
  }
};

TEST_F(Test, testingCurvatureResidual)
{
  nav2_costmap_2d::Costmap2D costmap;
  TestableSmootherCostFunction fn(
    Eigen::Vector2d(1.0, 0.0), 1.0, false,
    &costmap, std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(),
    nav2_constrained_smoother::SmootherParams(), 0.0
  );

  // test for edge values
  Eigen::Vector2d pt(1.0, 0.0);
  Eigen::Vector2d pt_other(0.0, 0.0);
  EXPECT_EQ(fn.getCurvatureResidual(0.0, pt, pt_other, pt_other), 0.0);

  nav2_constrained_smoother::SmootherParams params_no_min_turning_radius;
  params_no_min_turning_radius.max_curvature = 1.0f / 0.0;
  TestableSmootherCostFunction fn_no_min_turning_radius(
    Eigen::Vector2d(1.0, 0.0), 1.0, false,
    &costmap, std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>>(),
    params_no_min_turning_radius, 0.0
  );
  EXPECT_EQ(fn_no_min_turning_radius.getCurvatureResidual(1.0, pt, pt_other, pt_other), 0.0);
}

TEST_F(Test, testingUtils)
{
  Eigen::Vector2d pt(1.0, 0.0);
  Eigen::Vector2d pt_prev(0.0, 0.0);
  Eigen::Vector2d pt_next(0.0, 0.0);

  // test for intermediate values
  auto center = nav2_constrained_smoother::arcCenter(pt_prev, pt, pt_next, false);
  // although in this situation the center would be at (0.5, 0.0),
  // cases where pt_prev == pt_next are very rare and thus unhandled
  // during the smoothing points will be separated (and thus made valid) by smoothness cost anyways
  EXPECT_EQ(center[0], std::numeric_limits<double>::infinity());
  EXPECT_EQ(center[1], std::numeric_limits<double>::infinity());

  auto tangent =
    nav2_constrained_smoother::tangentDir(pt_prev, pt, pt_next, false).normalized();
  EXPECT_NEAR(tangent[0], 0, 1e-10);
  EXPECT_NEAR(std::abs(tangent[1]), 1, 1e-10);

  // no rotation when mid point is a cusp
  tangent = nav2_constrained_smoother::tangentDir(pt_prev, pt, pt_next, true).normalized();
  EXPECT_NEAR(std::abs(tangent[0]), 1, 1e-10);
  EXPECT_NEAR(tangent[1], 0, 1e-10);

  pt_prev[0] = -1.0;
  // rotation is mathematically invalid, picking direction of a shorter segment
  tangent = nav2_constrained_smoother::tangentDir(pt_prev, pt, pt_next, true).normalized();
  EXPECT_NEAR(std::abs(tangent[0]), 1, 1e-10);
  EXPECT_NEAR(tangent[1], 0, 1e-10);

  pt_prev[0] = 0.0;
  pt_next[0] = -1.0;
  // rotation is mathematically invalid, picking direction of a shorter segment
  tangent = nav2_constrained_smoother::tangentDir(pt_prev, pt, pt_next, true).normalized();
  EXPECT_NEAR(std::abs(tangent[0]), 1, 1e-10);
  EXPECT_NEAR(tangent[1], 0, 1e-10);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
