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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"

// Tests model classes with methods

using namespace mppi::models;  // NOLINT

TEST(ModelsTest, ControlSequenceTest)
{
  // populate the object
  ControlSequence sequence;
  sequence.vx = Eigen::ArrayXf::Ones(10);
  sequence.vy = Eigen::ArrayXf::Ones(10);
  sequence.wz = Eigen::ArrayXf::Ones(10);

  // Show you can get contents
  EXPECT_EQ(sequence.vx(4), 1);
  EXPECT_EQ(sequence.vy(4), 1);
  EXPECT_EQ(sequence.wz(4), 1);

  sequence.reset(20);

  // Show contents are gone and new size
  EXPECT_EQ(sequence.vx(4), 0);
  EXPECT_EQ(sequence.vy(4), 0);
  EXPECT_EQ(sequence.wz(4), 0);
  EXPECT_EQ(sequence.vx.rows(), 20);
  EXPECT_EQ(sequence.vy.rows(), 20);
  EXPECT_EQ(sequence.wz.rows(), 20);
}

TEST(ModelsTest, PathTest)
{
  // populate the object
  Path path;
  path.x = Eigen::ArrayXf::Ones(10);
  path.y = Eigen::ArrayXf::Ones(10);
  path.yaws = Eigen::ArrayXf::Ones(10);

  // Show you can get contents
  EXPECT_EQ(path.x(4), 1);
  EXPECT_EQ(path.y(4), 1);
  EXPECT_EQ(path.yaws(4), 1);

  path.reset(20);

  // Show contents are gone and new size
  EXPECT_EQ(path.x(4), 0);
  EXPECT_EQ(path.y(4), 0);
  EXPECT_EQ(path.yaws(4), 0);
  EXPECT_EQ(path.x.rows(), 20);
  EXPECT_EQ(path.y.rows(), 20);
  EXPECT_EQ(path.yaws.rows(), 20);
}

TEST(ModelsTest, StateTest)
{
  // populate the object
  State state;
  state.vx = Eigen::ArrayXXf::Ones(10, 10);
  state.vy = Eigen::ArrayXXf::Ones(10, 10);
  state.wz = Eigen::ArrayXXf::Ones(10, 10);
  state.cvx = Eigen::ArrayXXf::Ones(10, 10);
  state.cvy = Eigen::ArrayXXf::Ones(10, 10);
  state.cwz = Eigen::ArrayXXf::Ones(10, 10);

  // Show you can get contents
  EXPECT_EQ(state.cvx(4), 1);
  EXPECT_EQ(state.cvy(4), 1);
  EXPECT_EQ(state.cwz(4), 1);
  EXPECT_EQ(state.vx(4), 1);
  EXPECT_EQ(state.vy(4), 1);
  EXPECT_EQ(state.wz(4), 1);

  state.reset(20, 40);

  // Show contents are gone and new size
  EXPECT_EQ(state.cvx(4), 0);
  EXPECT_EQ(state.cvy(4), 0);
  EXPECT_EQ(state.cwz(4), 0);
  EXPECT_EQ(state.vx(4), 0);
  EXPECT_EQ(state.vy(4), 0);
  EXPECT_EQ(state.wz(4), 0);
  EXPECT_EQ(state.cvx.rows(), 20);
  EXPECT_EQ(state.cvy.rows(), 20);
  EXPECT_EQ(state.cwz.rows(), 20);
  EXPECT_EQ(state.cvx.cols(), 40);
  EXPECT_EQ(state.cvy.cols(), 40);
  EXPECT_EQ(state.cwz.cols(), 40);
  EXPECT_EQ(state.vx.rows(), 20);
  EXPECT_EQ(state.vy.rows(), 20);
  EXPECT_EQ(state.wz.rows(), 20);
  EXPECT_EQ(state.vx.cols(), 40);
  EXPECT_EQ(state.vy.cols(), 40);
  EXPECT_EQ(state.wz.cols(), 40);
}

TEST(ModelsTest, TrajectoriesTest)
{
  // populate the object
  Trajectories trajectories;
  trajectories.x = Eigen::ArrayXXf::Ones(10, 10);
  trajectories.y = Eigen::ArrayXXf::Ones(10, 10);
  trajectories.yaws = Eigen::ArrayXXf::Ones(10, 10);

  // Show you can get contents
  EXPECT_EQ(trajectories.x(4), 1);
  EXPECT_EQ(trajectories.y(4), 1);
  EXPECT_EQ(trajectories.yaws(4), 1);

  trajectories.reset(20, 40);

  // Show contents are gone and new size
  EXPECT_EQ(trajectories.x(4), 0);
  EXPECT_EQ(trajectories.y(4), 0);
  EXPECT_EQ(trajectories.yaws(4), 0);
  EXPECT_EQ(trajectories.x.rows(), 20);
  EXPECT_EQ(trajectories.y.rows(), 20);
  EXPECT_EQ(trajectories.yaws.rows(), 20);
  EXPECT_EQ(trajectories.x.cols(), 40);
  EXPECT_EQ(trajectories.y.cols(), 40);
  EXPECT_EQ(trajectories.yaws.cols(), 40);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
