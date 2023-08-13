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

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi::models;  // NOLINT

TEST(ModelsTest, ControlSequenceTest)
{
  // populate the object
  ControlSequence sequence;
  sequence.vx = xt::ones<float>({10});
  sequence.vy = xt::ones<float>({10});
  sequence.wz = xt::ones<float>({10});

  // Show you can get contents
  EXPECT_EQ(sequence.vx(4), 1);
  EXPECT_EQ(sequence.vy(4), 1);
  EXPECT_EQ(sequence.wz(4), 1);

  sequence.reset(20);

  // Show contents are gone and new size
  EXPECT_EQ(sequence.vx(4), 0);
  EXPECT_EQ(sequence.vy(4), 0);
  EXPECT_EQ(sequence.wz(4), 0);
  EXPECT_EQ(sequence.vx.shape(0), 20u);
  EXPECT_EQ(sequence.vy.shape(0), 20u);
  EXPECT_EQ(sequence.wz.shape(0), 20u);
}

TEST(ModelsTest, PathTest)
{
  // populate the object
  Path path;
  path.x = xt::ones<float>({10});
  path.y = xt::ones<float>({10});
  path.yaws = xt::ones<float>({10});

  // Show you can get contents
  EXPECT_EQ(path.x(4), 1);
  EXPECT_EQ(path.y(4), 1);
  EXPECT_EQ(path.yaws(4), 1);

  path.reset(20);

  // Show contents are gone and new size
  EXPECT_EQ(path.x(4), 0);
  EXPECT_EQ(path.y(4), 0);
  EXPECT_EQ(path.yaws(4), 0);
  EXPECT_EQ(path.x.shape(0), 20u);
  EXPECT_EQ(path.y.shape(0), 20u);
  EXPECT_EQ(path.yaws.shape(0), 20u);
}

TEST(ModelsTest, StateTest)
{
  // populate the object
  State state;
  state.vx = xt::ones<float>({10, 10});
  state.vy = xt::ones<float>({10, 10});
  state.wz = xt::ones<float>({10, 10});
  state.cvx = xt::ones<float>({10, 10});
  state.cvy = xt::ones<float>({10, 10});
  state.cwz = xt::ones<float>({10, 10});

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
  EXPECT_EQ(state.cvx.shape(0), 20u);
  EXPECT_EQ(state.cvy.shape(0), 20u);
  EXPECT_EQ(state.cwz.shape(0), 20u);
  EXPECT_EQ(state.cvx.shape(1), 40u);
  EXPECT_EQ(state.cvy.shape(1), 40u);
  EXPECT_EQ(state.cwz.shape(1), 40u);
  EXPECT_EQ(state.vx.shape(0), 20u);
  EXPECT_EQ(state.vy.shape(0), 20u);
  EXPECT_EQ(state.wz.shape(0), 20u);
  EXPECT_EQ(state.vx.shape(1), 40u);
  EXPECT_EQ(state.vy.shape(1), 40u);
  EXPECT_EQ(state.wz.shape(1), 40u);
}

TEST(ModelsTest, TrajectoriesTest)
{
  // populate the object
  Trajectories trajectories;
  trajectories.x = xt::ones<float>({10, 10});
  trajectories.y = xt::ones<float>({10, 10});
  trajectories.yaws = xt::ones<float>({10, 10});

  // Show you can get contents
  EXPECT_EQ(trajectories.x(4), 1);
  EXPECT_EQ(trajectories.y(4), 1);
  EXPECT_EQ(trajectories.yaws(4), 1);

  trajectories.reset(20, 40);

  // Show contents are gone and new size
  EXPECT_EQ(trajectories.x(4), 0);
  EXPECT_EQ(trajectories.y(4), 0);
  EXPECT_EQ(trajectories.yaws(4), 0);
  EXPECT_EQ(trajectories.x.shape(0), 20u);
  EXPECT_EQ(trajectories.y.shape(0), 20u);
  EXPECT_EQ(trajectories.yaws.shape(0), 20u);
  EXPECT_EQ(trajectories.x.shape(1), 40u);
  EXPECT_EQ(trajectories.y.shape(1), 40u);
  EXPECT_EQ(trajectories.yaws.shape(1), 40u);
}
