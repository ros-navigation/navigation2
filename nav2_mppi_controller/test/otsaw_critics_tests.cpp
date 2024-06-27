// Copyright (c) 2024 OTSAW

#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/critics/otsaw_critic.hpp"
#include <xtensor/xio.hpp>  // for debugging, to support std::out

#include "utils_test.cpp"  // NOLINT

// Tests the various critic plugin functions

// ROS lock used from utils_test.cpp

using namespace mppi;  // NOLINT
using namespace mppi::critics;  // NOLINT
using namespace mppi::utils;  // NOLINT
using xt::evaluation_strategy::immediate;

TEST(CriticTests, OtsawCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();

  // Initialization testing

  // Make sure initializes correctly and that defaults are reasonable
  OtsawCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");
  EXPECT_TRUE(critic.getMaxVelConstraint() > 0.0);
  EXPECT_TRUE(critic.getMinVelConstraint() < 0.0);

  // Scoring testing

  // provide velocities in constraints, should not have any costs
  state.vx = 0.40 * xt::ones<float>({1000, 30});
  state.vy = xt::zeros<float>({1000, 30});
  state.wz = xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);

  // provide out of maximum velocity constraint
  auto last_batch_traj_in_full = xt::view(state.vx, -1, xt::all());
  last_batch_traj_in_full = 0.60 * xt::ones<float>({30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  // 4.0 weight * 0.1 model_dt * 0.1 error introduced * 30 timesteps = 1.2
  EXPECT_NEAR(costs(999), 1.2, 0.01);
  costs = xt::zeros<float>({10});

  // provide out of minimum velocity constraint
  auto first_batch_traj_in_full = xt::view(state.vx, 1, xt::all());
  first_batch_traj_in_full = -0.45 * xt::ones<float>({30});

  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  // 4.0 weight * 0.1 model_dt * 0.1 error introduced * 30 timesteps = 1.2
  EXPECT_NEAR(costs(1), 1.2, 0.01);
  costs = xt::zeros<float>({10});
}
