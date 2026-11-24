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

#include <benchmark/benchmark.h>

#include <Eigen/Dense>

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/goal_checker.hpp>
#include "nav2_ros_common/tf2_factories.hpp"

#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/motion_models.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

#include "utils.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};

RosLockGuard g_rclcpp;

void prepareAndRunBenchmark(
  bool consider_footprint, std::string motion_model,
  std::vector<std::string> critics, benchmark::State & state, bool asymmetric_accel = false)
{
  int batch_size = 2000;
  int time_steps = 56;
  unsigned int path_points = 50u;
  int iteration_count = 2;
  double lookahead_distance = 10.0;

  TestCostmapSettings costmap_settings{};
  auto costmap_ros = getDummyCostmapRos(costmap_settings);
  auto costmap = costmap_ros->getCostmap();

  TestPose start_pose = costmap_settings.getCenterPose();
  double path_step = costmap_settings.resolution;

  TestPathSettings path_settings{start_pose, path_points, path_step, path_step};
  TestOptimizerSettings optimizer_settings{batch_size, time_steps, iteration_count,
    lookahead_distance, motion_model, consider_footprint};

  unsigned int offset = 4;
  unsigned int obstacle_size = offset * 2;

  unsigned char obstacle_cost = 250;

  auto [obst_x, obst_y] = costmap_settings.getCenterIJ();

  obst_x = obst_x - offset;
  obst_y = obst_y - offset;
  addObstacle(costmap, {obst_x, obst_y, obstacle_size, obstacle_cost});

  printInfo(optimizer_settings, path_settings, critics);
  auto node = getDummyNode(optimizer_settings, critics);
  if (asymmetric_accel) {
    // Accelerating slower than braking
    node->declare_parameter("dummy.ax_max", rclcpp::ParameterValue(0.6));
    node->declare_parameter("dummy.ax_min", rclcpp::ParameterValue(-1.7));
    node->declare_parameter("dummy.ay_max", rclcpp::ParameterValue(0.6));
    node->declare_parameter("dummy.ay_min", rclcpp::ParameterValue(-1.7));
  }
  std::string name = "test";
  auto parameters_handler = std::make_unique<mppi::ParametersHandler>(node, name);
  auto tf_buffer = nav2::create_transform_buffer(node);
  auto optimizer = getDummyOptimizer(node, costmap_ros, tf_buffer, parameters_handler.get());

  // evalControl args
  auto pose = getDummyPointStamped(node, start_pose);
  auto velocity = getDummyTwist();
  auto path = getIncrementalDummyPath(node, path_settings);
  nav2_core::GoalChecker * dummy_goal_checker{nullptr};

  for (auto _ : state) {
    auto [cmd, trajectory] = optimizer->evalControl(
      pose, velocity, path, path.poses.back().pose,
      dummy_goal_checker);
  }
}

static void BM_DiffDrivePointFootprint(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "diff_drive";
  std::vector<std::string> critics = {{"ConstraintCritic"}, {"CostCritic"}, {"GoalCritic"},
    {"GoalAngleCritic"}, {"PathAlignCritic"}, {"PathFollowCritic"}, {"PathAngleCritic"},
    {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_DiffDrive(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "diff_drive";
  std::vector<std::string> critics = {{"ConstraintCritic"}, {"CostCritic"}, {"GoalCritic"},
    {"GoalAngleCritic"}, {"PathAlignCritic"}, {"PathFollowCritic"}, {"PathAngleCritic"},
    {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_DiffDriveAsymmetricAccel(benchmark::State & state)
{
  std::vector<std::string> critics = {{"ConstraintCritic"}, {"CostCritic"}, {"GoalCritic"},
    {"GoalAngleCritic"}, {"PathAlignCritic"}, {"PathFollowCritic"}, {"PathAngleCritic"},
    {"PreferForwardCritic"}};
  prepareAndRunBenchmark(true, "diff_drive", critics, state, true);
}

static void BM_OmniAsymmetricAccel(benchmark::State & state)
{
  std::vector<std::string> critics = {{"ConstraintCritic"}, {"CostCritic"}, {"GoalCritic"},
    {"GoalAngleCritic"}, {"PathAlignCritic"}, {"PathFollowCritic"}, {"PathAngleCritic"},
    {"PreferForwardCritic"}};
  prepareAndRunBenchmark(true, "omni", critics, state, true);
}

static void BM_Omni(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "omni";
  std::vector<std::string> critics = {{"ConstraintCritic"}, {"CostCritic"}, {"GoalCritic"},
    {"GoalAngleCritic"}, {"PathAlignCritic"}, {"PathFollowCritic"}, {"PathAngleCritic"},
    {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_Ackermann(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "ackermann";
  std::vector<std::string> critics = {{"ConstraintCritic"}, {"CostCritic"}, {"GoalCritic"},
    {"GoalAngleCritic"}, {"PathAlignCritic"}, {"PathFollowCritic"}, {"PathAngleCritic"},
    {"PreferForwardCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_GoalCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "ackermann";
  std::vector<std::string> critics = {{"GoalCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_GoalAngleCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "ackermann";
  std::vector<std::string> critics = {{"GoalAngleCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_ObstaclesCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "ackermann";
  std::vector<std::string> critics = {{"ObstaclesCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_ObstaclesCriticPointFootprint(benchmark::State & state)
{
  bool consider_footprint = false;
  std::string motion_model = "ackermann";
  std::vector<std::string> critics = {{"ObstaclesCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_OmniConstraintCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "omni";
  std::vector<std::string> critics = {{"OmniConstraintCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_MecanumCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "omni";
  std::vector<std::string> critics = {{"MecanumCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_TwilringCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "ackermann";
  std::vector<std::string> critics = {{"TwirlingCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_PathFollowCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "ackermann";
  std::vector<std::string> critics = {{"PathFollowCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

static void BM_PathAngleCritic(benchmark::State & state)
{
  bool consider_footprint = true;
  std::string motion_model = "ackermann";
  std::vector<std::string> critics = {{"PathAngleCritic"}};

  prepareAndRunBenchmark(consider_footprint, motion_model, critics, state);
}

// Rollout of the sampled controls through the motion model's dynamics, the hot path of predict()
template<typename ModelT>
void runPredictBenchmark(
  benchmark::State & state, float delay, float ax_max = 3.0f, float ax_min = -3.0f)
{
  constexpr unsigned int batch_size = 2000, time_steps = 56;
  ModelT model;
  auto node = std::make_shared<nav2::LifecycleNode>("predict_benchmark");
  for (const char * axis : {"vx", "vy", "wz"}) {
    node->declare_parameter(
      std::string("bench.model_delay_") + axis, rclcpp::ParameterValue(delay));
  }
  std::string name = "bench";
  mppi::ParametersHandler param_handler(node, name);
  model.on_configure(&param_handler, name, name + ".model");
  mppi::models::ControlConstraints constraints{0.5f, -0.35f, 0.5f, 1.9f, ax_max, ax_min, ax_min,
    ax_max, 3.5f};
  model.setConstraints(constraints, 0.05f, false);
  for (double v : {0.1, 0.2, 0.3, 0.4}) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.linear.y = v;
    cmd.angular.z = v;
    model.onCommandSent(cmd);
  }

  mppi::models::State s;
  s.reset(batch_size, time_steps);
  s.vx.col(0).setConstant(0.2f);
  s.vy.col(0).setConstant(0.1f);
  s.wz.col(0).setConstant(0.3f);
  s.cvx = Eigen::ArrayXXf::Random(batch_size, time_steps) * 0.6f;
  s.cvy = Eigen::ArrayXXf::Random(batch_size, time_steps) * 0.6f;
  s.cwz = Eigen::ArrayXXf::Random(batch_size, time_steps) * 2.0f;

  for (auto _ : state) {
    model.predict(s);
    benchmark::DoNotOptimize(s.vx.data());
    benchmark::ClobberMemory();
  }
}

static void BM_PredictDiffDrive(benchmark::State & state)
{
  runPredictBenchmark<mppi::DiffDriveMotionModel>(state, 0.0f);
}

static void BM_PredictDiffDriveDelay(benchmark::State & state)
{
  runPredictBenchmark<mppi::DiffDriveMotionModel>(state, 0.2f);
}

static void BM_PredictOmni(benchmark::State & state)
{
  runPredictBenchmark<mppi::OmniMotionModel>(state, 0.0f);
}

static void BM_PredictDiffDriveAsymmetricAccel(benchmark::State & state)
{
  runPredictBenchmark<mppi::DiffDriveMotionModel>(state, 0.0f, 0.6f, -1.7f);
}

static void BM_PredictOmniAsymmetricAccel(benchmark::State & state)
{
  runPredictBenchmark<mppi::OmniMotionModel>(state, 0.0f, 0.6f, -1.7f);
}

BENCHMARK(BM_PredictDiffDrive)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_PredictDiffDriveAsymmetricAccel)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_PredictOmniAsymmetricAccel)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_DiffDriveAsymmetricAccel)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_OmniAsymmetricAccel)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_PredictDiffDriveDelay)->Unit(benchmark::kMicrosecond);
BENCHMARK(BM_PredictOmni)->Unit(benchmark::kMicrosecond);

BENCHMARK(BM_DiffDrivePointFootprint)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_DiffDrive)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Omni)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Ackermann)->Unit(benchmark::kMillisecond);

BENCHMARK(BM_GoalCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_GoalAngleCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_PathAngleCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_PathFollowCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_ObstaclesCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_ObstaclesCriticPointFootprint)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_OmniConstraintCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_MecanumCritic)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_TwilringCritic)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
