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

#include <chrono>
#include <cmath>
#include <deque>
#include <random>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/utilities.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_ros_common/tf2_factories.hpp"

// Tests main optimizer functions

using namespace mppi;  // NOLINT
using namespace mppi::critics;  // NOLINT
using namespace mppi::utils;  // NOLINT

class OptimizerTester : public Optimizer
{
public:
  OptimizerTester()
  : Optimizer() {}

  void testSetDiffModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("diff_drive"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<DiffDriveMotionModel *>(motion_model_.get()));
    EXPECT_FALSE(isHolonomic());
  }

  void testSetOmniModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("omni"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<OmniMotionModel *>(motion_model_.get()));
    EXPECT_TRUE(isHolonomic());
  }

  void testSetAckModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("ackermann"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<AckermannMotionModel *>(motion_model_.get()));
    EXPECT_FALSE(isHolonomic());
  }

  void testSetModelMissingPlugin()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    // Do not declare plugin type so the loader throws
    try {
      setMotionModel("no_plugin_model");
      FAIL();
    } catch (...) {
      SUCCEED();
    }
    EXPECT_EQ(motion_model_.get(), nullptr);
  }

  void testSetRandModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    // Pre-declare an invalid plugin type so the loader throws
    auto node = parent_.lock();
    node->declare_parameter(
      name_ + ".invalid_model.plugin",
      rclcpp::ParameterValue("mppi::NoSuchMotionModel"));
    try {
      setMotionModel("invalid_model");
      FAIL();
    } catch (...) {
      SUCCEED();
    }
    EXPECT_EQ(motion_model_.get(), nullptr);
  }

  void resetMotionModel()
  {
    motion_model_.reset();
  }

  void setOffsetWrapper(const double controller_period)
  {
    return setOffset(controller_period);
  }

  bool getShiftControlSequence()
  {
    return settings_.shift_control_sequence;
  }

  void fillOptimizerWithGarbage()
  {
    state_.vx = 0.43432 * Eigen::ArrayXXf::Ones(1000, 10);
    control_sequence_.vx = 342.0 * Eigen::ArrayXf::Ones(30);
    control_history_[0] = {43, 5646, 32432};
    costs_ = 5.32 * Eigen::ArrayXf::Ones(56453);
    generated_trajectories_.x = 432.234 * Eigen::ArrayXf::Ones(7865);
  }

  void testReset()
  {
    reset();

    EXPECT_TRUE(state_.vx.isApproxToConstant(0.0f));
    EXPECT_TRUE(control_sequence_.vx.isApproxToConstant(0.0f));
    EXPECT_EQ(control_history_[0].vx, 0.0);
    EXPECT_EQ(control_history_[0].vy, 0.0);
    EXPECT_NEAR(costs_.sum(), 0, 1e-6);
    EXPECT_TRUE(generated_trajectories_.x.isApproxToConstant(0.0f));
  }

  bool fallbackWrapper(bool fail)
  {
    return fallback(fail);
  }

  void prepareWrapper(
    const geometry_msgs::msg::Twist & robot_speed, const geometry_msgs::msg::Twist & last_command)
  {
    last_command_vel_ = last_command;
    geometry_msgs::msg::PoseStamped pose;
    nav_msgs::msg::Path plan;
    plan.poses.resize(2);
    prepare(pose, robot_speed, plan, geometry_msgs::msg::Pose(), nullptr);
  }

  void testPrepare(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::Pose & goal,
    nav2_core::GoalChecker * goal_checker)
  {
    prepare(robot_pose, robot_speed, plan, goal, goal_checker);

    EXPECT_EQ(critics_data_.goal_checker, nullptr);
    EXPECT_NEAR(costs_.sum(), 0, 1e-6);  // should be reset
    EXPECT_FALSE(critics_data_.fail_flag);  // should be reset
    EXPECT_FALSE(critics_data_.motion_model->isHolonomic());  // object is valid + diff drive
    EXPECT_FALSE(critics_data_.furthest_reached_path_point.has_value());  // val is not set
    EXPECT_FALSE(critics_data_.path_pts_valid.has_value());  // val is not set
    EXPECT_EQ(state_.pose.pose.position.x, 999);
    EXPECT_EQ(state_.speed.linear.y, 4.0);
    EXPECT_EQ(path_.x.rows(), 17);
  }

  void shiftControlSequenceWrapper()
  {
    return shiftControlSequence();
  }

  std::pair<double, double> getVelLimits()
  {
    auto & s = settings_;
    return {s.constraints.vx_min, s.constraints.vx_max};
  }

  const geometry_msgs::msg::Twist & getLastCommandVel()
  {
    return last_command_vel_;
  }

  models::ControlConstraints & getControlConstraints()
  {
    return settings_.constraints;
  }

  models::ControlConstraints & getBaseConstraints()
  {
    return settings_.base_constraints;
  }

  bool isSpeedLimitActiveWrapper() const
  {
    return isSpeedLimitActive();
  }

  void applyControlSequenceConstraintsWrapper()
  {
    return applyControlSequenceConstraints();
  }

  /**
   * @brief Run a control cycle as evalControl() does, keeping the optimal control sequence before
   * it is shifted and the velocity it was constrained to continue from
   */
  geometry_msgs::msg::Twist runCycle(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    const nav_msgs::msg::Path & plan, const geometry_msgs::msg::Pose & goal,
    geometry_msgs::msg::Twist & start, models::ControlSequence & sequence)
  {
    prepare(pose, speed, plan, goal, nullptr);
    optimize();
    start = motion_model_->getControlStart(state_.speed);
    sequence = control_sequence_;
    auto cmd = getControlFromSequenceAsTwist(plan.header.stamp);
    last_command_vel_ = cmd.twist;
    if (settings_.shift_control_sequence) {
      shiftControlSequence();
    }
    return cmd.twist;
  }

  void generateNoisedTrajectoriesWrapper()
  {
    generateNoisedTrajectories();
  }

  void applyControlSequenceInterIterationConstraintsWrapper()
  {
    return applyControlSequenceInterIterationConstraints();
  }

  models::OptimizerSettings & grabSettings()
  {
    return settings_;
  }

  models::State & grabState()
  {
    return state_;
  }

  models::ControlSequence & grabControlSequence()
  {
    return control_sequence_;
  }

  std::shared_ptr<MotionModel> getMotionModel()
  {
    return motion_model_;
  }

  void testupdateStateVels()
  {
    // updateInitialStateVelocities
    models::State state;
    state.reset(1000, 50);
    state.speed.linear.x = 5.0;
    state.speed.linear.y = 1.0;
    state.speed.angular.z = 6.0;
    state.cvx = 0.75 * Eigen::ArrayXXf::Ones(1000, 50);
    state.cvy = 0.5 * Eigen::ArrayXXf::Ones(1000, 50);
    state.cwz = 0.1 * Eigen::ArrayXXf::Ones(1000, 50);
    updateInitialStateVelocities(state);
    EXPECT_NEAR(state.vx(0, 0), 5.0, 1e-6);
    EXPECT_NEAR(state.vy(0, 0), 1.0, 1e-6);
    EXPECT_NEAR(state.wz(0, 0), 6.0, 1e-6);

    // propagateStateVelocitiesFromInitials
    float max_delta_vx = settings_.constraints.ax_max * settings_.model_dt;
    float min_delta_vx = settings_.constraints.ax_min * settings_.model_dt;
    float max_delta_vy = settings_.constraints.ay_max * settings_.model_dt;
    float min_delta_vy = settings_.constraints.ay_min * settings_.model_dt;
    float max_delta_wz = settings_.constraints.az_max * settings_.model_dt;
    propagateStateVelocitiesFromInitials(state);
    EXPECT_NEAR(state.vx(0, 0), 5.0, 1e-6);
    EXPECT_NEAR(state.vy(0, 0), 1.0, 1e-6);
    EXPECT_NEAR(state.wz(0, 0), 6.0, 1e-6);
    EXPECT_NEAR(state.vx(0, 1), std::clamp(0.75, 5.0 + min_delta_vx, 5.0 + max_delta_vx), 1e-6);
    EXPECT_NEAR(state.vy(0, 1), std::clamp(0.5, 1.0 + min_delta_vy, 1.0 + max_delta_vy), 1e-6);
    EXPECT_NEAR(state.wz(0, 1), std::clamp(0.1, 6.0 - max_delta_wz, 6.0 + max_delta_wz), 1e-6);

    // Putting them together: updateStateVelocities
    state.reset(1000, 50);
    state.speed.linear.x = -5.0;
    state.speed.linear.y = -1.0;
    state.speed.angular.z = -6.0;
    state.cvx = -0.75 * Eigen::ArrayXXf::Ones(1000, 50);
    state.cvy = -0.5 * Eigen::ArrayXXf::Ones(1000, 50);
    state.cwz = -0.1 * Eigen::ArrayXXf::Ones(1000, 50);
    updateStateVelocities(state);
    EXPECT_NEAR(state.vx(0, 0), -5.0, 1e-6);
    EXPECT_NEAR(state.vy(0, 0), -1.0, 1e-6);
    EXPECT_NEAR(state.wz(0, 0), -6.0, 1e-6);
    EXPECT_NEAR(state.vx(0, 1), std::clamp(-0.75, -5.0 + min_delta_vx, -5.0 + max_delta_vx), 1e-6);
    EXPECT_NEAR(state.vy(0, 1), std::clamp(-0.5, -1.0 - max_delta_vy, -1.0 + max_delta_vy), 1e-6);
    EXPECT_NEAR(state.wz(0, 1), std::clamp(-0.1, -6.0 - max_delta_wz, -6.0 + max_delta_wz), 1e-6);
  }

  geometry_msgs::msg::TwistStamped getControlFromSequenceAsTwistWrapper()
  {
    builtin_interfaces::msg::Time stamp;
    return getControlFromSequenceAsTwist(stamp);
  }

  void integrateStateVelocitiesWrapper(
    models::Trajectories & traj,
    const models::State & state)
  {
    return integrateStateVelocities(traj, state);
  }
};

TEST(OptimizerTests, DefaultInvalidInitialSize)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(100001));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(1001));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);

  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);
  EXPECT_EQ(optimizer_tester.grabSettings().batch_size, 1000u);
  EXPECT_EQ(optimizer_tester.grabSettings().time_steps, 56u);
}

TEST(OptimizerTests, BasicInitializedFunctions)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.ax_min", rclcpp::ParameterValue(3.0));
  node->declare_parameter("mppic.ay_min", rclcpp::ParameterValue(3.0));

  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  node->declare_parameter(
    "mppic.omni.plugin", rclcpp::ParameterValue("mppi::OmniMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "mppic";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);
  param_handler.start();

  auto result = node->set_parameter(rclcpp::Parameter("mppic.batch_size", 0));
  EXPECT_FALSE(result.successful);
  result = node->set_parameter(rclcpp::Parameter("mppic.batch_size", 100001));
  EXPECT_FALSE(result.successful);
  result = node->set_parameter(rclcpp::Parameter("mppic.time_steps", 0));
  EXPECT_FALSE(result.successful);
  result = node->set_parameter(rclcpp::Parameter("mppic.time_steps", 1001));
  EXPECT_FALSE(result.successful);

  // Test value of ax_min, ay_min it should be negative
  auto & constraints = optimizer_tester.getControlConstraints();
  EXPECT_EQ(constraints.ax_min, -3.0);
  EXPECT_EQ(constraints.ay_min, -3.0);

  // Should be empty of size batches x time steps
  // and tests getting set params: time_steps, batch_size, controller_frequency
  auto trajs = optimizer_tester.getGeneratedTrajectories();
  EXPECT_EQ(trajs.x.rows(), 1000);
  EXPECT_EQ(trajs.x.cols(), 50);
  EXPECT_TRUE(trajs.x.isApproxToConstant(0.0f));

  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  auto traj = optimizer_tester.getOptimizedTrajectory();
  EXPECT_EQ(traj(5, 0), 0.0);  // x
  EXPECT_EQ(traj(5, 1), 0.0);  // y
  EXPECT_EQ(traj(5, 2), 0.0);  // yaw
  EXPECT_EQ(traj.rows(), 50);
  EXPECT_EQ(traj.cols(), 3);

  optimizer_tester.reset();
  optimizer_tester.shutdown();
}

TEST(OptimizerTests, TestOptimizerMotionModels)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  // Declare plugin types for the motion model tests
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  node->declare_parameter(
    "mppic.omni.plugin", rclcpp::ParameterValue("mppi::OmniMotionModel"));
  node->declare_parameter(
    "mppic.ackermann.plugin", rclcpp::ParameterValue("mppi::AckermannMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Diff Drive should be non-holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetDiffModel();

  // Omni Drive should be holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  // // Ackermann should be non-holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetAckModel();

  // // Missing motion model plugin type, it should fail
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetModelMissingPlugin();

  // // Rand should fail
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetRandModel();
}

TEST(OptimizerTests, setOffsetTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.1));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Test offsets are properly set based on relationship of model_dt and controller period
  // Also tests getting set model_dt parameter.
  EXPECT_THROW(optimizer_tester.setOffsetWrapper(1.0), std::runtime_error);  // period >> model_dt
  EXPECT_NO_THROW(optimizer_tester.setOffsetWrapper(1.0 / 30.0));  // period < model_dt
  EXPECT_FALSE(optimizer_tester.getShiftControlSequence());
  EXPECT_NO_THROW(optimizer_tester.setOffsetWrapper(0.1));  // period == model_dt
  EXPECT_TRUE(optimizer_tester.getShiftControlSequence());
}

TEST(OptimizerTests, resetTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Tests resetting the full state of all the functions after filling with garbage
  optimizer_tester.fillOptimizerWithGarbage();
  optimizer_tester.testReset();
}

TEST(OptimizerTests, FallbackTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Test fallback logic, also tests getting set param retry_attempt_limit
  // Because retry set to 2, it should attempt soft resets 2x before throwing exception
  // for hard reset.
  // A successful cycle on another optimizer must not reset this one.
  EXPECT_FALSE(optimizer_tester.fallbackWrapper(false));
  EXPECT_TRUE(optimizer_tester.fallbackWrapper(true));
  OptimizerTester other_optimizer;
  EXPECT_FALSE(other_optimizer.fallbackWrapper(false));
  EXPECT_TRUE(optimizer_tester.fallbackWrapper(true));
  EXPECT_THROW(optimizer_tester.fallbackWrapper(true), std::runtime_error);
}

TEST(OptimizerTests, PrepareTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Test Prepare function to set the state of the robot pose/speed on new cycle
  // Populate the contents with things easily identifiable if correct
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 999;
  geometry_msgs::msg::Twist speed;
  speed.linear.y = 4.0;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::Pose goal;
  path.poses.resize(17);

  optimizer_tester.testPrepare(pose, speed, path, goal, nullptr);
}

class OptimizerIterationTests : public ::testing::TestWithParam<int> {};

TEST_P(OptimizerIterationTests, CostsOnlyIncludeLatestRollouts)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(32));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(12));
  node->declare_parameter("mppic.iteration_count", rclcpp::ParameterValue(GetParam()));
  node->declare_parameter("mppic.gamma", rclcpp::ParameterValue(0.0));
  node->declare_parameter("mppic.regenerate_noises", rclcpp::ParameterValue(false));
  node->declare_parameter(
    "mppic.critics", rclcpp::ParameterValue(std::vector<std::string>{"GoalCritic"}));
  node->declare_parameter("mppic.GoalCritic.cost_weight", rclcpp::ParameterValue(1.0));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  costmap_ros->on_configure(rclcpp_lifecycle::State{});
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist speed;
  auto goal = pose.pose;
  goal.position.x = 1.0;
  goal.position.y = 0.6;
  nav_msgs::msg::Path path;
  path.poses = {pose, pose};
  path.poses.back().pose = goal;
  optimizer_tester.evalControl(pose, speed, path, goal, nullptr);

  // With only GoalCritic and gamma zero, costs must equal the current mean goal distance.
  const auto & trajectories = optimizer_tester.getGeneratedTrajectories();
  const auto & costs = optimizer_tester.getCosts();
  for (Eigen::Index row = 0; row < trajectories.x.rows(); ++row) {
    double distance_sum = 0.0;
    for (Eigen::Index col = 0; col < trajectories.x.cols(); ++col) {
      distance_sum += std::hypot(
        trajectories.x(row, col) - goal.position.x,
        trajectories.y(row, col) - goal.position.y);
    }
    EXPECT_NEAR(costs(row), distance_sum / trajectories.x.cols(), 1e-5);
  }
  optimizer_tester.shutdown();
}

INSTANTIATE_TEST_SUITE_P(IterationCounts, OptimizerIterationTests, ::testing::Values(1, 2, 3));

TEST(OptimizerTests, shiftControlSequenceTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  node->declare_parameter(
    "mppic.omni.plugin", rclcpp::ParameterValue("mppi::OmniMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Test shiftControlSequence by setting the 2nd value to something unique to neighbors
  auto & sequence = optimizer_tester.grabControlSequence();
  sequence.reset(100);
  sequence.vx(0) = 9999;
  sequence.vx(1) = 6;
  sequence.vx(2) = 888;
  sequence.vy(0) = 9999;
  sequence.vy(1) = 6;
  sequence.vy(2) = 888;
  sequence.wz(0) = 9999;
  sequence.wz(1) = 6;
  sequence.wz(2) = 888;

  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  optimizer_tester.shiftControlSequenceWrapper();

  EXPECT_EQ(sequence.vx(0), 6);
  EXPECT_EQ(sequence.vy(0), 6);
  EXPECT_EQ(sequence.wz(0), 6);
  EXPECT_EQ(sequence.vx(1), 888);
  EXPECT_EQ(sequence.vy(1), 888);
  EXPECT_EQ(sequence.wz(1), 888);
  EXPECT_EQ(sequence.vx(2), 0);
  EXPECT_EQ(sequence.vy(2), 0);
  EXPECT_EQ(sequence.wz(2), 0);
}

TEST(OptimizerTests, SpeedLimitTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Test Speed limits API
  auto [v_min, v_max] = optimizer_tester.getVelLimits();
  EXPECT_EQ(v_max, 0.5f);
  EXPECT_EQ(v_min, -0.35f);
  optimizer_tester.setSpeedLimit(0, false);
  auto [v_min2, v_max2] = optimizer_tester.getVelLimits();
  EXPECT_EQ(v_max2, 0.5f);
  EXPECT_EQ(v_min2, -0.35f);
  optimizer_tester.setSpeedLimit(50.0, true);
  auto [v_min3, v_max3] = optimizer_tester.getVelLimits();
  EXPECT_NEAR(v_max3, 0.5 / 2.0, 1e-3);
  EXPECT_NEAR(v_min3, -0.35 / 2.0, 1e-3);
  optimizer_tester.setSpeedLimit(0, true);
  auto [v_min4, v_max4] = optimizer_tester.getVelLimits();
  EXPECT_EQ(v_max4, 0.5f);
  EXPECT_EQ(v_min4, -0.35f);
  optimizer_tester.setSpeedLimit(0.75, false);
  auto [v_min5, v_max5] = optimizer_tester.getVelLimits();
  EXPECT_NEAR(v_max5, 0.75, 1e-3);
  EXPECT_NEAR(v_min5, -0.5249, 1e-2);
}

TEST(OptimizerTests, SpeedLimitReverseOnlyTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(0.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-0.30));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(1.9));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  const auto base = optimizer_tester.getBaseConstraints();
  EXPECT_EQ(base.vx_max, 0.0f);
  EXPECT_EQ(base.vx_min, -0.30f);

  // Absolute limit on a reverse-only base is relative to |vx_min|
  optimizer_tester.setSpeedLimit(0.2, false);
  const auto & c = optimizer_tester.getControlConstraints();
  const double ratio = 0.2 / 0.30;
  EXPECT_TRUE(std::isfinite(c.vx_max));
  EXPECT_TRUE(std::isfinite(c.vx_min));
  EXPECT_TRUE(std::isfinite(c.vy));
  EXPECT_TRUE(std::isfinite(c.wz));
  EXPECT_EQ(c.vx_max, 0.0f);
  EXPECT_NEAR(c.vx_min, -0.2, 1e-4);
  EXPECT_NEAR(c.vy, base.vy * ratio, 1e-4);
  EXPECT_NEAR(c.wz, 1.9 * ratio, 1e-4);
  EXPECT_TRUE(optimizer_tester.isSpeedLimitActiveWrapper());

  // Clearing the limit restores the base constraints
  optimizer_tester.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_EQ(c.vx_max, 0.0f);
  EXPECT_EQ(c.vx_min, -0.30f);
  EXPECT_EQ(c.wz, 1.9f);
  EXPECT_FALSE(optimizer_tester.isSpeedLimitActiveWrapper());
}

TEST(OptimizerTests, SpeedLimitZeroLinearVelocityTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(0.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(0.0));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(1.9));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  const auto base = optimizer_tester.getBaseConstraints();
  const auto & c = optimizer_tester.getControlConstraints();

  // An absolute limit cannot be applied when vx_max and vx_min are both zero:
  // the update is rejected and the constraints are left untouched
  optimizer_tester.setSpeedLimit(0.2, false);
  EXPECT_EQ(c.vx_max, 0.0f);
  EXPECT_EQ(c.vx_min, 0.0f);
  EXPECT_EQ(c.vy, base.vy);
  EXPECT_EQ(c.wz, base.wz);
  EXPECT_FALSE(optimizer_tester.isSpeedLimitActiveWrapper());

  // A rejected update keeps the previously applied limit instead of resetting it
  optimizer_tester.setSpeedLimit(50.0, true);
  EXPECT_NEAR(c.wz, base.wz * 0.5, 1e-4);
  optimizer_tester.setSpeedLimit(0.2, false);
  EXPECT_EQ(c.vx_max, 0.0f);
  EXPECT_EQ(c.vx_min, 0.0f);
  EXPECT_NEAR(c.vy, base.vy * 0.5, 1e-4);
  EXPECT_NEAR(c.wz, base.wz * 0.5, 1e-4);
  EXPECT_TRUE(std::isfinite(c.vy));
  EXPECT_TRUE(std::isfinite(c.wz));
}

TEST(OptimizerTests, applyControlSequenceConstraintsTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.vy_max", rclcpp::ParameterValue(0.75));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(2.0));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  node->declare_parameter(
    "mppic.omni.plugin", rclcpp::ParameterValue("mppi::OmniMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Test constraints being applied to ensure feasibility of trajectories
  // Also tests param get of set vx/vy/wz min/maxes

  // Set model to omni to consider holonomic vy elements
  // Ack is not tested here because `applyConstraints` is covered in detail
  // in motion_models_test.cpp
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  auto & sequence = optimizer_tester.grabControlSequence();
  auto & state = optimizer_tester.grabState();

  // Test boundary of limits
  // Set state speed to match so acceleration constraints are satisfied
  state.speed.linear.x = 1.0;
  state.speed.linear.y = 0.75;
  state.speed.angular.z = 2.0;
  sequence.vx = Eigen::ArrayXf::Ones(50);
  sequence.vy = 0.75 * Eigen::ArrayXf::Ones(50);
  sequence.wz = 2.0 * Eigen::ArrayXf::Ones(50);
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_TRUE(sequence.vx.isApproxToConstant(1.0f));
  EXPECT_TRUE(sequence.vy.isApproxToConstant(0.75f));
  EXPECT_TRUE(sequence.wz.isApproxToConstant(2.0f));

  // Test breaking limits sets to maximum
  state.speed.linear.x = 1.0;
  state.speed.linear.y = 0.75;
  state.speed.angular.z = 2.0;
  sequence.vx = 5.0 * Eigen::ArrayXf::Ones(50);
  sequence.vy = 5.0 * Eigen::ArrayXf::Ones(50);
  sequence.wz = 5.0 * Eigen::ArrayXf::Ones(50);
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_TRUE(sequence.vx.isApproxToConstant(1.0f));
  EXPECT_TRUE(sequence.vy.isApproxToConstant(0.75f));
  EXPECT_TRUE(sequence.wz.isApproxToConstant(2.0f));

  // Test breaking limits sets to minimum
  state.speed.linear.x = -1.0;
  state.speed.linear.y = -0.75;
  state.speed.angular.z = -2.0;
  sequence.vx = -5.0 * Eigen::ArrayXf::Ones(50);
  sequence.vy = -5.0 * Eigen::ArrayXf::Ones(50);
  sequence.wz = -5.0 * Eigen::ArrayXf::Ones(50);
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_TRUE(sequence.vx.isApproxToConstant(-1.0f));
  EXPECT_TRUE(sequence.vy.isApproxToConstant(-0.75f));
  EXPECT_TRUE(sequence.wz.isApproxToConstant(-2.0f));
}

TEST(OptimizerTests, CommandsInFlightTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(10));
  // vx commands are in flight for two model_dt steps, wz only has the inherent latency
  node->declare_parameter("mppic.model_delay_vx", rclcpp::ParameterValue(0.15));
  node->declare_parameter("mppic.model_delay_wz", rclcpp::ParameterValue(0.05));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // The robot lags behind the commands sent, which are still in flight
  auto & state = optimizer_tester.grabState();
  auto & sequence = optimizer_tester.grabControlSequence();
  state.speed.linear.x = 0.1;
  state.speed.angular.z = 0.2;
  for (int i = 0; i < 3; ++i) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.4;
    cmd.angular.z = 0.6;
    optimizer_tester.getMotionModel()->onCommandSent(cmd);
  }

  // So the next command continues from the last command rather than the measured velocity
  sequence.vx = Eigen::ArrayXf::Ones(10);
  sequence.wz = Eigen::ArrayXf::Ones(10);
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_NEAR(sequence.vx(0), 0.4, 1e-6);
  EXPECT_NEAR(sequence.wz(0), 0.2, 1e-6);

  // And the optimal trajectory includes the motion from the commands in flight
  sequence.vx.setZero();
  sequence.wz.setZero();
  auto trajectory = optimizer_tester.getOptimizedTrajectory();
  EXPECT_GT(trajectory(2, 0), 0.02);
}

TEST(OptimizerTests, OptimalTrajectoryMatchesSampledRolloutTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(10));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // At rest, with an optimal sequence asking for 0.5 m/s
  auto & state = optimizer_tester.grabState();
  auto & sequence = optimizer_tester.grabControlSequence();
  state.speed = geometry_msgs::msg::Twist();
  sequence.vx = Eigen::ArrayXf::Constant(10, 0.5f);
  sequence.wz.setZero();
  auto trajectory = optimizer_tester.getOptimizedTrajectory();

  // As the sampled trajectories, the first step moves at the current speed and the controls take
  // effect after it within the acceleration limits (3.0 m/s^2 over 0.05s steps), rather than the
  // robot moving at the optimal controls at once
  EXPECT_NEAR(trajectory(0, 0), 0.0, 1e-6);
  EXPECT_NEAR(trajectory(1, 0), 0.15 * 0.05, 1e-6);
  EXPECT_NEAR(trajectory(2, 0), (0.15 + 0.3) * 0.05, 1e-6);
}

TEST(OptimizerTests, updateStateVelocitiesTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.vy_max", rclcpp::ParameterValue(0.60));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(2.0));
  node->declare_parameter("mppic.ax_max", rclcpp::ParameterValue(3.0));
  node->declare_parameter("mppic.ax_min", rclcpp::ParameterValue(-3.0));
  node->declare_parameter("mppic.ay_max", rclcpp::ParameterValue(3.0));
  node->declare_parameter("mppic.ay_min", rclcpp::ParameterValue(-3.0));
  node->declare_parameter("mppic.az_max", rclcpp::ParameterValue(3.5));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  node->declare_parameter(
    "mppic.omni.plugin", rclcpp::ParameterValue("mppi::OmniMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Test settings of the state to the initial robot speed to start rollout
  // Set model to omni to consider holonomic vy elements
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  optimizer_tester.testupdateStateVels();
}

TEST(OptimizerTests, getControlFromSequenceAsTwistTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.vy_max", rclcpp::ParameterValue(0.60));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(2.0));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  node->declare_parameter(
    "mppic.omni.plugin", rclcpp::ParameterValue("mppi::OmniMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Test conversion of control sequence into a Twist command to execute
  auto & sequence = optimizer_tester.grabControlSequence();
  sequence.vx = 0.25 * Eigen::ArrayXf::Ones(10);
  sequence.vy = 0.5 * Eigen::ArrayXf::Ones(10);
  sequence.wz = 0.1 * Eigen::ArrayXf::Ones(10);

  auto diff_t = optimizer_tester.getControlFromSequenceAsTwistWrapper();
  EXPECT_NEAR(diff_t.twist.linear.x, 0.25, 1e-6);
  EXPECT_NEAR(diff_t.twist.linear.y, 0.0, 1e-6);  // Y should not be populated
  EXPECT_NEAR(diff_t.twist.angular.z, 0.1, 1e-6);

  // Set model to omni to consider holonomic vy elements
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  auto omni_t = optimizer_tester.getControlFromSequenceAsTwistWrapper();
  EXPECT_NEAR(omni_t.twist.linear.x, 0.25, 1e-6);
  EXPECT_NEAR(omni_t.twist.linear.y, 0.5, 1e-6);  // Now it should be
  EXPECT_NEAR(omni_t.twist.angular.z, 0.1, 1e-6);
}

TEST(OptimizerTests, integrateStateVelocitiesTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.1));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  node->declare_parameter(
    "mppic.omni.plugin", rclcpp::ParameterValue("mppi::OmniMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  // Test integration of velocities for trajectory rollout poses

  // Give it a couple of easy const traj and check rollout, start from 0
  models::State state;
  state.reset(1000, 50);
  models::Trajectories traj;
  traj.reset(1000, 50);
  state.vx = 0.1 * Eigen::ArrayXXf::Ones(1000, 50);
  state.vx.col(0) = Eigen::ArrayXf::Zero(1000);
  state.vy = Eigen::ArrayXXf::Zero(1000, 50);
  state.wz = Eigen::ArrayXXf::Zero(1000, 50);
  optimizer_tester.integrateStateVelocitiesWrapper(traj, state);
  EXPECT_TRUE(traj.y.isApproxToConstant(0.0f));
  EXPECT_TRUE(traj.yaws.isApproxToConstant(0.0f));
  for (unsigned int i = 0; i != traj.x.cols(); i++) {
    EXPECT_NEAR(traj.x(1, i), i * 0.1 /*vel*/ * 0.1 /*dt*/, 1e-3);
  }

  // Give it a bit of a more complex trajectory to crunch
  state.vy = 0.2 * Eigen::ArrayXXf::Ones(1000, 50);
  state.vy.col(0) = Eigen::ArrayXf::Zero(1000);
  optimizer_tester.integrateStateVelocitiesWrapper(traj, state);

  EXPECT_TRUE(traj.yaws.isApproxToConstant(0.0f));
  for (unsigned int i = 0; i != traj.x.cols(); i++) {
    EXPECT_NEAR(traj.x(1, i), i * 0.1 /*vel*/ * 0.1 /*dt*/, 1e-3);
    EXPECT_NEAR(traj.y(1, i), i * 0.2 /*vel*/ * 0.1 /*dt*/, 1e-3);
  }

  // Let's add some angular motion to the mix
  state.vy = Eigen::ArrayXXf::Zero(1000, 50);
  state.wz = 0.2 * Eigen::ArrayXXf::Ones(1000, 50);
  state.wz.col(0) = Eigen::ArrayXf::Zero(1000);
  optimizer_tester.integrateStateVelocitiesWrapper(traj, state);

  float x = 0;
  float y = 0;
  for (unsigned int i = 1; i != traj.x.cols(); i++) {
    x += (0.1 /*vx*/ * cosf(0.2 /*wz*/ * 0.1 /*model_dt*/ * (i - 1))) * 0.1 /*model_dt*/;
    y += (0.1 /*vx*/ * sinf(0.2 /*wz*/ * 0.1 /*model_dt*/ * (i - 1))) * 0.1 /*model_dt*/;

    EXPECT_NEAR(traj.x(1, i), x, 1e-6);
    EXPECT_NEAR(traj.y(1, i), y, 1e-6);
  }
}

TEST(OptimizerTests, TestGetters)
{
  OptimizerTester optimizer_tester;
  optimizer_tester.fillOptimizerWithGarbage();

  auto control_seq = optimizer_tester.getOptimalControlSequence();
  EXPECT_EQ(control_seq.vx(0), 342.0);
  EXPECT_EQ(control_seq.vx.rows(), 30);

  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string test = "test";
  ParametersHandler param_handler(node, test);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);
  EXPECT_EQ(optimizer_tester.getSettings().model_dt, 0.05f);
}

TEST(OptimizerTests, Omni_openLoopMppiTest)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.05));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(80));
  node->declare_parameter("mppic.ax_max", rclcpp::ParameterValue(0.5));
  node->declare_parameter("mppic.az_max", rclcpp::ParameterValue(0.5));
  node->declare_parameter("mppic.ay_max", rclcpp::ParameterValue(0.5));
  node->declare_parameter("mppic.open_loop", rclcpp::ParameterValue(true));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  node->declare_parameter(
    "mppic.omni.plugin", rclcpp::ParameterValue("mppi::OmniMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 999;
  geometry_msgs::msg::Twist robot_speed;
  robot_speed.linear.x = 100.0;
  robot_speed.angular.z = 100.0;
  robot_speed.linear.y = 100.0;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::Pose goal;
  path.poses.resize(17);

  auto [cmd1, optimal_trajectory1] = optimizer_tester.evalControl(
    pose, robot_speed, path,
    goal, nullptr);

  EXPECT_LE(
    std::abs(cmd1.twist.linear.x),
    optimizer_tester.getSettings().controller_period *
    optimizer_tester.getControlConstraints().ax_max);
  EXPECT_LE(
    std::abs(cmd1.twist.angular.z),
    optimizer_tester.getSettings().controller_period *
    optimizer_tester.getControlConstraints().az_max);
  EXPECT_LE(
    std::abs(cmd1.twist.linear.y),
    optimizer_tester.getSettings().controller_period *
    optimizer_tester.getControlConstraints().ay_max);

  auto [cmd2, optimal_trajectory2] = optimizer_tester.evalControl(
    pose, robot_speed, path,
    goal, nullptr);

  const double vx_delta = std::abs(cmd2.twist.linear.x - cmd1.twist.linear.x);
  const double wz_delta = std::abs(cmd2.twist.angular.z - cmd1.twist.angular.z);
  const double vy_delta = std::abs(cmd2.twist.linear.y - cmd1.twist.linear.y);

  EXPECT_LE(
    vx_delta,
    optimizer_tester.getSettings().controller_period *
    optimizer_tester.getControlConstraints().ax_max);
  EXPECT_LE(
    wz_delta,
    optimizer_tester.getSettings().controller_period *
    optimizer_tester.getControlConstraints().az_max);
  EXPECT_LE(
    vy_delta,
    optimizer_tester.getSettings().controller_period *
    optimizer_tester.getControlConstraints().ay_max);
}

TEST(OptimizerTests, AccelerationLimitsTests)
{
  // Accelerating slowly but braking hard: 0.3 m/s^2 and -3.0 m/s^2 over 0.05s control periods
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.ax_max", rclcpp::ParameterValue(0.3));
  node->declare_parameter("mppic.ax_min", rclcpp::ParameterValue(-3.0));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "mppic";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);
  param_handler.start();

  // The closed loop prediction toward the last command speeds up in reverse at ax_max
  geometry_msgs::msg::Twist speed, last_command;
  speed.linear.x = -0.4;
  last_command.linear.x = -1.0;
  optimizer_tester.prepareWrapper(speed, last_command);
  EXPECT_NEAR(optimizer_tester.grabState().speed.linear.x, -0.415, 1e-6);

  // And from rest, in either direction, at ax_max rather than the braking rate
  speed.linear.x = 0.0;
  optimizer_tester.prepareWrapper(speed, last_command);
  EXPECT_NEAR(optimizer_tester.grabState().speed.linear.x, -0.015, 1e-6);
  last_command.linear.x = 1.0;
  speed.linear.x = -0.001;
  optimizer_tester.prepareWrapper(speed, last_command);
  EXPECT_LT(optimizer_tester.grabState().speed.linear.x, 0.015);

  // Acceleration limits of the wrong sign are rejected, as they would invert the bounds
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("mppic.ax_min", 1.0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("mppic.ax_max", -1.0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("mppic.az_max", -1.0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("mppic.ay_min", 1.0)).successful);
  EXPECT_FLOAT_EQ(optimizer_tester.getBaseConstraints().ax_min, -3.0f);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("mppic.ax_min", -2.0)).successful);
  EXPECT_FLOAT_EQ(optimizer_tester.getBaseConstraints().ax_min, -2.0f);
}

TEST(OptimizerTests, SoftResetKeepsCommandsAndSpeedLimitTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.open_loop", rclcpp::ParameterValue(true));
  node->declare_parameter("mppic.ax_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.model_delay_vx", rclcpp::ParameterValue(0.15));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "mppic";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);
  param_handler.start();

  // Moving at 0.5 m/s in open loop, with commands at 0.5 m/s in flight
  geometry_msgs::msg::Twist moving;
  moving.linear.x = 0.5;
  for (int i = 0; i < 3; ++i) {
    optimizer_tester.getMotionModel()->onCommandSent(moving);
  }
  optimizer_tester.prepareWrapper(moving, moving);
  EXPECT_NEAR(optimizer_tester.grabState().speed.linear.x, 0.5, 1e-6);

  // A parameter update while moving keeps the commands sent, so open loop still starts from the
  // last command and the next command continues from the commands in flight, rather than both
  // being reset to rest and commanding a stop far beyond the braking limit
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("mppic.gamma", 0.02)).successful);
  optimizer_tester.prepareWrapper(moving, optimizer_tester.getLastCommandVel());
  EXPECT_NEAR(optimizer_tester.grabState().speed.linear.x, 0.5, 1e-6);
  EXPECT_NEAR(
    optimizer_tester.getMotionModel()->getControlStart(geometry_msgs::msg::Twist()).linear.x,
    0.5, 1e-6);
  auto & sequence = optimizer_tester.grabControlSequence();
  sequence.vx.setZero();
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_GE(sequence.vx(1), 0.5 - 1.0 * 0.05 - 1e-6);

  // And it keeps an active speed limit, which the speed filter would not publish again
  optimizer_tester.setSpeedLimit(50.0, true);
  const float limited_vx_max = optimizer_tester.getControlConstraints().vx_max;
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("mppic.gamma", 0.03)).successful);
  EXPECT_TRUE(optimizer_tester.isSpeedLimitActiveWrapper());
  EXPECT_FLOAT_EQ(optimizer_tester.getControlConstraints().vx_max, limited_vx_max);

  // While a base acceleration update still applies, under the same speed limit
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("mppic.ax_min", -2.0)).successful);
  EXPECT_FLOAT_EQ(optimizer_tester.getControlConstraints().ax_min, -2.0f);
  EXPECT_FLOAT_EQ(optimizer_tester.getControlConstraints().vx_max, limited_vx_max);

  // A fallback keeps them too
  EXPECT_TRUE(optimizer_tester.fallbackWrapper(true));
  EXPECT_NEAR(optimizer_tester.getLastCommandVel().linear.x, 0.5, 1e-6);
  EXPECT_TRUE(optimizer_tester.isSpeedLimitActiveWrapper());

  // While a hard reset, between goals, forgets them
  optimizer_tester.reset(false);
  EXPECT_NEAR(optimizer_tester.getLastCommandVel().linear.x, 0.0, 1e-6);
  EXPECT_NEAR(
    optimizer_tester.getMotionModel()->getControlStart(geometry_msgs::msg::Twist()).linear.x,
    0.0, 1e-6);
}

TEST(OptimizerTests, PinnedControlNotSampledTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(200));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.motion_model", rclcpp::ParameterValue("omni"));
  node->declare_parameter(
    "mppic.omni.plugin", rclcpp::ParameterValue("mppi::OmniMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Shifting, vx(0) is the current speed and vx(1) the sent command, asking for full speed
  auto & state = optimizer_tester.grabState();
  state.speed.linear.x = 0.2;
  state.speed.linear.y = 0.1;
  state.speed.angular.z = 0.3;
  auto & sequence = optimizer_tester.grabControlSequence();
  sequence.vx.setConstant(0.5f);
  sequence.vy.setConstant(0.5f);
  sequence.wz.setConstant(1.9f);
  optimizer_tester.generateNoisedTrajectoriesWrapper();

  // The current speed is not sampled, so every sample's response to the sent command, vx(2),
  // is within one step of the acceleration limits (3.0 and 3.5 over 0.05s) of the current speed,
  // as the sent command itself is
  EXPECT_TRUE(state.cvx.col(0).isApproxToConstant(0.2f));
  EXPECT_TRUE(state.cvy.col(0).isApproxToConstant(0.1f));
  EXPECT_TRUE(state.cwz.col(0).isApproxToConstant(0.3f));
  EXPECT_LE((state.vx.col(2) - 0.2f).abs().maxCoeff(), 0.15f + 1e-5f);
  EXPECT_LE((state.vy.col(2) - 0.1f).abs().maxCoeff(), 0.15f + 1e-5f);
  EXPECT_LE((state.wz.col(2) - 0.3f).abs().maxCoeff(), 0.175f + 1e-5f);
}

struct FeasibilityCase
{
  std::string model;
  bool open_loop;
  double ax_max, ax_min, az_max;
  double delay;
  double frequency;
};

class AccelerationFeasibilityTests : public ::testing::TestWithParam<FeasibilityCase> {};

// Every control cycle, the sent command, the optimal control sequence and the sampled rollouts
// stay within the direction-aware acceleration limits, between and within iterations
TEST_P(AccelerationFeasibilityTests, LimitsHeldEveryCycle)
{
  const auto c = GetParam();
  const bool omni = c.model == "omni";
  auto node = std::make_shared<nav2::LifecycleNode>("feasibility_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(c.frequency));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(300));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(30));
  node->declare_parameter("mppic.open_loop", rclcpp::ParameterValue(c.open_loop));
  node->declare_parameter("mppic.ax_max", rclcpp::ParameterValue(c.ax_max));
  node->declare_parameter("mppic.ax_min", rclcpp::ParameterValue(c.ax_min));
  node->declare_parameter("mppic.ay_max", rclcpp::ParameterValue(c.ax_max));
  node->declare_parameter("mppic.ay_min", rclcpp::ParameterValue(c.ax_min));
  node->declare_parameter("mppic.az_max", rclcpp::ParameterValue(c.az_max));
  node->declare_parameter("mppic.model_delay_vx", rclcpp::ParameterValue(c.delay));
  node->declare_parameter("mppic.model_delay_vy", rclcpp::ParameterValue(c.delay));
  node->declare_parameter("mppic.model_delay_wz", rclcpp::ParameterValue(c.delay));
  node->declare_parameter("mppic.motion_model", rclcpp::ParameterValue(c.model));
  node->declare_parameter(
    "mppic." + c.model + ".plugin", rclcpp::ParameterValue(
      omni ? "mppi::OmniMotionModel" : "mppi::DiffDriveMotionModel"));
  node->declare_parameter(
    "mppic.critics", rclcpp::ParameterValue(std::vector<std::string>{"GoalCritic"}));
  node->declare_parameter("mppic.GoalCritic.threshold_to_consider", rclcpp::ParameterValue(10.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  costmap_ros->on_configure(rclcpp_lifecycle::State{});
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  const auto & settings = optimizer_tester.getSettings();
  const double period = 1.0 / c.frequency, dt = settings.model_dt;
  const auto & lim = optimizer_tester.getControlConstraints();

  // Independent reference of the velocities reachable over a duration: slowing down toward zero
  // at up to decel, and speeding up away from it (forward or reverse) at up to accel
  struct Limits
  {
    double accel, decel, duration;
  };
  auto reachable = [](const Limits & l, double v) -> std::pair<double, double> {
      const double to_zero = std::fabs(v) / l.decel;  // time to slow down to zero
      const double slowest = to_zero >= l.duration ?
        std::fabs(v) - l.decel * l.duration : -l.accel * (l.duration - to_zero);
      const double fastest = std::fabs(v) + l.accel * l.duration;
      return v >= 0.0 ? std::make_pair(slowest, fastest) : std::make_pair(-fastest, -slowest);
    };
  const Limits step_vx{lim.ax_max, -lim.ax_min, dt}, first_vx{lim.ax_max, -lim.ax_min, period};
  const Limits step_vy{lim.ay_max, -lim.ay_min, dt}, first_vy{lim.ay_max, -lim.ay_min, period};
  const Limits step_wz{lim.az_max, lim.az_max, dt}, first_wz{lim.az_max, lim.az_max, period};
  constexpr double tol = 1e-5;
  auto within = [&](const Limits & l, double from, double to) {
      const auto [lo, hi] = reachable(l, from);
      return to >= lo - tol && to <= hi + tol;
    };
  const unsigned int k = std::max(1u, static_cast<unsigned int>(std::lround(c.delay / dt)));
  const unsigned int offset = settings.shift_control_sequence ? 1 : 0;

  // One drive through goals ahead, behind (reversing while moving) and to the side, so the
  // velocities cross zero. The robot follows the commands within the acceleration limits after
  // the delay, and its measured velocity has noise around zero.
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist robot, prev_cmd;
  std::deque<geometry_msgs::msg::Twist> in_flight(k - 1);
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.002);
  double min_cmd_vx = 0.0, max_cmd_vx = 0.0;
  int sign_changes = 0;
  for (const auto & [gx, gy] : std::vector<std::pair<double, double>>{{2.0, 0.0}, {-1.0, 0.0},
      {0.5, 1.5}})
  {
    geometry_msgs::msg::Pose goal;
    goal.position.x = gx;
    goal.position.y = gy;
    goal.orientation.w = 1.0;
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "odom";
    plan.poses.resize(2);
    for (int cycle = 0; cycle < 80; ++cycle) {
      plan.poses[0] = pose;
      plan.poses[1].pose = goal;
      geometry_msgs::msg::Twist measured = robot;
      measured.linear.x += noise(rng);
      measured.linear.y = omni ? measured.linear.y + noise(rng) : 0.0;
      measured.angular.z += noise(rng);
      geometry_msgs::msg::Twist start;
      models::ControlSequence seq;
      const auto cmd = optimizer_tester.runCycle(pose, measured, plan, goal, start, seq);
      const auto & state = optimizer_tester.grabState();
      const std::string id = "cycle " + std::to_string(cycle) + " goal " + std::to_string(gx) +
        "," + std::to_string(gy);

      // Between iterations: the sent command is reachable from the velocity the control
      // sequence continues from, within a control period
      EXPECT_TRUE(within(first_vx, start.linear.x, cmd.linear.x)) << id;
      EXPECT_TRUE(within(first_wz, start.angular.z, cmd.angular.z)) << id;
      if (omni) {
        EXPECT_TRUE(within(first_vy, start.linear.y, cmd.linear.y)) << id;
      }
      if (offset == 1) {
        EXPECT_FLOAT_EQ(seq.vx(0), start.linear.x) << id;
      }
      // Which, in open loop or with commands in flight, is the previous command
      if (c.open_loop || k > 1) {
        EXPECT_TRUE(within(first_vx, prev_cmd.linear.x, cmd.linear.x)) << id;
        EXPECT_TRUE(within(first_wz, prev_cmd.angular.z, cmd.angular.z)) << id;
      } else {
        // And in closed loop, the predicted velocity is reachable from the measured one
        EXPECT_TRUE(
          within(first_vx, measured.linear.x, optimizer_tester.grabState().speed.linear.x)) << id;
      }

      // Within an iteration: each control of the optimal sequence is reachable from the last
      for (unsigned int i = offset + 1; i < seq.vx.size(); ++i) {
        EXPECT_TRUE(within(step_vx, seq.vx(i - 1), seq.vx(i))) << id << " step " << i;
        EXPECT_TRUE(within(step_wz, seq.wz(i - 1), seq.wz(i))) << id << " step " << i;
        if (omni) {
          EXPECT_TRUE(within(step_vy, seq.vy(i - 1), seq.vy(i))) << id << " step " << i;
        }
        // And stays within the velocity limits once within them
        if (seq.vx(i - 1) <= lim.vx_max + tol && seq.vx(i - 1) >= lim.vx_min - tol) {
          EXPECT_LE(seq.vx(i), lim.vx_max + tol) << id;
          EXPECT_GE(seq.vx(i), lim.vx_min - tol) << id;
        }
      }

      // The sampled rollouts: each velocity after the commands in flight is reachable from the
      // one before it
      for (unsigned int i = k + 1; i < state.vx.cols(); ++i) {
        for (Eigen::Index b = 0; b < state.vx.rows(); ++b) {
          if (!within(step_vx, state.vx(b, i - 1), state.vx(b, i)) ||
            !within(step_wz, state.wz(b, i - 1), state.wz(b, i)) ||
            (omni && !within(step_vy, state.vy(b, i - 1), state.vy(b, i))))
          {
            ADD_FAILURE() << id << " rollout " << b << " step " << i;
            i = state.vx.cols();
            break;
          }
        }
      }

      // Step the robot forward a control period
      in_flight.push_back(cmd);
      const auto applied = in_flight.front();
      in_flight.pop_front();
      robot.linear.x = clampVelocityByAccel(
        robot.linear.x, applied.linear.x, period * c.ax_min, period * c.ax_max);
      robot.linear.y = omni ? clampVelocityByAccel(
        robot.linear.y, applied.linear.y, period * c.ax_min, period * c.ax_max) : 0.0;
      robot.angular.z = clampVelocityByAccel(
        robot.angular.z, applied.angular.z, -period * c.az_max, period * c.az_max);
      const double yaw = tf2::getYaw(pose.pose.orientation) + robot.angular.z * period;
      pose.pose.position.x += (robot.linear.x * std::cos(yaw) - robot.linear.y * std::sin(yaw)) *
        period;
      pose.pose.position.y += (robot.linear.x * std::sin(yaw) + robot.linear.y * std::cos(yaw)) *
        period;
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
      min_cmd_vx = std::min(min_cmd_vx, cmd.linear.x);
      max_cmd_vx = std::max(max_cmd_vx, cmd.linear.x);
      sign_changes += (cmd.linear.x > 0.0) != (prev_cmd.linear.x > 0.0);
      prev_cmd = cmd;
    }
  }
  // The scenarios drive forward and in reverse, through zero
  EXPECT_GT(max_cmd_vx, 0.05);
  EXPECT_LT(min_cmd_vx, -0.02);
  EXPECT_GE(sign_changes, 2);
  optimizer_tester.shutdown();
}

std::vector<FeasibilityCase> feasibilityCases()
{
  std::vector<FeasibilityCase> cases;
  for (const std::string model : {"diff_drive", "omni"}) {
    for (bool open_loop : {false, true}) {
      // High, low, accelerating slower than braking, and braking slower than accelerating
      for (const auto & [ax_max, ax_min, az_max] : std::vector<std::tuple<double, double, double>>{
        {3.0, -3.0, 3.5}, {0.3, -0.3, 0.5}, {0.6, -1.7, 1.0}, {1.7, -0.6, 1.0}})
      {
        for (double delay : {0.0, 0.2}) {
          // Shifting the control sequence (controller period == model_dt) or not
          for (double frequency : {20.0, 30.0}) {
            cases.push_back({model, open_loop, ax_max, ax_min, az_max, delay, frequency});
          }
        }
      }
    }
  }
  return cases;
}

INSTANTIATE_TEST_SUITE_P(
  Feasibility, AccelerationFeasibilityTests, ::testing::ValuesIn(feasibilityCases()),
  [](const ::testing::TestParamInfo<FeasibilityCase> & param_info) {
    const auto & c = param_info.param;
    char name[128];
    snprintf(
      name, sizeof(name), "%s_%s_ax%.0f_%.0f_delay%.0f_hz%.0f", c.model.c_str(),
      c.open_loop ? "open" : "closed", c.ax_max * 10, -c.ax_min * 10, c.delay * 100, c.frequency);
    return std::string(name);
  });

TEST(OptimizerTests, SpeedLimitDynamicParameterGuard)
{
  // This test verifies that kinematic parameters (vx_max, etc.) are rejected
  // when a speed limit is active, but allowed when no speed limit is active.
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(0.5));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-0.35));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  // Use "mppic" to match the optimizer's name_ so ParametersHandler correctly routes callbacks
  std::string name = "mppic";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);

  // Start the parameter handler to enable dynamic parameter callbacks
  param_handler.start();

  // Verify initial state: no speed limit active
  EXPECT_FALSE(optimizer_tester.isSpeedLimitActiveWrapper());

  // Store original base_constraints value
  float original_vx_max = optimizer_tester.getBaseConstraints().vx_max;
  EXPECT_EQ(original_vx_max, 0.5f);

  // Test 1: Dynamic parameter update SHOULD SUCCEED when no speed limit is active
  auto result1 = node->set_parameter(rclcpp::Parameter("mppic.vx_max", 0.8));
  EXPECT_TRUE(result1.successful);
  // Verify the parameter was updated
  EXPECT_EQ(optimizer_tester.getBaseConstraints().vx_max, 0.8f);

  // Apply a speed limit (50% reduction)
  optimizer_tester.setSpeedLimit(50.0, true);
  EXPECT_TRUE(optimizer_tester.isSpeedLimitActiveWrapper());

  // Verify constraints differ from base_constraints now
  EXPECT_NE(
    optimizer_tester.getControlConstraints().vx_max,
    optimizer_tester.getBaseConstraints().vx_max);

  // Test 2: Dynamic parameter update SHOULD BE REJECTED when speed limit is active
  auto result2 = node->set_parameter(rclcpp::Parameter("mppic.vx_max", 1.0));
  EXPECT_FALSE(result2.successful);
  // Verify the base_constraints value was NOT updated
  EXPECT_EQ(optimizer_tester.getBaseConstraints().vx_max, 0.8f);

  // Clear the speed limit using NO_SPEED_LIMIT constant
  optimizer_tester.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_FALSE(optimizer_tester.isSpeedLimitActiveWrapper());

  // Test 3: Dynamic parameter update SHOULD SUCCEED again after clearing speed limit
  auto result3 = node->set_parameter(rclcpp::Parameter("mppic.vx_max", 1.0));
  EXPECT_TRUE(result3.successful);
  EXPECT_EQ(optimizer_tester.getBaseConstraints().vx_max, 1.0f);
}

TEST(OptimizerTests, InterIterationConstraintsTests)
{
  auto node = std::make_shared<nav2::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(20.0));
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.05));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(100));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(10));
  node->declare_parameter("mppic.ax_max", rclcpp::ParameterValue(2.0));
  node->declare_parameter("mppic.ax_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.ay_max", rclcpp::ParameterValue(2.0));
  node->declare_parameter("mppic.ay_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.az_max", rclcpp::ParameterValue(2.0));
  node->declare_parameter("mppic.open_loop", rclcpp::ParameterValue(true));
  node->declare_parameter(
    "mppic.diff_drive.plugin", rclcpp::ParameterValue("mppi::DiffDriveMotionModel"));
  node->declare_parameter(
    "mppic.omni.plugin", rclcpp::ParameterValue("mppi::OmniMotionModel"));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", true);
  std::string name = "test";
  ParametersHandler param_handler(node, name);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto tf_buffer = nav2::create_transform_buffer(node);
  optimizer_tester.initialize(node, "mppic", costmap_ros, tf_buffer, &param_handler);
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  auto & settings = optimizer_tester.grabSettings();
  auto & state = optimizer_tester.grabState();
  auto & seq = optimizer_tester.grabControlSequence();

  // controller_period == model_dt == 0.05, shift ON, open_loop ON
  EXPECT_TRUE(settings.shift_control_sequence);
  EXPECT_TRUE(settings.open_loop);

  // Open-loop + shift: vx(0) should be pinned to state_.speed
  state.speed.linear.x = 0.1;
  state.speed.linear.y = 0.3;
  state.speed.angular.z = 0.2;
  seq.reset(10);
  seq.vx(0) = 5.0;
  seq.vy(0) = 5.0;
  seq.wz(0) = 5.0;
  optimizer_tester.applyControlSequenceInterIterationConstraintsWrapper();
  EXPECT_FLOAT_EQ(seq.vx(0), 0.1f);
  EXPECT_FLOAT_EQ(seq.vy(0), 0.3f);
  EXPECT_FLOAT_EQ(seq.wz(0), 0.2f);

  // Closed-loop + shift: vx(0) should still be pinned to state_.speed
  settings.open_loop = false;
  state.speed.linear.x = 0.0;
  state.speed.linear.y = 0.0;
  state.speed.angular.z = 0.0;
  seq.reset(10);
  seq.vx(0) = 5.0;
  seq.vy(0) = 5.0;
  seq.wz(0) = 5.0;
  optimizer_tester.applyControlSequenceInterIterationConstraintsWrapper();
  // shift always pins vx(0) to current speed regardless of open/closed loop
  EXPECT_FLOAT_EQ(seq.vx(0), 0.0f);
  EXPECT_FLOAT_EQ(seq.vy(0), 0.0f);
  EXPECT_FLOAT_EQ(seq.wz(0), 0.0f);

  // Open-loop + no shift (controller faster): clamp with controller_period
  settings.open_loop = true;
  settings.shift_control_sequence = false;
  settings.controller_period = 0.025f;  // controller at 40Hz
  state.speed.linear.x = 0.0;
  state.speed.linear.y = 0.0;
  state.speed.angular.z = 0.0;
  seq.reset(10);
  seq.vx(0) = 5.0;
  seq.vy(0) = 5.0;
  seq.wz(0) = 5.0;
  optimizer_tester.applyControlSequenceInterIterationConstraintsWrapper();
  // max delta = controller_period * accel = 0.025 * 2.0 = 0.05
  EXPECT_NEAR(seq.vx(0), 0.05f, 1e-6);
  EXPECT_NEAR(seq.vy(0), 0.05f, 1e-6);
  EXPECT_NEAR(seq.wz(0), 0.05f, 1e-6);
  // Tighter than model_dt would give (0.1)
  EXPECT_LT(seq.vx(0), 0.05f * 2.0f);

  // Closed-loop + no shift: uses controller_period for clamping
  settings.open_loop = false;
  seq.reset(10);
  seq.vx(0) = 5.0;
  seq.vy(0) = 5.0;
  seq.wz(0) = 5.0;
  optimizer_tester.applyControlSequenceInterIterationConstraintsWrapper();
  // max delta = controller_period * accel = 0.025 * 2.0 = 0.05
  EXPECT_NEAR(seq.vx(0), 0.05f, 1e-6);
  EXPECT_NEAR(seq.vy(0), 0.05f, 1e-6);
  EXPECT_NEAR(seq.wz(0), 0.05f, 1e-6);

  // Deceleration: from positive speed with asymmetric accel/decel limits
  settings.open_loop = true;
  state.speed.linear.x = 0.5;
  state.speed.linear.y = 0.0;
  state.speed.angular.z = 0.0;
  seq.reset(10);
  seq.vx(0) = -5.0;  // Hard decel / reversal attempt
  optimizer_tester.applyControlSequenceInterIterationConstraintsWrapper();
  // lower_bound = 0.5 + controller_period * ax_min = 0.5 + 0.025 * (-1.0) = 0.475
  EXPECT_NEAR(seq.vx(0), 0.475f, 1e-6);

  // Acceleration from same speed
  seq.vx(0) = 5.0;
  optimizer_tester.applyControlSequenceInterIterationConstraintsWrapper();
  // upper_bound = 0.5 + controller_period * ax_max = 0.5 + 0.025 * 2.0 = 0.55
  EXPECT_NEAR(seq.vx(0), 0.55f, 1e-6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
