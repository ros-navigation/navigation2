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
#include <thread>

#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#include <xtensor/xrandom.hpp>
#pragma GCC diagnostic pop

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/models/path.hpp"

// Tests noise generator object

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi::utils;  // NOLINT
using namespace mppi;  // NOLINT

class TestGoalChecker : public nav2_core::GoalChecker
{
public:
  TestGoalChecker() {}

  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & /*parent*/,
    const std::string & /*plugin_name*/,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/) {}

  virtual void reset() {}

  virtual bool isGoalReached(
    const geometry_msgs::msg::Pose & /*query_pose*/,
    const geometry_msgs::msg::Pose & /*goal_pose*/,
    const geometry_msgs::msg::Twist & /*velocity*/) {return false;}

  virtual bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & /*vel_tolerance*/)
  {
    pose_tolerance.position.x = 0.25;
    pose_tolerance.position.y = 0.25;
    return true;
  }
};

TEST(UtilsTests, MarkerPopulationUtils)
{
  auto pose = createPose(1.0, 2.0, 3.0);
  EXPECT_EQ(pose.position.x, 1.0);
  EXPECT_EQ(pose.position.y, 2.0);
  EXPECT_EQ(pose.position.z, 3.0);
  EXPECT_EQ(pose.orientation.w, 1.0);

  auto scale = createScale(1.0, 2.0, 3.0);
  EXPECT_EQ(scale.x, 1.0);
  EXPECT_EQ(scale.y, 2.0);
  EXPECT_EQ(scale.z, 3.0);

  auto color = createColor(1.0, 2.0, 3.0, 0.0);
  EXPECT_EQ(color.r, 1.0);
  EXPECT_EQ(color.g, 2.0);
  EXPECT_EQ(color.b, 3.0);
  EXPECT_EQ(color.a, 0.0);

  auto marker = createMarker(999, pose, scale, color, "map", "ns");
  EXPECT_EQ(marker.header.frame_id, "map");
  EXPECT_EQ(marker.id, 999);
  EXPECT_EQ(marker.pose, pose);
  EXPECT_EQ(marker.scale, scale);
  EXPECT_EQ(marker.color, color);
  EXPECT_EQ(marker.ns, "ns");
}

TEST(UtilsTests, ConversionTests)
{
  geometry_msgs::msg::TwistStamped output;
  builtin_interfaces::msg::Time time;

  // Check population is correct
  output = toTwistStamped(0.5, 0.3, time, "map");
  EXPECT_NEAR(output.twist.linear.x, 0.5, 1e-6);
  EXPECT_NEAR(output.twist.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(output.twist.angular.z, 0.3, 1e-6);
  EXPECT_EQ(output.header.frame_id, "map");
  EXPECT_EQ(output.header.stamp, time);

  output = toTwistStamped(0.5, 0.4, 0.3, time, "map");
  EXPECT_NEAR(output.twist.linear.x, 0.5, 1e-6);
  EXPECT_NEAR(output.twist.linear.y, 0.4, 1e-6);
  EXPECT_NEAR(output.twist.angular.z, 0.3, 1e-6);
  EXPECT_EQ(output.header.frame_id, "map");
  EXPECT_EQ(output.header.stamp, time);

  nav_msgs::msg::Path path;
  path.poses.resize(5);
  path.poses[2].pose.position.x = 5;
  path.poses[2].pose.position.y = 50;
  models::Path path_t = toTensor(path);

  // Check population is correct
  EXPECT_EQ(path_t.x.shape(0), 5u);
  EXPECT_EQ(path_t.y.shape(0), 5u);
  EXPECT_EQ(path_t.yaws.shape(0), 5u);
  EXPECT_EQ(path_t.x(2), 5);
  EXPECT_EQ(path_t.y(2), 50);
  EXPECT_NEAR(path_t.yaws(2), 0.0, 1e-6);
}

TEST(UtilsTests, WithTolTests)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 10.0;
  pose.position.y = 1.0;

  nav2_core::GoalChecker * goal_checker = new TestGoalChecker;

  nav_msgs::msg::Path path;
  path.poses.resize(2);
  geometry_msgs::msg::Pose & goal = path.poses.back().pose;

  // Create CriticData with state and goal initialized
  models::State state;
  state.pose.pose = pose;
  models::Trajectories generated_trajectories;
  models::Path path_critic;
  xt::xtensor<float, 1> costs;
  float model_dt;
  CriticData data = {
    state, generated_trajectories, path_critic, goal,
    costs, model_dt, false, nullptr, nullptr, std::nullopt, std::nullopt};

  // Test not in tolerance
  goal.position.x = 0.0;
  goal.position.y = 0.0;
  EXPECT_FALSE(withinPositionGoalTolerance(goal_checker, pose, goal));
  EXPECT_FALSE(withinPositionGoalTolerance(0.25, pose, goal));

  // Test in tolerance
  goal.position.x = 9.8;
  goal.position.y = 0.95;
  EXPECT_TRUE(withinPositionGoalTolerance(goal_checker, pose, goal));
  EXPECT_TRUE(withinPositionGoalTolerance(0.25, pose, goal));

  goal.position.x = 10.0;
  goal.position.y = 0.76;
  EXPECT_TRUE(withinPositionGoalTolerance(goal_checker, pose, goal));
  EXPECT_TRUE(withinPositionGoalTolerance(0.25, pose, goal));

  goal.position.x = 9.76;
  goal.position.y = 1.0;
  EXPECT_TRUE(withinPositionGoalTolerance(goal_checker, pose, goal));
  EXPECT_TRUE(withinPositionGoalTolerance(0.25, pose, goal));

  delete goal_checker;
  goal_checker = nullptr;
  EXPECT_FALSE(withinPositionGoalTolerance(goal_checker, pose, goal));
}

TEST(UtilsTests, AnglesTests)
{
  // Test angle normalization by creating insane angles
  xt::xtensor<float, 1> angles, zero_angles;
  angles = xt::ones<float>({100});
  for (unsigned int i = 0; i != angles.shape(0); i++) {
    angles(i) = i * i;
    if (i % 2 == 0) {
      angles(i) *= -1;
    }
  }

  auto norm_ang = normalize_angles(angles);
  for (unsigned int i = 0; i != norm_ang.shape(0); i++) {
    EXPECT_TRUE((norm_ang(i) >= -M_PI) && (norm_ang(i) <= M_PI));
  }

  // Test shortest angular distance
  zero_angles = xt::zeros<float>({100});
  auto ang_dist = shortest_angular_distance(angles, zero_angles);
  for (unsigned int i = 0; i != ang_dist.shape(0); i++) {
    EXPECT_TRUE((ang_dist(i) >= -M_PI) && (ang_dist(i) <= M_PI));
  }

  // Test point-pose angle
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.orientation.w = 1.0;
  double point_x = 1.0, point_y = 0.0;
  bool forward_preference = true;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, forward_preference), 0.0, 1e-6);
  forward_preference = false;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, forward_preference), 0.0, 1e-6);
  point_x = -1.0;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, forward_preference), 0.0, 1e-6);
  forward_preference = true;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, forward_preference), M_PI, 1e-6);

  // Test point-pose angle with goal yaws
  point_x = 1.0;
  double point_yaw = 0.0;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, point_yaw), 0.0, 1e-6);
  point_yaw = M_PI;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, point_yaw), M_PI, 1e-6);
  point_yaw = 0.1;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, point_yaw), 0.0, 1e-3);
  point_yaw = 3.04159;
  EXPECT_NEAR(posePointAngle(pose, point_x, point_y, point_yaw), M_PI, 1e-3);
}

TEST(UtilsTests, FurthestAndClosestReachedPoint)
{
  models::State state;
  models::Trajectories generated_trajectories;
  models::Path path;
  geometry_msgs::msg::Pose goal;
  xt::xtensor<float, 1> costs;
  float model_dt = 0.1;

  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};  /// Caution, keep references

  // Attempt to set furthest point if notionally set, should not change
  data.furthest_reached_path_point = 99999;
  setPathFurthestPointIfNotSet(data);
  EXPECT_EQ(data.furthest_reached_path_point, 99999);

  // Attempt to set if not set already with no other information, should fail
  CriticData data2 =
  {state, generated_trajectories, path, goal, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};  /// Caution, keep references
  setPathFurthestPointIfNotSet(data2);
  EXPECT_EQ(data2.furthest_reached_path_point, 0);

  // Test the actual computation of the path point reached
  generated_trajectories.x = xt::ones<float>({100, 2});
  generated_trajectories.y = xt::zeros<float>({100, 2});
  generated_trajectories.yaws = xt::zeros<float>({100, 2});

  nav_msgs::msg::Path plan;
  plan.poses.resize(10);
  for (unsigned int i = 0; i != plan.poses.size(); i++) {
    plan.poses[i].pose.position.x = 0.2 * i;
    plan.poses[i].pose.position.y = 0.0;
  }
  path = toTensor(plan);

  CriticData data3 =
  {state, generated_trajectories, path, goal, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};  /// Caution, keep references
  EXPECT_EQ(findPathFurthestReachedPoint(data3), 5u);
}

TEST(UtilsTests, findPathCosts)
{
  models::State state;
  models::Trajectories generated_trajectories;
  models::Path path;
  geometry_msgs::msg::Pose goal;
  xt::xtensor<float, 1> costs;
  float model_dt = 0.1;

  CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};  /// Caution, keep references

  // Test not set if already set, should not change
  data.path_pts_valid = std::vector<bool>(10, false);
  for (unsigned int i = 0; i != 10; i++) {
    (*data.path_pts_valid)[i] = false;
  }
  EXPECT_TRUE(data.path_pts_valid);
  setPathCostsIfNotSet(data, nullptr);
  EXPECT_EQ(data.path_pts_valid->size(), 10u);

  CriticData data3 =
  {state, generated_trajectories, path, goal, costs, model_dt, false, nullptr, nullptr,
    std::nullopt, std::nullopt};  /// Caution, keep references

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap", true);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  auto * costmap = costmap_ros->getCostmap();
  // island in the middle of lethal cost to cross. Costmap defaults to size 5x5 @ 10cm resolution
  for (unsigned int i = 10; i <= 30; ++i) {  // 1m-3m
    for (unsigned int j = 10; j <= 30; ++j) {  // 1m-3m
      costmap->setCost(i, j, 254);
    }
  }
  for (unsigned int i = 40; i <= 45; ++i) {  // 4m-4.5m
    for (unsigned int j = 45; j <= 45; ++j) {  // 4m-4.5m
      costmap->setCost(i, j, 253);
    }
  }

  path.reset(50);
  path.x(1) = 999999999;  // OFF COSTMAP
  path.y(1) = 999999999;
  path.x(10) = 1.5;  // IN LETHAL
  path.y(10) = 1.5;
  path.x(20) = 4.2;  // IN INFLATED
  path.y(20) = 4.2;

  // This should be evaluated and have real outputs now
  setPathCostsIfNotSet(data3, costmap_ros);
  EXPECT_TRUE(data3.path_pts_valid.has_value());
  for (unsigned int i = 0; i != path.x.shape(0) - 1; i++) {
    if (i == 1 || i == 10) {
      EXPECT_FALSE((*data3.path_pts_valid)[i]);
    } else {
      EXPECT_TRUE((*data3.path_pts_valid)[i]);
    }
  }
}

TEST(UtilsTests, SmootherTest)
{
  models::ControlSequence noisey_sequence, sequence_init;
  noisey_sequence.vx = 0.2 * xt::ones<float>({30});
  noisey_sequence.vy = 0.0 * xt::ones<float>({30});
  noisey_sequence.wz = 0.3 * xt::ones<float>({30});

  // Make the sequence noisy
  auto noises = xt::random::randn<float>({30}, 0.0, 0.2);
  noisey_sequence.vx += noises;
  noisey_sequence.vy += noises;
  noisey_sequence.wz += noises;
  sequence_init = noisey_sequence;

  std::array<mppi::models::Control, 4> history, history_init;
  history[3].vx = 0.1;
  history[3].vy = 0.0;
  history[3].wz = 0.3;
  history[2].vx = 0.1;
  history[2].vy = 0.0;
  history[2].wz = 0.3;
  history[1].vx = 0.1;
  history[1].vy = 0.0;
  history[1].wz = 0.3;
  history[0].vx = 0.0;
  history[0].vy = 0.0;
  history[0].wz = 0.0;
  history_init = history;

  models::OptimizerSettings settings;
  settings.shift_control_sequence = false;  // so result stores 0th value in history

  savitskyGolayFilter(noisey_sequence, history, settings);

  // Check history is propogated backward
  EXPECT_NEAR(history_init[3].vx, history[2].vx, 0.02);
  EXPECT_NEAR(history_init[3].vy, history[2].vy, 0.02);
  EXPECT_NEAR(history_init[3].wz, history[2].wz, 0.02);

  // Check history element is updated for first command
  EXPECT_NEAR(history[3].vx, 0.2, 0.05);
  EXPECT_NEAR(history[3].vy, 0.0, 0.035);
  EXPECT_NEAR(history[3].wz, 0.23, 0.02);

  // Check that path is smoother
  float smoothed_val{0}, original_val{0};
  for (unsigned int i = 1; i != noisey_sequence.vx.shape(0) - 1; i++) {
    smoothed_val += fabs(noisey_sequence.vx(i) - 0.2);
    smoothed_val += fabs(noisey_sequence.vy(i) - 0.0);
    smoothed_val += fabs(noisey_sequence.wz(i) - 0.3);

    original_val += fabs(sequence_init.vx(i) - 0.2);
    original_val += fabs(sequence_init.vy(i) - 0.0);
    original_val += fabs(sequence_init.wz(i) - 0.3);
  }

  EXPECT_LT(smoothed_val, original_val);
}

TEST(UtilsTests, FindPathInversionTest)
{
  // Straight path, no inversions to be found
  nav_msgs::msg::Path path;
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(utils::findFirstPathInversion(path), 10u);

  // To short to process
  path.poses.erase(path.poses.begin(), path.poses.begin() + 7);
  EXPECT_EQ(utils::findFirstPathInversion(path), 3u);

  // Has inversion at index 10, so should return 11 for the first point afterwards
  // 0 1 2 3 4 5 6 7 8 9 10 **9** 8 7 6 5 4 3 2 1
  path.poses.clear();
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 10 - i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(utils::findFirstPathInversion(path), 11u);
}

TEST(UtilsTests, RemovePosesAfterPathInversionTest)
{
  nav_msgs::msg::Path path;
  // straight path
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(utils::removePosesAfterFirstInversion(path), 0u);

  // try empty path
  path.poses.clear();
  EXPECT_EQ(utils::removePosesAfterFirstInversion(path), 0u);

  // cusping path
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = i;
    path.poses.push_back(pose);
  }
  for (unsigned int i = 0; i != 10; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = 10 - i;
    path.poses.push_back(pose);
  }
  EXPECT_EQ(utils::removePosesAfterFirstInversion(path), 11u);
  // Check to see if removed
  EXPECT_EQ(path.poses.size(), 11u);
  EXPECT_EQ(path.poses.back().pose.position.x, 10);
}
