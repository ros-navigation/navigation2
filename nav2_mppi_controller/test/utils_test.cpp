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
#include <random>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_mppi_controller/models/path.hpp"

// Tests noise generator object

using namespace mppi::utils;  // NOLINT
using namespace mppi;  // NOLINT

class TestGoalChecker : public nav2_core::GoalChecker
{
public:
  TestGoalChecker() {}

  virtual void initialize(
    const nav2::LifecycleNode::WeakPtr & /*parent*/,
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
  EXPECT_EQ(path_t.x.rows(), 5u);
  EXPECT_EQ(path_t.y.rows(), 5u);
  EXPECT_EQ(path_t.yaws.rows(), 5u);
  EXPECT_EQ(path_t.x(2), 5);
  EXPECT_EQ(path_t.y(2), 50);
  EXPECT_NEAR(path_t.yaws(2), 0.0, 1e-6);
}

TEST(UtilsTests, AnglesTests)
{
  // Test angle normalization by creating insane angles
  Eigen::ArrayXf angles(100);
  angles.setConstant(1.0f);

  for (unsigned int i = 0; i != angles.size(); i++) {
    angles(i) = i * i;
    if (i % 2 == 0) {
      angles(i) *= -1;
    }
  }

  auto norm_ang = normalize_angles(angles).eval();
  for (unsigned int i = 0; i != norm_ang.size(); i++) {
    EXPECT_TRUE((norm_ang(i) >= -M_PIF) && (norm_ang(i) <= M_PIF));
  }

  // Test shortest angular distance
  Eigen::ArrayXf zero_angles(100);
  zero_angles.setZero();
  auto ang_dist = shortest_angular_distance(angles, zero_angles).eval();
  for (unsigned int i = 0; i != ang_dist.size(); i++) {
    EXPECT_TRUE((ang_dist(i) >= -M_PIF) && (ang_dist(i) <= M_PIF));
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
  generated_trajectories.reset(100, 2);
  models::Path path;
  geometry_msgs::msg::Pose goal;
  path.reset(10);
  Eigen::ArrayXf costs;
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
  generated_trajectories.x = Eigen::ArrayXXf::Ones(100, 2);
  generated_trajectories.y = Eigen::ArrayXXf::Zero(100, 2);
  generated_trajectories.yaws = Eigen::ArrayXXf::Zero(100, 2);

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
  EXPECT_EQ(findPathFurthestReachedPoint(data3), 5);
}

TEST(UtilsTests, findPathCosts)
{
  models::State state;
  models::Trajectories generated_trajectories;
  models::Path path;
  geometry_msgs::msg::Pose goal;
  path.reset(50);
  Eigen::ArrayXf costs;
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
    "dummy_costmap", "", true);
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

  path.x(1) = 999999999;  // OFF COSTMAP
  path.y(1) = 999999999;
  path.x(10) = 1.5;  // IN LETHAL
  path.y(10) = 1.5;
  path.x(20) = 4.2;  // IN INFLATED
  path.y(20) = 4.2;

  // This should be evaluated and have real outputs now
  setPathCostsIfNotSet(data3, costmap_ros);
  EXPECT_TRUE(data3.path_pts_valid.has_value());
  for (unsigned int i = 0; i != path.x.size() - 1; i++) {
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
  noisey_sequence.vx = 0.2 * Eigen::ArrayXf::Ones(30);
  noisey_sequence.vy = 0.0 * Eigen::ArrayXf::Ones(30);
  noisey_sequence.wz = 0.3 * Eigen::ArrayXf::Ones(30);

  // Make the sequence noisy
  std::mt19937 engine;
  std::normal_distribution<float> normal_dist = std::normal_distribution(0.0f, 0.2f);
  auto noises = Eigen::ArrayXf::NullaryExpr(
    30, [&] () {return normal_dist(engine);});
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

  savitskyGolayFilter(noisey_sequence, history);

  // History should remain unchanged
  EXPECT_NEAR(history_init[0].vx, history[0].vx, 1e-6);
  EXPECT_NEAR(history_init[0].vy, history[0].vy, 1e-6);
  EXPECT_NEAR(history_init[0].wz, history[0].wz, 1e-6);
  EXPECT_NEAR(history_init[1].vx, history[1].vx, 1e-6);
  EXPECT_NEAR(history_init[1].vy, history[1].vy, 1e-6);
  EXPECT_NEAR(history_init[1].wz, history[1].wz, 1e-6);
  EXPECT_NEAR(history_init[2].vx, history[2].vx, 1e-6);
  EXPECT_NEAR(history_init[2].vy, history[2].vy, 1e-6);
  EXPECT_NEAR(history_init[2].wz, history[2].wz, 1e-6);
  EXPECT_NEAR(history_init[3].vx, history[3].vx, 1e-6);
  EXPECT_NEAR(history_init[3].vy, history[3].vy, 1e-6);
  EXPECT_NEAR(history_init[3].wz, history[3].wz, 1e-6);

  // Check that path is smoother
  float smoothed_val{0}, original_val{0};
  for (unsigned int i = 1; i != noisey_sequence.vx.size() - 1; i++) {
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

TEST(UtilsTests, ShiftColumnsByOnePlaceTest)
{
  // Try with scalar value
  Eigen::ArrayXf scalar_val(1);
  scalar_val(0) = 5;
  utils::shiftColumnsByOnePlace(scalar_val, 1);
  EXPECT_EQ(scalar_val.size(), 1);
  EXPECT_EQ(scalar_val(0), 5);

  // Try with one dimensional array, shift right
  Eigen::ArrayXf array_1d(4);
  array_1d << 1, 2, 3, 4;
  utils::shiftColumnsByOnePlace(array_1d, 1);
  EXPECT_EQ(array_1d.size(), 4);
  EXPECT_EQ(array_1d(0), 1);
  EXPECT_EQ(array_1d(1), 1);
  EXPECT_EQ(array_1d(2), 2);
  EXPECT_EQ(array_1d(3), 3);

  // Try with one dimensional array, shift left
  array_1d(1) = 5;
  utils::shiftColumnsByOnePlace(array_1d, -1);
  EXPECT_EQ(array_1d.size(), 4);
  EXPECT_EQ(array_1d(0), 5);
  EXPECT_EQ(array_1d(1), 2);
  EXPECT_EQ(array_1d(2), 3);
  EXPECT_EQ(array_1d(2), 3);

  // Try with two dimensional array, shift right
  // 1 2 3 4        1 1 2 3
  // 5 6 7 8    ->  5 5 6 7
  // 9 10 11 12     9 9 10 11
  Eigen::ArrayXXf array_2d(3, 4);
  array_2d << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
  utils::shiftColumnsByOnePlace(array_2d, 1);
  EXPECT_EQ(array_2d.rows(), 3);
  EXPECT_EQ(array_2d.cols(), 4);
  EXPECT_EQ(array_2d(0, 0), 1);
  EXPECT_EQ(array_2d(1, 0), 5);
  EXPECT_EQ(array_2d(2, 0), 9);
  EXPECT_EQ(array_2d(0, 1), 1);
  EXPECT_EQ(array_2d(1, 1), 5);
  EXPECT_EQ(array_2d(2, 1), 9);
  EXPECT_EQ(array_2d(0, 2), 2);
  EXPECT_EQ(array_2d(1, 2), 6);
  EXPECT_EQ(array_2d(2, 2), 10);
  EXPECT_EQ(array_2d(0, 3), 3);
  EXPECT_EQ(array_2d(1, 3), 7);
  EXPECT_EQ(array_2d(2, 3), 11);

  array_2d.col(0).setZero();

  // Try with two dimensional array, shift left
  // 0 1 2 3      1 2 3 3
  // 0 5 6 7   -> 5 6 7 7
  // 0 9 10 11    9 10 11 11
  utils::shiftColumnsByOnePlace(array_2d, -1);
  EXPECT_EQ(array_2d.rows(), 3);
  EXPECT_EQ(array_2d.cols(), 4);
  EXPECT_EQ(array_2d(0, 0), 1);
  EXPECT_EQ(array_2d(1, 0), 5);
  EXPECT_EQ(array_2d(2, 0), 9);
  EXPECT_EQ(array_2d(0, 1), 2);
  EXPECT_EQ(array_2d(1, 1), 6);
  EXPECT_EQ(array_2d(2, 1), 10);
  EXPECT_EQ(array_2d(0, 2), 3);
  EXPECT_EQ(array_2d(1, 2), 7);
  EXPECT_EQ(array_2d(2, 2), 11);
  EXPECT_EQ(array_2d(0, 3), 3);
  EXPECT_EQ(array_2d(1, 3), 7);
  EXPECT_EQ(array_2d(2, 3), 11);

  // Try with invalid direction value.
  EXPECT_THROW(utils::shiftColumnsByOnePlace(array_2d, -2), std::logic_error);
}

TEST(UtilsTests, NormalizeYawsBetweenPointsTest)
{
  Eigen::ArrayXf last_yaws(10);
  last_yaws.setZero();

  Eigen::ArrayXf yaw_between_points(10);
  yaw_between_points.setZero(10);

  // Try with both angles 0
  Eigen::ArrayXf yaws_between_points_corrected = utils::normalize_yaws_between_points(last_yaws,
    yaw_between_points);
  EXPECT_TRUE(yaws_between_points_corrected.isApprox(yaw_between_points));

  // Try with yaw between points as pi/4
  yaw_between_points.setConstant(M_PIF_2 / 2);
  yaws_between_points_corrected = utils::normalize_yaws_between_points(last_yaws,
    yaw_between_points);
  EXPECT_TRUE(yaws_between_points_corrected.isApprox(yaw_between_points));

  // Try with yaw between points as pi/2
  yaw_between_points.setConstant(M_PIF_2);
  yaws_between_points_corrected = utils::normalize_yaws_between_points(last_yaws,
    yaw_between_points);
  EXPECT_TRUE(yaws_between_points_corrected.isApprox(yaw_between_points));

  // Try with a few yaw between points  more than pi/2
  yaw_between_points[1] = 1.2 * M_PIF_2;
  yaws_between_points_corrected = utils::normalize_yaws_between_points(last_yaws,
    yaw_between_points);
  EXPECT_NEAR(yaws_between_points_corrected[1], -0.8 * M_PIF_2, 1e-3);
  EXPECT_NEAR(yaws_between_points_corrected[0], yaw_between_points[0], 1e-3);
  EXPECT_NEAR(yaws_between_points_corrected[9], yaw_between_points[9], 1e-3);

  // Try with goal angle 0
  float goal_angle = 0;
  yaws_between_points_corrected = utils::normalize_yaws_between_points(goal_angle,
    yaw_between_points);
  EXPECT_NEAR(yaws_between_points_corrected[1], -0.8 * M_PIF_2, 1e-3);
}

TEST(UtilsTests, toTrajectoryMsgTest)
{
  Eigen::ArrayXXf trajectory(5, 3);
  trajectory <<
    0.0, 0.0, 0.0,
    1.0, 1.0, 1.0,
    2.0, 2.0, 2.0,
    3.0, 3.0, 3.0,
    4.0, 4.0, 4.0;

  models::ControlSequence control_sequence;
  control_sequence.vx = Eigen::ArrayXf::Ones(5);
  control_sequence.wz = Eigen::ArrayXf::Ones(5);
  control_sequence.vy = Eigen::ArrayXf::Zero(5);

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = rclcpp::Time(100, 0, RCL_ROS_TIME);

  auto trajectory_msg = utils::toTrajectoryMsg(
    trajectory, control_sequence, 1.0, header);

  EXPECT_EQ(trajectory_msg->header.frame_id, "map");
  EXPECT_EQ(trajectory_msg->header.stamp, header.stamp);
  EXPECT_EQ(trajectory_msg->points.size(), 5u);
  EXPECT_EQ(trajectory_msg->points[0].pose.position.x, 0.0);
  EXPECT_EQ(trajectory_msg->points[0].pose.position.y, 0.0);
  EXPECT_EQ(trajectory_msg->points[1].pose.position.x, 1.0);
  EXPECT_EQ(trajectory_msg->points[1].pose.position.y, 1.0);
  EXPECT_EQ(trajectory_msg->points[2].pose.position.x, 2.0);
  EXPECT_EQ(trajectory_msg->points[2].pose.position.y, 2.0);
  EXPECT_EQ(trajectory_msg->points[3].pose.position.x, 3.0);
  EXPECT_EQ(trajectory_msg->points[3].pose.position.y, 3.0);
  EXPECT_EQ(trajectory_msg->points[4].pose.position.x, 4.0);
  EXPECT_EQ(trajectory_msg->points[4].pose.position.y, 4.0);

  EXPECT_EQ(trajectory_msg->points[0].velocity.linear.x, 1.0);
  EXPECT_EQ(trajectory_msg->points[0].velocity.linear.y, 0.0);
  EXPECT_EQ(trajectory_msg->points[0].velocity.angular.z, 1.0);
  EXPECT_EQ(trajectory_msg->points[1].velocity.linear.x, 1.0);
  EXPECT_EQ(trajectory_msg->points[1].velocity.linear.y, 0.0);
  EXPECT_EQ(trajectory_msg->points[1].velocity.angular.z, 1.0);
  EXPECT_EQ(trajectory_msg->points[2].velocity.linear.x, 1.0);
  EXPECT_EQ(trajectory_msg->points[2].velocity.linear.y, 0.0);
  EXPECT_EQ(trajectory_msg->points[2].velocity.angular.z, 1.0);
  EXPECT_EQ(trajectory_msg->points[3].velocity.linear.x, 1.0);
  EXPECT_EQ(trajectory_msg->points[3].velocity.linear.y, 0.0);
  EXPECT_EQ(trajectory_msg->points[3].velocity.angular.z, 1.0);
  EXPECT_EQ(trajectory_msg->points[4].velocity.linear.x, 1.0);
  EXPECT_EQ(trajectory_msg->points[4].velocity.linear.y, 0.0);
  EXPECT_EQ(trajectory_msg->points[4].velocity.angular.z, 1.0);

  EXPECT_EQ(trajectory_msg->points[0].time_from_start, rclcpp::Duration(0, 0));
  EXPECT_EQ(trajectory_msg->points[1].time_from_start, rclcpp::Duration(1, 0));
  EXPECT_EQ(trajectory_msg->points[2].time_from_start, rclcpp::Duration(2, 0));
  EXPECT_EQ(trajectory_msg->points[3].time_from_start, rclcpp::Duration(3, 0));
  EXPECT_EQ(trajectory_msg->points[4].time_from_start, rclcpp::Duration(4, 0));
}

TEST(UtilsTests, getLastPathPoseTest)
{
  nav_msgs::msg::Path path;
  path.poses.resize(10);
  path.poses[9].pose.position.x = 5.0;
  path.poses[9].pose.position.y = 50.0;
  path.poses[9].pose.orientation.x = 0.0;
  path.poses[9].pose.orientation.y = 0.0;
  path.poses[9].pose.orientation.z = 1.0;
  path.poses[9].pose.orientation.w = 0.0;

  models::Path path_t = toTensor(path);
  geometry_msgs::msg::Pose last_path_pose = utils::getLastPathPose(path_t);

  EXPECT_EQ(last_path_pose.position.x, 5);
  EXPECT_EQ(last_path_pose.position.y, 50);
  EXPECT_NEAR(last_path_pose.orientation.x, 0.0, 1e-3);
  EXPECT_NEAR(last_path_pose.orientation.y, 0.0, 1e-3);
  EXPECT_NEAR(last_path_pose.orientation.z, 1.0, 1e-3);
  EXPECT_NEAR(last_path_pose.orientation.w, 0.0, 1e-3);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
