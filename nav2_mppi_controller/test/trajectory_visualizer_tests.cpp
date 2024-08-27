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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"

// Tests trajectory visualization

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace mppi;  // NOLINT

TEST(TrajectoryVisualizerTests, StateTransition)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto parameters_handler = std::make_unique<ParametersHandler>(node);

  TrajectoryVisualizer vis;
  vis.on_configure(node, "my_name", "map", parameters_handler.get());
  vis.on_activate();
  vis.on_deactivate();
  vis.on_cleanup();
}

TEST(TrajectoryVisualizerTests, VisPathRepub)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto parameters_handler = std::make_unique<ParametersHandler>(node);
  nav_msgs::msg::Path recieved_path;
  nav_msgs::msg::Path pub_path;
  pub_path.header.frame_id = "fake_frame";
  pub_path.poses.resize(5);

  auto my_sub = node->create_subscription<nav_msgs::msg::Path>(
    "transformed_global_plan", 10,
    [&](const nav_msgs::msg::Path msg) {recieved_path = msg;});

  TrajectoryVisualizer vis;
  vis.on_configure(node, "my_name", "map", parameters_handler.get());
  vis.on_activate();
  vis.visualize(pub_path);

  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_EQ(recieved_path.poses.size(), 5u);
  EXPECT_EQ(recieved_path.header.frame_id, "fake_frame");
}

TEST(TrajectoryVisualizerTests, VisOptimalTrajectory)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto parameters_handler = std::make_unique<ParametersHandler>(node);

  visualization_msgs::msg::MarkerArray recieved_msg;
  auto my_sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/trajectories", 10,
    [&](const visualization_msgs::msg::MarkerArray msg) {recieved_msg = msg;});

  // optimal_trajectory empty, should fail to publish
  xt::xtensor<float, 2> optimal_trajectory;
  TrajectoryVisualizer vis;
  vis.on_configure(node, "my_name", "fkmap", parameters_handler.get());
  vis.on_activate();
  builtin_interfaces::msg::Time bogus_stamp;
  vis.add(optimal_trajectory, "Optimal Trajectory", bogus_stamp);
  nav_msgs::msg::Path bogus_path;
  vis.visualize(bogus_path);

  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_EQ(recieved_msg.markers.size(), 0u);

  // Now populated with content, should publish
  optimal_trajectory = xt::ones<float>({20, 2});
  vis.add(optimal_trajectory, "Optimal Trajectory", bogus_stamp);
  vis.visualize(bogus_path);

  rclcpp::spin_some(node->get_node_base_interface());

  // Should have 20 trajectory points in the map frame
  EXPECT_EQ(recieved_msg.markers.size(), 20u);
  EXPECT_EQ(recieved_msg.markers[0].header.frame_id, "fkmap");

  // Check IDs are properly populated
  EXPECT_EQ(recieved_msg.markers[0].id, 0);
  EXPECT_EQ(recieved_msg.markers[1].id, 1);
  EXPECT_EQ(recieved_msg.markers[10].id, 10);

  // Check poses are correct
  EXPECT_EQ(recieved_msg.markers[0].pose.position.x, 1);
  EXPECT_EQ(recieved_msg.markers[0].pose.position.y, 1);
  EXPECT_EQ(recieved_msg.markers[0].pose.position.z, 0.06);

  // Check that scales are rational
  EXPECT_EQ(recieved_msg.markers[0].scale.x, 0.03);
  EXPECT_EQ(recieved_msg.markers[0].scale.y, 0.03);
  EXPECT_EQ(recieved_msg.markers[0].scale.z, 0.07);

  EXPECT_EQ(recieved_msg.markers[19].scale.x, 0.07);
  EXPECT_EQ(recieved_msg.markers[19].scale.y, 0.07);
  EXPECT_EQ(recieved_msg.markers[19].scale.z, 0.09);

  // Check that the colors are rational
  for (unsigned int i = 0; i != recieved_msg.markers.size() - 1; i++) {
    EXPECT_LT(recieved_msg.markers[i].color.g, recieved_msg.markers[i + 1].color.g);
    EXPECT_LT(recieved_msg.markers[i].color.b, recieved_msg.markers[i + 1].color.b);
    EXPECT_EQ(recieved_msg.markers[i].color.r, recieved_msg.markers[i + 1].color.r);
    EXPECT_EQ(recieved_msg.markers[i].color.a, recieved_msg.markers[i + 1].color.a);
  }
}

TEST(TrajectoryVisualizerTests, VisCandidateTrajectories)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto parameters_handler = std::make_unique<ParametersHandler>(node);

  visualization_msgs::msg::MarkerArray recieved_msg;
  auto my_sub = node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/trajectories", 10,
    [&](const visualization_msgs::msg::MarkerArray msg) {recieved_msg = msg;});

  models::Trajectories candidate_trajectories;
  candidate_trajectories.x = xt::ones<float>({200, 12});
  candidate_trajectories.y = xt::ones<float>({200, 12});
  candidate_trajectories.yaws = xt::ones<float>({200, 12});

  TrajectoryVisualizer vis;
  vis.on_configure(node, "my_name", "fkmap", parameters_handler.get());
  vis.on_activate();
  vis.add(candidate_trajectories, "Candidate Trajectories");
  nav_msgs::msg::Path bogus_path;
  vis.visualize(bogus_path);

  rclcpp::spin_some(node->get_node_base_interface());
  // 40 * 4, for 5 trajectory steps + 3 point steps
  EXPECT_EQ(recieved_msg.markers.size(), 160u);
}

TEST(TrajectoryVisualizerTests, VisOptimalPath)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto parameters_handler = std::make_unique<ParametersHandler>(node);
  builtin_interfaces::msg::Time cmd_stamp;
  cmd_stamp.sec = 5;
  cmd_stamp.nanosec = 10;

  nav_msgs::msg::Path recieved_path;
  auto my_sub = node->create_subscription<nav_msgs::msg::Path>(
    "optimal_trajectory", 10,
    [&](const nav_msgs::msg::Path msg) {recieved_path = msg;});

  // optimal_trajectory empty, should fail to publish
  xt::xtensor<float, 2> optimal_trajectory;
  TrajectoryVisualizer vis;
  vis.on_configure(node, "my_name", "fkmap", parameters_handler.get());
  vis.on_activate();
  vis.add(optimal_trajectory, "Optimal Trajectory", cmd_stamp);
  nav_msgs::msg::Path bogus_path;
  vis.visualize(bogus_path);

  rclcpp::spin_some(node->get_node_base_interface());
  EXPECT_EQ(recieved_path.poses.size(), 0u);

  // Now populated with content, should publish
  optimal_trajectory.resize({20, 2});
  for (unsigned int i = 0; i != optimal_trajectory.shape()[0] - 1; i++) {
    optimal_trajectory(i, 0) = static_cast<float>(i);
    optimal_trajectory(i, 1) = static_cast<float>(i);
  }
  vis.add(optimal_trajectory, "Optimal Trajectory", cmd_stamp);
  vis.visualize(bogus_path);

  rclcpp::spin_some(node->get_node_base_interface());

  // Should have a 20 points path in the map frame and with same stamp than velocity command
  EXPECT_EQ(recieved_path.poses.size(), 20u);
  EXPECT_EQ(recieved_path.header.frame_id, "fkmap");
  EXPECT_EQ(recieved_path.header.stamp.sec, cmd_stamp.sec);
  EXPECT_EQ(recieved_path.header.stamp.nanosec, cmd_stamp.nanosec);

  tf2::Quaternion quat;
  for (unsigned int i = 0; i != recieved_path.poses.size() - 1; i++) {
    // Poses should be in map frame too
    EXPECT_EQ(recieved_path.poses[i].header.frame_id, "fkmap");

    // Check positions are correct
    EXPECT_EQ(recieved_path.poses[i].pose.position.x, static_cast<float>(i));
    EXPECT_EQ(recieved_path.poses[i].pose.position.y, static_cast<float>(i));
    EXPECT_EQ(recieved_path.poses[i].pose.position.z, 0.06);

    // Check orientations are correct
    quat.setRPY(0., 0., optimal_trajectory(i, 2));
    auto expected_orientation = tf2::toMsg(quat);
    EXPECT_EQ(recieved_path.poses[i].pose.orientation.x, expected_orientation.x);
    EXPECT_EQ(recieved_path.poses[i].pose.orientation.y, expected_orientation.y);
    EXPECT_EQ(recieved_path.poses[i].pose.orientation.z, expected_orientation.z);
    EXPECT_EQ(recieved_path.poses[i].pose.orientation.w, expected_orientation.w);
  }
}
