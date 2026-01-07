// Copyright (c) 2025, Open Navigation LLC
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_core/route_exceptions.hpp"
#include "nav2_route/route_tracker.hpp"
#include "nav2_route/route_server.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT


class RouteTrackerWrapper : public RouteTracker
{
public:
  RouteTrackerWrapper() = default;

  void setRouteMsgSize(const int & size)
  {
    nav2_msgs::msg::Route msg;
    msg.edges.resize(size);
    route_msg_ = msg;
  }
};

TEST(RouteTrackerTest, test_lifecycle)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("router_test");

  RouteTracker tracker;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  tracker.configure(node, nullptr, costmap_subscriber, nullptr, "map", "base_link");
}

TEST(RouteTrackerTest, test_get_robot_pose)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("router_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tf->setCreateTimerInterface(timer_interface);
  auto transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf);
  tf2_ros::TransformBroadcaster broadcaster(node);
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;

  RouteTracker tracker;
  tracker.configure(node, tf, costmap_subscriber, nullptr, "map", "base_link");

  EXPECT_THROW(tracker.getRobotPose(), nav2_core::RouteTFError);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = node->now();
  transform.child_frame_id = "base_link";
  broadcaster.sendTransform(transform);
  EXPECT_NO_THROW(tracker.getRobotPose());
}

TEST(RouteTrackerTest, test_route_start_end)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("router_test");

  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  RouteTrackerWrapper tracker;
  tracker.configure(node, nullptr, costmap_subscriber, nullptr, "map", "base_link");
  Route route;
  route.edges.resize(7);
  DirectionalEdge edge;
  RouteTrackingState state;

  state.route_edges_idx = -1;
  EXPECT_TRUE(tracker.isStartOrEndNode(state, route));  // Attempting to get to first node

  state.current_edge = &edge;
  EXPECT_FALSE(tracker.isStartOrEndNode(state, route));  // with tracking continued

  state.route_edges_idx = 0;
  EXPECT_FALSE(tracker.isStartOrEndNode(state, route));

  state.route_edges_idx = 1;
  EXPECT_FALSE(tracker.isStartOrEndNode(state, route));

  state.route_edges_idx = 3;
  EXPECT_FALSE(tracker.isStartOrEndNode(state, route));

  state.route_edges_idx = 5;
  EXPECT_FALSE(tracker.isStartOrEndNode(state, route));

  state.route_edges_idx = 6;
  EXPECT_TRUE(tracker.isStartOrEndNode(state, route));  // Approaching final node
}

TEST(RouteTrackerTest, test_feedback)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("router_test");
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  RouteTrackerWrapper tracker;
  tracker.configure(node, nullptr, costmap_subscriber, nullptr, "map", "base_link");

  Route route;
  tracker.setRouteMsgSize(7);
  std::vector<std::string> ops;

  // This will segfault since there is no action server to publish feedback upon
  ASSERT_EXIT(
    tracker.publishFeedback(false, 0u, 1u, 2u, ops), ::testing::KilledBySignal(SIGSEGV), ".*");
}

TEST(RouteTrackerTest, test_node_achievement_simple)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("router_test");
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;

  // Test with straight line to do exact analysis easier. More realistic routes in the next test
  RouteTrackerWrapper tracker;
  tracker.configure(node, nullptr, costmap_subscriber, nullptr, "map", "base_link");

  // Create a demo route to test upon
  Node node1, node2, node3;
  node1.nodeid = 1;
  node1.coords.x = 0.0;
  node1.coords.y = 0.0;
  node2.nodeid = 2;
  node2.coords.x = 10.0;
  node2.coords.y = 0.0;
  node3.nodeid = 3;
  node3.coords.x = 20.0;
  node3.coords.y = 0.0;

  DirectionalEdge edge1, edge2;
  edge1.edgeid = 5;
  edge1.start = &node1;
  edge1.end = &node2;
  edge2.edgeid = 6;
  edge2.start = &node2;
  edge2.end = &node3;

  Route route;
  route.start_node = &node1;
  route.edges.push_back(&edge1);
  route.edges.push_back(&edge2);

  RouteTrackingState state;
  state.last_node = nullptr;
  state.next_node = &node1;
  state.current_edge = nullptr;
  state.route_edges_idx = -1;
  state.within_radius = false;

  geometry_msgs::msg::PoseStamped pose;

  // Test a few cases surrounding the line y = 10 where the triggering should occur.
  // In a single straight line for a simple test that the mathematical criteria works exactly
  // at the boundary. Tests below will test for odd / real angled routes to ensure functionality
  // in realistic conditions but without having to find the exact line equations after being proven
  state.last_node = &node1;
  state.next_node = &node2;
  state.current_edge = &edge1;
  state.route_edges_idx = 0;
  state.within_radius = false;
  pose.pose.position.x = 10.0;  // exact
  pose.pose.position.y = 0.0;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));
  state.within_radius = false;

  pose.pose.position.x = 9.99;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = 10.01;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));
  state.within_radius = false;

  // Set some tracking error
  pose.pose.position.y = 0.1;
  pose.pose.position.x = 9.99;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = 10.01;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));
  state.within_radius = false;

  // Test symmetry
  pose.pose.position.y = -0.1;
  pose.pose.position.x = 9.99;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = 10.01;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));
  state.within_radius = false;
}

TEST(RouteTrackerTest, test_node_achievement)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("router_test");
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;

  // Minimum threshold is 2m by default
  RouteTrackerWrapper tracker;
  tracker.configure(node, nullptr, costmap_subscriber, nullptr, "map", "base_link");

  // Create a demo route to test upon
  Node node1, node2, node3, node4;
  node1.nodeid = 1;
  node1.coords.x = 0.0;
  node1.coords.y = 0.0;
  node2.nodeid = 2;
  node2.coords.x = -10.0;
  node2.coords.y = 10.0;
  node3.nodeid = 3;
  node3.coords.x = -10.0;
  node3.coords.y = 20.0;
  node4.nodeid = 4;
  node4.coords.x = -20.0;
  node4.coords.y = 10.0;

  DirectionalEdge edge1, edge2, edge3;
  edge1.edgeid = 5;
  edge1.start = &node1;
  edge1.end = &node2;
  edge2.edgeid = 6;
  edge2.start = &node2;
  edge2.end = &node3;
  edge3.edgeid = 7;
  edge3.start = &node3;
  edge3.end = &node4;

  Route route;
  route.start_node = &node1;
  route.edges.push_back(&edge1);
  route.edges.push_back(&edge2);
  route.edges.push_back(&edge3);

  RouteTrackingState state;
  state.last_node = nullptr;
  state.next_node = &node1;
  state.current_edge = nullptr;
  state.route_edges_idx = -1;
  state.within_radius = false;

  geometry_msgs::msg::PoseStamped pose;

  // Test radius for start
  pose.pose.position.x = -1.5;
  pose.pose.position.y = 1.5;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -0.7;
  pose.pose.position.y = 0.7;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));

  // Test radius for end
  state.last_node = &node3;
  state.next_node = &node4;
  state.current_edge = &edge3;
  state.route_edges_idx = 2;
  state.within_radius = false;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -20.7;
  pose.pose.position.y = 10.7;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));

  // Test exactly on top of a node mid-execution
  state.last_node = &node2;
  state.next_node = &node3;
  state.current_edge = &edge2;
  state.route_edges_idx = 1;
  state.within_radius = false;
  pose.pose.position.x = -5.0;
  pose.pose.position.y = 5.0;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -10.0;
  pose.pose.position.y = 20.0;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));

  // Test within radius in last iteration, and now not
  state.within_radius = true;
  pose.pose.position.x = -1000.0;
  pose.pose.position.y = 1500.0;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));
  state.within_radius = false;

  // Test approaching in mid-execution with acute angle edge
  // A: Just on initial side in various locations
  pose.pose.position.x = -9.5;
  pose.pose.position.y = 19.5;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -9.9;
  pose.pose.position.y = 19.2;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -10.0;
  pose.pose.position.y = 19.9;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  // B: Just on other side in various locations
  pose.pose.position.x = -10.0;
  pose.pose.position.y = 20.01;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -9.9;
  pose.pose.position.y = 20.4;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -10.5;
  pose.pose.position.y = 20.5;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));

  // Repeat similar edges but now with Edge1 with an obtuse angle
  state.last_node = &node1;
  state.next_node = &node2;
  state.current_edge = &edge1;
  state.route_edges_idx = 0;
  state.within_radius = false;

  // A: Just on initial side
  pose.pose.position.x = -9.9;
  pose.pose.position.y = 9.9;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -9.7;
  pose.pose.position.y = 10.0;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -10.3;
  pose.pose.position.y = 9.7;
  EXPECT_FALSE(tracker.nodeAchieved(pose, state, route));
  // B: Just on other side
  pose.pose.position.x = -10.1;
  pose.pose.position.y = 10.1;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -10.3;
  pose.pose.position.y = 10.0;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));
  pose.pose.position.x = -9.5;
  pose.pose.position.y = 10.5;
  EXPECT_TRUE(tracker.nodeAchieved(pose, state, route));
}
