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
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_route/operations_manager.hpp"
#include "nav2_route/types.hpp"
#include "nav2_core/route_exceptions.hpp"
#include "nav2_route/plugins/route_operations/collision_monitor.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT

class CollisionMonitorWrapper : public CollisionMonitor
{
public:
  CollisionMonitorWrapper() = default;

  Coordinates backoutValidEndPointWrapper(
    const Coordinates & start, const Coordinates & end, const float dist)
  {
    return backoutValidEndPoint(start, end, dist);
  }

  bool lineToMapWrapper(
    const Coordinates & start, const Coordinates & end, LineSegment & line)
  {
    return lineToMap(start, end, line);
  }

  bool backoutValidEndPointWrapper(
    const Coordinates & start, LineSegment & line)
  {
    return backoutValidEndPoint(start, line);
  }

  bool isInCollisionWrapper(const LineSegment & line)
  {
    return isInCollision(line);
  }

  void getCostmapWrapper()
  {
    getCostmap();
  }
};

TEST(TestCollisionMonitor, test_lifecycle)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test");
  node->declare_parameter("costmap_topic", rclcpp::ParameterValue("dummy_topic"));
  CollisionMonitor monitor;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  monitor.configure(node, costmap_subscriber, "name");
  EXPECT_EQ(monitor.getName(), std::string("name"));
  EXPECT_EQ(monitor.processType(), RouteOperationType::ON_QUERY);
}

TEST(TestCollisionMonitor, test_geometric_backout_vector)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test");
  node->declare_parameter("costmap_topic", rclcpp::ParameterValue("local_costmap/costmap_raw"));
  node->declare_parameter("name.max_collision_dist", rclcpp::ParameterValue(-1.0));
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  CollisionMonitorWrapper monitor;
  monitor.configure(node, costmap_subscriber, "name");

  geometry_msgs::msg::PoseStamped pose;
  Coordinates start, end;
  start.x = 0.0;
  start.y = 0.0;
  end.x = 10.0;
  end.y = 0.0;
  float dist = 2.0;

  // Test with straight X / Y changes
  Coordinates rtn = monitor.backoutValidEndPointWrapper(start, end, dist);
  EXPECT_NEAR(rtn.x, 2.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  dist = 5.0;
  rtn = monitor.backoutValidEndPointWrapper(start, end, dist);
  EXPECT_NEAR(rtn.x, 5.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.0, 0.01);

  start.x = 0.0;
  start.y = 0.0;
  end.x = 0.0;
  end.y = 1.0;

  dist = 0.1;
  rtn = monitor.backoutValidEndPointWrapper(start, end, dist);
  EXPECT_NEAR(rtn.x, 0.0, 0.01);
  EXPECT_NEAR(rtn.y, 0.1, 0.01);

  dist = 15.0;
  rtn = monitor.backoutValidEndPointWrapper(start, end, dist);
  EXPECT_NEAR(rtn.x, 0.0, 0.01);
  EXPECT_NEAR(rtn.y, 15.0, 0.01);

  // Now that we know the basis work, let's try a none vector
  start.x = 0.0;
  start.y = 10.0;
  end.x = 0.0;
  end.y = 10.0;
  dist = 5.0;
  rtn = monitor.backoutValidEndPointWrapper(start, end, dist);
  EXPECT_NEAR(rtn.x, 0.0, 0.01);
  EXPECT_NEAR(rtn.y, 10.0, 0.01);

  // Now an x=y vector
  start.x = 0.0;
  start.y = 0.0;
  end.x = 10.0;
  end.y = 10.0;
  dist = 5.0;
  rtn = monitor.backoutValidEndPointWrapper(start, end, dist);
  EXPECT_NEAR(rtn.x, 3.53, 0.01);
  EXPECT_NEAR(rtn.y, 3.53, 0.01);

  // Let's try a random vector that I solved by hand
  start.x = 4.0;
  start.y = 10.0;
  end.x = 50.0;
  end.y = 100.0;
  dist = 45.0;
  rtn = monitor.backoutValidEndPointWrapper(start, end, dist);
  EXPECT_NEAR(rtn.x, 24.479, 0.01);
  EXPECT_NEAR(rtn.y, 50.069, 0.01);
}


TEST(TestCollisionMonitor, test_costmap_apis)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("test");
  node->declare_parameter("costmap_topic", rclcpp::ParameterValue("dummy_topic"));
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_subscriber;
  CollisionMonitorWrapper monitor;
  monitor.configure(node, costmap_subscriber, "name");

  // No costmap received yet
  EXPECT_THROW(monitor.getCostmapWrapper(), nav2_core::OperationFailed);

  // Create a demo costmap: * = 100, - = 0, / = 254
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  // * * * * / / / / - - - -
  // * * * * / / / / - - - -
  // * * * * / / / / - - - -
  // * * * * / / / / - - - -
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  // * * * * - - - - - - - -
  nav2_costmap_2d::Costmap2D * costmap =
    new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);
  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmap->setCost(i, j, 254);
    }
  }
  for (unsigned int i = 0; i < 40; ++i) {
    for (unsigned int j = 0; j < 100; ++j) {
      costmap->setCost(i, j, 100);
    }
  }

  nav2_costmap_2d::Costmap2DPublisher publisher(
    node, costmap, "map", "local_costmap/costmap", true);
  publisher.on_activate();
  publisher.publishCostmap();

  // Give it a moment to receive the costmap
  rclcpp::Rate r(10);
  r.sleep();
  monitor.getCostmapWrapper();  // Since would otherwise be called in `perform`

  Coordinates start, end;
  start.x = 1.0;
  start.y = 1.0;
  end.x = 10.0;
  end.y = 10.0;
  CollisionMonitor::LineSegment line;
  EXPECT_TRUE(monitor.lineToMapWrapper(start, end, line));
  EXPECT_EQ(line.x0, 9u);
  EXPECT_EQ(line.y0, 9u);
  EXPECT_EQ(line.x1, 99u);
  EXPECT_EQ(line.y1, 99u);

  end.x = 11.0;
  end.y = 11.0;
  EXPECT_FALSE(monitor.lineToMapWrapper(start, end, line));
  EXPECT_EQ(line.x0, 9u);
  EXPECT_EQ(line.y0, 9u);
  EXPECT_EQ(line.x1, 109u);
  EXPECT_EQ(line.y1, 109u);

  // Should backout to the costmap edge
  EXPECT_TRUE(monitor.backoutValidEndPointWrapper(start, line));
  EXPECT_EQ(line.x1, 99u);
  EXPECT_EQ(line.y1, 99u);

  // In this case, even starting point is invalid, reject
  start.x = -1.0;
  start.y = -1.0;
  EXPECT_FALSE(monitor.backoutValidEndPointWrapper(start, line));

  // Check collision checker
  start.x = 1.0;
  start.y = 1.0;
  end.x = 9.0;
  end.y = 1.0;
  EXPECT_TRUE(monitor.lineToMapWrapper(start, end, line));
  EXPECT_FALSE(monitor.isInCollisionWrapper(line));

  start.x = 1.0;
  start.y = 1.0;
  end.x = 1.0;
  end.y = 9.0;
  EXPECT_TRUE(monitor.lineToMapWrapper(start, end, line));
  EXPECT_FALSE(monitor.isInCollisionWrapper(line));

  start.x = 1.0;
  start.y = 1.0;
  end.x = 9.0;
  end.y = 9.0;
  EXPECT_TRUE(monitor.lineToMapWrapper(start, end, line));
  EXPECT_TRUE(monitor.isInCollisionWrapper(line));

  // Sanity check via a realistic graph route
  Node node1, node2, node3;
  node1.coords.x = 1.0;  // lower left corner
  node1.coords.y = 1.0;
  node2.coords.x = 1.0;  // upper left corner
  node2.coords.y = 9.0;
  node3.coords.x = 11.0;  // lower right corner, through center (collision), off edge
  node3.coords.y = -1.0;
  DirectionalEdge edge1, edge2;
  edge1.start = &node1;
  edge1.end = &node2;
  edge2.start = &node2;
  edge2.end = &node3;
  edge1.edgeid = 1u;
  edge2.edgeid = 2u;

  Route route;
  route.start_node = &node1;
  route.edges.push_back(&edge1);
  route.edges.push_back(&edge2);

  geometry_msgs::msg::PoseStamped pose;
  EdgePtr curr_edge = &edge1;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.0;

  // We are along the first edge, outside of 5m check from costmap lethal block
  // Need to wait long enough between calls due to collision monitor throttling
  OperationResult result = monitor.perform(
    &node1 /*unused*/, curr_edge, curr_edge /*unused*/, route, pose, nullptr);
  EXPECT_FALSE(result.reroute);
  rclcpp::Rate r1(0.8);
  r1.sleep();

  // Still along first bit of the route, 5m ahead still not in lethal block
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 5.0;
  result = monitor.perform(
    &node1 /*unused*/, curr_edge, curr_edge /*unused*/, route, pose, nullptr);
  EXPECT_FALSE(result.reroute);
  rclcpp::Rate r2(0.8);
  r2.sleep();

  // Now should be within range of lethal block
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 9.0;
  result = monitor.perform(
    &node1 /*unused*/, curr_edge, curr_edge /*unused*/, route, pose, nullptr);
  EXPECT_TRUE(result.reroute);
  EXPECT_EQ(result.blocked_ids.size(), 1u);
  EXPECT_EQ(result.blocked_ids[0], 2u);

  // But should be not reporting now if we immediately try again due
  // to not meeting the timer needs, even if we give it a better edge to use
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 9.0;
  curr_edge = &edge2;
  result = monitor.perform(
    &node1 /*unused*/, curr_edge, curr_edge /*unused*/, route, pose, nullptr);
  EXPECT_FALSE(result.reroute);
  EXPECT_EQ(result.blocked_ids.size(), 0u);

  rclcpp::Rate r3(0.8);
  r3.sleep();

  // Finally let's try once more after the collision block to make sure handles terminal situations
  pose.pose.position.x = 8.0;
  pose.pose.position.y = 2.0;
  curr_edge = &edge2;
  result = monitor.perform(
    &node1 /*unused*/, curr_edge, curr_edge /*unused*/, route, pose, nullptr);
  EXPECT_FALSE(result.reroute);
  EXPECT_EQ(result.blocked_ids.size(), 0u);
}
